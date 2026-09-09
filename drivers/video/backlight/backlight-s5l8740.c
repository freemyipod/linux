// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform backlight for Samsung/Apple S5L8740 (iPod nano 7G / N31)
 *
 * MMIO block @ 0x3E000000 (LCD AUX / backlight):
 *   +0x04  enable — bit0
 *   +0x08  level  — low 8 bits hold 1..62; bit0 also acts as enable
 *
 * Init (matches U-Boot / panel bring-up): write 62 to +0x08, then set bit0
 * on +0x04 and +0x08. Brightness 0 = off; userspace 1..max → HW 1..62.
 *
 * This is MMIO, not the Dialog PMIC, and that is from the decomp rather
 * than from anyone's recollection:
 *
 *   - The stock bootloader ends its display bring-up with
 *     sub_3522(0x3E000008, 62) -- sub_3522 is its plain MMIO write
 *     helper, the same one used for CLKCON and IIC -- immediately after
 *     the 0x3D7000xx LCDIF/DSI block and just before 0x3D700008 = 1.
 *     62 is exactly this register's full-scale level.
 *   - OSOS sub_4399FC sets 0x3E000008 bit 0 when SoC power domain 2
 *     comes up, and domain 2 is the LCD domain. So the domain gates the
 *     block; it does not supply the level.
 *
 * The brightness level IS on the PMIC, and an earlier revision of this
 * comment saying no D1830 register carries it was wrong. sub_A2650 writes
 * a 16-bit level as D1830 reg 0x24 = level bits 15:8 and reg 0x25 bits
 * 2:0 = level bits 7:5 (0x25 is PMU_WLED_ISET in gpio-d1830.c), or, when
 * the PMIC handle has its fast-path function installed, as one 16-bit
 * word on the single-wire transmitter at 0x3DE00000. See
 * docs/N31-BACKLIGHT-BRIGHTNESS-PATH.md. Not implemented here yet.
 *
 * Kconfig fragment (wire Makefile / Kconfig separately):
 *   config BACKLIGHT_S5L8740
 *   	tristate "Samsung/Apple S5L8740 backlight"
 *   	depends on BACKLIGHT_CLASS_DEVICE && (ARCH_S5L8740 || COMPILE_TEST)
 *   	default y if ARCH_S5L8740
 *   	help
 *   	  LCD backlight at 0x3E000000 for iPod nano 7G (N31).
 */
#include <linux/backlight.h>
#include <linux/apple-n31.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

/*
 * The brightness control is on the PMIC, not in this MMIO block.
 *
 * What this driver used to write, and why it never did anything:
 *
 *	#define S5L8740_BL_ENABLE_OFF   0x04
 *	#define S5L8740_BL_LEVEL_OFF    0x08
 *	#define S5L8740_BL_MAX          62
 *
 * with 62 described as "this register's full-scale level" and bits 1..5 of
 * 0x3E000008 treated as a magnitude. 62 is 0b111110 -- five separate bits
 * with bit 0 clear -- and bit 0 is the only bit at that address the hardware
 * owns. Its two writers in the whole image are the SoC power-domain enable
 * and its counterpart:
 *
 *	sub_4399FC  MEMORY[0x3E000008] |= 1u;    (display domain comes up)
 *	sub_1234    MEMORY[0x3E000008] &= ~1u;   (domain goes down)
 *
 * Nothing anywhere writes a variable to it. The whole 0x3E000000 block is
 * single-bit and two-bit fields; 0x3E00000C takes 0, 1 and 3 from sub_A06,
 * and 0x3E000004 bit 0 is set and cleared by the media-engine start and
 * stop. It is a power gate, and a sweep of 62 -> 1 -> 0 -> 62 on the real
 * panel changed nothing, exactly as a power gate would not.
 *
 * Where the level lives was found on 2026-09-07: D1830 regs 0x24/0x25 via
 * sub_A2650 (see the header comment and docs/N31-BACKLIGHT-BRIGHTNESS-PATH.md).
 * It was earlier believed to be D1830 register 0x2A bits 5:0; that register
 * is charge current, owned by ChargeMgmtTask, and the reasoning behind that
 * mistake is recorded on it in gpio-d1830.c. This driver still writes
 * nothing anywhere -- see s5l8740_bl_hw_set() -- until the 0x24/0x25 path
 * is wired through the PMIC provider.
 */
/* A scale for userspace only; nothing downstream consumes it. */
#define S5L8740_BL_MAX		255	/* the D1830's coarse level byte, 1:1 */

/*
 * The LED boost is the expensive part of the display, so screen sleep
 * ramps it rather than cutting it. Stepping happens in a work item so a
 * caller in a button handler does not block for the length of the fade.
 */
struct s5l8740_bl {
	void __iomem *base;
	struct backlight_device *bd;
	struct delayed_work fade;
	struct mutex lock;
	int level;		/* what the hardware currently has */
	int last_lit;		/* the last level above zero it had */
	int target;
	unsigned int step_ms;
};

static struct s5l8740_bl *s5l8740_bl_dev;

/*
 * The PMIC provider, registered rather than called directly.
 *
 * This driver is built into the kernel and gpio-d1830 is a module userspace
 * loads later, so a direct call does not link: vmlinux cannot reference a
 * module's exports. Same hook shape as bcm2078_register_bt_rails() and for the
 * same reason.
 *
 * With a deferred replay, because here the ordering is harmless. The last
 * requested level is remembered and applied when the provider arrives, so a
 * brightness set during boot is not silently lost -- the panel is already lit by
 * the bootloader at that point, so replaying late only moves it to what was
 * actually asked for.
 */
static int (*s5l8740_bl_wled_fn)(unsigned int level, unsigned int max);
static int s5l8740_bl_wled_want = -1;

void n31_backlight_register_wled(int (*set)(unsigned int level, unsigned int max),
				 int (*get)(void))
{
	struct s5l8740_bl *bl = s5l8740_bl_dev;

	s5l8740_bl_wled_fn = set;

	if (set && s5l8740_bl_wled_want >= 0) {
		int ret = set((unsigned int)s5l8740_bl_wled_want, S5L8740_BL_MAX);

		pr_info("s5l8740-bl: wled provider arrived; applied %d: %d\n",
			s5l8740_bl_wled_want, ret);
		s5l8740_bl_wled_want = -1;
	} else if (set && get && bl) {
		/*
		 * Nothing has been asked for yet: report what the hardware
		 * holds (the bootloader's level) instead of a made-up full
		 * scale, so the first userspace read is true.
		 */
		int cur = get();

		if (cur >= 0) {
			mutex_lock(&bl->lock);
			bl->level = bl->target = cur;
			if (bl->bd)
				bl->bd->props.brightness = cur;
			mutex_unlock(&bl->lock);
			pr_info("s5l8740-bl: wled provider arrived; hardware at %d/%d\n",
				cur, S5L8740_BL_MAX);
		}
	}
}
EXPORT_SYMBOL_GPL(n31_backlight_register_wled);

static void s5l8740_bl_hw_set(struct s5l8740_bl *bl, int level)
{
	int ret;

	if (level > S5L8740_BL_MAX)
		level = S5L8740_BL_MAX;
	if (level < 0)
		level = 0;
	bl->level = level;
	if (level > 0)
		bl->last_lit = level;

	/*
	 * The backlight is in the PMIC's white-LED driver, not in this block.
	 *
	 * WLED_ISET at PMIC 0x25 is the LED current -- the image writes bits 2:0,
	 * so eight levels -- and WLED_CTRL at 0x26 bit 0 is the enable. Traced
	 * from the display power-on: sub_1C20 -> sub_1D04 -> sub_4D08 ->
	 * sub_BA50 -> sub_D438, which read-modify-writes 0x26 bit 0, with
	 * sub_1C20 asserting pad 14 immediately afterwards.
	 *
	 * So pad 14 was never the backlight enable on its own; it is asserted
	 * together with the PMIC enable and power-domain resource 9.
	 *
	 * gpio-d1830 owns the PMIC bus and clamps to the current the bootloader
	 * left, so this can only ask for less than the panel is already running.
	 * -ENODEV means that module is not loaded yet, which is ordinary during
	 * boot and not worth a message.
	 */
	if (!s5l8740_bl_wled_fn) {
		s5l8740_bl_wled_want = level;
		return;
	}
	ret = s5l8740_bl_wled_fn((unsigned int)level, S5L8740_BL_MAX);
	if (ret && ret != -ENODEV)
		pr_warn_ratelimited("s5l8740-bl: wled %d: %d\n", level, ret);

}

static void s5l8740_bl_fade_work(struct work_struct *work)
{
	struct s5l8740_bl *bl = container_of(to_delayed_work(work),
					     struct s5l8740_bl, fade);
	bool more;

	mutex_lock(&bl->lock);
	if (bl->level < bl->target)
		s5l8740_bl_hw_set(bl, bl->level + 1);
	else if (bl->level > bl->target)
		s5l8740_bl_hw_set(bl, bl->level - 1);
	more = bl->level != bl->target;
	mutex_unlock(&bl->lock);

	if (more)
		schedule_delayed_work(&bl->fade,
				      msecs_to_jiffies(bl->step_ms));
}

/*
 * Ramp to `level` over roughly `ms`. ms = 0 jumps straight there, which is
 * what the backlight class writes should do. Returns the level that was
 * programmed before the fade started, so a caller can restore it on wake.
 */
int n31_backlight_fade(int level, unsigned int ms)
{
	struct s5l8740_bl *bl = s5l8740_bl_dev;
	int previous, delta;

	if (!bl)
		return -ENODEV;
	if (level < 0)
		level = 0;
	if (level > S5L8740_BL_MAX)
		level = S5L8740_BL_MAX;

	cancel_delayed_work_sync(&bl->fade);
	mutex_lock(&bl->lock);
	previous = bl->level;
	bl->target = level;
	delta = abs(level - bl->level);
	if (!ms || !delta) {
		s5l8740_bl_hw_set(bl, level);
		mutex_unlock(&bl->lock);
		return previous;
	}
	bl->step_ms = max_t(unsigned int, ms / delta, 1);
	mutex_unlock(&bl->lock);

	schedule_delayed_work(&bl->fade, msecs_to_jiffies(bl->step_ms));
	return previous;
}
EXPORT_SYMBOL_GPL(n31_backlight_fade);

/*
 * The level to come back to after a screen sleep.
 *
 * Not the hardware's current level: by the time the sleep is requested
 * the class may already have been blanked by whoever asked for it, and
 * restoring that zero wakes the panel into the dark (measured 2026-09-08:
 * display on, glass off). The class's own brightness is the level the
 * user set; failing that, the last level the hardware was actually lit
 * at; failing that, the current level.
 */
int n31_backlight_level(void)
{
	struct s5l8740_bl *bl = s5l8740_bl_dev;

	if (!bl)
		return -ENODEV;
	if (bl->bd && bl->bd->props.brightness > 0)
		return bl->bd->props.brightness;
	if (bl->level > 0)
		return bl->level;
	return bl->last_lit > 0 ? bl->last_lit : bl->level;
}
EXPORT_SYMBOL_GPL(n31_backlight_level);

static int s5l8740_bl_update_status(struct backlight_device *bd)
{
	struct s5l8740_bl *bl = bl_get_data(bd);
	int brightness = backlight_get_brightness(bd);

	cancel_delayed_work_sync(&bl->fade);
	mutex_lock(&bl->lock);
	bl->target = brightness;
	s5l8740_bl_hw_set(bl, brightness);
	mutex_unlock(&bl->lock);
	return 0;
}

static const struct backlight_ops s5l8740_bl_ops = {
	.update_status = s5l8740_bl_update_status,
};

static int s5l8740_bl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8740_bl *bl;
	struct backlight_properties props = { };
	struct backlight_device *bd;

	bl = devm_kzalloc(dev, sizeof(*bl), GFP_KERNEL);
	if (!bl)
		return -ENOMEM;

	bl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bl->base))
		return PTR_ERR(bl->base);

	mutex_init(&bl->lock);
	INIT_DELAYED_WORK(&bl->fade, s5l8740_bl_fade_work);
	/*
	 * The panel is already lit by the bootloader at its own level; do
	 * not ask for anything until userspace does. The class starts at
	 * full scale only until the PMIC provider arrives and reports the
	 * real value (n31_backlight_register_wled()).
	 */
	bl->level = S5L8740_BL_MAX;
	bl->target = S5L8740_BL_MAX;

	props.type = BACKLIGHT_RAW;
	props.max_brightness = S5L8740_BL_MAX;
	props.brightness = S5L8740_BL_MAX;

	bd = devm_backlight_device_register(dev, "s5l8740-backlight", dev, bl,
					   &s5l8740_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	bl->bd = bd;
	s5l8740_bl_dev = bl;
	platform_set_drvdata(pdev, bl);
	dev_info(dev, "S5L8740 backlight @%pR max=%u\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0), S5L8740_BL_MAX);
	return 0;
}

static const struct of_device_id s5l8740_bl_of_match[] = {
	{ .compatible = "apple,s5l8740-backlight" },
	{ .compatible = "samsung,s5l8740-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_bl_of_match);

static void s5l8740_bl_remove(struct platform_device *pdev)
{
	struct s5l8740_bl *bl = platform_get_drvdata(pdev);

	s5l8740_bl_dev = NULL;
	cancel_delayed_work_sync(&bl->fade);
}

static struct platform_driver s5l8740_bl_driver = {
	.probe = s5l8740_bl_probe,
	.remove = s5l8740_bl_remove,
	.driver = {
		.name = "backlight-s5l8740",
		.of_match_table = s5l8740_bl_of_match,
	},
};
module_platform_driver(s5l8740_bl_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8740 LCD backlight");
MODULE_LICENSE("GPL");
