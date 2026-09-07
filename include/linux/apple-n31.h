/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Cross-driver interfaces for the iPod nano 7 (N31, Samsung S5L8740).
 *
 * A handful of symbols on this board legitimately cross driver boundaries:
 * the PMIC gates the touch and audio rails, its interrupt arrives on an SoC
 * GPIO owned by another driver, and the touch controller reads its calibration
 * blob through the NAND FTL. Collecting the declarations here keeps them in
 * one place instead of being repeated as bare externs in each .c file, where
 * they were already starting to disagree with each other.
 *
 * Everything below is exported with EXPORT_SYMBOL_GPL() by the driver named
 * in the comment. Consumers must cope with the provider being absent, since
 * these are separate modules that can load in any order.
 */
#ifndef __LINUX_APPLE_N31_H
#define __LINUX_APPLE_N31_H

#include <linux/types.h>

struct device;
struct dma_chan;

/* irq-s5l8740-eic.c — route an SoC GPIO to the external interrupt controller. */
int s5l8740_eic_enable_gpio(unsigned int gpio, unsigned int irq_type);
/*
 * irq-s5l8740-eic.c - bring an SoC power domain up, as RetailOS sub_1234
 * does through the same block. Domain 2 is the display/LCDIF domain.
 */
int s5l8740_eic_domain_up(unsigned int domain);

/* gpio-s5l8740.c — report a key press to the board input device. */
void s5l8740_n31_report_key(unsigned int code, int pressed);

/* gpio-s5l8740.c — raw level of GPIO 86, the PMIC nIRQ line. */
int s5l8740_n31_din86(void);

/*
 * gpio-d1830.c — set by the PMIC driver so the GPIO edge handler can fold a
 * missed EIC edge back into the button poll. NULL until gpio-d1830 probes.
 */
extern void (*d1830_n31_din_nirq_hook)(void);

/* gpio-d1830.c — apply the audio LDO trim, for the CS42L81 codec. */
int d1830_audio_rails(void);

/* gpio-d1830.c — power the touch controller rail, for the Grape driver. */
int d1830_grape_rail(bool on);
/*
 * gpio-d1830.c — the touch controller's PMU_GPIO_8 line at register 0x51,
 * the other half of sub_439B00(1, on). 0x51 on, 0x11 off.
 */
int d1830_touch_gpio8(bool on);
/* Bluetooth companion rails: reg 87 bits 7:6, reg 88 bit 0 and bits 6:4,
 * the fields sub_51688C zeroes on de-init. Saves the boot values on the
 * first power-off so power-on can restore them. */
int d1830_bt_rails(bool on);

/*
 * The BCM2078's enable line, which is a PMIC GPIO-block output and not a
 * SoC pad. The steps are stock's, in stock's order; see d1830_bt_enable().
 */
#define D1830_BT_STEP_PREP	0	/* sub_51681C: 0x52, 0x56, then hold */
#define D1830_BT_STEP_RELEASE	1	/* sub_5169A8: hold, 50 ms, run */
#define D1830_BT_STEP_HOLD	2	/* sub_516700(0): hold, on its own */
int d1830_bt_enable(unsigned int step);


/*
 * gpio-d1830.c — refcounted rail control. Rail ids index the PMU rail
 * table; a rail stays up while any consumer holds it and powers down a
 * few seconds after the last release.
 *
 * Ownership below comes from the stock firmware's own rail dispatcher,
 * which maps a consumer id onto a bit in PMU_ACTIVE_1/2; the call sites
 * name the consumer.
 *
 *   LDO_3  ACTIVE_1 bit5  touch controller
 *   LDO_4  ACTIVE_1 bit6  display, enabled immediately before LCDIF init
 *   LDO_5  ACTIVE_1 bit7  accessory port, voltage negotiated 2.5-3.3 V
 *
 * No stock path enables a rail for the audio codec: its analog supply is
 * always on. Do not add one.
 */
/*
 * Indexes into n31_pmu_rails[], NOT RetailOS logical rail IDs. The two
 * numbering systems overlap and disagree, which is a trap worth naming:
 *
 *   this table index 2  -> 0x10 bit 5 -> Grape
 *   RetailOS logical 4  -> 0x10 bit 5 -> Grape
 *
 * Same physical bit, different number, and sub_6644 converts logical
 * IDs 1..10 into selectors 6..15 before sub_7484 turns those into
 * register and bit. So a bare 4 in a decompiler listing and a bare 4
 * here mean different rails. Always carry the {register, mask} pair.
 *
 * The physical assignments, from sub_7484:
 *   0x10 bits 2..7 are selectors 6..11
 *   0x11 bits 0..3 are selectors 12..15
 * Only 0x10 bit 5 has a proven consumer -- Grape, via the call chain
 * sub_20766(1) to sub_439B00(1) to sub_6644(4) to sub_7484(9). The
 * display and accessory names below are this project's mapping and are
 * not re-proven from the firmware.
 */
#define N31_PMU_RAIL_TOUCH	2	/* PMU_LDO_3, 0x10 bit 5, Grape */
#define N31_PMU_RAIL_DISPLAY	3	/* PMU_LDO_4, 0x10 bit 6, unproven */
#define N31_PMU_RAIL_ACCESSORY	4	/* PMU_LDO_5, 0x10 bit 7, unproven */

int n31_pmu_rail_get(unsigned int id);
void n31_pmu_rail_put(unsigned int id);

/*
 * backlight-s5l8740.c — ramp the LED boost. n31_backlight_fade() returns
 * the level that was set before the ramp began, so a screen-sleep caller
 * can restore exactly what the user had. ms = 0 applies immediately.
 */
int n31_backlight_fade(int level, unsigned int ms);
int n31_backlight_level(void);

/*
 * apple-grape.c — screen-sleep hooks for the touch controller. Whether
 * the rail is cut is the driver's own touch_power_down parameter; the
 * caller only says sleep or wake.
 */
int n31_touch_suspend(void);
int n31_touch_resume(void);

/*
 * s5l8740.c (DRM) — display power. Turning the panel off means stopping
 * the LCDIF and dropping its rail; turning it back on resets and
 * reprograms the interface and repaints. The panel itself needs no
 * command sequence.
 */
int n31_lcd_power(bool on);
bool n31_lcd_is_on(void);

/* spi-s5l8702.c — reapply the SPI2 engine setup after a pinmux change. */
void s5l8702_spi2_reinit(void);

/*
 * gpio-d1830.c -- the backlight, which is the PMIC's white-LED driver and not
 * anything in the SoC: WLED_ISET at 0x25 is the LED current (7 bits) and
 * WLED_CTRL at 0x26 bit 0 the enable. Brightness is clamped to the code the
 * bootloader left, so a caller cannot ask for more current than the panel is
 * already running on.
 */
int d1830_wled_set(unsigned int level, unsigned int max);
int d1830_wled_get(void);

/*
 * backlight-s5l8740.c is built in and gpio-d1830 is a module, so the built-in
 * publishes the hook and the module fills it in -- the same shape as
 * bcm2078_register_bt_rails().
 */
void n31_backlight_register_wled(int (*fn)(unsigned int level, unsigned int max));

/* cs42l81-spi.c — true while the analog play graph is latched. */
bool n31_audio_playback_active(void);

/*
 * bcm2078-bt.c — the FM tuner's own mute (AUDIO_CTRL MANUAL_MUTE, sent as
 * 0xFC15). The machine driver owns the mixer and this owns the radio, so the
 * control lives there and the register write lives here. There is no FM volume
 * register in the recovered map; level on the FM path is the codec's playback
 * volume once the audio is looped to the headphones.
 */
int bcm2078_fm_mute_get(void);
int bcm2078_fm_mute_set(bool mute);

/*
 * bcm2078-bt.c -- the SoC side of the FM audio enable. Stock's sub_42A8C
 * reprograms the IIS2 dividers from a bit clock chosen by the tuned station
 * before it puts tuner audio on the PCM port, so the tuner drives this and
 * s5l8740-i2s (a module) fills the hook in.
 */
void bcm2078_register_fm_pcm_clk(int (*fn)(unsigned int bitclk_hz));

/*
 * cs42l81-spi.c — the headset model, per stock's numbering (sub_140EC8 at
 * 0x140EC8). 0 is removed and 11 an open circuit; anything else is something in
 * the jack. Negative is an error and must not be read as "unplugged".
 */
int cs42l81_headset_model(void);

/*
 * cs42l81-spi.c — MikeyBus, which is a byte FIFO at codec registers
 * 0x051E..0x0528. Not a UART, and nothing to do with the audio path: RetailOS
 * contains 0x3DC00000 only in a generic descriptor table, with no owner.
 */
int cs42l81_mbox_level(void);
int cs42l81_mbox_read_frame(u8 *buf, size_t buf_size);
int cs42l81_mbox_enable(bool on);
int cs42l81_mbox_reset(void);

/* apple-mikeybus.c — presence, derived from the model above. */
int apple_mikeybus_jack_present(void);
int apple_mikeybus_headset_ready(void);

/*
 * hci_bcm (patched, see patch-hci-bcm-link-recovery.py) — the board's radio
 * reset, for its escalating recovery from a desynchronised UART and for the top
 * of every setup run. Must leave the part unpatched at its initial baud, which
 * is where setup expects to find it.
 */
void n31_bcm_register_radio_reset(int (*fn)(void));

/*
 * apple-syscfg.c -- this unit's identity from the bootloader's "IsyS" RAM
 * handoff, with no storage stack involved. All return -ENODEV until that
 * driver has probed and validated the object.
 */
/* Bluetooth address, 6 bytes, most significant octet first. */
int apple_syscfg_bd_addr(u8 out[6]);
/* Serial, NUL-terminated; needs a 33-byte buffer. -ENODATA if unprogrammed. */
int apple_syscfg_serial(char *out, size_t len);
/* Accelerometer per-axis zero offsets, signed 16.16. -ENODATA if unprogrammed. */
int apple_syscfg_accel_offsets(s32 out[3]);

/* nand-s5l8740.c — true once the FTL has a usable logical-to-virtual map. */
bool nand_ftl_present(void);

/* nand-s5l8740.c — read one 4096-byte logical sector through the FTL. */
int nand_ftl_read_sector(u64 logical_sector, void *buf);
/*
 * ftl-s5l8740.c — the touch calibration out of SysCfg, as the 0x560-byte
 * object apple-grape expects with the calibration at +350. Returns its
 * length, or 0 when SysCfg has not been read or carries no calibration.
 */
size_t whimory_syscfg_touch_cal(const u8 **out);

/*
 * dma-s5l8740-pl080.c — slave-channel lookup. The I2S and IIS2 request lines
 * are fixed by the SoC rather than described in the device tree, so consumers
 * ask for them by index or by peripheral number instead of going through the
 * usual of_dma path.
 */
struct dma_chan *s5l_pl080_request_slave(struct device *consumer,
					 unsigned int idx);
struct dma_chan *s5l_pl080_lookup_peri(unsigned int peri);
int s5l_pl080_rearm_set_src(struct dma_chan *c, dma_addr_t addr,
			    size_t bytes);
/*
 * Largest byte count one LLI node can carry at the width the channel is
 * configured for. A period bigger than this is split across several nodes,
 * which the self-linked ring cannot express -- see s5l_pl080_rearm_set_ring().
 */
size_t s5l_pl080_max_seg_bytes(void);

int s5l_pl080_rearm_set_ring(struct dma_chan *c, dma_addr_t base, size_t bytes,
			     size_t period);
int s5l_pl080_peri_snapshot(unsigned int peri, u32 *src, u32 *dst, u32 *en);
/* SRC, DST, LLI, CTL, CFG, CONTROL2, RAW_TC, ENBLD; returns the channel. */
int s5l_pl080_peri_regs(unsigned int peri, u32 *out, unsigned int n);
/*
 * dma-s5l8740-pl080.c — raise burst requests on a channel's peripheral line
 * from software, for a peripheral that never asserts its own. Opt-in per
 * channel: every other peripheral here does assert.
 */
void s5l_pl080_soft_req(struct dma_chan *chan, bool on);

/*
 * i2c-s5l8702.c — one bus's IICCON prescaler, addressed by physical base
 * because these buses are numbered by the SoC and not by probe order.
 *
 * The FM tuner's bus runs slower for as long as the radio is on. Stock does it
 * in the FM audio enable itself, immediately before it programs the IIS2
 * dividers: sub_570590(1, 7) on the way up and sub_570590(1, 2) on the way
 * down, which are the two field values below against I2C1 at 0x3C900000.
 */
#define S5L8702_I2C1_PHYS		0x3c900000u
#define S5L8702_I2C_SCALE_DEFAULT	1
#define S5L8702_I2C_SCALE_FM_ON		6
int s5l8702_i2c_set_clock_scale(u32 phys_base, unsigned int scale);

struct spi_device;

/*
 * What the last transfer on @spi's controller actually received, from
 * spi-s5l8702.
 *
 * @status is SPISTATUS as it stood immediately before the final RXDATA
 * read; @shortfall is how many requested receive bytes never arrived.
 * A shortfall equal to the transfer length means nothing came back, and
 * whatever is in the receive buffer is an empty-FIFO read rather than an
 * answer -- on this block that reads out as the 0x48/0x79 pattern, so
 * the bytes alone cannot be trusted to say a device replied.
 *
 * Returns -ENODEV when @spi is not on an s5l8702 controller.
 */
int s5l8702_spi_last_rx(struct spi_device *spi, u32 *status,
			unsigned int *shortfall, u32 *wait_status);

#endif /* __LINUX_APPLE_N31_H */
