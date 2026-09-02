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
/* Bluetooth companion rails: reg 87 bits 7:6, reg 88 bit 0 and bits 6:4,
 * the fields sub_51688C zeroes on de-init. Saves the boot values on the
 * first power-off so power-on can restore them. */
int d1830_bt_rails(bool on);

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

/* cs42l81-spi.c — true while the analog play graph is latched. */
bool n31_audio_playback_active(void);

/* nand-s5l8740.c — true once the FTL has a usable logical-to-virtual map. */
bool nand_ftl_present(void);

/* nand-s5l8740.c — read one 4096-byte logical sector through the FTL. */
int nand_ftl_read_sector(u64 logical_sector, void *buf);

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

#endif /* __LINUX_APPLE_N31_H */
