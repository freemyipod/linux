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

/* gpio-d1830.c — power the touch controller rail, for the Nimbus driver. */
int d1830_nimbus_rail(bool on);

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
int s5l_pl080_peri_snapshot(unsigned int peri, u32 *src, u32 *dst, u32 *en);

#endif /* __LINUX_APPLE_N31_H */
