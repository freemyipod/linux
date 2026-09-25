/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Device Tree binding constants for Samsung S5L8702 clock controller.
 *
 * IDs 0-2 are shared with the S5L8720-family controller
 * ("samsung,s5l8720-clock", N20/N31); everything else is only provided
 * by the S5L8702 controller ("samsung,s5l8702-clock", N46).
 */

#ifndef _DT_BINDINGS_CLOCK_SAMSUNG_S5L8702_CLOCK_H
#define _DT_BINDINGS_CLOCK_SAMSUNG_S5L8702_CLOCK_H

#define CLK_SHA1    0
#define CLK_AES     1
#define CLK_PRNG    2	/* S5L8720 family only */

/* S5L8702: PLLs and CG16 clock generators */
#define CLK_PLL0    3
#define CLK_PLL1    4
#define CLK_PLL2    5
#define CLK_FCLK    6	/* CG16_SYS */
#define CLK_CCLK    7	/* CPU */
#define CLK_HCLK    8	/* AHB */
#define CLK_PCLK    9	/* APB */
#define CLK_ECLK    10	/* CG16_RTIME: timers, MIU refresh */
#define CLK_AUD0    11	/* CG16_AUD0..2: I2S master clocks */
#define CLK_AUD1    12
#define CLK_AUD2    13

/* S5L8702: PWRCON_AHB gates */
#define CLK_LCD     14
#define CLK_USBOTG  15
#define CLK_SMX     16
#define CLK_SM1     17
#define CLK_ATA     18
#define CLK_NAND    19
#define CLK_SDCI    20
#define CLK_NANDECC 21
#define CLK_DMAC0   22
#define CLK_DMAC1   23
#define CLK_ROM     24

/* S5L8702: PWRCON_APB gates */
#define CLK_RTC     25
#define CLK_CWHEEL  26
#define CLK_SPI0    27
#define CLK_USBPHY  28
#define CLK_I2C0    29
#define CLK_TIMER   30
#define CLK_I2C1    31
#define CLK_I2S0    32
#define CLK_UART    33
#define CLK_I2S1    34
#define CLK_SPI1    35
#define CLK_GPIO    36
#define CLK_CHIPID  37
#define CLK_I2S2    38
#define CLK_SPI2    39

#endif /* _DT_BINDINGS_CLOCK_SAMSUNG_S5L8702_CLOCK_H */
