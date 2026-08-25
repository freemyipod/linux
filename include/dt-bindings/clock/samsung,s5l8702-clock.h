/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DT bindings for Samsung/Apple S5L8702/S5L8740 CLKCON @0x3C500000
 *
 * Gate IDs match Rockbox CLOCKGATE_* (bank = id>>5, bit = id&31).
 * Enable polarity: CLEAR bit in PWRCONn (CLK_GATE_SET_TO_DISABLE).
 */

#ifndef _DT_BINDINGS_CLOCK_SAMSUNG_S5L8702_CLOCK_H
#define _DT_BINDINGS_CLOCK_SAMSUNG_S5L8702_CLOCK_H

/* Legacy crypto IDs (keep stable) */
#define CLK_SHA1		0
#define CLK_AES			1
#define CLK_PRNG		2

/* Fixed / derived */
#define CLK_OSC24		3
#define CLK_PCLK		4

/* PWRCON0 / AHB @ +0x48 (gates 0..31) */
#define CLK_LCD			5
#define CLK_USBOTG		6
#define CLK_SMX			7
#define CLK_SM1			8
#define CLK_ATA			9
#define CLK_NAND		10
#define CLK_AES_AHB		11
#define CLK_NANDECC		12
#define CLK_DMAC0		13
#define CLK_DMAC1		14
#define CLK_ROM			15

/* PWRCON1 / APB @ +0x4C (gates 32..63) — N31 proven subset */
#define CLK_RTC			16
#define CLK_CWHEEL		17
#define CLK_SPI0		18
#define CLK_USBPHY		19
#define CLK_I2C0		20
#define CLK_TIMER		21
#define CLK_I2C1		22
#define CLK_I2S0		23
#define CLK_UART		24
#define CLK_I2S1		25
#define CLK_SPI1		26
#define CLK_GPIO		27
#define CLK_CHIPID		28
#define CLK_I2S2		29
#define CLK_SPI2		30
/* Ambiguous 8720 map: SPI2 sometimes bit15 (gate 47) */
#define CLK_SPI2_ALT		31

/* PWRCON2 @ +0x58 */
#define CLK_SPI3		32

/* PWRCON4 secondary @ +0x6C (8720-style; harmless if RO on 8740) */
#define CLK_SPI0_2		33
#define CLK_SPI1_2		34
#define CLK_SPI2_2		35
#define CLK_I2C0_2		36
#define CLK_I2C1_2		37
#define CLK_UART_2		38
#define CLK_LCD_2		39
#define CLK_TIMERA_2		40

/* CG16 / RetailOS sub_41CBD8 gate IDs 7..13 (div-half enable = clear 0x8000/0x80000000) */
#define CLK_CG16_7		41
#define CLK_CG16_8		42
#define CLK_CG16_9		43
#define CLK_CG16_10		44
#define CLK_CG16_11		45
#define CLK_CG16_12		46
#define CLK_CG16_13		47

#define CLK_S5L8702_NR_CLKS	48

#endif
