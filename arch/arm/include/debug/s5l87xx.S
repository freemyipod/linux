/* SPDX-License-Identifier: GPL-2.0-only */

#define S3C_ADDR_BASE   0xF6000000
#define S3C_VA_UART	S3C_ADDR_BASE + 0x01000000
#define S5L87XX_PA_UART	0x3CC00000

	/* note, for the boot process to work we have to keep the UART
	 * virtual address aligned to an 1MiB boundary for the L1
	 * mapping the head code makes. We keep the UART virtual address
	 * aligned and add in the offset when we load the value here.
	 */

	.macro addruart, rp, rv, tmp
		ldr	\rp, =S5L87XX_PA_UART
		ldr	\rv, =S3C_VA_UART
#if CONFIG_DEBUG_S3C_UART != 0
        add	\rp, \rp, 0xe00000
        add	\rv, \rv, 0xe00000
		add	\rp, \rp, #(0x100000 * CONFIG_DEBUG_S3C_UART)
		add	\rv, \rv, #(0x100000 * CONFIG_DEBUG_S3C_UART)
#endif
	.endm

#define fifo_full fifo_full_s3c2410
#define fifo_level fifo_level_s3c2410

#include <debug/samsung.S>
