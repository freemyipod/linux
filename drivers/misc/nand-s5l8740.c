// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 FMSS/FMC NAND controller driver (N31).
 *
 * Provides FIL primitives used by the FTL: READ ID, parameter page, page
 * read (PIO and CS command-list paths), and geometry helpers. Array
 * program and erase are not implemented.
 *
 * CS physical reads use span-4 / 4112-byte records with four 4096+16 slots
 * per page. True metadata DMA remains optional and disabled by default.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/math.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/vmalloc.h>
#include <linux/unaligned.h>
#include <asm/cacheflush.h>

#include "nand-s5l8740-seq.h"
#include "nand-s5l8740.h"
#include "whimory-ftl.h"

#define FMSS_PHYS		0x38A00000ul
#define FMSS_SIZE		0x1000

#define FMCTRL0			0x00
#define FMCTRL1			0x04
#define FMCMD			0x08
#define FMADDR			0x0c
#define FMCE			0x14
#define FMUNK18			0x18
#define FMUNK24			0x24
#define FMUNK28			0x28
#define FMCYCLES		0x2c
#define FMLEN			0x30
#define FMUNK38			0x38
#define FMSTAT48		0x48
#define NANDSTAT		0x4c
#define FMDATA			0x80
#define FMSEQ			0xc00
#define FMSEQBASE		0xc04
#define FMSEQSTAT		0xc08
#define FMSEQIRQ		0xc0c
#define FMGEN0			0xd00
#define FMGEN1			0xd04
#define FMGEN2			0xd08
#define FMGEN3			0xd0c
#define FMGEN4			0xd10
#define FMGEN5			0xd14
#define FMUNK81C		0x81c

#define FMSS_DMA_STATUS_LEN	512
#define FMSS_DMA_CMDLIST_LEN	256
#define FMSS_DMA_SPARE_LEN	256
#define FMSS_SECTOR_LEN		4096

#define FMSS_MAX_CHUNKS		16
#define FMSS_CHUNK		1024
#define FMSS_PAGE_LEN		(FMSS_MAX_CHUNKS * FMSS_CHUNK)
#define FMSS_PARAM_LEN		0x200

/*
 * PPN address packing — OSOS 5173CA / Sogeti PPNVFL:
 * page | (block << page_bits) | (cau << (page_bits+block_bits)) | (slc <<...)
 * Param page: page_bits=7, block_bits=12, cau_bits=1.
 * VFL context lives in the last ~5% of each CAU, SLC page 0
 * (spare type 0x20, index 0xFFFF). SFTL context spare type 0x1F.
 */
#define FMSS_PAGE_BITS		7
#define FMSS_BLOCK_BITS		12
#define FMSS_CAU_BITS		1
#define FMSS_BLOCKS_PER_CAU	2088
#define FMSS_NUM_CE		2
#define FMSS_NUM_CAU		2
#define FMSS_VFL_TAIL		128
#define FMSS_VFL_HDR_LEN	512
#define FMSS_BTOC_PAGE		127
#define FMSS_VBAS_PER_PAGE	4	/* s_g_vbas_per_page: 16 KiB / 4096 */
#define FMSS_LPN_INDEX_MAX	512
#define FMSS_GREP_MAX_BLOCKS	64
#define FMSS_VFL_MAP_MAX	512
#define FMSS_L2V_DEFAULT_BLOCKS	256
/* Map is keyed by page LPN (YaFTL); full LBA map preferred for block I/O. */
#define FMSS_L2V_DEFAULT_MAX_LPN \
	((NAND_FTL_DEFAULT_CAPACITY / NAND_FTL_SECTORS_PER_LPN) + 64)

/* Classic Whimory mount (freemyipod) adapted for N31 PPN geometry. */
#define WMR_PAGES_PER_BLOCK	128u
#define WMR_BLOCK_MAP_MAX	16384u
#define WMR_MOUNT_MAX_BLOCKS	64u
#define WMR_FTLCTRL_MAX		3u

/* Packed L2V entry:
 * valid|ce[1:0]|cau[1:0]|PHYS|sec[1:0]|block[11:0]|page[6:0]
 * sec = 4K index within the NAND page (SFTL VBA). 0x3 = “use LBA%4”.
 * PHYS: block is already physical (BTOC/BTE/META/carve) — do NOT VFL-remap.
 */
#define L2V_VALID		BIT(31)
#define L2V_CE_SHIFT		29
#define L2V_CAU_SHIFT		27
#define L2V_PHYS		BIT(26)	/* already-physical block */
#define L2V_SEC_SHIFT		19
#define L2V_SEC_MASK		0x3u
#define L2V_SEC_FROM_LBA	0x3u	/* sentinel: derive sec from lba%4 */
#define L2V_BLOCK_SHIFT		7
#define L2V_PAGE_MASK		0x7fu
#define L2V_BLOCK_MASK		0xfffu

/* Claim source priority for newest-wins (higher wins on equal weave). */
enum {
	L2V_SRC_NONE = 0,
	L2V_SRC_CARVE = 1,
	L2V_SRC_WMR = 2,	/* classic virt block-map (may need VFL) */
	L2V_SRC_BTOC = 3,
	L2V_SRC_BTE = 4,
	L2V_SRC_META = 5,
};

struct fmss_vfl_map {
	unsigned int cau;
	unsigned int virt;
	unsigned int phys;
};

struct fmss_lpn_map {
	unsigned int lpn;
	unsigned int ce;
	unsigned int cau;
	unsigned int block;
	unsigned int page;
};

static char vfl_log[PAGE_SIZE];
static unsigned int vfl_log_len;
static char sector_log[512];
static unsigned int sector_log_len;
static unsigned int lpn_scan_blocks = 512;
module_param(lpn_scan_blocks, uint, 0644);
MODULE_PARM_DESC(lpn_scan_blocks, "max CAU blocks to scan for lpn_read (default 512)");

static unsigned int l2v_scan_blocks = FMSS_L2V_DEFAULT_BLOCKS;
module_param(l2v_scan_blocks, uint, 0644);
MODULE_PARM_DESC(l2v_scan_blocks,
		 "default block count per CE/CAU for l2v_build (default 256)");

static unsigned int grep_max_blocks = 64;
module_param(grep_max_blocks, uint, 0644);
MODULE_PARM_DESC(grep_max_blocks, "max blocks per ftl_grep call (default 64)");

static int quiet = 1;
module_param(quiet, int, 0644);
MODULE_PARM_DESC(quiet, "1=minimal logs, skip ECC diag (faster FTL scans, default on)");

#define fmss_info(fmt, ...) \
	do { if (!quiet) pr_info("s5l8740-nand: " fmt, ##__VA_ARGS__); } while (0)

#define nand_dev_info(dev, fmt, ...) \
	do { if (!quiet) dev_info(dev, fmt, ##__VA_ARGS__); } while (0)

static unsigned int vfl_build_blocks = 32;
module_param(vfl_build_blocks, uint, 0644);
MODULE_PARM_DESC(vfl_build_blocks, "max blocks per CAU for vfl_build (default 32, tail only)");

/*
 * VFL remap of map entries that are NOT marked L2V_PHYS.
 * BTOC/BTE/carve store physical scan blocks — remapping those double-translates.
 * Default off until wrmx+0x100 is proven as direct virt→phys.
 */
static char vfl_remap_mode[16] = "off";
module_param_string(vfl_remap_mode, vfl_remap_mode, sizeof(vfl_remap_mode), 0644);
MODULE_PARM_DESC(vfl_remap_mode,
		 "off (default) | direct256 | tail_only — VFL remap for non-PHYS map entries");

static unsigned int vfl_remap_applied;
static unsigned int vfl_remap_skipped_phys;

/* Tiny list for sysfs lpn_index (debug); dense map is authoritative. */
static struct fmss_lpn_map lpn_index[FMSS_LPN_INDEX_MAX];
static unsigned int lpn_index_count;

/* Dense LPN → physical page map (vzalloc). */
static u32 *l2v_map;
static u64 *l2v_weave;
static u8 *l2v_src;
static unsigned int l2v_map_size;
static unsigned int l2v_mapped;
static unsigned int l2v_max_lpn;
static unsigned int l2v_btoc_hits;
static unsigned int l2v_bmap_hits;
static unsigned int l2v_meta_hits;

/*
 * Full LBA → packed phys+sec map (SFTL BTE / boot carve). Preferred over LPN
 * dense map for block I/O.
 *
 * WARNING: NAND_FTL_DEFAULT_CAPACITY is ~3.8M *sectors* (~15GB media). A dense
 * map of that size is ~50MB+ RAM (u32+u64+u8) and OOMs N31 before RNDIS.
 * Cap with lba_map_max (default 262144 ≈ 1GB LBA space ≈ 3.4MB RAM).
 */
#define FMSS_LBA_MAP_HARDMAX	NAND_FTL_DEFAULT_CAPACITY
static unsigned int lba_map_max = 262144;
module_param(lba_map_max, uint, 0644);
MODULE_PARM_DESC(lba_map_max,
		 "max LBA entries for dense map (default 262144; full media is ~3.8M / ~50MB)");
static u32 *lba_map;
static u64 *lba_weave;
static u8 *lba_src;
static unsigned int lba_mapped;

/* Last resolution chain for sysfs resolve_log. */
static char resolve_log[512];
static unsigned int resolve_log_len;

/* Carved Apple FAT boot (*UOKJIHC); sector 0 served from here. */
static bool boot_carve_valid;
static unsigned int boot_carve_ce;
static unsigned int boot_carve_cau;
static unsigned int boot_carve_block;
static unsigned int boot_carve_page;
static unsigned int boot_carve_off;
/* From carved BPB; reserved area must not be served from a poisoned L2V[0]. */
static unsigned int boot_reserved_sects = 32;
static unsigned int boot_data_start = 1916;
static unsigned int boot_fat_sects = 942; /* FATSz32 from live BPB */

/*
 * Optional cached aligned boot page (page must have BPB at offset 0).
 * Defaults 0 = rediscover via BTOC(0,1) + aligned BPB. Do NOT cache the
 * mid-page *UOKJIHC @ blk63/pg88/off7816 — that is a file copy, not LBA0.
 */
static unsigned int boot_carve_ce_param;
static unsigned int boot_carve_cau_param;
static unsigned int boot_carve_block_param; /* 0 = scan */
static unsigned int boot_carve_page_param;
static unsigned int boot_carve_off_param; /* must be 0 for a real boot page */
module_param_named(boot_carve_ce, boot_carve_ce_param, uint, 0644);
module_param_named(boot_carve_cau, boot_carve_cau_param, uint, 0644);
module_param_named(boot_carve_block, boot_carve_block_param, uint, 0644);
module_param_named(boot_carve_page, boot_carve_page_param, uint, 0644);
module_param_named(boot_carve_off, boot_carve_off_param, uint, 0644);
MODULE_PARM_DESC(boot_carve_block,
		 "cached aligned boot block (0=BTOC hunt for LPN0)");

/* Default l2v_build width when sysfs omits NBLOCKS (user blocks to scan). */
static unsigned int l2v_auto_blocks = 256;
module_param(l2v_auto_blocks, uint, 0644);
MODULE_PARM_DESC(l2v_auto_blocks,
		 "default l2v_build block span per CE/CAU (0=carve/boot hunt only)");

/*
 * Legacy dense-map META ingest. Off unless l2v_build/whimory_mount is
 * running — FIL reads from the Whimory driver must not allocate the
 * ~50MB LBA map as a side effect.
 */
static bool fmss_legacy_meta_ingest;

static int (*nand_ftl_read_hook)(u64 lba, void *buf);

/* Root-dir physical page when LPN(DataStart) is otherwise unmapped. */
static bool root_dir_valid;
static unsigned int root_dir_ce;
static unsigned int root_dir_cau;
static unsigned int root_dir_block;
static unsigned int root_dir_page;
static unsigned int root_dir_lpn;

static struct fmss_vfl_map vfl_map[FMSS_VFL_MAP_MAX];
static unsigned int vfl_map_count;
static unsigned int vfl_ctx_cau[FMSS_NUM_CAU];
static unsigned int vfl_ctx_block[FMSS_NUM_CAU];

/* Classic Whimory FTL block map + mount status (phase). */
static u16 *wmr_block_map;
static unsigned int wmr_block_map_n;
static unsigned int wmr_dis_hits;
static unsigned int wmr_vfl_hits;
static unsigned int wmr_ftlctrl_hits;
static unsigned int wmr_bmap_pages;
static unsigned int wmr_l2v_filled;
static int wmr_mount_ret;
static u16 wmr_ftlctrl[WMR_FTLCTRL_MAX];
static unsigned int wmr_ftlctrl_n;
static unsigned int wmr_dis_ce, wmr_dis_cau, wmr_dis_block, wmr_dis_page;

static u32 fmss_ppn_addr(unsigned int cau, unsigned int block,
			 unsigned int page, unsigned int slc)
{
	return page
	     | (block << FMSS_PAGE_BITS)
	     | (cau << (FMSS_PAGE_BITS + FMSS_BLOCK_BITS))
	     | ((slc ? 1u : 0u) << (FMSS_PAGE_BITS + FMSS_BLOCK_BITS + FMSS_CAU_BITS));
}

struct nand_s5l8740 {
	void __iomem *base;
	struct mutex lock;
	u8 last_id[8];
	int last_ce;
	u8 last_page[FMSS_PAGE_LEN];
	u8 last_spare[64];
	unsigned int last_spare_len;
	u8 last_parity[FMSS_MAX_CHUNKS][64];
	unsigned int last_parity_len[FMSS_MAX_CHUNKS];
	u32 last_page_addr;
	int last_page_ce;
	int last_page_ret;
	int last_page_chunk;
	unsigned int last_page_len;
	unsigned int last_clean_chunks;
	u8 last_param[FMSS_PARAM_LEN];
	int last_param_ce;
	int last_param_ret;
	u32 last_stat48;
	u32 last_nandstat;
	unsigned int pages_since_reset;
	unsigned int ecc_soft_fails;
	int dma_ok;
	int dma_mapped;
	struct device *dev;
	void *seq;
	void *cmdl;
	void *data;
	void *spare;
	void *stbuf;
	dma_addr_t seq_dma;
	dma_addr_t cmdl_dma;
	dma_addr_t data_dma;
	dma_addr_t spare_dma;
	dma_addr_t stbuf_dma;
	u32 last_dma_c0c;
	u32 last_dma_d00;
	u32 last_dma_c00;
	int irq;
	struct completion cs_irq;
	u32 last_vic_raw;
	u32 last_vic_en;
};

static struct nand_s5l8740 *nand_dev;

/* 12ED9C: PPN >= 0x10500 uses 4 address cycles. */
static unsigned int addr_cycles = 4;
module_param(addr_cycles, uint, 0644);
MODULE_PARM_DESC(addr_cycles, "FMSS address cycles (OSOS 8D102EC, default 4)");

/* 12ECCC calls 50D960(..., 1): hardware eats 53-byte parity before each 1K. */
static bool with_parity = true;
module_param(with_parity, bool, 0644);
MODULE_PARM_DESC(with_parity, "Issue 50D960 parity transfer (default Y)");

static bool use_dma;
module_param(use_dma, bool, 0644);
MODULE_PARM_DESC(use_dma, "Use FMSS DMA page read when dma_ok (default N — DMA spare path still OPEN on N31)");

/* 4EC6F4: MEMORY[0x8980CA4] = 0xFF000, OR'd into FMCTRL0 for ID/param/features. */
static unsigned int ctrl0_or = 0xFF000;
module_param(ctrl0_or, uint, 0644);
MODULE_PARM_DESC(ctrl0_or, "FMCTRL0 extra bits for ID/param/features (default 0xFF000)");

/*
 * FFE70 timing row 171 MHz: CA4 = 0x20011000. 50D960 uses this, not 0xFF000.
 * 0xFF000 for array reads returns zeros and wedges PPN.
 */
static unsigned int page_ctrl0_or = 0x20011000;
module_param(page_ctrl0_or, uint, 0644);
MODULE_PARM_DESC(page_ctrl0_or, "FMCTRL0 extra bits for 50D960 page read (171MHz timing)");

/* 50D960 wedges PPN after many reads; re-run 10453C+0xFF+power-state. */
static unsigned int reset_every = 6;
module_param(reset_every, uint, 0644);
MODULE_PARM_DESC(reset_every, "nand_reset after this many page_read calls (0=off)");

/* 50D960 uses 3; full PPN page is 16 x 1K data (+64 spare not in this PIO). */
/*
 * The CS/command-list path bumps pages_since_reset but historically never
 * acted on it — only the PIO page_read paths reset. A recover is thousands
 * of back-to-back live C00 kicks with no controller reset in between, and
 * the glass reboots partway through a wide scan. Reset every N CS reads.
 */
/* Total live CS kicks since insmod; heartbeat + post-mortem marker. */
static unsigned int cs_reads_total;
/* Off by default: console writes lengthen recover measurably. */
static bool cs_heartbeat;
module_param(cs_heartbeat, bool, 0644);
MODULE_PARM_DESC(cs_heartbeat, "Log CS read progress every 1024 pages");
module_param(cs_reads_total, uint, 0444);

/* Where CS read wall time actually goes; reported by the heartbeat. */
static u64 cs_ns_kick, cs_ns_copy;

/*
 * On. The comment above explains exactly why this is needed and it then
 * defaulted to 0, so the counter was incremented in five places and acted
 * on in none. A recovery walks eight thousand pages of back-to-back live
 * C00 kicks with nothing reasserting the sequencer across the whole run,
 * which is the condition the comment describes.
 *
 * 4096 is two register writes and two 10 us delays roughly twice per
 * recovery -- unmeasurable against the reads between them, and it bounds
 * how far the sequencer can drift before something puts it back.
 */
static unsigned int cs_reset_every = 4096;
module_param(cs_reset_every, uint, 0644);
MODULE_PARM_DESC(cs_reset_every,
		 "fmss_nand_reset after this many CS phys reads (0=off)");

static unsigned int page_chunks = 16;
module_param(page_chunks, uint, 0644);
MODULE_PARM_DESC(page_chunks, "1K PIO chunks per page_read (16=full 16KiB data)");

static unsigned int spare_len = 64;
module_param(spare_len, uint, 0644);
MODULE_PARM_DESC(spare_len,
		 "PIO META bytes after data (default 64 = 4×16B slots; was 16)");

/*
 * 50D960 parity FIFO is 53 bytes per 1K: 16B host spare (Sogeti META) + ECC.
 * Draining it before 4EB458 steals the syndrome (whitened data). A second
 * 50D960 with ecc_before_drain=0 copies META and keeps the first pass's data.
 * Extra style-1 FMLEN=15 beats after a style-0 page returned 0xFF and made
 * the next programmed page ECC-clean (glass 2026-08-27).
 */
static bool meta_pass = true;
module_param(meta_pass, bool, 0644);
MODULE_PARM_DESC(meta_pass,
		 "Second 50D960 to stash 53-byte parity into META slots (default Y)");

/*
 * 4EDDDC: D14 = (v40 ? 8D102F0 : 8D102EC) - 1.
 * v40 = (8D10300 != 1) — PPN multi-cycle path uses 8, so D14=7.
 */
static unsigned int dma_d14 = 7;
module_param(dma_d14, uint, 0644);
MODULE_PARM_DESC(dma_d14, "FMSS D14 address-cycles-1 for DMA (default 7 = PPN v40)");

/* One 4096-byte host sector first (oracle G1); FTL callers override via span. */
static unsigned int dma_nsect = 1;
module_param(dma_nsect, uint, 0644);
MODULE_PARM_DESC(dma_nsect, "DMA span (# logical LBAs) per CS read (default 1)");

/*
 * Decomp wants command-list META. Live CS kick (C00=0xFFF5) still wedges
 * the SoC in testing — keep default off until that path is safe. Callers that
 * ask for meta with this off get -EOPNOTSUPP (never PIO fake spare).
 */
static bool meta_dma_read;
module_param(meta_dma_read, bool, 0644);
MODULE_PARM_DESC(meta_dma_read,
		 "Allow page_read META via CS span4 (default N). "
		 "ftl_sftl_recover uses dma_session live CS without this.");

/*
 * PIO page path uses the legacy reset/reinit sequence.
 * CS command-list path is a separate sequencer path and should not assume
 * hard reset immediately before kick. in testing this sequence can wedge.
 */
static bool meta_dma_reset_before;
module_param(meta_dma_reset_before, bool, 0644);
MODULE_PARM_DESC(meta_dma_reset_before,
		 "Reset NAND controller before command-list metadata read");

static bool dma_reset_before;
module_param(dma_reset_before, bool, 0644);
MODULE_PARM_DESC(dma_reset_before,
		 "Reset NAND controller before manual CS DMA read");

/*
 * PPN physical page = N × (4096 DATA + 16 META) records (N=2 or 4).
 * 5172A0: qword.lo = (rec*span) | ((rec*slot) << 16); qword.hi = encoded_ppn.
 */
#define FMSS_PPN_REC		4112u	/* 4096 + 16 */
static unsigned int dma_slot;
module_param(dma_slot, uint, 0644);
MODULE_PARM_DESC(dma_slot, "starting logical slot within PPN page (0..3, default 0)");

static unsigned int dma_rec = FMSS_PPN_REC;
module_param(dma_rec, uint, 0644);
MODULE_PARM_DESC(dma_rec, "PPN on-flash record bytes (default 4112 = 4096+16)");

/*
 * 4EDDDC v40 packing with qword=(0,ppn): addr in cmdlist[3].
 * 1 = force addr into cmdlist[2] (experiment). Default 0.
 */
static unsigned int dma_row_in_lo;
module_param(dma_row_in_lo, uint, 0644);
MODULE_PARM_DESC(dma_row_in_lo, "1=addr in cmdlist[2]; 0=v40 high dword in [3] (default)");

static unsigned int dma_c6c = 16;
module_param(dma_c6c, uint, 0644);
MODULE_PARM_DESC(dma_c6c, "value written to +0xC6C before DMA kick (default 16)");

/* D39EC: MEMORY[C00] = 65525 = 0xFFF5 (NOT 0xFFFD). */
static unsigned int dma_kick = 0xfff5;
module_param(dma_kick, uint, 0644);
MODULE_PARM_DESC(dma_kick, "FMSEQ (C00) kick value (OSOS D39EC = 0xFFF5)");

/* Default dry/disarmed: permanent live C00 kick reboots (glass 2026-08-27).
 * ftl_sftl_recover / cs_phys use dma_session_begin to arm live CS temporarily.
 */
static bool dma_dry = true;
module_param(dma_dry, bool, 0644);
MODULE_PARM_DESC(dma_dry,
		 "program CS regs/descriptors but do not write C00 (default Y)");

static bool dma_armed;
module_param(dma_armed, bool, 0644);
MODULE_PARM_DESC(dma_armed, "Allow CS DMA kick (default N — session arms for recover)");

static bool dma_one_shot = true;
module_param(dma_one_shot, bool, 0644);
MODULE_PARM_DESC(dma_one_shot,
		 "Clear dma_armed after each CS kick (default Y outside sessions)");

/* Canary path: one page, no FTL/lba_map ingest. */
static bool dma_skip_ingest;

/*
 * D39EC: if 0x8982448 then C6C=0 + pulse C60; else C6C=16.
 * Pulse never raised C64 in testing. Default matches the else path.
 */
static bool dma_pulse;
module_param(dma_pulse, bool, 0644);
MODULE_PARM_DESC(dma_pulse, "pulse +0xC60 before DMA kick (D39EC if-path)");

/*
 * Seq blob[0..31] is CS ops. CPU-poking C0C=0xFF is W1C of FMSEQIRQ and
 * made wait_cs see 0xFF&0xD==0xD. Leave off unless debugging C10/C58/C4C.
 */
static bool dma_preamble;
module_param(dma_preamble, bool, 0644);
MODULE_PARM_DESC(dma_preamble, "CPU-write C10/C58/C4C from seq header (not C0C)");

/* OSOS D39B8: interrupt 54 = VIC1 hwirq 22 = Linux irq 70 on this GATE0 map. */
static int dma_irq = 70;
module_param(dma_irq, int, 0644);
MODULE_PARM_DESC(dma_irq, "Linux IRQ for FMSS CS (OSOS 54 / VIC1 22, default 70)");

static u32 fmss_ctrl0(unsigned int ce)
{
	return ctrl0_or | (2u * (1u << ce)) | 1u;
}

static u32 fmss_page_ctrl0(unsigned int ce)
{
	return page_ctrl0_or | (2u * (1u << ce)) | 1u;
}

/* 1858DC: poll FMSTAT48 bit(s), then W1C. */
static int fmss_wait48_n(struct nand_s5l8740 *f, u32 mask, unsigned int loops)
{
	unsigned int i;
	u32 st;

	for (i = 0; i < loops; i++) {
		st = readl(f->base + FMSTAT48);
		if (st & mask) {
			if (mask == 0x800000u)
				writel(32, f->base + FMCTRL1);
			writel(mask, f->base + FMSTAT48);
			return 0;
		}
		udelay(1);
	}
	f->last_stat48 = readl(f->base + FMSTAT48);
	f->last_nandstat = readl(f->base + NANDSTAT);
	writel(mask, f->base + FMSTAT48);
	return -ETIMEDOUT;
}

static int fmss_wait48(struct nand_s5l8740 *f, u32 mask)
{
	return fmss_wait48_n(f, mask, 20000);
}

static int fmss_cmd(struct nand_s5l8740 *f, u8 cmd)
{
	writel(cmd, f->base + FMCMD);
	return fmss_wait48(f, 2);
}

/* D622C: drain PIO FIFO at +0x80. Must read +0x80 first (OSOS + live ID). */
static int fmss_pio_read(struct nand_s5l8740 *f, void *dst, unsigned int len)
{
	u8 *p = dst;
	unsigned int n = 0, words = len >> 2, spins = 0;
	u32 word;

	writel(3, f->base + FMLEN);
	writel(1u << 8, f->base + FMCE);
	writel(readl(f->base + FMCTRL0) & 0xFEFFFBFF, f->base + FMCTRL0);
	writel(0, f->base + FMUNK28);
	writel(480, f->base + FMCTRL1);

	while (n < words && spins < 20000u * (words + 1)) {
		word = readl(f->base + FMDATA);
		if ((u8)readl(f->base + FMUNK28) == (u8)(4 * n + 4)) {
			memcpy(p + 4 * n, &word, 4);
			n++;
		}
		spins++;
		cpu_relax();
	}
	writel(0x100000, f->base + FMSTAT48);
	writel(32, f->base + FMCTRL1);
	return n == words ? 0 : -ETIMEDOUT;
}

/* 41AE38: NAND→controller beat then D622C PIO. len <= M2_BYTES_PER_SECTOR. */
static int fmss_data_in(struct nand_s5l8740 *f, void *dst, unsigned int len)
{
	if (len < 1 || len > 0x400)
		return -EINVAL;
	writel(len - 1, f->base + FMLEN);
	writel(1, f->base + FMCE);
	writel(0, f->base + FMUNK24);
	writel(34, f->base + FMCTRL1);
	if (fmss_wait48(f, 8)) {
		pr_info("s5l8740-nand: data_in wait48(8) timeout len=%u st=%08x\n",
			len, f->last_stat48);
		return -ETIMEDOUT;
	}
	return fmss_pio_read(f, dst, len);
}

/* 41DA70: wait NANDSTAT ready after 0x77/0x7D. */
static int fmss_wait_status(struct nand_s5l8740 *f, unsigned int loops, u8 *nandstat)
{
	int ret;

	writel(0x800000u, f->base + FMSTAT48);
	writel(234, f->base + FMCTRL1);
	ret = fmss_wait48_n(f, 0x800000u, loops);
	if (nandstat)
		*nandstat = (u8)readl(f->base + NANDSTAT);
	writel(0x1000000u, f->base + FMSTAT48);
	return ret;
}

static int fmss_addr_n(struct nand_s5l8740 *f, u32 addr, unsigned int cycles)
{
	if (cycles < 1 || cycles > 8)
		cycles = 1;
	writel(cycles - 1, f->base + FMCYCLES);
	writel(addr, f->base + FMADDR);
	writel(1, f->base + FMCTRL1);
	return fmss_wait48(f, 4);
}

static int fmss_addr1(struct nand_s5l8740 *f, u32 addr)
{
	return fmss_addr_n(f, addr, 1);
}

/*
 * D6388 + 112A7C: PIO write to +0x80 then kick. Used only for SET FEATURES
 * (NAND device registers), never for array program (50DC34).
 */
static int fmss_pio_write(struct nand_s5l8740 *f, const void *src, unsigned int len)
{
	const u32 *p = src;
	unsigned int i, words;

	if (len < 4 || (len & 3) || len > 0x400)
		return -EINVAL;
	words = len >> 2;
	writel(readl(f->base + FMCTRL0) | 0x02000000, f->base + FMCTRL0);
	writel(readl(f->base + FMCTRL0) & 0xFEFFFBFF, f->base + FMCTRL0);
	writel(1, f->base + FMCE);
	writel(0, f->base + FMUNK24);
	writel(736, f->base + FMCTRL1);
	for (i = 0; i < words; i++)
		writel(p[i], f->base + FMDATA);
	writel(0x100000, f->base + FMSTAT48);
	writel(32, f->base + FMCTRL1);
	writel(readl(f->base + FMCTRL0) & ~0x02000000u, f->base + FMCTRL0);

	writel(len - 1, f->base + FMLEN);
	writel(256, f->base + FMCE);
	writel(65764, f->base + FMCTRL1);
	return fmss_wait48(f, 8);
}

/* 1303B4: PPN SET FEATURES (cmd 0xEF). */
static int fmss_set_feature(struct nand_s5l8740 *f, unsigned int ce, u16 feat, u32 val)
{
	u8 st = 0;
	u32 feat_word = feat;
	int ret = 0;

	if (ce > 7)
		return -EINVAL;
	writel((2u * (1u << ce)) | 0xFF001u, f->base + FMCTRL0);
	if (fmss_cmd(f, 0xef) || fmss_addr_n(f, feat_word, 2) ||
	    fmss_pio_write(f, &val, 4))
		ret = -ETIMEDOUT;
	fmss_cmd(f, 0xe7);
	fmss_cmd(f, 0x77);
	fmss_cmd(f, 0x7d);
	if (fmss_wait_status(f, 100000, &st))
		ret = -ETIMEDOUT;
	fmss_cmd(f, 0x77);
	writel(1, f->base + FMCTRL0);
	fmss_info("set_feature ce=%u feat=0x%04x val=0x%08x ret=%d nandstat=%02x\n",
		  ce, feat, val, ret, st);
	return ret;
}

/* 41CEDC: PPN GET FEATURES (cmd 0xEE). */
static u8 last_feat[16];
static u16 last_feat_id;
static int last_feat_ce = -1;
static int last_feat_ret = -1;

static int fmss_get_feature(struct nand_s5l8740 *f, unsigned int ce, u16 feat,
			    void *dst, unsigned int len)
{
	u8 st = 0;
	u32 feat_word = feat;
	u32 saved;
	int ret = 0;

	if (ce > 7 || len < 4 || len > 0x200)
		return -EINVAL;
	memset(dst, 0, len);
	saved = readl(f->base + FMCTRL0);
	writel((2u * (1u << ce)) | 0xFF001u, f->base + FMCTRL0);
	if (fmss_cmd(f, 0xee) || fmss_addr_n(f, feat_word, 2))
		ret = -ETIMEDOUT;
	fmss_cmd(f, 0xe7);
	fmss_cmd(f, 0x77);
	fmss_cmd(f, 0x7d);
	if (fmss_wait_status(f, 100000, &st))
		ret = -ETIMEDOUT;
	else {
		fmss_cmd(f, 0x7a);
		if (fmss_data_in(f, dst, len))
			ret = -ETIMEDOUT;
		fmss_cmd(f, 0x77);
	}
	writel(saved, f->base + FMCTRL0);
	fmss_info("get_feature ce=%u feat=0x%04x ret=%d nandstat=%02x %02x %02x %02x %02x\n",
		  ce, feat, ret, st,
		  ((u8 *)dst)[0], ((u8 *)dst)[1], ((u8 *)dst)[2], ((u8 *)dst)[3]);
	return ret;
}

/*(ce, addr=0, buf): READ ID, no 10453C reset. */
static int fmss_read_id(struct nand_s5l8740 *f, unsigned int ce)
{
	u8 *dst = f->last_id;

	if (ce > 7)
		return -EINVAL;

	memset(dst, 0, 8);
	/* Drop leftover ready bits so 1858DC does not return immediately. */
	writel(0x0e, f->base + FMSTAT48);

	writel(fmss_ctrl0(ce), f->base + FMCTRL0);
	fmss_cmd(f, 0x90);

	if (fmss_addr1(f, 0))
		pr_info("s5l8740-nand: wait48(4) after addr timed out st=%08x\n",
			readl(f->base + FMSTAT48));

	if (fmss_data_in(f, dst, 8)) {
		f->last_ce = (int)ce;
		return -ETIMEDOUT;
	}
	writel(1, f->base + FMCTRL0);

	f->last_ce = (int)ce;
	return 0;
}

/* 4F1CE8: cmd 0x77/0x7D then wait NAND ready (STAT48 bit 0x800000). */
static int fmss_wait_ready(struct nand_s5l8740 *f)
{
	int ret;

	fmss_cmd(f, 0x77);
	fmss_cmd(f, 0x7d);
	writel(1, f->base + FMCE);
	writel(234, f->base + FMCTRL1);
	ret = fmss_wait48_n(f, 0x800000u, 200000);
	writel(32, f->base + FMCTRL1);
	writel(0x800000u, f->base + FMSTAT48);
	if (ret)
		pr_info("s5l8740-nand: ready timeout NANDSTAT=%08x STAT48=%08x\n",
			f->last_nandstat, f->last_stat48);
	return ret;
}

static bool ecc_before_drain = true;
module_param(ecc_before_drain, bool, 0644);
MODULE_PARM_DESC(ecc_before_drain,
		 "Run OSOS 4EB458 ECC/descramble before PIO drain (default Y)");

/*
 * 0 = diagnostic: FMLEN=52 FMCE=16 CTRL1=34 then data FMCE=1
 * 1 = production-seq style: FMLEN=15 FMCE=0x102 CTRL1=0x1E2, data FMCE=0x201 +0x18=2
 */
static unsigned int xfer_style;
module_param(xfer_style, uint, 0644);
MODULE_PARM_DESC(xfer_style,
		 "0=50D960 parity52 (default), 1=DMA-seq-like meta15/CTRL1=0x1E2");

static unsigned int ecc_op = 0x1e1;
module_param(ecc_op, uint, 0644);
MODULE_PARM_DESC(ecc_op,
		 "low bits OR'd into +0x804 for 4EB458 (default 0x1E1; encode uses 0x1E2)");

/* OSOS 50D960 uses 16; sweep if UECC persists. */
static unsigned int parity_fmce = 16;
module_param(parity_fmce, uint, 0644);
MODULE_PARM_DESC(parity_fmce, "FMCE value for 52-byte parity beat (default 16)");

static unsigned int data_fmce = 1;
module_param(data_fmce, uint, 0644);
MODULE_PARM_DESC(data_fmce, "FMCE value for 1024-byte data beat (default 1)");

static unsigned int xfer_ctrl1 = 34;
module_param(xfer_ctrl1, uint, 0644);
MODULE_PARM_DESC(xfer_ctrl1, "FMCTRL1 for parity/data beats (default 34)");

/*
 * OSOS(a1=0, len=1024): kick FMSS ECC/descramble engine.
 * Must run AFTER parity+data transfer waits, BEFORE FIFO drain.
 *
 * Exact RetailOS order — do NOT clear +0x810 before kick (preload stays
 * 0x3f1f73af). Poll bit0 of +0x810 for completion; if preload already has
 * bit0, treat as synchronous after the +0x804 write (first read).
 *
 * Returns 0 OK, 1 clean/erased page, -ETIMEDOUT / -EIO on hard fail.
 */
static int fmss_ecc_chunk(struct nand_s5l8740 *f, unsigned int seed_a1)
{
	u32 ecc, hist, st;
	unsigned int t;

	writel(1, f->base + 0x81c);
	writel(0x04000400u, f->base + 0x818);
	writel(1, f->base + 0x80c);
	writel(0x3f1f73afu, f->base + 0x810);
	writel(0x80000000u, f->base + 0x820);
	writel(0x02000000u, f->base + 0x808);
	writel(seed_a1 | (1024u << 16) | (ecc_op & 0xfffu), f->base + 0x804);

	/* Kick may clear bit0; poll until set again (or already set). */
	for (t = 0; t < 20000; t++) {
		st = readl(f->base + 0x810);
		if (st & 1u)
			break;
		udelay(1);
	}
	if (t >= 20000) {
		pr_info("s5l8740-nand: 4EB458 timeout st=%08x\n", st);
		return -ETIMEDOUT;
	}
	ecc = readl(f->base + 0x80c);
	hist = readl(f->base + 0x818);
	writel(1, f->base + 0x810);
	if (!quiet)
		fmss_info("4EB458 ecc=0x%08x hist=0x%08x t=%u st=%08x op=%x\n",
			  ecc, hist, t, st, ecc_op);
	if (ecc & 1u)
		return -EIO;
	if (ecc & 2u)
		return 1;
	return 0;
}

/*
 *(ce, page_addr, buf, with_parity).
 * Per 1KiB chunk: parity beat → data xfer wait → 4EB458 → PIO drain.
 * Linux previously drained before ECC — that left DATA pages whitened.
 */
static void fmss_meta_ingest_spare(struct nand_s5l8740 *f, unsigned int ce,
				   unsigned int cau, unsigned int block,
				   unsigned int page, unsigned int slot0);

static int fmss_page_read(struct nand_s5l8740 *f, unsigned int ce, u32 addr)
{
	int i, ret = -EIO, ecc_ret;
	u8 *dst = f->last_page;
	unsigned int cycles = addr_cycles;
	unsigned int chunks = page_chunks;

	if (ce > 7)
		return -EINVAL;
	if (cycles < 1 || cycles > 8)
		cycles = 4;
	if (chunks < 1 || chunks > FMSS_MAX_CHUNKS)
		chunks = 3;

	memset(dst, 0, FMSS_PAGE_LEN);
	memset(f->last_spare, 0, sizeof(f->last_spare));
	f->last_spare_len = 0;
	f->last_page_ce = (int)ce;
	f->last_page_addr = addr;
	f->last_page_chunk = -1;
	f->last_page_len = 0;
	f->last_clean_chunks = 0;

	writel(7, f->base + FMUNK38);
	writel(fmss_page_ctrl0(ce), f->base + FMCTRL0);
	writel(0x0FF00FFE, f->base + FMSTAT48);
	writel(1, f->base + FMUNK81C);
	fmss_cmd(f, 0x0a);

	writel(cycles - 1, f->base + FMCYCLES);
	writel(addr, f->base + FMADDR);
	writel(1, f->base + FMCTRL1);
	if (fmss_wait48(f, 4)) {
		pr_info("s5l8740-nand: addr cycle timeout ce=%u addr=%08x st=%08x nand=%08x\n",
			ce, addr, f->last_stat48, f->last_nandstat);
		goto fail_ctrl0;
	}

	if (fmss_cmd(f, 0x37)) {
		pr_info("s5l8740-nand: cmd 0x37 timeout ce=%u addr=%08x st=%08x\n",
			ce, addr, f->last_stat48);
		goto fail_ctrl0;
	}

	if (fmss_wait_ready(f)) {
		pr_info("s5l8740-nand: not ready after read cmd ce=%u addr=%08x\n",
			ce, addr);
		goto fail_ctrl0;
	}

	fmss_cmd(f, 0x7a);
	for (i = 0; i < (int)chunks; i++) {
		f->last_page_chunk = i;
		f->last_parity_len[i] = 0;
		writel(0x08000000, f->base + FMSTAT48);
		if (with_parity) {
			if (xfer_style == 1) {
				writel(15, f->base + FMLEN);
				writel(0x102, f->base + FMCE);
				writel(1, f->base + FMUNK18);
				writel(0, f->base + FMUNK24);
				writel(0x1e2, f->base + FMCTRL1);
			} else {
				/* OSOS 50D960 parity: FMLEN=52 FMCE=16 +0x24=0 CTRL1=34
				 * — do not touch +0x18 or +0x28 here.
				 */
				writel(52, f->base + FMLEN);
				writel(parity_fmce, f->base + FMCE);
				writel(0, f->base + FMUNK24);
				writel(xfer_ctrl1, f->base + FMCTRL1);
			}
			if (fmss_wait48(f, 8)) {
				pr_info("s5l8740-nand: parity xfer timeout ce=%u addr=%08x chunk=%d st=%08x style=%u\n",
					ce, addr, i, f->last_stat48, xfer_style);
				goto fail_ctrl0;
			}
			/*
			 * Live: after FMCE=16 FMLEN=52, D622C can read bytes
			 * (head often 02 …) — parity lands on the DATA FIFO.
			 * Never drain it when ecc_before_drain=1 (OSOS leaves
			 * it for 4EB458). Optional strip only when ECC off.
			 */
			if (xfer_style == 0 && !ecc_before_drain) {
				u8 dig[64];

				memset(dig, 0, sizeof(dig));
				if (!fmss_pio_read(f, dig, 53)) {
					memcpy(f->last_parity[i], dig, 53);
					f->last_parity_len[i] = 53;
					/*
					 * Host-visible spare is the first 16 of
					 * the 53-byte beat, once per 4K slot.
					 */
					if ((i & 3) == 0) {
						unsigned int slot = (unsigned int)i / 4u;
						unsigned int pick = 0;

						if (slot < 4) {
							if (dig[0] != 0x30) {
								if (dig[16] == 0x30)
									pick = 16;
								else if (dig[32] == 0x30)
									pick = 32;
							}
							memcpy(f->last_spare + slot * 16,
							       dig + pick, 16);
							f->last_spare_len = (slot + 1) * 16;
						}
					}
					if (!quiet)
						fmss_info("stripped parity-fifo ch=%d %02x%02x%02x%02x…\n",
							  i, dig[0], dig[1], dig[2], dig[3]);
				}
			}
		}
		writel(1023, f->base + FMLEN);
		if (xfer_style == 1) {
			writel(2, f->base + FMUNK18);
			writel(0x201, f->base + FMCE);
			writel(0, f->base + FMUNK24);
			writel(0x1e2, f->base + FMCTRL1);
		} else {
			writel(1, f->base + FMUNK18);
			writel(data_fmce, f->base + FMCE);
			writel(0, f->base + FMUNK24);
			writel(xfer_ctrl1, f->base + FMCTRL1);
		}
		if (fmss_wait48(f, 8)) {
			pr_info("s5l8740-nand: data xfer timeout ce=%u addr=%08x chunk=%d st=%08x nand=%08x style=%u\n",
				ce, addr, i, f->last_stat48, f->last_nandstat,
				xfer_style);
			goto fail_ctrl0;
		}
		/* OSOS: ECC/descramble BEFORE FIFO drain when parity path used. */
		if (with_parity && ecc_before_drain) {
			ecc_ret = fmss_ecc_chunk(f, 0);
			if (ecc_ret == 1) {
				/*
				 * Chunk is erased/clean — leave zeros and
				 * keep going. Aborting the whole page on
				 * chunk0 made FPart/VFL miss SLC specials
				 * (glass: 4096 tail reads, tag30=0).
				 */
				f->last_clean_chunks++;
				continue;
			}
			if (ecc_ret) {
				/* Blank BTOC pages are normal during l2v_build — count, don't spam. */
				f->ecc_soft_fails++;
				if (!quiet && i == 0)
					fmss_info("ECC soft-fail ce=%u addr=%08x ret=%d (raw drain)\n",
						  ce, addr, ecc_ret);
			}
		}
		if (fmss_pio_read(f, dst + i * FMSS_CHUNK, FMSS_CHUNK)) {
			pr_info("s5l8740-nand: PIO drain timeout ce=%u addr=%08x chunk=%d\n",
				ce, addr, i);
			goto fail_ctrl0;
		}
	}

	/*
	 * Trailing fmss_data_in(64) after 16×1K is an empty FIFO (zeros) and
	 * must not be treated as META (that polluted lba_map with type 0x00).
	 * Extra style-1 FMLEN=15 beats after this path desynced the next page.
	 */

	fmss_cmd(f, 0x77);
	writel(0, f->base + FMCTRL0);
	f->last_page_ret = 0;
	f->last_page_len = chunks * FMSS_CHUNK;
	/* Full-page PIO: ingest all META slots into lba_map (pass 2). */
	if (chunks >= 16 && f->last_spare_len >= 16) {
		unsigned int pg = addr & L2V_PAGE_MASK;
		unsigned int blk = (addr >> FMSS_PAGE_BITS) & L2V_BLOCK_MASK;
		unsigned int cau = (addr >> (FMSS_PAGE_BITS + FMSS_BLOCK_BITS)) &
				   ((1u << FMSS_CAU_BITS) - 1u);

		fmss_meta_ingest_spare(f, ce, cau, blk, pg, 0);
	}
	return 0;

fail_ctrl0:
	writel(0, f->base + FMCTRL0);
	f->last_page_ret = ret;
	return ret;
}

/*
 * FIL needs both descrambled data (pass 1, ECC) and 16B Sogeti META
 * (pass 2, drain 53-byte parity FIFO). Mutex is already held.
 */
static int fmss_page_read_with_meta(struct nand_s5l8740 *f, unsigned int ce,
				    u32 addr)
{
	static u8 page_bak[FMSS_PAGE_LEN];
	unsigned int dlen;
	int ret, ret2;
	bool saved_ecc;
	int saved_ce, saved_chunk;
	u32 saved_addr;

	ret = fmss_page_read(f, ce, addr);
	if (ret || !meta_pass)
		return ret;
	/*
	 * Erased PPN page: all 16 chunks ECC-clean. Spare is 0xFF; a second
	 * 50D960 only burns the controller (tail+brute are mostly empty).
	 */
	if (f->last_clean_chunks &&
	    f->last_clean_chunks >= (page_chunks ? page_chunks : 16)) {
		memset(f->last_spare, 0xff, sizeof(f->last_spare));
		f->last_spare_len = 64;
		return ret;
	}
	dlen = f->last_page_len;
	if (dlen > FMSS_PAGE_LEN)
		dlen = FMSS_PAGE_LEN;
	memcpy(page_bak, f->last_page, dlen);
	saved_ce = f->last_page_ce;
	saved_addr = f->last_page_addr;
	saved_chunk = f->last_page_chunk;
	saved_ecc = ecc_before_drain;
	ecc_before_drain = false;
	ret2 = fmss_page_read(f, ce, addr);
	ecc_before_drain = saved_ecc;
	memcpy(f->last_page, page_bak, dlen);
	f->last_page_len = dlen;
	f->last_page_ce = saved_ce;
	f->last_page_addr = saved_addr;
	f->last_page_chunk = saved_chunk;
	f->last_page_ret = ret;
	if (ret2 && !quiet)
		pr_info("s5l8740-nand: meta pass ce=%u addr=%08x ret=%d (data kept)\n",
			ce, addr, ret2);
	return ret;
}

/*
 * OSOS 4EDDDC / D39EC: FMSS command-list DMA page read.
 * Sequence program is the OSOS blob at 0x8980EA0 (embedded).
 * 16-byte PPN spare per 4K sector lands in the spare DMA buffer
 * (Sogeti type/bank/weaveSeq/lpn). Status bytes go to stbuf.
 *
 * Treat CS as a peripheral sequencer (not PL080): clear status, prove
 * idle, program pointers, read back posted MMIO, kick last. USB OTG
 * already proves SoC bus-master DMA / coherent buffers work.
 */
static int fmss_wait_cs_poll(struct nand_s5l8740 *f, unsigned int loops)
{
	unsigned int i;
	u32 irq;

	for (i = 0; i < loops; i++) {
		irq = readl(f->base + FMSEQIRQ);
		if ((irq & 0xd) == 1) {
			f->last_dma_c0c = irq;
			return 0;
		}
		if (irq & 0xc) {
			f->last_dma_c0c = irq;
			return -EIO;
		}
		udelay(10);
	}
	f->last_dma_c0c = readl(f->base + FMSEQIRQ);
	return -ETIMEDOUT;
}

static bool fmss_cs_preflight(struct nand_s5l8740 *f)
{
	u32 c08, c0c, c00;

	c00 = readl(f->base + FMSEQ);
	c08 = readl(f->base + FMSEQSTAT);
	c0c = readl(f->base + FMSEQIRQ);

	f->last_dma_c00 = c00;
	f->last_dma_c0c = c0c;

	/* Clear stale completion/error before programming. */
	if (c0c & 0x0d) {
		writel(c0c & 0x0d, f->base + FMSEQIRQ);
		readl(f->base + FMSEQIRQ);
		udelay(10);
		c0c = readl(f->base + FMSEQIRQ);
		f->last_dma_c0c = c0c;
	}

	/*
	 * Conservative idle gate. Adjust allowed states only after glass logs.
	 * Avoid kick if status already advertises completion/error/busy noise.
	 */
	if (c0c & 0x0d)
		return false;

	if (c08 != 0 && c08 != 3)
		return false;

	return true;
}

static void fmss_dma_teardown(struct nand_s5l8740 *f)
{
	struct device *dev = f->dev;

	if (f->irq > 0) {
		free_irq(f->irq, f);
		f->irq = 0;
	}
	if (dev) {
		if (f->seq)
			dma_free_coherent(dev, FMSS_SEQ_READ_LEN, f->seq, f->seq_dma);
		if (f->cmdl)
			dma_free_coherent(dev, FMSS_DMA_CMDLIST_LEN, f->cmdl, f->cmdl_dma);
		if (f->data)
			dma_free_coherent(dev, FMSS_PAGE_LEN, f->data, f->data_dma);
		if (f->spare)
			dma_free_coherent(dev, FMSS_DMA_SPARE_LEN, f->spare, f->spare_dma);
		if (f->stbuf)
			dma_free_coherent(dev, FMSS_DMA_STATUS_LEN, f->stbuf, f->stbuf_dma);
	}
	f->seq = f->cmdl = f->data = f->spare = f->stbuf = NULL;
	f->dma_ok = 0;
	f->dma_mapped = 0;
}

static irqreturn_t fmss_cs_irq(int irq, void *data)
{
	struct nand_s5l8740 *f = data;
	u32 st;

	st = readl(f->base + FMSEQIRQ);
	f->last_dma_c0c = st;

	/*
	 * Level-style VIC source: clear peripheral before parent EOI, or it
	 * can retrigger/stick. Snapshot first — waiter must not require C0C
	 * to remain asserted after W1C.
	 */
	if (st & 0x0d) {
		writel(st & 0x0d, f->base + FMSEQIRQ);
		readl(f->base + FMSEQIRQ);
	}

	complete(&f->cs_irq);
	return IRQ_HANDLED;
}

static void fmss_peek_vic1(struct nand_s5l8740 *f)
{
	void __iomem *vic1;

	vic1 = ioremap(0x38e01000ul, 0x20);
	if (!vic1)
		return;
	f->last_vic_raw = readl(vic1 + 0x08);
	f->last_vic_en = readl(vic1 + 0x10);
	iounmap(vic1);
}

static int fmss_dma_setup(struct nand_s5l8740 *f, struct device *dev)
{
	int ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		dev_warn(dev, "dma mask: %d\n", ret);
	f->dev = dev;
	init_completion(&f->cs_irq);

	f->seq = dma_alloc_coherent(dev, FMSS_SEQ_READ_LEN, &f->seq_dma, GFP_KERNEL);
	f->cmdl = dma_alloc_coherent(dev, FMSS_DMA_CMDLIST_LEN, &f->cmdl_dma, GFP_KERNEL);
	f->data = dma_alloc_coherent(dev, FMSS_PAGE_LEN, &f->data_dma, GFP_KERNEL);
	f->spare = dma_alloc_coherent(dev, FMSS_DMA_SPARE_LEN, &f->spare_dma, GFP_KERNEL);
	f->stbuf = dma_alloc_coherent(dev, FMSS_DMA_STATUS_LEN, &f->stbuf_dma, GFP_KERNEL);
	if (!f->seq || !f->cmdl || !f->data || !f->spare || !f->stbuf) {
		nand_dev_info(dev, "DMA coherent alloc failed, PIO only\n");
		fmss_dma_teardown(f);
		return -ENOMEM;
	}
	memcpy(f->seq, fmss_seq_read_blob, FMSS_SEQ_READ_LEN);
	memset(f->cmdl, 0, FMSS_DMA_CMDLIST_LEN);
	memset(f->data, 0, FMSS_PAGE_LEN);
	memset(f->spare, 0, FMSS_DMA_SPARE_LEN);
	memset(f->stbuf, 0, FMSS_DMA_STATUS_LEN);
	f->dma_mapped = 1;
	f->dma_ok = 1;

	if (dma_irq > 0) {
		ret = request_irq(dma_irq, fmss_cs_irq, 0, "fmss-cs", f);
		if (ret) {
			dev_warn(dev, "CS IRQ %d: %d (polling C0C)\n", dma_irq, ret);
			f->irq = 0;
		} else {
			f->irq = dma_irq;
			nand_dev_info(dev, "CS IRQ %d (OSOS 54 / VIC1 22)\n", f->irq);
		}
	}

	nand_dev_info(dev, "DMA seq_phys=0x%08lx cmdl=0x%08lx data=0x%08lx seq0=%02x %02x %02x %02x coherent=1\n",
		 (unsigned long)f->seq_dma, (unsigned long)f->cmdl_dma,
		 (unsigned long)f->data_dma,
		 ((u8 *)f->seq)[0], ((u8 *)f->seq)[1],
		 ((u8 *)f->seq)[2], ((u8 *)f->seq)[3]);
	return 0;
}

static int fmss_dma_page_read(struct nand_s5l8740 *f, unsigned int ce, u32 addr)
{
	u32 *cl;
	u32 ce_bit;
	unsigned int nsect = dma_nsect;
	unsigned int i, spare_bytes;
	u8 *dp;
	int ret = 0;
	u32 dregs[32];

	memset(dregs, 0, sizeof(dregs));

	if (!f->dma_ok)
		return -ENODEV;
	if (ce > 7 || nsect < 1 || nsect > 4)
		return -EINVAL;

	/* Refresh CS microcode + wipe all coherent targets (device-visible). */
	memcpy(f->seq, fmss_seq_read_blob, FMSS_SEQ_READ_LEN);
	memset(f->data, 0, FMSS_PAGE_LEN);
	memset(f->spare, 0, FMSS_DMA_SPARE_LEN);
	memset(f->stbuf, 0, FMSS_DMA_STATUS_LEN);
	memset(f->cmdl, 0, FMSS_DMA_CMDLIST_LEN);

	cl = f->cmdl;
	ce_bit = 1u << (16 + ce);

	/*
	 * 4EDDDC one-page descriptor list:
	 * desc0: CE-select dword0=1<<(ce+16), later |0x80000000 if last for CE
	 * dword2/3 = packed physical address (v40: low in [2])
	 * desc1: transfer dword0=(1<<(ce+16))|1, [1]=span, [2]=meta, [3]=data
	 * term: 0x00010002
	 */
	/*
	 * 4EDDDC address qword from 5172A0 (READ, v40 / multi-LBA page):
	 * lo = (rec * span) | ((rec * slot) << 16) // length | column<<16
	 * hi = encoded_ppn (5173CA, mode 0)
	 * desc[2]=lo, desc[3]=hi when dma_d14>=7 (v40).
	 */
	cl[0] = ce_bit;
	cl[1] = 0;
	{
		unsigned int span = nsect;
		unsigned int slot = dma_slot;
		u32 rec = dma_rec ? dma_rec : FMSS_PPN_REC;
		u32 col_len;

		if (slot > 3)
			slot = 3;
		if (span < 1)
			span = 1;
		if (span > 4)
			span = 4;
		if (slot + span > 4)
			span = 4 - slot;
		col_len = (rec * span) | ((rec * slot) << 16);

		if (dma_row_in_lo) {
			/* experiment: put ppn in lo (broken for normal path) */
			cl[2] = addr;
			cl[3] = (addr >> 31);
		} else if (dma_d14 >= 7) {
			cl[2] = col_len;
			cl[3] = addr;
		} else {
			/* !v40: only HIDWORD consumed as ppn; keep col_len in lo anyway */
			cl[2] = addr;
			cl[3] = 0;
		}
		fmss_info("dma slice slot=%u span=%u rec=%u col_len=%08x ppn=%08x\n",
			  slot, span, rec, col_len, addr);
	}
	/* Post-pass: OR bit31 onto last CE-select for each CE present. */
	cl[0] |= 0x80000000u;
	cl[4] = ce_bit | 1u;
	cl[5] = nsect;
	cl[6] = (u32)f->spare_dma;
	cl[7] = (u32)f->data_dma;
	cl[8] = 0x00010002u;

	if (dma_pulse) {
		unsigned int t;

		writel(0, f->base + 0xc6c);
		writel(1, f->base + 0xc60);
		for (t = 0; t < 200000; t++) {
			if (readl(f->base + 0xc64) == 1)
				break;
			udelay(1);
		}
		writel(0, f->base + 0xc60);
	} else {
		writel(dma_c6c, f->base + 0xc6c);
	}

	if (dma_preamble) {
		writel(1, f->base + 0xc10);
		writel(4, f->base + 0xc58);
		writel(0x0b00, f->base + 0xc4c);
	}

	/*
	 * Clear/idle before programming D-regs / C04 (USB/PL080 pattern:
	 * clear status, prove idle, program pointers, kick last).
	 */
	if (!fmss_cs_preflight(f)) {
		ret = -EBUSY;
		goto dma_done;
	}

	/* 4EDDDC register skeleton — bus addresses only. */
	writel(page_ctrl0_or, f->base + FMGEN1);		/* D04 timing template */
	writel((u32)f->cmdl_dma, f->base + FMGEN2);	/* D08 descriptor list */
	writel(FMSS_SECTOR_LEN, f->base + FMGEN3);	/* D0C = 4096 */
	writel((u32)f->stbuf_dma, f->base + FMGEN4);	/* D10 status */
	writel(dma_d14, f->base + FMGEN5);		/* D14 = addr_cycles-1 */
	writel((u32)f->seq_dma, f->base + FMSEQBASE);	/* C04 = seq program */
	/*
	 * Do NOT poke +0x81C here — that is 4EB458 ECC, not in 4EDDDC/D39EC.
	 * A spurious 81C write before CS previously correlated with SoC wedges.
	 */
	/* D39EC: C00 = 0xFFF5. Do NOT use 0x80000 (reset) here. */
	reinit_completion(&f->cs_irq);
	fmss_info("dma kick ce=%u addr=%08x seq=%08x cmdl=%08x data=%08x meta=%08x st=%08x d14=%u kick=%04x dry=%d armed=%d\n",
		  ce, addr, (u32)f->seq_dma, (u32)f->cmdl_dma, (u32)f->data_dma,
		  (u32)f->spare_dma, (u32)f->stbuf_dma, dma_d14, dma_kick,
		  dma_dry, dma_armed);
	if (dma_dry) {
		f->last_dma_c0c = readl(f->base + FMSEQIRQ);
		f->last_dma_d00 = readl(f->base + FMGEN0);
		f->last_dma_c00 = readl(f->base + FMSEQ);
		ret = -EAGAIN;
		goto dma_done;
	}
	if (!dma_armed) {
		ret = -EPERM;
		goto dma_done;
	}
	if (dma_one_shot)
		dma_armed = false;

	/*
	 * Publish the descriptor, data and status buffers before the
	 * sequencer fetches them. Coherent allocations need no explicit
	 * cache maintenance, but they do need ordering.
	 */
	dma_wmb();
	/* Pair the DMA-visible write above with a full barrier. */
	wmb();

	/* Flush posted MMIO programming before the sequencer kick. */
	readl(f->base + FMGEN2);
	readl(f->base + FMGEN3);
	readl(f->base + FMGEN4);
	readl(f->base + FMGEN5);
	readl(f->base + FMSEQBASE);

	writel(dma_kick, f->base + FMSEQ);
	readl(f->base + FMSEQ); /* posted write flush */

	/*
	 * Prefer IRQ completion snapshot (ISR already W1C'd C0C). Fall back
	 * to short poll only when no IRQ or completion never arrived.
	 */
	if (f->irq > 0) {
		if (wait_for_completion_timeout(&f->cs_irq,
						msecs_to_jiffies(200))) {
			ret = 0;
		} else {
			ret = fmss_wait_cs_poll(f, 2000);
		}
	} else {
		ret = fmss_wait_cs_poll(f, 20000);
	}

	/*
	 * Avoid require C0C to still be live — ISR clears it for VIC EOI.
	 * Trust the saved snapshot from ISR or poll.
	 */
	if (!ret && f->last_dma_c0c &&
	    ((f->last_dma_c0c & 0x0d) != 1))
		ret = -EIO;

	f->last_dma_d00 = readl(f->base + FMGEN0);
	f->last_dma_c00 = readl(f->base + FMSEQ);
	fmss_peek_vic1(f);

	for (i = 0; i < 32; i++)
		dregs[i] = readl(f->base + FMGEN0 + i * 4);

	writel(13, f->base + FMSEQIRQ);
	for (i = 0; i < 10000; i++) {
		if ((readl(f->base + FMSEQIRQ) & 0xd) == 0)
			break;
		udelay(1);
	}
	writel(1, f->base + FMCTRL0);

dma_done:
	if (ret == -EAGAIN) {
		for (i = 0; i < 32; i++)
			dregs[i] = readl(f->base + FMGEN0 + i * 4);
		fmss_peek_vic1(f);
	}

	memcpy(f->last_page, f->data, FMSS_PAGE_LEN);
	spare_bytes = nsect * 16;
	if (spare_bytes > sizeof(f->last_spare))
		spare_bytes = sizeof(f->last_spare);
	memcpy(f->last_spare, f->spare, spare_bytes);
	f->last_spare_len = spare_bytes;
	/* 16 B Sogeti spare per 4 KiB sector → copy into last_parity[group]. */
	for (i = 0; i < nsect; i++) {
		unsigned int c, idx = i * 4;

		if (idx >= FMSS_MAX_CHUNKS)
			break;
		memcpy(f->last_parity[idx], f->spare + i * 16, 16);
		f->last_parity_len[idx] = 16;
		for (c = 1; c < 4 && idx + c < FMSS_MAX_CHUNKS; c++)
			f->last_parity_len[idx + c] = 0;
	}
	f->last_page_ce = (int)ce;
	f->last_page_addr = addr;
	f->last_page_len = nsect * FMSS_SECTOR_LEN;
	f->last_page_chunk = (int)((u8 *)f->stbuf)[0];
	f->last_page_ret = ret;
	f->last_stat48 = readl(f->base + FMSTAT48);
	f->last_nandstat = readl(f->base + NANDSTAT);

	dp = f->data;
	fmss_info("dma ce=%u addr=%08x ret=%d kick=%04x nsect=%u rowlo=%u d14=%u c00=%08x c04=%08x c0c=%08x c6c=%08x d00=%08x d04=%08x d08=%08x d0c=%08x d10=%08x d14=%08x st=%08x spare=%02x%02x%02x%02x data=%02x%02x%02x%02x%02x%02x%02x%02x\n",
		  ce, addr, ret, dma_kick, nsect, dma_row_in_lo, dma_d14,
		  f->last_dma_c00, readl(f->base + FMSEQBASE), f->last_dma_c0c,
		  readl(f->base + 0xc6c), dregs[0], dregs[1], dregs[2], dregs[3],
		  dregs[4], dregs[5], *(u32 *)f->stbuf,
		  f->last_spare[0], f->last_spare[1], f->last_spare[2],
		  f->last_spare[3],
		  dp[0], dp[1], dp[2], dp[3], dp[4], dp[5], dp[6], dp[7]);
	fmss_info("dma D00..D7C: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x | %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		  dregs[0], dregs[1], dregs[2], dregs[3], dregs[4], dregs[5],
		  dregs[6], dregs[7], dregs[8], dregs[9], dregs[10], dregs[11],
		  dregs[12], dregs[13], dregs[14], dregs[15], dregs[16], dregs[17],
		  dregs[18], dregs[19], dregs[20], dregs[21], dregs[22], dregs[23],
		  dregs[24], dregs[25], dregs[26], dregs[27], dregs[28], dregs[29],
		  dregs[30], dregs[31]);
	fmss_info("dma cmdl: %08x %08x %08x %08x | %08x %08x %08x %08x | %08x  seq_dma=%08x cmdl_dma=%08x data_dma=%08x meta_dma=%08x\n",
		  cl[0], cl[1], cl[2], cl[3], cl[4], cl[5], cl[6], cl[7], cl[8],
		  (u32)f->seq_dma, (u32)f->cmdl_dma, (u32)f->data_dma,
		  (u32)f->spare_dma);
	if (!ret && !dma_skip_ingest && f->last_spare_len >= 16) {
		unsigned int pg = addr & L2V_PAGE_MASK;
		unsigned int blk = (addr >> FMSS_PAGE_BITS) & L2V_BLOCK_MASK;
		unsigned int cau = (addr >> (FMSS_PAGE_BITS + FMSS_BLOCK_BITS)) &
				   ((1u << FMSS_CAU_BITS) - 1u);

		fmss_meta_ingest_spare(f, ce, cau, blk, pg, dma_slot);
	}
	return ret;
}

/*
 * Read the 512-byte PPN parameter page.
 *
 * Command sequence: 0x92, address 0, 0x97, then 0x77/0x7D to select the
 * page, then 0x7A and 512 bytes of PIO.
 */
static int fmss_param_read(struct nand_s5l8740 *f, unsigned int ce)
{
	u8 st = 0;
	int ret;

	if (ce > 7)
		return -EINVAL;
	memset(f->last_param, 0, FMSS_PARAM_LEN);
	f->last_param_ce = (int)ce;

	writel(fmss_ctrl0(ce), f->base + FMCTRL0);
	if (fmss_cmd(f, 0x92)) {
		pr_info("s5l8740-nand: param cmd 0x92 timeout\n");
		ret = -ETIMEDOUT;
		goto out_idle;
	}
	if (fmss_addr1(f, 0)) {
		pr_info("s5l8740-nand: param addr timeout st=%08x\n", f->last_stat48);
		ret = -ETIMEDOUT;
		goto out_idle;
	}
	if (fmss_cmd(f, 0x97)) {
		pr_info("s5l8740-nand: param cmd 0x97 timeout\n");
		ret = -ETIMEDOUT;
		goto out_idle;
	}
	fmss_cmd(f, 0x77);
	fmss_cmd(f, 0x7d);
	if (fmss_wait_status(f, 100000, &st)) {
		pr_info("s5l8740-nand: param ready timeout NANDSTAT=%02x st48=%08x\n",
			st, f->last_stat48);
		ret = -ETIMEDOUT;
		goto out_idle;
	}
	fmss_cmd(f, 0x7a);
	ret = fmss_data_in(f, f->last_param, FMSS_PARAM_LEN);
	fmss_cmd(f, 0x77);
out_idle:
	writel(1, f->base + FMCTRL0);
	f->last_param_ret = ret;
	fmss_info("param ce=%u ret=%d nandstat=%02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		  ce, ret, st,
		  f->last_param[0], f->last_param[1], f->last_param[2], f->last_param[3],
		  f->last_param[4], f->last_param[5], f->last_param[6], f->last_param[7]);
	return ret;
}

/*
 * Reset the FMSS controller only; this touches no NAND array state.
 * Each CE is then issued command 0xFF, matching the stock reset path.
 */
static int fmss_ctrl_reset(struct nand_s5l8740 *f)
{
	unsigned int i;
	u32 v;

	writel(1, f->base + FMCTRL0);
	writel(8, f->base + FMSEQ);
	v = readl(f->base + FMSEQSTAT);
	fmss_info("10453C after C00=8 c00=%08x c08=%08x\n",
		  readl(f->base + FMSEQ), v);
	for (i = 0; i < 200000; i++) {
		v = readl(f->base + FMSEQSTAT);
		if (v == 4) {
			writel(2, f->base + FMSEQ);
			for (i = 0; i < 200000; i++) {
				v = readl(f->base + FMSEQSTAT);
				if (v == 3 || v == 0)
					break;
				udelay(1);
			}
			break;
		}
		if (!v)
			break;
		udelay(1);
	}
	if (readl(f->base + 0xc6c) != 16) {
		writel(1, f->base + 0xc60);
		for (i = 0; i < 200000; i++) {
			if (readl(f->base + 0xc64) == 1)
				break;
			udelay(1);
		}
		writel(0, f->base + 0xc60);
	}
	writel(0x80000, f->base + FMSEQ);
	writel(2048, f->base + FMCTRL1);
	for (i = 0; i < 200000; i++) {
		if (readl(f->base + FMCTRL1) & 0x40000000u)
			break;
		udelay(1);
	}
	if (!(readl(f->base + FMCTRL1) & 0x40000000u)) {
		pr_info("s5l8740-nand: 10453C FMCTRL1 bit30 timeout v=%08x\n",
			readl(f->base + FMCTRL1));
		return -ETIMEDOUT;
	}
	writel(0x80000000, f->base + FMCTRL1);
	writel(1, f->base + FMUNK81C);
	return 0;
}

/* 130060: 10453C, then NAND RESET (0xFF) on CE0/CE1. */
static int fmss_nand_reset(struct nand_s5l8740 *f)
{
	unsigned int ce;
	int ret;

	/* Abort a wedged 50D960 (glass: STAT48 stuck 0x080c3002, FIL -110). */
	writel(0, f->base + FMCTRL0);
	writel(0x0FF00FFE, f->base + FMSTAT48);
	writel(13, f->base + FMSEQIRQ);
	udelay(100);
	writel(1, f->base + FMCTRL0);

	ret = fmss_ctrl_reset(f);
	if (ret)
		return ret;
	for (ce = 0; ce < 2; ce++) {
		writel((2u * (1u << ce)) | 0xFF001u, f->base + FMCTRL0);
		writel(2, f->base + FMSTAT48);
		if (fmss_cmd(f, 0xff))
			pr_info("s5l8740-nand: cmd 0xFF timeout ce=%u st=%08x\n",
				ce, f->last_stat48);
	}
	msleep(50);
	writel(1, f->base + FMCTRL0);
	/* 130544: PPN_FEATURE__POWER_STATE (384) = 2 on each CE. */
	for (ce = 0; ce < 2; ce++)
		fmss_set_feature(f, ce, 384, 2);
	fmss_info("nand_reset done NANDSTAT=%08x STAT48=%08x\n",
		  readl(f->base + NANDSTAT), readl(f->base + FMSTAT48));
	f->pages_since_reset = 0;
	return 0;
}

static u32 fmss_le32(const u8 *p, unsigned int off)
{
	u32 v;

	memcpy(&v, p + off, 4);
	return le32_to_cpu(v);
}

static ssize_t regs_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct nand_s5l8740 *f = nand_dev;
	static const u32 offs[] = {
		FMCTRL0, FMCTRL1, FMCMD, FMADDR, FMCE, FMUNK18, FMUNK24, FMUNK28,
		FMCYCLES, FMLEN, FMUNK38, FMSTAT48, NANDSTAT, FMDATA, FMSEQ, FMSEQSTAT,
		0xc04, 0xc10, 0xc38, 0xc4c, 0xc58, 0xc60, 0xc64, 0xc6c, FMUNK81C,
	};
	int i, n = 0;

	if (!f || !f->base)
		return -ENODEV;
	for (i = 0; i < ARRAY_SIZE(offs); i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "+0x%03x: 0x%08x\n",
			       offs[i], readl(f->base + offs[i]));
	return n;
}
static DEVICE_ATTR_RO(regs);

static ssize_t id_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct nand_s5l8740 *f = nand_dev;

	if (!f)
		return -ENODEV;
	return sysfs_emit(buf, "ce=%d %02x %02x %02x %02x %02x %02x %02x %02x\n",
			  f->last_ce,
			  f->last_id[0], f->last_id[1], f->last_id[2],
			  f->last_id[3], f->last_id[4], f->last_id[5],
			  f->last_id[6], f->last_id[7]);
}
static DEVICE_ATTR_RO(id);

static ssize_t read_id_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce;
	int ret;

	if (!f)
		return -ENODEV;
	if (kstrtouint(buf, 0, &ce))
		return -EINVAL;
	mutex_lock(&f->lock);
	ret = fmss_read_id(f, ce);
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "read_id ce=%u ret=%d id=%02x%02x%02x%02x%02x%02x%02x%02x\n",
		 ce, ret,
		 f->last_id[0], f->last_id[1], f->last_id[2], f->last_id[3],
		 f->last_id[4], f->last_id[5], f->last_id[6], f->last_id[7]);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(read_id);

static ssize_t page_status_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct nand_s5l8740 *f = nand_dev;

	if (!f)
		return -ENODEV;
	return sysfs_emit(buf,
			  "ce=%d addr=0x%08x ret=%d chunk=%d len=%u parity=%d cycles=%u chunks=%u stat48=0x%08x nand=0x%08x dma=%d c0c=0x%08x d00=0x%08x c00=0x%08x\n",
			  f->last_page_ce, f->last_page_addr, f->last_page_ret,
			  f->last_page_chunk, f->last_page_len, with_parity, addr_cycles,
			  page_chunks, f->last_stat48, f->last_nandstat, f->dma_ok,
			  f->last_dma_c0c, f->last_dma_d00, f->last_dma_c00);
}
static DEVICE_ATTR_RO(page_status);

static ssize_t page_hex_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct nand_s5l8740 *f = nand_dev;
	int i, n = 0;

	if (!f)
		return -ENODEV;
	for (i = 0; i < 256; i++) {
		n += scnprintf(buf + n, PAGE_SIZE - n, "%02x%s",
			       f->last_page[i], ((i + 1) % 16) ? " " : "\n");
	}
	return n;
}
static DEVICE_ATTR_RO(page_hex);

static ssize_t spare_hex_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct nand_s5l8740 *f = nand_dev;
	int i, n = 0;

	if (!f)
		return -ENODEV;
	n += scnprintf(buf + n, PAGE_SIZE - n, "len=%u ", f->last_spare_len);
	for (i = 0; i < (int)f->last_spare_len && i < 64; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "%02x ", f->last_spare[i]);
	n += scnprintf(buf + n, PAGE_SIZE - n, "\n");
	return n;
}
static DEVICE_ATTR_RO(spare_hex);

static u32 fmss_meta_lpn(const u8 *m, unsigned int len)
{
	if (len < 12)
		return ~0u;
	return get_unaligned_le32(m + 8);
}

static u8 fmss_meta_type(const u8 *m, unsigned int len)
{
	if (!len)
		return 0;
	return m[0];
}

static ssize_t parity_hex_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int i, j, n = 0;

	if (!f)
		return -ENODEV;
	for (i = 0; i < FMSS_MAX_CHUNKS; i++) {
		if (!f->last_parity_len[i])
			continue;
		n += scnprintf(buf + n, PAGE_SIZE - n, "chunk%u len=%u",
			       i, f->last_parity_len[i]);
		for (j = 0; j < f->last_parity_len[i] && j < 32; j++)
			n += scnprintf(buf + n, PAGE_SIZE - n, " %02x",
				       f->last_parity[i][j]);
		n += scnprintf(buf + n, PAGE_SIZE - n, " lpn=%u type=%02x\n",
			       fmss_meta_lpn(f->last_parity[i], f->last_parity_len[i]),
			       fmss_meta_type(f->last_parity[i], f->last_parity_len[i]));
	}
	if (!n)
		return sysfs_emit(buf, "(no parity captured — page_read with with_parity=1)\n");
	return n;
}
static DEVICE_ATTR_RO(parity_hex);

static ssize_t page_read_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, a, b, c, d, cycles;
	u32 addr;
	int nf, ret;

	if (!f)
		return -ENODEV;
	/* "CE CAU BLOCK PAGE SLC" or "CE ADDR [cycles]" */
	nf = sscanf(buf, "%u %i %u %u %u", &ce, &a, &b, &c, &d);
	if (nf == 5) {
		if (a > 1 || b >= FMSS_BLOCKS_PER_CAU || c > FMSS_BTOC_PAGE)
			return -EINVAL;
		addr = fmss_ppn_addr(a, b, c, d);
	} else {
		nf = sscanf(buf, "%u %i %u", &ce, &addr, &cycles);
		if (nf < 2)
			return -EINVAL;
		if (nf >= 3 && cycles >= 1 && cycles <= 8)
			addr_cycles = cycles;
	}
	mutex_lock(&f->lock);
	if (reset_every && f->pages_since_reset >= reset_every) {
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}
	ret = fmss_page_read(f, ce, addr);
	f->pages_since_reset++;
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(page_read);

static ssize_t dma_read_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, a, b, c, d, cycles;
	u32 addr;
	int nf, ret;

	if (!f)
		return -ENODEV;
	if (!f->dma_ok)
		return -ENODEV;
	nf = sscanf(buf, "%u %i %u %u %u", &ce, &a, &b, &c, &d);
	if (nf == 5) {
		if (a > 1 || b >= FMSS_BLOCKS_PER_CAU || c > FMSS_BTOC_PAGE)
			return -EINVAL;
		addr = fmss_ppn_addr(a, b, c, d);
	} else {
		nf = sscanf(buf, "%u %i %u", &ce, &addr, &cycles);
		if (nf < 2)
			return -EINVAL;
	}
	mutex_lock(&f->lock);

	if (dma_reset_before)
		fmss_nand_reset(f);

	ret = fmss_dma_page_read(f, ce, addr);
	f->pages_since_reset++;
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(dma_read);

/*
 * CS descriptor ABI canary — no FTL ingest / no lba_map mutation.
 *
 * Physical page: ce=0 cau=0 block=63 page=88 slc=0
 *
 * PIO may show *UOKJIHC at non-aligned offset 7816 — that is a raw
 * positive-control artifact only. CS full-page shows sector-aligned
 * boot-like payloads at 0 and 8192 (= 2×4096). Prefer rec=4112
 * (4096 data + 16 meta). Span4 is the primary trusted CS path;
 * sub-slot is optional.
 */
#define CS_CANARY_MARK		"*UOKJIHC"
#define CS_CANARY_MARK_LEN	8
#define CS_CANARY_CE		0u
#define CS_CANARY_CAU		0u
#define CS_CANARY_BLOCK		63u
#define CS_CANARY_PAGE		88u
#define CS_CANARY_SLC		0u
#define CS_CANARY_PIO_OFF	7816u	/* PIO-only artifact; not CS expect */
#define CS_CANARY_SLOT2_OFF	8192u
#define CS_CANARY_OEM_DELTA	3u
#define CS_CANARY_SECTOR	4096u
#define CS_CANARY_META		16u

static char cs_canary_log[8192];

static int fmss_find_bytes(const u8 *p, unsigned int n,
			   const char *s, unsigned int sl)
{
	unsigned int i;

	if (!sl || sl > n)
		return -1;
	for (i = 0; i + sl <= n; i++) {
		if (!memcmp(p + i, s, sl))
			return (int)i;
	}
	return -1;
}

static bool fmss_meta_nonblank(const u8 *m, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (m[i] != 0x00 && m[i] != 0xff)
			return true;
	}
	return false;
}

static bool fmss_looks_like_bpb(const u8 *p)
{
	return (p[0] == 0xeb || p[0] == 0xe9) &&
	       (p[2] == 0x90 || p[2] == 0x00);
}

/* Sector-aligned CS hit: BPB jump at expect_off and/or OEM at +3. */
static bool fmss_cs_canary_hit(const u8 *dp, unsigned int data_len,
			       unsigned int expect_off, int *mark_off)
{
	int hit;

	hit = fmss_find_bytes(dp, data_len, CS_CANARY_MARK, CS_CANARY_MARK_LEN);
	if (mark_off)
		*mark_off = hit;
	if (expect_off + 3 <= data_len && fmss_looks_like_bpb(dp + expect_off))
		return true;
	if (hit == (int)(expect_off + CS_CANARY_OEM_DELTA))
		return true;
	return false;
}

static u64 fmss_le48(const u8 *p)
{
	return (u64)p[0] | ((u64)p[1] << 8) | ((u64)p[2] << 16) |
	       ((u64)p[3] << 24) | ((u64)p[4] << 32) | ((u64)p[5] << 40);
}

static int fmss_cs_slot_report(const u8 *dp, const u8 *mp,
			       unsigned int slot, char *out, size_t out_sz)
{
	const u8 *d = dp + slot * CS_CANARY_SECTOR;
	const u8 *m = mp + slot * CS_CANARY_META;
	u16 bps = (u16)d[11] | ((u16)d[12] << 8);
	u8 spc = d[13];
	u32 lba = (u32)m[8] | ((u32)m[9] << 8) |
		  ((u32)m[10] << 16) | ((u32)m[11] << 24);
	int mark = fmss_find_bytes(d, CS_CANARY_SECTOR,
				   CS_CANARY_MARK, CS_CANARY_MARK_LEN);

	return scnprintf(out, out_sz,
			 "  slot%u data[0..15]=%*ph oem=%*ph bpb=%d "
			 "bps=%u spc=%u mark_off=%d\n"
			 "  slot%u meta=%*ph type=%02x flags=%02x "
			 "weave=%012llx lba=%u aux=%*ph nb=%d\n",
			 slot, 16, d, 8, d + 3, fmss_looks_like_bpb(d),
			 bps, spc, mark,
			 slot, 16, m, m[0], m[1],
			 (unsigned long long)fmss_le48(m + 2), lba,
			 4, m + 12, fmss_meta_nonblank(m, 16));
}

static int fmss_cs_canary_one(struct nand_s5l8740 *f, unsigned int ce, u32 addr,
			      unsigned int slot, unsigned int span,
			      unsigned int rec, unsigned int expect_off,
			      bool slot_report, char *out, size_t out_sz)
{
	unsigned int saved_slot = dma_slot;
	unsigned int saved_nsect = dma_nsect;
	unsigned int saved_rec = dma_rec;
	bool saved_armed = dma_armed;
	int ret, hit, n, s;
	bool pass;
	u8 *dp, *sp, *st;
	unsigned int data_len;

	dma_slot = slot;
	dma_nsect = span;
	dma_rec = rec;

	if (dma_reset_before)
		fmss_nand_reset(f);

	if (!dma_dry)
		dma_armed = true;

	dma_skip_ingest = true;
	ret = fmss_dma_page_read(f, ce, addr);
	dma_skip_ingest = false;

	dma_slot = saved_slot;
	dma_nsect = saved_nsect;
	dma_rec = saved_rec;
	if (dma_dry)
		dma_armed = saved_armed;

	dp = f->last_page;
	sp = f->last_spare;
	st = f->stbuf ? f->stbuf : f->last_spare;
	data_len = f->last_page_len;
	if (data_len > FMSS_PAGE_LEN)
		data_len = FMSS_PAGE_LEN;

	pass = (expect_off != 0xffffffffu) &&
	       fmss_cs_canary_hit(dp, data_len, expect_off, &hit);
	if (expect_off == 0xffffffffu)
		fmss_cs_canary_hit(dp, data_len, 0, &hit);

	n = scnprintf(out, out_sz,
		      "case slot=%u span=%u rec=%u ret=%d dry=%d expect_off=%u "
		      "mark_off=%d meta_nb=%d\n"
		      "  c00=%08x c04=%08x c08=%08x c0c=%08x c6c=%08x\n"
		      "  d00=%08x d04=%08x d08=%08x d0c=%08x d10=%08x d14=%08x\n"
		      "  st[0..15]=%*ph\n"
		      "  meta[0..15]=%*ph\n"
		      "  data[0..15]=%*ph\n",
		      slot, span, rec, ret, dma_dry,
		      expect_off == 0xffffffffu ? 0 : expect_off, hit,
		      fmss_meta_nonblank(sp, 16),
		      f->last_dma_c00, readl(f->base + FMSEQBASE),
		      readl(f->base + FMSEQSTAT), f->last_dma_c0c,
		      readl(f->base + 0xc6c),
		      f->last_dma_d00, readl(f->base + FMGEN1),
		      readl(f->base + FMGEN2), readl(f->base + FMGEN3),
		      readl(f->base + FMGEN4), readl(f->base + FMGEN5),
		      16, st, 16, sp, 16, dp);

	if (expect_off != 0xffffffffu && expect_off < data_len) {
		n += scnprintf(out + n, out_sz - n,
			       "  data@expect=%*ph%s\n",
			       16, dp + expect_off,
			       pass ? " HIT" :
			       (hit >= 0) ? " ELSEWHERE" : " MISS");
	}
	if (slot_report && span == 4 && data_len >= FMSS_PAGE_LEN) {
		for (s = 0; s < 4; s++)
			n += fmss_cs_slot_report(dp, sp, s,
						 out + n, out_sz - n);
	}
	if (pass)
		n += scnprintf(out + n, out_sz - n, "  RESULT=PASS_MARKER\n");
	else if (hit >= 0)
		n += scnprintf(out + n, out_sz - n,
			       "  RESULT=MARKER_AT_%d\n", hit);
	else if (!ret || ret == -EAGAIN)
		n += scnprintf(out + n, out_sz - n, "  RESULT=NO_MARKER\n");
	else
		n += scnprintf(out + n, out_sz - n, "  RESULT=ERR\n");

	return n;
}

static ssize_t cs_canary_read_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	if (!cs_canary_log[0])
		return sysfs_emit(buf, "no canary yet\n");
	return sysfs_emit(buf, "%s", cs_canary_log);
}

static ssize_t cs_canary_read_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, a, b, c, d;
	unsigned int slot = 0, span = 1, rec = FMSS_PPN_REC;
	unsigned int expect_off = 0xffffffffu;
	u32 addr;
	int nf, n;

	if (!f)
		return -ENODEV;
	if (!f->dma_ok)
		return -ENODEV;

	nf = sscanf(buf, "%u %i %u %u %u %u %u %u",
		    &ce, &a, &b, &c, &d, &slot, &span, &rec);
	if (nf >= 5) {
		if (a > 1 || b >= FMSS_BLOCKS_PER_CAU || c > FMSS_BTOC_PAGE)
			return -EINVAL;
		addr = fmss_ppn_addr(a, b, c, d);
		if (nf < 6)
			slot = dma_slot;
		if (nf < 7)
			span = 1;
		if (nf < 8)
			rec = dma_rec ? dma_rec : FMSS_PPN_REC;
	} else {
		nf = sscanf(buf, "%u %i %u %u %u",
			    &ce, &addr, &slot, &span, &rec);
		if (nf < 2)
			return -EINVAL;
		if (nf < 3)
			slot = dma_slot;
		if (nf < 4)
			span = 1;
		if (nf < 5)
			rec = dma_rec ? dma_rec : FMSS_PPN_REC;
	}

	if (slot > 3 || span < 1 || span > 4 || slot + span > 4)
		return -EINVAL;
	if (rec != 4096 && rec != 4112)
		return -EINVAL;

	/* Sector-aligned expects only. */
	if (span == 1)
		expect_off = 0;
	else if (span == 4 && slot == 0)
		expect_off = CS_CANARY_SLOT2_OFF; /* Apple boot in slot2 */

	if (!dma_dry && !dma_armed)
		return -EPERM;

	mutex_lock(&f->lock);
	n = scnprintf(cs_canary_log, sizeof(cs_canary_log),
		      "cs_canary ce=%u addr=%08x slot=%u span=%u rec=%u\n",
		      ce, addr, slot, span, rec);
	n += fmss_cs_canary_one(f, ce, addr, slot, span, rec, expect_off,
				span == 4, cs_canary_log + n,
				sizeof(cs_canary_log) - n);
	f->pages_since_reset++;
	nand_dev_info(dev, "%s", cs_canary_log);
	mutex_unlock(&f->lock);
	return count;
}
static DEVICE_ATTR_RW(cs_canary_read);

static ssize_t cs_canary_matrix_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	size_t len;

	if (!cs_canary_log[0])
		return sysfs_emit(buf,
				  "usage: echo 1 > cs_canary_matrix\n"
				  "page ce=%u cau=%u blk=%u pg=%u\n"
				  "PIO artifact off=%u (not CS expect)\n"
				  "E slot2/span1/rec4112 expect data@0 BPB/OEM\n"
				  "F slot2/span1/rec4096 (meta-in-data check)\n"
				  "G slot0/span4/rec4112 per-slot BPB+meta\n"
				  "A/B slot1 legacy; C/D span4 legacy\n"
				  "(full log also in dmesg)\n",
				  CS_CANARY_CE, CS_CANARY_CAU, CS_CANARY_BLOCK,
				  CS_CANARY_PAGE, CS_CANARY_PIO_OFF);
	len = strnlen(cs_canary_log, sizeof(cs_canary_log));
	if (len >= PAGE_SIZE)
		len = PAGE_SIZE - 1;
	memcpy(buf, cs_canary_log, len);
	buf[len] = '\0';
	return len;
}

static ssize_t cs_canary_matrix_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	u32 addr;
	unsigned int on = 0;
	int n;
	struct {
		unsigned int slot, span, rec, expect;
		bool slot_report;
		char tag;
	} cases[] = {
		/* Primary: sector-aligned slot2 + span4 decode. */
		{ 2, 1, 4112, 0, false, 'E' },
		{ 2, 1, 4096, 0, false, 'F' },
		{ 0, 4, 4112, CS_CANARY_SLOT2_OFF, true, 'G' },
	};
	unsigned int i;

	if (!f)
		return -ENODEV;
	if (!f->dma_ok)
		return -ENODEV;
	if (kstrtouint(buf, 0, &on) || !on)
		return -EINVAL;
	if (!dma_dry && !dma_armed)
		return -EPERM;

	addr = fmss_ppn_addr(CS_CANARY_CAU, CS_CANARY_BLOCK,
			     CS_CANARY_PAGE, CS_CANARY_SLC);

	mutex_lock(&f->lock);
	n = scnprintf(cs_canary_log, sizeof(cs_canary_log),
		      "cs_canary_matrix ce=%u cau=%u blk=%u pg=%u addr=%08x "
		      "dry=%d\n"
		      "note: PIO off=%u is artifact; CS expect sector @0/@8192\n",
		      CS_CANARY_CE, CS_CANARY_CAU, CS_CANARY_BLOCK,
		      CS_CANARY_PAGE, addr, dma_dry, CS_CANARY_PIO_OFF);

	/* PIO control — proves page exists; do not require CS at 7816. */
	{
		int pret, phit;
		unsigned int saved_chunks = page_chunks;
		unsigned int plen;

		page_chunks = FMSS_MAX_CHUNKS;
		pret = fmss_page_read(f, CS_CANARY_CE, addr);
		page_chunks = saved_chunks;
		plen = f->last_page_len;
		if (plen > FMSS_PAGE_LEN)
			plen = FMSS_PAGE_LEN;
		phit = fmss_find_bytes(f->last_page, plen,
				       CS_CANARY_MARK, CS_CANARY_MARK_LEN);
		n += scnprintf(cs_canary_log + n, sizeof(cs_canary_log) - n,
			       "PIO ret=%d mark_off=%d (artifact expect~%u) %s\n",
			       pret, phit, CS_CANARY_PIO_OFF,
			       (phit >= 0) ? "PASS_CONTROL" : "FAIL_CONTROL");
		if (pret || phit < 0) {
			nand_dev_info(dev, "%s", cs_canary_log);
			mutex_unlock(&f->lock);
			return -EIO;
		}
	}

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		n += scnprintf(cs_canary_log + n, sizeof(cs_canary_log) - n,
			       "\n=== %c ===\n", cases[i].tag);
		n += fmss_cs_canary_one(f, CS_CANARY_CE, addr,
					cases[i].slot, cases[i].span,
					cases[i].rec, cases[i].expect,
					cases[i].slot_report,
					cs_canary_log + n,
					sizeof(cs_canary_log) - n);
		f->pages_since_reset++;
	}

	nand_dev_info(dev, "%s", cs_canary_log);
	mutex_unlock(&f->lock);
	return count;
}
static DEVICE_ATTR_RW(cs_canary_matrix);

static char cs_phys_log[2048];

static ssize_t cs_phys_read_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	if (!cs_phys_log[0])
		return sysfs_emit(buf,
				  "usage: echo CE CAU BLK PG [fmss_lba] > cs_phys_read\n"
				  "CS span4/rec4112 physical page; optional fmss_lba pick\n"
				  "requires dma_dry=0 dma_armed=1; no lba_map ingest\n");
	return sysfs_emit(buf, "%s", cs_phys_log);
}

static ssize_t cs_phys_last_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return cs_phys_read_show(dev, attr, buf);
}
static DEVICE_ATTR_RO(cs_phys_last);

static ssize_t cs_phys_read_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, block, page;
	u32 want_lba = 0xffffffffu;
	struct s5l8740_cs_page *pg;
	int nf, ret, n, s, pick;

	if (!f || !f->dma_ok)
		return -ENODEV;
	/* Accept optional trailing SLC (ignored; CS phys is always MLC=0). */
	nf = sscanf(buf, "%u %u %u %u %u",
		    &ce, &cau, &block, &page, &want_lba);
	if (nf < 4)
		return -EINVAL;
	/*
	 * Legacy: "ce cau blk pg slc [lba]" — if 5th token is 0/1 treat as
	 * SLC and take optional 6th as fmss_lba.
	 */
	if (nf == 5 && want_lba <= 1) {
		unsigned int slc_ignored = want_lba;
		u32 lba6 = 0xffffffffu;
		int n6 = sscanf(buf, "%u %u %u %u %u %u",
				&ce, &cau, &block, &page, &slc_ignored, &lba6);
		(void)slc_ignored;
		want_lba = (n6 >= 6) ? lba6 : 0xffffffffu;
	}

	pg = kzalloc(sizeof(*pg), GFP_KERNEL);
	if (!pg)
		return -ENOMEM;

	ret = s5l8740_nand_cs_phys_read(ce, cau, block, page, pg);
	n = scnprintf(cs_phys_log, sizeof(cs_phys_log),
		      "cs_phys_read ce=%u cau=%u blk=%u pg=%u ret=%d "
		      "rec=%u span=4\n",
		      ce, cau, block, page, ret, N31_CS_REC_SIZE);
	if (!ret) {
		for (s = 0; s < N31_DATA_SLOTS; s++) {
			const struct s5l8740_meta_decoded *sm = &pg->meta[s];

			n += scnprintf(cs_phys_log + n, sizeof(cs_phys_log) - n,
				       "slot%u data=%*ph bpb=%d meta type=%02x "
				       "weave=%012llx fmss_lba=%u valid=%d "
				       "data_rec=%d\n",
				       s, 8, pg->data[s],
				       (pg->data[s][0] == 0xeb ||
					pg->data[s][0] == 0xe9),
				       sm->type, (unsigned long long)sm->weave,
				       sm->lba, sm->valid,
				       n31_meta_is_data_record(sm));
		}
		if (want_lba != 0xffffffffu) {
			pick = s5l8740_nand_meta_pick_lba(pg, want_lba);
			n += scnprintf(cs_phys_log + n, sizeof(cs_phys_log) - n,
				       "pick_fmss_lba=%u -> slot=%d weave=%012llx "
				       "type=%02x\n",
				       want_lba, pick,
				       pick >= 0 ?
				       (unsigned long long)pg->meta[pick].weave :
				       0ULL,
				       pick >= 0 ? pg->meta[pick].type : 0);
		}
	}
	nand_dev_info(dev, "%s", cs_phys_log);
	kfree(pg);
	return ret && ret != -EAGAIN && ret != -EPERM ? ret : count;
}
static DEVICE_ATTR_RW(cs_phys_read);

static int fmss_page_blankish(const u8 *p, unsigned int n);

/*
 * PPN DATA spare (CS META stream, type 0x01):
 * +0 type, +1 bank/flags, +2..+7 weaveSeq48, +8..+11 LBA LE, +12..+15 aux.
 * Whimory chooses the newest weave claimant before FMSS; manual phys reads
 * can hit stale historical LBA copies. lba_weave_scan lists them newest-first.
 */
#define FMSS_LBA_CLAIM_MAX	64
#define FMSS_LBA_TARGETS_MAX	8
struct fmss_lba_claim {
	u64 weave;
	u32 lba;
	u32 ppn;
	u16 block;
	u8 ce;
	u8 cau;
	u8 page;
	u8 slot;
	u8 typ;
};

static struct fmss_lba_claim lba_claims[FMSS_LBA_CLAIM_MAX];
static unsigned int lba_claim_count;
static unsigned int lba_claim_target; /* primary / first target */
static unsigned int lba_claim_targets[FMSS_LBA_TARGETS_MAX];
static unsigned int lba_claim_ntargets;
static unsigned int lba_claim_scanned;
static unsigned int lba_claim_hits;
static unsigned int lba_claim_small; /* type01 && lba < 4096 sightings */
static unsigned int lba_claim_be_alt; /* BE@+8 matched a target */
static char lba_claim_log[PAGE_SIZE];
static unsigned int lba_claim_log_len;

/* N31 META: weave[15:0]@+2 LE16 | weave[47:16]@+4 LE32 (5688C4 / 568ED4) */
static u64 fmss_ppn_weave48(const u8 *m)
{
	return (u64)get_unaligned_le16(m + 2) |
	       ((u64)get_unaligned_le32(m + 4) << 16);
}

static u32 fmss_ppn_meta_lba(const u8 *m)
{
	return get_unaligned_le32(m + 8);
}

static bool fmss_lba_is_target(u32 lba)
{
	unsigned int i;

	for (i = 0; i < lba_claim_ntargets; i++) {
		if (lba_claim_targets[i] == lba)
			return true;
	}
	return false;
}

static void fmss_lba_claim_insert(const struct fmss_lba_claim *c)
{
	unsigned int i, j;

	for (i = 0; i < lba_claim_count; i++) {
		if (lba_claims[i].ce == c->ce && lba_claims[i].cau == c->cau &&
		    lba_claims[i].block == c->block &&
		    lba_claims[i].page == c->page &&
		    lba_claims[i].slot == c->slot &&
		    lba_claims[i].lba == c->lba)
			return;
	}
	for (i = 0; i < lba_claim_count; i++) {
		if (c->weave > lba_claims[i].weave)
			break;
	}
	if (i >= FMSS_LBA_CLAIM_MAX)
		return;
	if (lba_claim_count < FMSS_LBA_CLAIM_MAX)
		lba_claim_count++;
	for (j = lba_claim_count - 1; j > i; j--)
		lba_claims[j] = lba_claims[j - 1];
	lba_claims[i] = *c;
}

static void fmss_lba_claim_note_page(struct nand_s5l8740 *f, unsigned int ce,
				     unsigned int cau, unsigned int block,
				     unsigned int page, u32 ppn)
{
	unsigned int s, nslots;
	const u8 *meta;

	if (f->last_page_ret)
		return;
	nslots = f->last_spare_len / 16;
	if (nslots > FMSS_VBAS_PER_PAGE)
		nslots = FMSS_VBAS_PER_PAGE;
	for (s = 0; s < nslots; s++) {
		struct fmss_lba_claim c;
		u32 lba_le, lba_be;

		meta = f->last_spare + s * 16;
		/*
		 * Diagnostic: any type whose LE LBA matches a target.
		 * Production mapping still prefers type 0x01.
		 */
		lba_le = fmss_ppn_meta_lba(meta);
		lba_be = get_unaligned_be32(meta + 8);
		if (meta[0] == 0x01 && lba_le < 4096u)
			lba_claim_small++;
		if (fmss_lba_is_target(lba_be) && !fmss_lba_is_target(lba_le))
			lba_claim_be_alt++;
		if (!fmss_lba_is_target(lba_le))
			continue;
		c.weave = fmss_ppn_weave48(meta);
		c.lba = lba_le;
		c.ppn = ppn;
		c.block = (u16)block;
		c.ce = (u8)ce;
		c.cau = (u8)cau;
		c.page = (u8)page;
		c.slot = (u8)s;
		c.typ = meta[0];
		fmss_lba_claim_insert(&c);
		lba_claim_hits++;
		/* Raw META for positive-control / endian debug. */
		pr_info("s5l8740-nand: META hit lba=%u typ=%02x weave=%012llx ce=%u cau=%u blk=%u pg=%u slot=%u meta=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
			lba_le, meta[0], (unsigned long long)c.weave,
			ce, cau, block, page, s,
			meta[0], meta[1], meta[2], meta[3],
			meta[4], meta[5], meta[6], meta[7],
			meta[8], meta[9], meta[10], meta[11],
			meta[12], meta[13], meta[14], meta[15]);
	}
}

/* Erased NAND only — do not treat all-zero DMA failure residue as blank. */
static int fmss_page_erased(const u8 *p, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (p[i] != 0xff)
			return 0;
	}
	return 1;
}

/*
 * CS full-page (slot0/span4) meta hunt for one or more LBAs.
 * echo "LBA [START [NBLOCKS]]" > lba_weave_scan
 * echo "121,122,123 [START [NBLOCKS]]" > lba_weave_scan
 * Default: START=0 NBLOCKS=256 (user area; skip VFL tail).
 */
static int fmss_lba_weave_scan(struct nand_s5l8740 *f, unsigned int start,
			       unsigned int nblocks)
{
	unsigned int ce, cau, b, pg;
	unsigned int saved_nsect, saved_slot, user_max;
	u32 addr;
	int ret = 0;

	user_max = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;
	if (!nblocks)
		nblocks = 256;
	if (start >= user_max)
		return -EINVAL;
	if (start + nblocks > user_max)
		nblocks = user_max - start;
	if (!lba_claim_ntargets)
		return -EINVAL;

	lba_claim_count = 0;
	lba_claim_hits = 0;
	lba_claim_scanned = 0;
	lba_claim_small = 0;
	lba_claim_be_alt = 0;
	lba_claim_target = lba_claim_targets[0];
	lba_claim_log_len = 0;

	saved_nsect = dma_nsect;
	saved_slot = dma_slot;
	dma_nsect = FMSS_VBAS_PER_PAGE;
	dma_slot = 0;

	for (unsigned int cei = 0; cei < FMSS_NUM_CE; cei++) {
		/* Prefer CE1 first — known user DATA / music live there. */
		ce = (cei == 0) ? 1u : 0u;
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (b = start; b < start + nblocks; b++) {
				unsigned int fail = 0;

				for (pg = 0; pg < FMSS_BTOC_PAGE; pg++) {
					/* Always re-init before CS — hung C0C wedges SoC. */
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
					addr = fmss_ppn_addr(cau, b, pg, 0);
					ret = fmss_dma_page_read(f, ce, addr);
					lba_claim_scanned++;
					if (ret) {
						fail++;
						if (fail >= 2)
							break;
						continue;
					}
					fail = 0;
					fmss_lba_claim_note_page(f, ce, cau, b,
								 pg, addr);
					/* Fully erased page0 → skip rest of block. */
					if (pg == 0 &&
					    fmss_page_erased(f->last_page, 64) &&
					    fmss_page_erased(f->last_spare, 16))
						break;
				}
				if ((b & 7) == 0) {
					pr_info("s5l8740-nand: lba_weave_scan prog targets=%u scanned=%u hits=%u small=%u be_alt=%u ce=%u cau=%u blk=%u\n",
						lba_claim_ntargets,
						lba_claim_scanned, lba_claim_hits,
						lba_claim_small, lba_claim_be_alt,
						ce, cau, b);
					lba_claim_log_len = scnprintf(
						lba_claim_log, sizeof(lba_claim_log),
						"INPROGRESS scanned=%u hits=%u small=%u be_alt=%u kept=%u ce=%u cau=%u blk=%u\n",
						lba_claim_scanned, lba_claim_hits,
						lba_claim_small, lba_claim_be_alt,
						lba_claim_count, ce, cau, b);
				}
			}
		}
	}

	dma_nsect = saved_nsect;
	dma_slot = saved_slot;

	lba_claim_log_len = scnprintf(lba_claim_log, sizeof(lba_claim_log),
		"targets=%u scanned=%u hits=%u small=%u be_alt=%u kept=%u (newest weave first)\n",
		lba_claim_ntargets, lba_claim_scanned, lba_claim_hits,
		lba_claim_small, lba_claim_be_alt, lba_claim_count);
	for (b = 0; b < lba_claim_count; b++) {
		const struct fmss_lba_claim *c = &lba_claims[b];

		lba_claim_log_len += scnprintf(
			lba_claim_log + lba_claim_log_len,
			sizeof(lba_claim_log) - lba_claim_log_len,
			"CSV %u,%012llx,%u,%u,%u,%u,%u,%08x\n",
			c->lba, (unsigned long long)c->weave, c->ce, c->cau,
			c->block, c->page, c->slot, c->ppn);
		lba_claim_log_len += scnprintf(
			lba_claim_log + lba_claim_log_len,
			sizeof(lba_claim_log) - lba_claim_log_len,
			"%u: lba=%u weave=%012llx ce=%u cau=%u blk=%u pg=%u slot=%u ppn=%08x\n",
			b, c->lba, (unsigned long long)c->weave, c->ce, c->cau,
			c->block, c->page, c->slot, c->ppn);
	}
	pr_info("s5l8740-nand: lba_weave_scan %s", lba_claim_log);
	return 0;
}

static int fmss_lba_parse_targets(const char *s, unsigned int *start,
				  unsigned int *nblocks)
{
	unsigned int n = 0, a = 0, b = 256;
	const char *p = s;
	char *list, *cursor, *tok;
	size_t len = 0;

	lba_claim_ntargets = 0;
	while (p[len] && p[len] != ' ' && p[len] != '\t' && p[len] != '\n')
		len++;
	list = kstrndup(p, len, GFP_KERNEL);
	if (!list)
		return -ENOMEM;
	cursor = list;
	while ((tok = strsep(&cursor, ",")) != NULL) {
		unsigned int v;

		if (n >= FMSS_LBA_TARGETS_MAX || kstrtouint(tok, 0, &v)) {
			kfree(list);
			return -EINVAL;
		}
		lba_claim_targets[n++] = v;
	}
	kfree(list);
	p += len;
	if (!n)
		return -EINVAL;
	while (*p == ' ' || *p == '\t')
		p++;
	if (*p) {
		if (sscanf(p, "%u %u", &a, &b) < 1)
			return -EINVAL;
	}
	*start = a;
	*nblocks = b;
	lba_claim_ntargets = n;
	return 0;
}

static ssize_t lba_weave_scan_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int start = 0, nblocks = 256;
	int ret;

	if (!f || !f->dma_ok)
		return -ENODEV;
	ret = fmss_lba_parse_targets(buf, &start, &nblocks);
	if (ret)
		return ret;
	mutex_lock(&f->lock);
	fmss_nand_reset(f);
	ret = fmss_lba_weave_scan(f, start, nblocks);
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}

static ssize_t lba_weave_scan_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	if (!lba_claim_log_len)
		return sysfs_emit(buf, "no scan yet\n");
	return sysfs_emit(buf, "%s", lba_claim_log);
}
static DEVICE_ATTR_RW(lba_weave_scan);

static ssize_t seq_kick_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int cmd;

	if (!f)
		return -ENODEV;
	if (kstrtouint(buf, 0, &cmd))
		return -EINVAL;
	mutex_lock(&f->lock);
	if (!dma_armed) {
		mutex_unlock(&f->lock);
		return -EPERM;
	}
	if (dma_one_shot)
		dma_armed = false;
	writel(cmd, f->base + FMSEQ);
	f->last_dma_c00 = readl(f->base + FMSEQ);
	f->last_dma_c0c = readl(f->base + FMSEQIRQ);
	nand_dev_info(dev, "seq_kick wrote 0x%x now c00=%08x c08=%08x c0c=%08x c04=%08x c38=%08x\n",
		 cmd, f->last_dma_c00, readl(f->base + FMSEQSTAT),
		 f->last_dma_c0c, readl(f->base + FMSEQBASE),
		 readl(f->base + 0xc38));
	mutex_unlock(&f->lock);
	return count;
}
static DEVICE_ATTR_WO(seq_kick);

static int fmss_find(const u8 *p, unsigned int n, const char *s, unsigned int sl)
{
	unsigned int i;

	if (sl == 0 || sl > n)
		return 0;
	for (i = 0; i + sl <= n; i++) {
		if (!memcmp(p + i, s, sl))
			return 1;
	}
	return 0;
}

/*
 * Match ASCII needle as UTF-16LE or UTF-16BE (FAT LFN / wide strings).
 * Returns match byte offset+1, or 0 if not found.
 */
static unsigned int fmss_find_utf16(const u8 *p, unsigned int n, const char *s,
				    unsigned int sl, bool be)
{
	unsigned int i, j;

	if (sl == 0 || n < sl * 2)
		return 0;
	for (i = 0; i + sl * 2 <= n; i++) {
		for (j = 0; j < sl; j++) {
			u8 lo = be ? p[i + j * 2 + 1] : p[i + j * 2];
			u8 hi = be ? p[i + j * 2] : p[i + j * 2 + 1];

			if (hi != 0 || lo != (u8)s[j])
				break;
		}
		if (j == sl)
			return i + 1;
	}
	return 0;
}

/* 16-bit byte-swap of each pair: "AB" -> "BA" stream — PIO/DMA mis-endian probe. */
static int fmss_find_bswap16(const u8 *p, unsigned int n, const char *s,
			     unsigned int sl)
{
	unsigned int i, j;

	if (sl < 2 || n < sl)
		return 0;
	for (i = 0; i + sl <= n; i++) {
		for (j = 0; j + 1 < sl; j += 2) {
			if (p[i + j] != (u8)s[j + 1] ||
			    p[i + j + 1] != (u8)s[j])
				break;
		}
		if (j >= sl - (sl & 1)) {
			if (!(sl & 1) || p[i + j] == (u8)s[j])
				return 1;
		}
	}
	return 0;
}

enum {
	FMSS_MATCH_ASCII = 1,
	FMSS_MATCH_UTF16LE = 2,
	FMSS_MATCH_UTF16BE = 4,
	FMSS_MATCH_BSWAP16 = 8,
};

/* OR of FMSS_MATCH_* flags; also returns first hit offset via *off_out. */
static unsigned int fmss_find_enc(const u8 *p, unsigned int n, const char *s,
				  unsigned int sl, unsigned int *off_out)
{
	unsigned int flags = 0, i, u;

	if (off_out)
		*off_out = 0;
	if (sl == 0 || sl > n)
		return 0;
	for (i = 0; i + sl <= n; i++) {
		if (!memcmp(p + i, s, sl)) {
			flags |= FMSS_MATCH_ASCII;
			if (off_out && !*off_out)
				*off_out = i;
			break;
		}
	}
	u = fmss_find_utf16(p, n, s, sl, false);
	if (u) {
		flags |= FMSS_MATCH_UTF16LE;
		if (off_out && !*off_out)
			*off_out = u - 1;
	}
	u = fmss_find_utf16(p, n, s, sl, true);
	if (u) {
		flags |= FMSS_MATCH_UTF16BE;
		if (off_out && !*off_out)
			*off_out = u - 1;
	}
	if (fmss_find_bswap16(p, n, s, sl))
		flags |= FMSS_MATCH_BSWAP16;
	return flags;
}

static const char *fmss_match_enc_name(unsigned int flags)
{
	if (flags & FMSS_MATCH_ASCII)
		return "ascii";
	if (flags & FMSS_MATCH_UTF16LE)
		return "utf16le";
	if (flags & FMSS_MATCH_UTF16BE)
		return "utf16be";
	if (flags & FMSS_MATCH_BSWAP16)
		return "bswap16";
	return "none";
}

static int fmss_page_blankish(const u8 *p, unsigned int n)
{
	unsigned int i, ff = 0, zz = 0;

	for (i = 0; i < n; i++) {
		if (p[i] == 0xff)
			ff++;
		else if (p[i] == 0x00)
			zz++;
	}
	return ff == n || zz == n;
}

static u32 fmss_btoc_entry_be(const u8 *btoc_page, unsigned int idx)
{
	return get_unaligned_be32(btoc_page + idx * 4);
}

static u32 fmss_btoc_entry_le(const u8 *btoc_page, unsigned int idx)
{
	return get_unaligned_le32(btoc_page + idx * 4);
}

/* Default YaFTL/Sogeti BTOC is big-endian (live blk64: 00 00 00 0b …). */
static u32 fmss_btoc_entry(const u8 *btoc_page, unsigned int idx)
{
	return fmss_btoc_entry_be(btoc_page, idx);
}

/* Pick BE vs LE for a BTOC page: prefer the endian with more plausible LPNs. */
static bool fmss_btoc_prefer_le(const u8 *btoc)
{
	unsigned int i, good_be = 0, good_le = 0;
	u32 prev_be = 0, prev_le = 0;
	unsigned int seq_be = 0, seq_le = 0;

	for (i = 0; i < 16; i++) {
		u32 be = fmss_btoc_entry_be(btoc, i);
		u32 le = fmss_btoc_entry_le(btoc, i);

		if (be != 0xffffffff && be < 0x01000000u)
			good_be++;
		if (le != 0xffffffff && le < 0x01000000u)
			good_le++;
		if (i && be == prev_be + 1)
			seq_be++;
		if (i && le == prev_le + 1)
			seq_le++;
		prev_be = be;
		prev_le = le;
	}
	if (seq_le > seq_be && good_le >= good_be)
		return true;
	if (good_le >= 4 && good_be <= 1)
		return true;
	return false;
}

/* D: probe 2026-08-24: EB 3C 90 OEM "*UOKJIHC", vol "AISPOD FAT32"
 * Live copy may have 55AA at 0x1C9 rather than 510 — match OEM, patch on carve.
 */
static bool fmss_apple_fat_sig(const u8 *s)
{
	return s[0] == 0xeb && s[1] == 0x3c && s[2] == 0x90 &&
	       s[3] == '*' && s[4] == 'U' && s[5] == 'O' && s[6] == 'K' &&
	       s[7] == 'J' && s[8] == 'I' && s[9] == 'H' && s[10] == 'C';
}

static bool fmss_apple_fat_boot(const u8 *s)
{
	return fmss_apple_fat_sig(s) && s[510] == 0x55 && s[511] == 0xaa;
}

static bool fmss_page_find_apple_bpb(const u8 *page, unsigned int len,
				     unsigned int *off_out)
{
	unsigned int off;

	if (len < FMSS_SECTOR_LEN)
		return false;
	for (off = 0; off + FMSS_SECTOR_LEN <= len; off++) {
		if (fmss_apple_fat_sig(page + off)) {
			if (off_out)
				*off_out = off;
			return true;
		}
	}
	return false;
}

static bool fmss_page_has_fat_boot(const u8 *page, unsigned int len,
				   unsigned int *off_out)
{
	unsigned int off;

	for (off = 0; off + 512 <= len; off += 512) {
		if (fmss_apple_fat_boot(page + off) ||
		    fmss_apple_fat_sig(page + off)) {
			if (off_out)
				*off_out = off;
			return true;
		}
	}
	return fmss_page_find_apple_bpb(page, len, off_out);
}

/* FAT32 DataStart = reserved + nFATS * FATSz32 (logical sectors). */
static unsigned int fmss_bpb_data_start(const u8 *bpb)
{
	u16 bps = get_unaligned_le16(bpb + 11);
	u16 rsvd = get_unaligned_le16(bpb + 14);
	u8 nfats = bpb[16];
	u32 fatz = get_unaligned_le32(bpb + 36);
	unsigned int start;

	if (bps != 512 && bps != 4096)
		return 1916;
	if (!nfats || nfats > 4 || !fatz)
		return 1916;
	start = (unsigned int)rsvd + (unsigned int)nfats * fatz;
	if (!start || start > NAND_FTL_DEFAULT_CAPACITY)
		return 1916;
	return start;
}

static u32 fmss_l2v_pack_sec(unsigned int ce, unsigned int cau,
			     unsigned int block, unsigned int page,
			     unsigned int sec, bool phys)
{
	u32 e = L2V_VALID |
		((ce & 3u) << L2V_CE_SHIFT) |
		((cau & 3u) << L2V_CAU_SHIFT) |
		((sec & L2V_SEC_MASK) << L2V_SEC_SHIFT) |
		((block & L2V_BLOCK_MASK) << L2V_BLOCK_SHIFT) |
		(page & L2V_PAGE_MASK);

	if (phys)
		e |= L2V_PHYS;
	return e;
}

static u32 fmss_l2v_pack(unsigned int ce, unsigned int cau,
			 unsigned int block, unsigned int page, bool phys)
{
	return fmss_l2v_pack_sec(ce, cau, block, page, L2V_SEC_FROM_LBA, phys);
}

static void fmss_l2v_unpack_sec(u32 e, unsigned int *ce, unsigned int *cau,
				unsigned int *block, unsigned int *page,
				unsigned int *sec)
{
	*ce = (e >> L2V_CE_SHIFT) & 3u;
	*cau = (e >> L2V_CAU_SHIFT) & 3u;
	*sec = (e >> L2V_SEC_SHIFT) & L2V_SEC_MASK;
	*block = (e >> L2V_BLOCK_SHIFT) & L2V_BLOCK_MASK;
	*page = e & L2V_PAGE_MASK;
}

static void fmss_l2v_unpack(u32 e, unsigned int *ce, unsigned int *cau,
			    unsigned int *block, unsigned int *page)
{
	unsigned int sec;

	fmss_l2v_unpack_sec(e, ce, cau, block, page, &sec);
	(void)sec;
}

static bool fmss_claim_better(u8 old_src, u64 old_weave, u8 new_src,
			      u64 new_weave)
{
	if (!old_src)
		return true;
	if (new_weave > old_weave)
		return true;
	if (new_weave < old_weave)
		return false;
	return new_src > old_src;
}

static void fmss_wmr_map_free(void)
{
	vfree(wmr_block_map);
	wmr_block_map = NULL;
	wmr_block_map_n = 0;
}

static void fmss_lba_map_free(void)
{
	vfree(lba_map);
	vfree(lba_weave);
	vfree(lba_src);
	lba_map = NULL;
	lba_weave = NULL;
	lba_src = NULL;
	lba_mapped = 0;
}

static unsigned int fmss_lba_map_cap(void)
{
	unsigned int cap = lba_map_max;

	if (!cap)
		cap = 262144;
	if (cap > FMSS_LBA_MAP_HARDMAX)
		cap = FMSS_LBA_MAP_HARDMAX;
	return cap;
}

static int fmss_lba_map_ensure(void)
{
	unsigned int cap;

	if (lba_map)
		return 0;
	cap = fmss_lba_map_cap();
	lba_map = vzalloc(array_size(cap, sizeof(*lba_map)));
	lba_weave = vzalloc(array_size(cap, sizeof(*lba_weave)));
	lba_src = vzalloc(array_size(cap, sizeof(*lba_src)));
	if (!lba_map || !lba_weave || !lba_src) {
		fmss_lba_map_free();
		return -ENOMEM;
	}
	pr_info("nand-s5l8740: LBA dense map cap=%u (~%u KiB)\n",
		cap, (cap * (4 + 8 + 1)) / 1024);
	return 0;
}

static void fmss_l2v_free(void)
{
	vfree(l2v_map);
	vfree(l2v_weave);
	vfree(l2v_src);
	l2v_map = NULL;
	l2v_weave = NULL;
	l2v_src = NULL;
	l2v_map_size = 0;
	l2v_mapped = 0;
	l2v_max_lpn = 0;
	l2v_btoc_hits = 0;
	l2v_bmap_hits = 0;
	l2v_meta_hits = 0;
	fmss_wmr_map_free();
}

static int fmss_l2v_ensure(unsigned int max_lpn)
{
	unsigned int need = max_lpn + 1;
	u32 *n;
	u64 *nw;
	u8 *ns;

	if (l2v_map && l2v_map_size >= need) {
		l2v_max_lpn = max_lpn;
		return 0;
	}
	n = vzalloc(array_size(need, sizeof(*n)));
	nw = vzalloc(array_size(need, sizeof(*nw)));
	ns = vzalloc(array_size(need, sizeof(*ns)));
	if (!n || !nw || !ns) {
		vfree(n);
		vfree(nw);
		vfree(ns);
		return -ENOMEM;
	}
	if (l2v_map) {
		memcpy(n, l2v_map, l2v_map_size * sizeof(*n));
		if (l2v_weave)
			memcpy(nw, l2v_weave, l2v_map_size * sizeof(*nw));
		if (l2v_src)
			memcpy(ns, l2v_src, l2v_map_size * sizeof(*ns));
		vfree(l2v_map);
		vfree(l2v_weave);
		vfree(l2v_src);
	}
	l2v_map = n;
	l2v_weave = nw;
	l2v_src = ns;
	l2v_map_size = need;
	l2v_max_lpn = max_lpn;
	return 0;
}

static void fmss_l2v_index_note(unsigned int lpn, unsigned int ce,
				unsigned int cau, unsigned int block,
				unsigned int page)
{
	unsigned int i;

	for (i = 0; i < lpn_index_count; i++) {
		if (lpn_index[i].lpn == lpn) {
			lpn_index[i].ce = ce;
			lpn_index[i].cau = cau;
			lpn_index[i].block = block;
			lpn_index[i].page = page;
			return;
		}
	}
	if (lpn_index_count >= FMSS_LPN_INDEX_MAX)
		return;
	lpn_index[lpn_index_count].lpn = lpn;
	lpn_index[lpn_index_count].ce = ce;
	lpn_index[lpn_index_count].cau = cau;
	lpn_index[lpn_index_count].block = block;
	lpn_index[lpn_index_count].page = page;
	lpn_index_count++;
}

static void fmss_l2v_set_ex(unsigned int lpn, unsigned int ce, unsigned int cau,
			    unsigned int block, unsigned int page, bool phys,
			    u8 src, u64 weave)
{
	u32 prev, packed;
	u8 old_src;
	u64 old_weave;

	if (!l2v_map || lpn >= l2v_map_size)
		return;
	prev = l2v_map[lpn];
	old_src = l2v_src ? l2v_src[lpn] : 0;
	old_weave = l2v_weave ? l2v_weave[lpn] : 0;
	if ((prev & L2V_VALID) &&
	    !fmss_claim_better(old_src, old_weave, src, weave))
		return;
	packed = fmss_l2v_pack(ce, cau, block, page, phys);
	l2v_map[lpn] = packed;
	if (l2v_src)
		l2v_src[lpn] = src;
	if (l2v_weave)
		l2v_weave[lpn] = weave;
	if (!(prev & L2V_VALID))
		l2v_mapped++;
	fmss_l2v_index_note(lpn, ce, cau, block, page);
	if (!quiet && l2v_mapped <= 8)
		pr_info("s5l8740-nand: l2v_set lpn=%u src=%u phys=%d ce=%u cau=%u blk=%u pg=%u weave=%llx\n",
			lpn, src, phys, ce, cau, block, page,
			(unsigned long long)weave);
}

static void fmss_lba_set(unsigned int lba, unsigned int ce, unsigned int cau,
			 unsigned int block, unsigned int page, unsigned int sec,
			 bool phys, u8 src, u64 weave)
{
	u32 prev, packed;
	u8 old_src;
	u64 old_weave;

	if (lba >= fmss_lba_map_cap() || fmss_lba_map_ensure())
		return;
	prev = lba_map[lba];
	old_src = lba_src[lba];
	old_weave = lba_weave[lba];
	if ((prev & L2V_VALID) &&
	    !fmss_claim_better(old_src, old_weave, src, weave))
		return;
	packed = fmss_l2v_pack_sec(ce, cau, block, page, sec & 3u, phys);
	lba_map[lba] = packed;
	lba_src[lba] = src;
	lba_weave[lba] = weave;
	if (!(prev & L2V_VALID))
		lba_mapped++;
	if (!quiet && lba_mapped <= 8)
		pr_info("s5l8740-nand: lba_set lba=%u src=%u phys=%d ce=%u cau=%u blk=%u pg=%u sec=%u weave=%llx\n",
			lba, src, phys, ce, cau, block, page, sec & 3u,
			(unsigned long long)weave);
}

static int fmss_lba_lookup(unsigned int lba, unsigned int *ce,
			   unsigned int *cau, unsigned int *block,
			   unsigned int *page, unsigned int *sec, u32 *packed)
{
	u32 e;

	if (!lba_map || lba >= fmss_lba_map_cap())
		return -ENOENT;
	e = lba_map[lba];
	if (!(e & L2V_VALID))
		return -ENOENT;
	fmss_l2v_unpack_sec(e, ce, cau, block, page, sec);
	if (packed)
		*packed = e;
	return 0;
}

static int fmss_l2v_lookup_ex(unsigned int lpn, unsigned int *ce,
			      unsigned int *cau, unsigned int *block,
			      unsigned int *page, u32 *packed)
{
	u32 e;

	if (!l2v_map || lpn >= l2v_map_size)
		return -ENOENT;
	e = l2v_map[lpn];
	if (!(e & L2V_VALID))
		return -ENOENT;
	fmss_l2v_unpack(e, ce, cau, block, page);
	if (packed)
		*packed = e;
	return 0;
}

static int fmss_l2v_lookup(unsigned int lpn, unsigned int *ce,
			   unsigned int *cau, unsigned int *block,
			   unsigned int *page)
{
	return fmss_l2v_lookup_ex(lpn, ce, cau, block, page, NULL);
}

static void fmss_early_lba_free(void)
{
	fmss_lba_map_free();
}

/*
 * Pass 2: promote PIO/DMA META slots into full lba_map.
 * type 0x01 data records; weave newest-wins via fmss_lba_set.
 */
static void fmss_meta_ingest_spare(struct nand_s5l8740 *f, unsigned int ce,
				   unsigned int cau, unsigned int block,
				   unsigned int page, unsigned int slot0)
{
	unsigned int s, nslots;

	if (!fmss_legacy_meta_ingest)
		return;
	if (!f || f->last_page_ret || f->last_spare_len < 16)
		return;
	if (fmss_lba_map_ensure())
		return;
	nslots = f->last_spare_len / 16;
	if (nslots > FMSS_VBAS_PER_PAGE)
		nslots = FMSS_VBAS_PER_PAGE;
	if (slot0 >= FMSS_VBAS_PER_PAGE)
		slot0 = 0;
	if (slot0 + nslots > FMSS_VBAS_PER_PAGE)
		nslots = FMSS_VBAS_PER_PAGE - slot0;
	for (s = 0; s < nslots; s++) {
		const u8 *meta = f->last_spare + s * 16;
		u32 lba, before;
		u64 weave;

		if (meta[0] != 0x01)
			continue;
		lba = get_unaligned_le32(meta + 8);
		if (lba >= fmss_lba_map_cap())
			continue;
		weave = fmss_ppn_weave48(meta);
		before = lba_map[lba];
		fmss_lba_set(lba, ce, cau, block, page, slot0 + s, true,
			     L2V_SRC_META, weave);
		if (lba_map[lba] != before)
			l2v_meta_hits++;
	}
}

/* Compat: physical BTOC-style set (legacy callers). */
static void __maybe_unused fmss_l2v_set(unsigned int lpn, unsigned int ce,
					unsigned int cau, unsigned int block,
					unsigned int page)
{
	fmss_l2v_set_ex(lpn, ce, cau, block, page, true, L2V_SRC_BTOC, 0);
}

/* Thin wrapper — BTOC/carve physical fills. */
static void __maybe_unused fmss_early_lba_set(unsigned int lba, unsigned int ce,
					      unsigned int cau,
					      unsigned int block,
					      unsigned int page,
					      unsigned int sec)
{
	fmss_lba_set(lba, ce, cau, block, page, sec, true, L2V_SRC_BTOC, 0);
}

static int __maybe_unused fmss_early_lba_lookup(unsigned int lba,
						unsigned int *ce,
						unsigned int *cau,
						unsigned int *block,
						unsigned int *page,
						unsigned int *sec)
{
	return fmss_lba_lookup(lba, ce, cau, block, page, sec, NULL);
}

static int __maybe_unused fmss_early_lba_ensure(void)
{
	return fmss_lba_map_ensure();
}

/*
 * Plausible YaFTL/Whimory BTOC: first entries are small LPNs and usually
 * sequential (we see 11,12,13 on blk 64 or 0,1,... on the boot superblock).
 * Used by on-demand lpn_read scan; l2v_build uses a looser ingest.
 */
static bool fmss_btoc_plausible(const u8 *btoc_page, unsigned int target_lpn,
				unsigned int *opage)
{
	unsigned int p, lpn0, lpn1, lpn2, hits = 0;

	lpn0 = fmss_btoc_entry(btoc_page, 0);
	lpn1 = fmss_btoc_entry(btoc_page, 1);
	lpn2 = fmss_btoc_entry(btoc_page, 2);
	if (target_lpn == 0) {
		/* Boot superblock: BTOC[0]==0 && BTOC[1]==1 (not a stray 0 elsewhere). */
		if (lpn0 == 0 && lpn1 == 1) {
			*opage = 0;
			return true;
		}
		return false;
	}
	if (lpn0 > 0x1000000 || lpn1 > 0x1000000)
		return false;
	for (p = 0; p < FMSS_BTOC_PAGE; p++) {
		if (fmss_btoc_entry(btoc_page, p) == target_lpn) {
			hits++;
			*opage = p;
		}
	}
	if (!hits)
		return false;
	/* Prefer superblocks whose first slots look like FTL metadata. */
	if (lpn0 <= target_lpn && lpn1 == lpn0 + 1)
		return true;
	if (lpn0 == 11 && lpn1 == 12 && lpn2 == 13)
		return true;
	return hits == 1;
}

/* Strict BTOC: need a dense BE (or LE) run — loose >=2 poisoned L2V. */
static bool fmss_btoc_ingestible(const u8 *btoc)
{
	unsigned int i, good_be = 0, good_le = 0, seq_be = 0, seq_le = 0;
	u32 prev_be = 0xffffffff, prev_le = 0xffffffff;

	if (fmss_page_blankish(btoc, 64))
		return false;
	for (i = 0; i < 32; i++) {
		u32 be = fmss_btoc_entry_be(btoc, i);
		u32 le = fmss_btoc_entry_le(btoc, i);

		if (be != 0xffffffff && be < 0x01000000u) {
			good_be++;
			if (prev_be != 0xffffffff && be == prev_be + 1)
				seq_be++;
			prev_be = be;
		} else {
			prev_be = 0xffffffff;
		}
		if (le != 0xffffffff && le < 0x01000000u) {
			good_le++;
			if (prev_le != 0xffffffff && le == prev_le + 1)
				seq_le++;
			prev_le = le;
		} else {
			prev_le = 0xffffffff;
		}
	}
	/* Prefer sequential tables (live blk64: 11,12,13…). */
	if (good_be >= 8 && seq_be >= 4)
		return true;
	if (good_le >= 8 && seq_le >= 4 && good_le > good_be)
		return true;
	/* Boot SB: (0,1) or (0,vbas_per_page) */
	if (fmss_btoc_entry_be(btoc, 0) == 0 &&
	    (fmss_btoc_entry_be(btoc, 1) == 1 ||
	     fmss_btoc_entry_be(btoc, 1) == FMSS_VBAS_PER_PAGE))
		return true;
	if (fmss_btoc_entry_le(btoc, 0) == 0 &&
	    (fmss_btoc_entry_le(btoc, 1) == 1 ||
	     fmss_btoc_entry_le(btoc, 1) == FMSS_VBAS_PER_PAGE))
		return true;
	return false;
}

static int fmss_boot_carve_try(struct nand_s5l8740 *f, unsigned int ce,
			       unsigned int cau, unsigned int block,
			       unsigned int page);

static void fmss_l2v_ingest_btoc(struct nand_s5l8740 *f, unsigned int ce,
				 unsigned int cau, unsigned int block,
				 const u8 *btoc, unsigned int max_lpn)
{
	unsigned int p;
	bool use_le;

	if (!fmss_btoc_ingestible(btoc))
		return;
	/* N31 live BTOCs are BE; only use LE when clearly better. */
	use_le = fmss_btoc_prefer_le(btoc);
	l2v_btoc_hits++;
	for (p = 0; p < FMSS_BTOC_PAGE; p++) {
		u32 lpn = use_le ? fmss_btoc_entry_le(btoc, p)
				 : fmss_btoc_entry_be(btoc, p);

		if (lpn == 0xffffffff || lpn > max_lpn)
			continue;
		if (lpn == 0) {
			/*
			 * Avoid fmss_boot_carve_try here — nested full-page
			 * reads during the BTOC walk wedge FMSS. Discover
			 * handles BTOC[0]==0 after the walk.
			 */
			continue;
		}
		fmss_l2v_set_ex(lpn, ce, cau, block, p, true, L2V_SRC_BTOC, 0);
		/* Fill LBA map for all 4 sectors of this page LPN. */
		{
			unsigned int s;

			for (s = 0; s < FMSS_VBAS_PER_PAGE; s++)
				fmss_lba_set(lpn * FMSS_VBAS_PER_PAGE + s,
					     ce, cau, block, p, s, true,
					     L2V_SRC_BTOC, 0);
		}
	}
}

/*
 * SFTL on-flash BTOC (meta type 28): 16-byte BE records
 * +0 weaveSeqAdd, +4 aux, +8 lba, +12 … +15 span in low byte (live:
 * 00 00 00 00 | a7 00 00 1d | 00 00 00 79 | 05 00 00 02 → lba=121 span=2).
 * Used when YaFTL u32 LPN table is not ingestible.
 */
static bool fmss_page_looks_bte(const u8 *page)
{
	u32 weave0, lba0, span0, lba1, span1;

	if (fmss_page_blankish(page, 64))
		return false;
	weave0 = get_unaligned_be32(page);
	lba0 = get_unaligned_be32(page + 8);
	span0 = page[15];
	if (weave0 != 0 || !span0 || span0 > 128 || lba0 >= 0x01000000u)
		return false;
	lba1 = get_unaligned_be32(page + 16 + 8);
	span1 = page[16 + 15];
	if (!span1 || span1 > 128 || lba1 >= 0x01000000u)
		return false;
	/* Prefer abutting/near spans (live 121+2 → 123). */
	if (lba1 != lba0 + span0 && lba1 + span1 != lba0 &&
	    (lba1 < lba0 || lba1 > lba0 + span0 + 8))
		return false;
	return true;
}

static void fmss_l2v_ingest_bte(struct nand_s5l8740 *f, unsigned int ce,
				unsigned int cau, unsigned int block,
				const u8 *page, unsigned int max_lpn)
{
	unsigned int i, recs, vba_ofs = 0, hit = 0;
	unsigned int usable = 1024;

	(void)max_lpn;
	if (!fmss_page_looks_bte(page))
		return;
	if (fmss_lba_map_ensure())
		return;
	/* Prefer full page when available (pass 2: fill full lba_map). */
	if (f && f->last_page_len)
		usable = f->last_page_len;
	if (usable > FMSS_PAGE_LEN)
		usable = FMSS_PAGE_LEN;
	recs = usable / 16;
	for (i = 0; i < recs; i++) {
		const u8 *r = page + i * 16;
		u32 lba = get_unaligned_be32(r + 8);
		u32 span = r[15];
		u64 weave = ((u64)get_unaligned_be32(r) << 16) |
			    (get_unaligned_be16(r + 4) & 0xffffu);
		unsigned int j;

		if (!span || span > 128)
			break;
		if (lba >= 0x01000000u)
			break;
		hit++;
		for (j = 0; j < span; j++) {
			u32 cur = lba + j;
			unsigned int pg = vba_ofs / FMSS_VBAS_PER_PAGE;
			unsigned int sec = vba_ofs % FMSS_VBAS_PER_PAGE;

			if (pg >= FMSS_BTOC_PAGE)
				goto done;
			if (cur < fmss_lba_map_cap())
				fmss_lba_set(cur, ce, cau, block, pg, sec,
					     true, L2V_SRC_BTE, weave);
			vba_ofs++;
		}
	}
done:
	if (hit)
		l2v_btoc_hits++;
}

/*
 * Classic Whimory block-map heuristic: dense array of u16 vblock ids.
 * Tries LE then BE. Returns entry count hint.
 */
static bool fmss_page_looks_block_map(const u8 *page, unsigned int len,
				      bool *be_out, unsigned int *nents)
{
	unsigned int max = min(len / 2u, 2048u);
	unsigned int i, good_le = 0, bad_le = 0, good_be = 0, bad_be = 0;

	if (max < 64)
		return false;
	for (i = 0; i < max; i++) {
		u16 le = get_unaligned_le16(page + i * 2);
		u16 be = get_unaligned_be16(page + i * 2);

		if (le && le != 0xffff) {
			if (le < FMSS_BLOCKS_PER_CAU)
				good_le++;
			else
				bad_le++;
		}
		if (be && be != 0xffff) {
			if (be < FMSS_BLOCKS_PER_CAU)
				good_be++;
			else
				bad_be++;
		}
	}
	if (good_le >= 32 && bad_le * 4 <= good_le && good_le >= good_be) {
		*be_out = false;
		*nents = max;
		return true;
	}
	if (good_be >= 32 && bad_be * 4 <= good_be) {
		*be_out = true;
		*nents = max;
		return true;
	}
	return false;
}

static unsigned int fmss_vfl_phys(unsigned int cau, unsigned int virt);
static unsigned int fmss_vfl_resolve(unsigned int cau, unsigned int virt);
static unsigned int fmss_map_to_phys(unsigned int cau, unsigned int block,
				    u32 packed);
static int fmss_vfl_ingest(struct nand_s5l8740 *f, unsigned int cau,
			   unsigned int block, const u8 *hdr);
static int fmss_read_lpn_page(struct nand_s5l8740 *f, unsigned int ce,
			      unsigned int cau, unsigned int block,
			      unsigned int page, u8 *dst, unsigned int dst_len);

/*
 * Classic VFL ftlctrlblocks[3] are LE u16 at +4/+6/+8 (after usn).
 * N31 wrmx headers differ; accept the triple only when all look like
 * in-range block ids. Also probe a few other offsets in the first 512B.
 */
static bool fmss_wmr_try_ftlctrl_at(const u8 *hdr, unsigned int off,
				    u16 *out3)
{
	u16 a, b, c;
	unsigned int usable = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;

	if (off + 6 > FMSS_VFL_HDR_LEN)
		return false;
	a = get_unaligned_le16(hdr + off);
	b = get_unaligned_le16(hdr + off + 2);
	c = get_unaligned_le16(hdr + off + 4);
	if (!a || a >= usable || b >= usable || c >= usable)
		return false;
	/* Prefer distinct-ish ctrl blocks (allow one duplicate). */
	if (a == b && b == c)
		return false;
	out3[0] = a;
	out3[1] = b;
	out3[2] = c;
	return true;
}

static bool fmss_wmr_extract_ftlctrl(const u8 *hdr, u16 *out3)
{
	static const unsigned int offs[] = {
		offsetof(struct wmr_vfl_cxt, ftlctrlblocks), /* 4 */
		0x08, 0x0c, 0x10, 0x14, 0x18, 0x20, 0x28, 0x30,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(offs); i++) {
		if (fmss_wmr_try_ftlctrl_at(hdr, offs[i], out3))
			return true;
	}
	return false;
}

static int fmss_wmr_ensure_map(void)
{
	if (wmr_block_map)
		return 0;
	wmr_block_map = vmalloc(array_size(WMR_BLOCK_MAP_MAX, sizeof(u16)));
	if (!wmr_block_map)
		return -ENOMEM;
	memset(wmr_block_map, 0xff, WMR_BLOCK_MAP_MAX * sizeof(u16));
	wmr_block_map_n = 0;
	return 0;
}

static unsigned int fmss_wmr_load_map_page(const u8 *page, unsigned int len,
					  bool be, unsigned int nents,
					  unsigned int dst_off)
{
	unsigned int i, take;

	if (fmss_wmr_ensure_map())
		return 0;
	if (dst_off >= WMR_BLOCK_MAP_MAX)
		return 0;
	take = min(nents, WMR_BLOCK_MAP_MAX - dst_off);
	take = min(take, len / 2u);
	for (i = 0; i < take; i++) {
		u16 v = be ? get_unaligned_be16(page + i * 2)
			   : get_unaligned_le16(page + i * 2);

		wmr_block_map[dst_off + i] = v;
	}
	if (dst_off + take > wmr_block_map_n)
		wmr_block_map_n = dst_off + take;
	return take;
}

/* Decode classic vPage → CE/CAU/block/page for N31 PPN. */
static bool fmss_wmr_vpage_to_phys(u32 vpage, unsigned int *ce,
				   unsigned int *cau, unsigned int *block,
				   unsigned int *page)
{
	unsigned int pg = vpage % WMR_PAGES_PER_BLOCK;
	unsigned int vbn = vpage / WMR_PAGES_PER_BLOCK;
	unsigned int usable = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;
	unsigned int phys;

	*page = pg;

	/* Prefer CE0/CAU0 with identity VFL remap. */
	if (vbn < usable) {
		phys = fmss_vfl_phys(0, vbn);
		if (phys < FMSS_BLOCKS_PER_CAU) {
			*ce = 0;
			*cau = 0;
			*block = phys;
			return true;
		}
	}

	/* Pack vblock across CAUs (then CEs) when identity is out of range. */
	if (!usable)
		return false;
	{
		unsigned int blk = vbn % usable;
		unsigned int cau_i = (vbn / usable) % FMSS_NUM_CAU;
		unsigned int ce_i = (vbn / (usable * FMSS_NUM_CAU)) %
				    FMSS_NUM_CE;

		phys = fmss_vfl_phys(cau_i, blk);
		*ce = ce_i;
		*cau = cau_i;
		*block = phys;
		return true;
	}
}

static void fmss_wmr_maybe_reset(struct nand_s5l8740 *f)
{
	if (reset_every && f->pages_since_reset >= reset_every) {
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}
}

/* Bounded DEVICEINFOSIGN hunt: early blocks + VFL tail, page 0 only. */
static void fmss_wmr_scan_deviceinfo(struct nand_s5l8740 *f, unsigned int nblocks)
{
	unsigned int ce, cau, b, saved;
	unsigned int start_tail;
	u32 addr;
	int ret;

	wmr_dis_hits = 0;
	if (!nblocks || nblocks > WMR_MOUNT_MAX_BLOCKS)
		nblocks = min(grep_max_blocks, WMR_MOUNT_MAX_BLOCKS);
	if (!nblocks)
		nblocks = 16;
	start_tail = FMSS_BLOCKS_PER_CAU - nblocks;

	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (b = 0; b < nblocks; b++) {
				fmss_wmr_maybe_reset(f);
				addr = fmss_ppn_addr(cau, b, 0, 0);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (!ret &&
				    fmss_find_enc(f->last_page, f->last_page_len,
						  WMR_DEVICEINFOSIGN,
						  strlen(WMR_DEVICEINFOSIGN),
						  NULL)) {
					wmr_dis_hits++;
					if (wmr_dis_hits == 1) {
						wmr_dis_ce = ce;
						wmr_dis_cau = cau;
						wmr_dis_block = b;
						wmr_dis_page = 0;
					}
				}
			}
			for (b = start_tail; b < FMSS_BLOCKS_PER_CAU; b++) {
				fmss_wmr_maybe_reset(f);
				addr = fmss_ppn_addr(cau, b, 0, 1);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (!ret &&
				    fmss_find_enc(f->last_page, f->last_page_len,
						  WMR_DEVICEINFOSIGN,
						  strlen(WMR_DEVICEINFOSIGN),
						  NULL)) {
					wmr_dis_hits++;
					if (wmr_dis_hits == 1) {
						wmr_dis_ce = ce;
						wmr_dis_cau = cau;
						wmr_dis_block = b;
						wmr_dis_page = 0;
					}
				}
			}
		}
	}
	page_chunks = saved;
}

/* Tail VFL wrmx/xrmw ingest + optional classic ftlctrlblocks. */
static void fmss_wmr_scan_vfl(struct nand_s5l8740 *f, unsigned int nblocks)
{
	unsigned int ce, cau, i, saved, start;
	u16 ctrl[3];
	int ret;

	wmr_vfl_hits = 0;
	wmr_ftlctrl_hits = 0;
	wmr_ftlctrl_n = 0;
	vfl_map_count = 0;
	if (!nblocks || nblocks > WMR_MOUNT_MAX_BLOCKS)
		nblocks = min(vfl_build_blocks ? vfl_build_blocks : 32u,
			      WMR_MOUNT_MAX_BLOCKS);
	if (nblocks > FMSS_VFL_TAIL)
		nblocks = FMSS_VFL_TAIL;
	start = FMSS_BLOCKS_PER_CAU - nblocks;

	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (i = 0; i < nblocks; i++) {
				unsigned int blk = start + i;
				u32 addr;

				fmss_wmr_maybe_reset(f);
				addr = fmss_ppn_addr(cau, blk, 0, 1);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (ret || fmss_page_blankish(f->last_page, 512))
					continue;
				if (fmss_vfl_ingest(f, cau, blk, f->last_page))
					wmr_vfl_hits++;
				if (fmss_wmr_extract_ftlctrl(f->last_page,
							     ctrl)) {
					wmr_ftlctrl_hits++;
					if (!wmr_ftlctrl_n) {
						wmr_ftlctrl[0] = ctrl[0];
						wmr_ftlctrl[1] = ctrl[1];
						wmr_ftlctrl[2] = ctrl[2];
						wmr_ftlctrl_n = 3;
					}
				}
			}
		}
	}
	page_chunks = saved;
}

static void fmss_wmr_try_load_bmap(struct nand_s5l8740 *f, unsigned int ce,
				   unsigned int cau, unsigned int block,
				   unsigned int *dst_off)
{
	bool be = false;
	unsigned int nents = 0, take;

	if (fmss_read_lpn_page(f, ce, cau, block, 0, NULL, 0))
		return;
	if (!fmss_page_looks_block_map(f->last_page, f->last_page_len,
				       &be, &nents))
		return;
	take = fmss_wmr_load_map_page(f->last_page, f->last_page_len, be,
				      nents, *dst_off);
	if (take) {
		wmr_bmap_pages++;
		*dst_off += take;
	}
}

/* Probe ftlctrl blocks + bounded early/tail for type-0x44-like maps. */
static void fmss_wmr_scan_block_maps(struct nand_s5l8740 *f, unsigned int nblocks)
{
	unsigned int ce, cau, b, i, saved, dst = 0;
	unsigned int start_tail;

	wmr_bmap_pages = 0;
	fmss_wmr_map_free();
	if (!nblocks || nblocks > WMR_MOUNT_MAX_BLOCKS)
		nblocks = min(grep_max_blocks, WMR_MOUNT_MAX_BLOCKS);
	if (!nblocks)
		nblocks = 16;
	start_tail = FMSS_BLOCKS_PER_CAU - nblocks;

	saved = page_chunks;
	page_chunks = 16;

	for (i = 0; i < wmr_ftlctrl_n && dst < WMR_BLOCK_MAP_MAX; i++) {
		u16 vb = wmr_ftlctrl[i];

		if (!vb || vb >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
			continue;
		for (ce = 0; ce < FMSS_NUM_CE && dst < WMR_BLOCK_MAP_MAX; ce++) {
			for (cau = 0; cau < FMSS_NUM_CAU &&
			     dst < WMR_BLOCK_MAP_MAX; cau++) {
				unsigned int phys = fmss_vfl_phys(cau, vb);

				fmss_wmr_try_load_bmap(f, ce, cau, phys, &dst);
			}
		}
	}

	for (ce = 0; ce < FMSS_NUM_CE && dst < WMR_BLOCK_MAP_MAX; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU &&
		     dst < WMR_BLOCK_MAP_MAX; cau++) {
			unsigned int probes = 0;

			for (b = 0; b < nblocks && probes < 24 &&
			     dst < WMR_BLOCK_MAP_MAX; b++) {
				fmss_wmr_try_load_bmap(f, ce, cau, b, &dst);
				probes++;
			}
			for (b = start_tail; b < FMSS_BLOCKS_PER_CAU &&
			     probes < 40 && dst < WMR_BLOCK_MAP_MAX; b++) {
				fmss_wmr_try_load_bmap(f, ce, cau, b, &dst);
				probes++;
			}
		}
	}
	page_chunks = saved;
}

static unsigned int fmss_wmr_fill_l2v(unsigned int max_lpn)
{
	unsigned int lpn, filled = 0;

	if (!wmr_block_map || !wmr_block_map_n)
		return 0;
	if (fmss_l2v_ensure(max_lpn))
		return 0;

	for (lpn = 0; lpn <= max_lpn; lpn++) {
		u32 vpage;
		unsigned int ce, cau, block, page;

		vpage = wmr_lpage_to_vpage(lpn, WMR_PAGES_PER_BLOCK,
					   wmr_block_map, wmr_block_map_n);
		if (vpage == ~0u)
			continue;
		if (!fmss_wmr_vpage_to_phys(vpage, &ce, &cau, &block, &page))
			continue;
		/* Avoid clobber denser BTOC hits already present. */
		if (l2v_map && lpn < l2v_map_size &&
		    (l2v_map[lpn] & L2V_VALID))
			continue;
		/* Classic WMR vpage already resolved to physical block. */
		fmss_l2v_set_ex(lpn, ce, cau, block, page, true,
				L2V_SRC_WMR, 0);
		filled++;
	}
	return filled;
}

/*
 * Classic freemyipod Whimory mount adapted for N31 PPN.
 * Does not clear existing l2v_build / boot_carve results.
 * Usage: echo 1 > whimory_mount
 * echo "NBLOCKS [MAX_LPN]" > whimory_mount
 */
static int fmss_whimory_mount(struct nand_s5l8740 *f, unsigned int nblocks,
			      unsigned int max_lpn)
{
	if (!max_lpn)
		max_lpn = FMSS_L2V_DEFAULT_MAX_LPN;
	if (!nblocks || nblocks > WMR_MOUNT_MAX_BLOCKS)
		nblocks = min(grep_max_blocks ? grep_max_blocks : 32u,
			      WMR_MOUNT_MAX_BLOCKS);

	wmr_l2v_filled = 0;
	wmr_mount_ret = 0;

	fmss_wmr_scan_deviceinfo(f, nblocks);
	fmss_wmr_scan_vfl(f, min(nblocks, FMSS_VFL_TAIL));
	fmss_wmr_scan_block_maps(f, nblocks);
	wmr_l2v_filled = fmss_wmr_fill_l2v(max_lpn);

	if (!wmr_block_map_n && !wmr_l2v_filled)
		wmr_mount_ret = -ENOENT;
	else
		wmr_mount_ret = 0;

	nand_dev_info(f->dev,
		      "whimory_mount n=%u max_lpn=%u dis=%u vfl=%u ftlctrl=%u bmap_pages=%u map_ents=%u filled=%u ret=%d\n",
		      nblocks, max_lpn, wmr_dis_hits, wmr_vfl_hits,
		      wmr_ftlctrl_hits, wmr_bmap_pages, wmr_block_map_n,
		      wmr_l2v_filled, wmr_mount_ret);
	return wmr_mount_ret;
}

static unsigned int fmss_vfl_phys(unsigned int cau, unsigned int virt)
{
	unsigned int i;

	for (i = 0; i < vfl_map_count; i++) {
		if (vfl_map[i].cau == cau && vfl_map[i].virt == virt)
			return vfl_map[i].phys;
	}
	return virt;
}

/*
 * Resolve virt→phys according to vfl_remap_mode.
 * Never call this for L2V_PHYS entries — use fmss_map_to_phys().
 */
static unsigned int fmss_vfl_resolve(unsigned int cau, unsigned int virt)
{
	unsigned int phys, i;

	if (!strncmp(vfl_remap_mode, "off", 3))
		return virt;

	phys = fmss_vfl_phys(cau, virt);
	if (phys == virt)
		return virt;

	if (!strncmp(vfl_remap_mode, "tail_only", 9)) {
		if (virt < FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
			return virt;
	} else if (strncmp(vfl_remap_mode, "direct256", 9)) {
		/* Unknown mode → treat as off. */
		return virt;
	}

	for (i = 0; i < vfl_map_count; i++) {
		if (vfl_map[i].cau == cau && vfl_map[i].virt == virt) {
			vfl_remap_applied++;
			if (!quiet && vfl_remap_applied <= 32)
				pr_info("s5l8740-nand: vfl_remap mode=%s cau=%u in=%u out=%u idx=%u\n",
					vfl_remap_mode, cau, virt, phys, i);
			return phys;
		}
	}
	return virt;
}

static unsigned int fmss_map_to_phys(unsigned int cau, unsigned int block,
				    u32 packed)
{
	if (packed & L2V_PHYS) {
		vfl_remap_skipped_phys++;
		return block;
	}
	return fmss_vfl_resolve(cau, block);
}

/*
 * wrmx/xrmw VFLCxt: 512-byte header, u32 remap table begins @ +0x100.
 * Live pod: entries are LE phys block numbers (e.g. 0x827 = 2087).
 */
static int fmss_vfl_ingest(struct nand_s5l8740 *f, unsigned int cau,
			   unsigned int block, const u8 *hdr)
{
	unsigned int i, virt, phys, added = 0;
	const u8 *tab = hdr + 0x100;
	const char *magic = "????";

	if (hdr[0] == 'w' && hdr[1] == 'r' && hdr[2] == 'm' && hdr[3] == 'x')
		magic = "wrmx";
	else if (hdr[0] == 'x' && hdr[1] == 'r' && hdr[2] == 'm' &&
		 hdr[3] == 'w')
		magic = "xrmw";
	else
		return 0;

	if (cau < FMSS_NUM_CAU) {
		vfl_ctx_cau[cau] = cau;
		vfl_ctx_block[cau] = block;
	}

	for (i = 0; i < 256; i++) {
		memcpy(&phys, tab + i * 4, 4);
		phys = le32_to_cpu(phys);
		if (!phys || phys >= FMSS_BLOCKS_PER_CAU)
			continue;
		virt = i;
		if (vfl_map_count < FMSS_VFL_MAP_MAX) {
			vfl_map[vfl_map_count].cau = cau;
			vfl_map[vfl_map_count].virt = virt;
			vfl_map[vfl_map_count].phys = phys;
			vfl_map_count++;
			added++;
		}
	}
	nand_dev_info(f->dev, "vfl_ingest cau=%u blk=%u magic=%s entries=%u total=%u\n",
		 cau, block, magic, added, vfl_map_count);
	return added;
}

static void fmss_vfl_format_log(struct nand_s5l8740 *f, unsigned int ce,
				unsigned int cau, unsigned int block, u32 addr)
{
	unsigned int i, off = 0;
	u32 t0, t1, t2, t3;
	const char *magic = "????";

	if (f->last_page[0] == 'w' && f->last_page[1] == 'r' &&
	    f->last_page[2] == 'm' && f->last_page[3] == 'x')
		magic = "wrmx";
	else if (f->last_page[0] == 'x' && f->last_page[1] == 'r' &&
		 f->last_page[2] == 'm' && f->last_page[3] == 'w')
		magic = "xrmw";

	memcpy(&t0, f->last_page + 256, 4);
	memcpy(&t1, f->last_page + 260, 4);
	memcpy(&t2, f->last_page + 264, 4);
	memcpy(&t3, f->last_page + 268, 4);
	/* Also dump first table dwords @ +0x100 (OSOS 4EB7E4). */
	if (off < PAGE_SIZE - 80) {
		u32 u0, u1, u2, u3;

		memcpy(&u0, f->last_page + 0x100, 4);
		memcpy(&u1, f->last_page + 0x104, 4);
		memcpy(&u2, f->last_page + 0x108, 4);
		memcpy(&u3, f->last_page + 0x10c, 4);
		off += scnprintf(vfl_log + off, PAGE_SIZE - off,
				 "table+0x100: %08x %08x %08x %08x\n",
				 le32_to_cpu(u0), le32_to_cpu(u1),
				 le32_to_cpu(u2), le32_to_cpu(u3));
	}

	off += scnprintf(vfl_log + off, PAGE_SIZE - off,
			 "ce=%u cau=%u blk=%u addr=0x%08x magic=%s\n",
			 ce, cau, block, addr, magic);
	off += scnprintf(vfl_log + off, PAGE_SIZE - off,
			 "table+256: %08x %08x %08x %08x\n",
			 le32_to_cpu(t0), le32_to_cpu(t1),
			 le32_to_cpu(t2), le32_to_cpu(t3));
	for (i = 0; i < FMSS_VFL_HDR_LEN && off < PAGE_SIZE - 4; i++) {
		off += scnprintf(vfl_log + off, PAGE_SIZE - off, "%02x%s",
				 f->last_page[i], ((i + 1) % 16) ? " " : "\n");
	}
	vfl_log_len = off;
}

static ssize_t vfl_log_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	if (!vfl_log_len)
		return sysfs_emit(buf, "(no vfl_dump yet)\n");
	memcpy(buf, vfl_log, min(vfl_log_len, PAGE_SIZE));
	return min(vfl_log_len, PAGE_SIZE);
}
static DEVICE_ATTR_RO(vfl_log);

static ssize_t sector_hex_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	if (!sector_log_len)
		return sysfs_emit(buf, "(no lpn_read yet)\n");
	memcpy(buf, sector_log, sector_log_len);
	return sector_log_len;
}
static DEVICE_ATTR_RO(sector_hex);

static ssize_t lpn_index_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	unsigned int i, n = 0;

	if (!lpn_index_count)
		return sysfs_emit(buf, "(empty — run lpn_read or lpn_build)\n");
	for (i = 0; i < lpn_index_count; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "lpn=%u ce=%u cau=%u blk=%u pg=%u\n",
			       lpn_index[i].lpn, lpn_index[i].ce,
			       lpn_index[i].cau, lpn_index[i].block,
			       lpn_index[i].page);
	return n;
}
static DEVICE_ATTR_RO(lpn_index);

/*
 * Read one VFL context page (SLC page 0) and capture 512-byte header in vfl_log.
 * Usage: echo "CE CAU BLOCK" > vfl_dump (CE/CAU default 0)
 */
static ssize_t vfl_dump_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce = 0, cau = 0, block;
	unsigned int saved;
	u32 addr;
	int ret, nf;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u %u", &ce, &cau, &block);
	if (nf == 1) {
		block = ce;
		ce = 0;
		cau = 0;
	} else if (nf != 3) {
		return -EINVAL;
	}
	if (ce >= FMSS_NUM_CE || cau >= FMSS_NUM_CAU ||
	    block >= FMSS_BLOCKS_PER_CAU)
		return -EINVAL;

	mutex_lock(&f->lock);
	saved = page_chunks;
	page_chunks = 1;
	if (reset_every && f->pages_since_reset >= reset_every) {
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}
	addr = fmss_ppn_addr(cau, block, 0, 1);
	ret = fmss_page_read(f, ce, addr);
	f->pages_since_reset++;
	page_chunks = saved;
	if (!ret)
		fmss_vfl_format_log(f, ce, cau, block, addr);
	if (!ret)
		fmss_vfl_ingest(f, cau, block, f->last_page);
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "vfl_dump ce=%u cau=%u blk=%u ret=%d\n", ce, cau, block, ret);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(vfl_dump);

static int fmss_find_lpn(struct nand_s5l8740 *f, unsigned int target_lpn,
			 unsigned int *oce, unsigned int *ocau,
			 unsigned int *oblock, unsigned int *opage);

static int fmss_lpn_resolve(struct nand_s5l8740 *f, unsigned int target_lpn,
			    unsigned int *oce, unsigned int *ocau,
			    unsigned int *oblock, unsigned int *opage)
{
	unsigned int i;

	if (!fmss_l2v_lookup(target_lpn, oce, ocau, oblock, opage))
		return 0;

	for (i = 0; i < lpn_index_count; i++) {
		if (lpn_index[i].lpn != target_lpn)
			continue;
		*oce = lpn_index[i].ce;
		*ocau = lpn_index[i].cau;
		*oblock = lpn_index[i].block;
		*opage = lpn_index[i].page;
		return 0;
	}
	return fmss_find_lpn(f, target_lpn, oce, ocau, oblock, opage);
}

static int fmss_read_lpn_page(struct nand_s5l8740 *f, unsigned int ce,
			      unsigned int cau, unsigned int block,
			      unsigned int page, u8 *dst, unsigned int dst_len)
{
	unsigned int pblock, saved;
	u32 addr;
	int ret;

	/* Callers pass physical scan blocks (find_lpn / BTOC). Avoid VFL-remap. */
	pblock = fmss_map_to_phys(cau, block, L2V_PHYS);
	saved = page_chunks;
	page_chunks = 16;
	if (reset_every && f->pages_since_reset >= reset_every) {
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}
	addr = fmss_ppn_addr(cau, pblock, page, 0);
	if (f->dma_ok && use_dma) {
		ret = fmss_dma_page_read(f, ce, addr);
		if (ret)
			ret = fmss_page_read(f, ce, addr);
	} else {
		ret = fmss_page_read(f, ce, addr);
	}
	f->pages_since_reset++;
	page_chunks = saved;
	if (!ret && dst && dst_len) {
		unsigned int n = min(dst_len, f->last_page_len);

		memcpy(dst, f->last_page, n);
	}
	return ret;
}

static int fmss_find_lpn(struct nand_s5l8740 *f, unsigned int target_lpn,
			 unsigned int *oce, unsigned int *ocau,
			 unsigned int *oblock, unsigned int *opage)
{
	unsigned int ce, cau, b, p, saved, limit, best = ~0u;
	u32 addr;
	int ret, found = 0;

	limit = lpn_scan_blocks;
	if (!limit || limit > FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
		limit = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;

	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (b = 0; b < limit; b++) {
				if (reset_every && f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				addr = fmss_ppn_addr(cau, b, FMSS_BTOC_PAGE, 0);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (ret || fmss_page_blankish(f->last_page, 64))
					continue;
				for (p = 0; p < FMSS_BTOC_PAGE; p++) {
					unsigned int score, pg = p;
					unsigned int fat_off = 0;
					bool fat_ok = false;

					if (fmss_btoc_entry(f->last_page, p) != target_lpn)
						continue;
					if (!fmss_btoc_plausible(f->last_page, target_lpn, &pg))
						continue;
					if (fmss_read_lpn_page(f, ce, cau, b, pg, NULL, 0))
						continue;
					if (target_lpn == 0) {
						fat_ok = fmss_page_has_fat_boot(
							f->last_page, f->last_page_len,
							&fat_off);
						if (!fat_ok)
							continue;
					} else if (fmss_page_blankish(f->last_page, 64)) {
						continue;
					}
					/* Prefer ce0/cau0, lower blocks, LPN0 with FAT @ 0. */
					score = ce * 1000000u + cau * 100000u + b * 100u + pg;
					if (target_lpn == 0 && fat_off == 0)
						score /= 10;
					if (score < best) {
						best = score;
						*oce = ce;
						*ocau = cau;
						*oblock = b;
						*opage = pg;
						found = 1;
					}
				}
			}
		}
	}
	page_chunks = saved;
	return found ? 0 : -ENOENT;
}

static void fmss_boot_apply_bpb(struct nand_s5l8740 *f, unsigned int ce,
				unsigned int cau, unsigned int block,
				unsigned int page, const u8 *bpb)
{
	u16 bps = get_unaligned_le16(bpb + 11);
	u16 rsv = get_unaligned_le16(bpb + 14);
	u32 fatz = get_unaligned_le32(bpb + 36);

	boot_carve_valid = true;
	boot_carve_ce = ce;
	boot_carve_cau = cau;
	boot_carve_block = block;
	boot_carve_page = page;
	boot_carve_off = 0;
	boot_data_start = fmss_bpb_data_start(bpb);
	if (bps == FMSS_SECTOR_LEN && rsv && rsv < 4096)
		boot_reserved_sects = rsv;
	else
		boot_reserved_sects = 32;
	if (fatz && fatz < 0x100000u)
		boot_fat_sects = fatz;
	/* L2V[0] / LBA0 must point at this real boot page (physical). */
	fmss_l2v_set_ex(0, ce, cau, block, page, true, L2V_SRC_CARVE, 0);
	fmss_lba_set(0, ce, cau, block, page, 0, true, L2V_SRC_CARVE, 0);
	nand_dev_info(f->dev,
		      "boot_sb ce=%u cau=%u blk=%u pg=%u DataStart=%u rsv=%u fatz=%u\n",
		      ce, cau, block, page, boot_data_start,
		      boot_reserved_sects, boot_fat_sects);
}

/*
 * Accept ONLY an aligned live boot sector: BPB at page offset 0 with
 * 55AA@510. Mid-page *UOKJIHC (e.g. off=7816) is a file copy — reject.
 */
static int fmss_boot_carve_try(struct nand_s5l8740 *f, unsigned int ce,
			       unsigned int cau, unsigned int block,
			       unsigned int page)
{
	int ret;

	ret = fmss_read_lpn_page(f, ce, cau, block, page, NULL, 0);
	if (ret)
		return 0;
	if (f->last_page_len < FMSS_SECTOR_LEN)
		return 0;
	if (!fmss_apple_fat_boot(f->last_page))
		return 0;
	fmss_boot_apply_bpb(f, ce, cau, block, page, f->last_page);
	return 1;
}

/*
 * Try aligned BPB on page p; on success ingest the saved BTOC table.
 */
static int fmss_boot_try_btoc_page(struct nand_s5l8740 *f, unsigned int ce,
				   unsigned int cau, unsigned int block,
				   const u8 *btoc_page, unsigned int page)
{
	u8 *btoc_save;
	int ok;

	btoc_save = kmemdup(btoc_page, FMSS_PAGE_LEN, GFP_KERNEL);
	if (!btoc_save)
		return 0;
	ok = fmss_boot_carve_try(f, ce, cau, block, page);
	if (ok) {
		fmss_l2v_ingest_btoc(f, ce, cau, block, btoc_save,
				     l2v_max_lpn ? l2v_max_lpn :
						   FMSS_L2V_DEFAULT_MAX_LPN);
	}
	kfree(btoc_save);
	return ok;
}

/*
 * Find boot superblock: ingestible BTOC with LPN0 in any slot (or boot-ish
 * BTOC[0]==0 && BTOC[1] in {1,vbas_per_page}), then aligned BPB on that page.
 * Falls back to page-0 aligned BPB scan (never mid-page OEM).
 */
static int fmss_boot_carve_discover(struct nand_s5l8740 *f, unsigned int start,
				    unsigned int nblocks)
{
	unsigned int ce, cau, b, p, saved;
	u32 addr, l0, l1;
	int ret;
	bool use_le;

	/* Optional explicit cache — only if params describe an aligned page. */
	if (boot_carve_block_param && boot_carve_off_param == 0) {
		if (fmss_boot_carve_try(f, boot_carve_ce_param,
					boot_carve_cau_param,
					boot_carve_block_param,
					boot_carve_page_param ?
						boot_carve_page_param : 0))
			return 0;
		nand_dev_info(f->dev,
			      "cached boot_sb blk%u miss — scanning BTOC LPN0\n",
			      boot_carve_block_param);
	}

	if (!nblocks)
		nblocks = l2v_auto_blocks ? l2v_auto_blocks : 512;
	if (nblocks > FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
		nblocks = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;

	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (b = start; b < start + nblocks; b++) {
				if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
					break;
				if (reset_every &&
				    f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				addr = fmss_ppn_addr(cau, b, FMSS_BTOC_PAGE, 0);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (ret || fmss_page_blankish(f->last_page, 64))
					continue;
				use_le = fmss_btoc_prefer_le(f->last_page);
				l0 = use_le ? fmss_btoc_entry_le(f->last_page, 0)
					    : fmss_btoc_entry_be(f->last_page, 0);
				l1 = use_le ? fmss_btoc_entry_le(f->last_page, 1)
					    : fmss_btoc_entry_be(f->last_page, 1);
				/*
				 * LPN0 candidate: BTOC[0]==0 (even if [1] is junk —
				 * live ce1/cau1/blk63). Avoid scan every random
				 * zero dword in non-ingestible pages (wedges NAND).
				 */
				if (l0 == 0) {
					page_chunks = 16;
					if (fmss_boot_try_btoc_page(f, ce, cau, b,
								    f->last_page,
								    0)) {
						page_chunks = saved;
						return 0;
					}
					page_chunks = 1;
				}
				if (!fmss_btoc_ingestible(f->last_page))
					continue;
				for (p = 1; p < FMSS_BTOC_PAGE; p++) {
					u32 lpn = use_le ?
						fmss_btoc_entry_le(f->last_page, p) :
						fmss_btoc_entry_be(f->last_page, p);

					if (lpn != 0)
						continue;
					page_chunks = 16;
					if (fmss_boot_try_btoc_page(f, ce, cau, b,
								    f->last_page,
								    p)) {
						page_chunks = saved;
						return 0;
					}
					page_chunks = 1;
				}
			}
		}
	}

	/*
	 * Aligned BPB: page0 of each block, then all pages of open SBs
	 * (page0 programmed && page127 not closed BTOC/BTE). Never mid-page OEM.
	 */
	page_chunks = 16;
	{
		unsigned int boot_scan = nblocks ? nblocks : 256;

		if (boot_scan > 512)
			boot_scan = 512;
		for (ce = 0; ce < FMSS_NUM_CE; ce++) {
			for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
				for (b = start; b < start + boot_scan; b++) {
					if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
						break;
					if (fmss_boot_carve_try(f, ce, cau, b, 0)) {
						page_chunks = saved;
						return 0;
					}
				}
			}
		}

		page_chunks = 1;
		for (ce = 0; ce < FMSS_NUM_CE; ce++) {
			for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
				for (b = start; b < start + boot_scan; b++) {
					unsigned int pg;
					u32 addr;
					int r;
					bool closed;

					if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
						break;
					if (reset_every &&
					    f->pages_since_reset >= reset_every) {
						fmss_nand_reset(f);
						f->pages_since_reset = 0;
					}
					addr = fmss_ppn_addr(cau, b, 0, 0);
					r = fmss_page_read(f, ce, addr);
					f->pages_since_reset++;
					if (r || fmss_page_blankish(f->last_page, 64))
						continue;
					addr = fmss_ppn_addr(cau, b, FMSS_BTOC_PAGE, 0);
					r = fmss_page_read(f, ce, addr);
					f->pages_since_reset++;
					closed = !r && !fmss_page_blankish(f->last_page, 64) &&
						 (fmss_btoc_ingestible(f->last_page) ||
						  fmss_page_looks_bte(f->last_page));
					if (closed)
						continue;

					page_chunks = 16;
					for (pg = 0; pg < FMSS_BTOC_PAGE; pg++) {
						unsigned int sec;

						if (fmss_boot_carve_try(f, ce, cau, b, pg)) {
							page_chunks = saved;
							return 0;
						}
						if (fmss_read_lpn_page(f, ce, cau, b, pg,
									NULL, 0))
							continue;
						for (sec = 1; sec < FMSS_VBAS_PER_PAGE; sec++) {
							unsigned int off = sec * FMSS_SECTOR_LEN;
							const u8 *s = f->last_page + off;

							if (off + FMSS_SECTOR_LEN > f->last_page_len)
								break;
							if (!fmss_apple_fat_boot(s))
								continue;
							fmss_boot_apply_bpb(f, ce, cau, b, pg, s);
							fmss_lba_set(0, ce, cau, b, pg, sec,
								     true, L2V_SRC_CARVE, 0);
							page_chunks = saved;
							nand_dev_info(f->dev,
								      "boot_sb open-SB sec=%u ce=%u cau=%u blk=%u pg=%u\n",
								      sec, ce, cau, b, pg);
							return 0;
						}
					}
					page_chunks = 1;
				}
			}
		}
	}
	page_chunks = saved;
	return -ENOENT;
}

static bool fmss_page_has_n31os_dirent(const u8 *page, unsigned int len)
{
	unsigned int off, flags;
	static const char *const needles[] = {
		"N31OS", "n31os", "README", "IPOD_CON", "iPod_C", NULL
	};
	unsigned int n;

	/* FAT 8.3 short names (ASCII, space-padded) — LE cluster fields elsewhere. */
	for (off = 0; off + 32 <= len; off += 32) {
		if (!memcmp(page + off, "N31OS   ", 8) ||
		    !memcmp(page + off, "README  ", 8) ||
		    !memcmp(page + off, "IPOD_CON", 8))
			return true;
	}
	for (n = 0; needles[n]; n++) {
		flags = fmss_find_enc(page, len, needles[n],
				      strlen(needles[n]), &off);
		if (flags)
			return true;
	}
	return false;
}

static int fmss_root_dir_discover(struct nand_s5l8740 *f, unsigned int start,
				  unsigned int nblocks)
{
	unsigned int ce, cau, b, p, lpn;
	unsigned int saved, pages = 0;
	const unsigned int page_cap = 4096;

	lpn = boot_data_start / NAND_FTL_SECTORS_PER_LPN;
	if (!fmss_l2v_lookup(lpn, &ce, &cau, &b, &p)) {
		root_dir_valid = true;
		root_dir_ce = ce;
		root_dir_cau = cau;
		root_dir_block = b;
		root_dir_page = p;
		root_dir_lpn = lpn;
		return 0;
	}

	if (!nblocks)
		nblocks = 24;
	if (nblocks > 48)
		nblocks = 48;

	saved = page_chunks;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (b = start; b < start + nblocks; b++) {
				if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
					break;
				for (p = 0; p < FMSS_BTOC_PAGE; p++) {
					if (pages >= page_cap)
						goto out;
					if (fmss_read_lpn_page(f, ce, cau, b, p,
							       NULL, 0))
						continue;
					pages++;
					if (!fmss_page_has_n31os_dirent(
						    f->last_page,
						    f->last_page_len))
						continue;
					root_dir_valid = true;
					root_dir_ce = ce;
					root_dir_cau = cau;
					root_dir_block = b;
					root_dir_page = p;
					root_dir_lpn = lpn;
					fmss_l2v_set_ex(lpn, ce, cau, b, p, true,
							L2V_SRC_CARVE, 0);
					page_chunks = saved;
					nand_dev_info(f->dev,
						      "root_dir N31OS ce=%u cau=%u blk=%u pg=%u lpn=%u\n",
						      ce, cau, b, p, lpn);
					return 0;
				}
			}
		}
	}
out:
	page_chunks = saved;
	return -ENOENT;
}

static void fmss_l2v_try_block_map_page(struct nand_s5l8740 *f, unsigned int ce,
					unsigned int cau, unsigned int block,
					unsigned int max_lpn)
{
	bool be = false;
	unsigned int nents = 0, i, saved, take;
	u32 addr;
	u8 *mapbuf;
	int ret;

	if (fmss_read_lpn_page(f, ce, cau, block, 0, NULL, 0))
		return;
	if (!fmss_page_looks_block_map(f->last_page, f->last_page_len,
				       &be, &nents))
		return;

	take = min(nents, 64u);
	mapbuf = kmalloc(take * 2, GFP_KERNEL);
	if (!mapbuf)
		return;
	memcpy(mapbuf, f->last_page, take * 2);
	l2v_bmap_hits++;

	saved = page_chunks;
	page_chunks = 1;
	for (i = 0; i < take; i++) {
		u16 vbn = be ? get_unaligned_be16(mapbuf + i * 2)
			     : get_unaligned_le16(mapbuf + i * 2);

		if (!vbn || vbn >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
			continue;
		if (reset_every && f->pages_since_reset >= reset_every) {
			fmss_nand_reset(f);
			f->pages_since_reset = 0;
		}
		{
			unsigned int pblk = fmss_vfl_resolve(cau, vbn);

			addr = fmss_ppn_addr(cau, pblk, FMSS_BTOC_PAGE, 0);
			ret = fmss_page_read(f, ce, addr);
			f->pages_since_reset++;
			if (ret || fmss_page_blankish(f->last_page, 64))
				continue;
			/* Store physical block + PHYS (already VFL-resolved). */
			fmss_l2v_ingest_btoc(f, ce, cau, pblk, f->last_page,
					     max_lpn);
		}
	}
	page_chunks = saved;
	kfree(mapbuf);
}

/*
 * Build dense L2V from BTOC page 127 (+ optional classic block-map pages).
 * Bounded: [start, start+nblocks) per CE/CAU. Also carve boot + root dir.
 */
static int fmss_l2v_build(struct nand_s5l8740 *f, unsigned int max_lpn,
			  unsigned int start, unsigned int nblocks)
{
	unsigned int ce, cau, b, saved;
	u32 addr;
	int ret;

	if (!max_lpn)
		max_lpn = FMSS_L2V_DEFAULT_MAX_LPN;
	if (!nblocks)
		nblocks = l2v_auto_blocks; /* 0 = carve-only, no BTOC walk */
	if (nblocks > FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
		nblocks = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;
	if (start >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
		start = 0;

	ret = fmss_l2v_ensure(max_lpn);
	if (ret)
		return ret;

	fmss_legacy_meta_ingest = true;

	/* Rebuild map contents for this pass (keep allocation). */
	memset(l2v_map, 0, l2v_map_size * sizeof(*l2v_map));
	if (l2v_weave)
		memset(l2v_weave, 0, l2v_map_size * sizeof(*l2v_weave));
	if (l2v_src)
		memset(l2v_src, 0, l2v_map_size * sizeof(*l2v_src));
	l2v_mapped = 0;
	l2v_btoc_hits = 0;
	l2v_bmap_hits = 0;
	l2v_meta_hits = 0;
	lpn_index_count = 0;
	boot_carve_valid = false;
	root_dir_valid = false;
	f->ecc_soft_fails = 0;
	if (lba_map) {
		memset(lba_map, 0, fmss_lba_map_cap() * sizeof(*lba_map));
		if (lba_weave)
			memset(lba_weave, 0, fmss_lba_map_cap() * sizeof(*lba_weave));
		if (lba_src)
			memset(lba_src, 0, fmss_lba_map_cap() * sizeof(*lba_src));
		lba_mapped = 0;
	} else {
		fmss_lba_map_ensure();
	}
	vfl_remap_applied = 0;
	vfl_remap_skipped_phys = 0;

	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; nblocks && ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			unsigned int bmap_probes = 0;

			for (b = start; b < start + nblocks; b++) {
				bool btoc_ok;

				if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
					break;
				if (reset_every &&
				    f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				addr = fmss_ppn_addr(cau, b, FMSS_BTOC_PAGE, 0);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				btoc_ok = !ret &&
					  !fmss_page_blankish(f->last_page, 64);
				if (btoc_ok) {
					if (fmss_btoc_ingestible(f->last_page))
						fmss_l2v_ingest_btoc(f, ce, cau, b,
								     f->last_page,
								     max_lpn);
					else if (fmss_page_looks_bte(f->last_page)) {
						/*
						 * Pass 2: BTE needs the full
						 * 16 KiB page; walk used 1-chunk
						 * probe — re-read full page.
						 */
						unsigned int saved2 = page_chunks;

						page_chunks = 16;
						ret = fmss_page_read(f, ce, addr);
						f->pages_since_reset++;
						page_chunks = saved2;
						if (!ret)
							fmss_l2v_ingest_bte(
								f, ce, cau, b,
								f->last_page,
								max_lpn);
					}
				}

				/*
				 * Classic block-map heuristic only when BTOC is
				 * blank — capped probes to avoid wedging.
				 */
				if (!btoc_ok && bmap_probes < 32) {
					page_chunks = 16;
					fmss_l2v_try_block_map_page(f, ce, cau,
								    b, max_lpn);
					page_chunks = 1;
					bmap_probes++;
				}
			}
		}
	}
	page_chunks = saved;

	fmss_boot_carve_discover(f, start, nblocks);
	if (nblocks)
		fmss_root_dir_discover(f, start ? start : 32,
				       min_t(unsigned int, nblocks, 48));

	fmss_legacy_meta_ingest = false;

	nand_dev_info(f->dev,
		      "l2v_build max_lpn=%u range=%u+%u mapped=%u btoc=%u bmap=%u boot=%d root=%d ecc_soft=%u\n",
		      max_lpn, start, nblocks, l2v_mapped, l2v_btoc_hits,
		      l2v_bmap_hits, boot_carve_valid, root_dir_valid,
		      f->ecc_soft_fails);
	return 0;
}

static int fmss_build_lpn_index(struct nand_s5l8740 *f, unsigned int max_lpn)
{
	unsigned int nblocks = l2v_scan_blocks;

	if (!nblocks)
		nblocks = lpn_scan_blocks;
	if (!nblocks)
		nblocks = FMSS_L2V_DEFAULT_BLOCKS;
	return fmss_l2v_build(f, max_lpn, 0, nblocks);
}

static ssize_t lpn_build_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int max_lpn = 32;
	int ret;

	if (!f)
		return -ENODEV;
	if (buf[0] && buf[0] != '\n' && kstrtouint(buf, 0, &max_lpn))
		return -EINVAL;
	mutex_lock(&f->lock);
	ret = fmss_build_lpn_index(f, max_lpn);
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(lpn_build);

/*
 * Explicit dense L2V build (bounded). Usage:
 * echo 1 > l2v_build
 * echo "NBLOCKS" > l2v_build
 * echo "START NBLOCKS [MAX_LPN]" > l2v_build
 */
static ssize_t l2v_build_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int start = 0, nblocks = 0, max_lpn = 0;
	int nf, ret;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u %u", &start, &nblocks, &max_lpn);
	if (nf == 1) {
		if (start == 1) {
			start = 0;
			nblocks = l2v_scan_blocks;
		} else {
			nblocks = start;
			start = 0;
		}
	} else if (nf == 2) {
		; /* start + nblocks */
	} else if (nf >= 3) {
		; /* start + nblocks + max_lpn */
	} else if (buf[0] && buf[0] != '\n') {
		return -EINVAL;
	}
	if (!nblocks)
		nblocks = l2v_scan_blocks ? l2v_scan_blocks
					  : FMSS_L2V_DEFAULT_BLOCKS;
	if (!max_lpn)
		max_lpn = FMSS_L2V_DEFAULT_MAX_LPN;

	mutex_lock(&f->lock);
	ret = fmss_l2v_build(f, max_lpn, start, nblocks);
	mutex_unlock(&f->lock);
	if (ret)
		return ret;
	dev_info(dev,
		 "l2v_build done mapped=%u btoc=%u bmap=%u boot=%u root=%u DataStart=%u\n",
		 l2v_mapped, l2v_btoc_hits, l2v_bmap_hits,
		 boot_carve_valid, root_dir_valid, boot_data_start);
	return count;
}
static DEVICE_ATTR_WO(l2v_build);

static ssize_t l2v_status_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return sysfs_emit(buf,
		"mapped=%u max_lpn=%u size=%u btoc_hits=%u bmap_hits=%u meta_hits=%u lba_mapped=%u\n"
		"boot_carve=%u ce=%u cau=%u blk=%u pg=%u off=%u DataStart=%u\n"
		"root_dir=%u ce=%u cau=%u blk=%u pg=%u lpn=%u\n"
		"lpn_index=%u vfl_mode=%s remap_applied=%u skipped_phys=%u\n"
		"whimory ret=%d dis=%u vfl=%u ftlctrl=%u bmap_pages=%u map_ents=%u filled=%u\n",
		l2v_mapped, l2v_max_lpn, l2v_map_size, l2v_btoc_hits,
		l2v_bmap_hits, l2v_meta_hits, lba_mapped,
		boot_carve_valid, boot_carve_ce, boot_carve_cau,
		boot_carve_block, boot_carve_page, boot_carve_off,
		boot_data_start,
		root_dir_valid, root_dir_ce, root_dir_cau, root_dir_block,
		root_dir_page, root_dir_lpn,
		lpn_index_count, vfl_remap_mode, vfl_remap_applied,
		vfl_remap_skipped_phys,
		wmr_mount_ret, wmr_dis_hits, wmr_vfl_hits, wmr_ftlctrl_hits,
		wmr_bmap_pages, wmr_block_map_n, wmr_l2v_filled);
}
static DEVICE_ATTR_RO(l2v_status);

static ssize_t whimory_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf,
		"ret=%d\n"
		"deviceinfosign_hits=%u first=ce%u/cau%u/blk%u/pg%u\n"
		"vfl_hits=%u ftlctrl_hits=%u ftlctrl=%u,%u,%u\n"
		"bmap_pages=%u map_ents=%u l2v_filled=%u\n",
		wmr_mount_ret,
		wmr_dis_hits, wmr_dis_ce, wmr_dis_cau, wmr_dis_block,
		wmr_dis_page,
		wmr_vfl_hits, wmr_ftlctrl_hits,
		wmr_ftlctrl_n > 0 ? wmr_ftlctrl[0] : 0,
		wmr_ftlctrl_n > 1 ? wmr_ftlctrl[1] : 0,
		wmr_ftlctrl_n > 2 ? wmr_ftlctrl[2] : 0,
		wmr_bmap_pages, wmr_block_map_n, wmr_l2v_filled);
}
static DEVICE_ATTR_RO(whimory_status);

/*
 * Classic Whimory mount (bounded). Usage:
 * echo 1 > whimory_mount
 * echo "NBLOCKS" > whimory_mount
 * echo "NBLOCKS MAX_LPN" > whimory_mount
 */
static ssize_t whimory_mount_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int nblocks = 0, max_lpn = 0;
	int nf, ret;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u", &nblocks, &max_lpn);
	if (nf == 1 && nblocks == 1)
		nblocks = 0; /* echo 1 → defaults */
	else if (nf < 1 && buf[0] && buf[0] != '\n')
		return -EINVAL;
	if (!nblocks)
		nblocks = min(grep_max_blocks ? grep_max_blocks : 32u,
			      WMR_MOUNT_MAX_BLOCKS);
	if (!max_lpn)
		max_lpn = FMSS_L2V_DEFAULT_MAX_LPN;

	mutex_lock(&f->lock);
	ret = fmss_whimory_mount(f, nblocks, max_lpn);
	mutex_unlock(&f->lock);
	dev_info(dev,
		 "whimory_mount done ret=%d map_ents=%u filled=%u (l2v mapped=%u)\n",
		 ret, wmr_block_map_n, wmr_l2v_filled, l2v_mapped);
	/* Mount soft-fails with -ENOENT when nothing found; still accept write. */
	if (ret && ret != -ENOENT)
		return ret;
	return count;
}
static DEVICE_ATTR_WO(whimory_mount);

/*
 * Resolve LPN → NAND page and copy one 4096-byte logical sector (must hold f->lock).
 * Uses dense L2V / root-dir cache only — no on-demand BTOC scan (avoids wedging
 * the device on sparse unmapped reads). Returns -ENOENT if unmapped.
 */
static int nand_ftl_read_lpn_locked(struct nand_s5l8740 *f, unsigned int target_lpn,
				    unsigned int sector, u8 *buf)
{
	unsigned int ce, cau, block, page, pblock, off, saved;
	u32 addr, packed = L2V_PHYS;
	int ret;

	if (sector > NAND_FTL_SECTORS_PER_LPN - 1)
		return -EINVAL;

	if (root_dir_valid && target_lpn == root_dir_lpn) {
		ce = root_dir_ce;
		cau = root_dir_cau;
		block = root_dir_block;
		page = root_dir_page;
		packed = L2V_PHYS;
	} else {
		ret = fmss_l2v_lookup_ex(target_lpn, &ce, &cau, &block, &page,
					 &packed);
		if (ret)
			return ret;
	}

	pblock = fmss_map_to_phys(cau, block, packed);
	saved = page_chunks;
	page_chunks = 16;
	if (reset_every && f->pages_since_reset >= reset_every) {
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}
	addr = fmss_ppn_addr(cau, pblock, page, 0);
	ret = fmss_page_read(f, ce, addr);
	f->pages_since_reset++;
	page_chunks = saved;
	if (ret)
		return ret;

	off = sector * FMSS_SECTOR_LEN;
	if (off + FMSS_SECTOR_LEN > f->last_page_len)
		return -ERANGE;

	memcpy(buf, f->last_page + off, FMSS_SECTOR_LEN);
	return 0;
}

/*
 * Read logical page N (BTOC LPN) and expose 4 KiB sector via sector_hex.
 * Usage: echo "LPN [sector_in_page]" > lpn_read (sector 0..3, default 0)
 */
static ssize_t lpn_read_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int target_lpn, sector = 0;
	unsigned int i, ce, cau, block, page, poff, saved;
	u8 *secbuf;
	u32 addr;
	int ret, nf;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u", &target_lpn, &sector);
	if (nf < 1)
		return -EINVAL;
	if (sector > 3)
		return -EINVAL;

	secbuf = kmalloc(FMSS_SECTOR_LEN, GFP_KERNEL);
	if (!secbuf)
		return -ENOMEM;

	mutex_lock(&f->lock);
	{
		u32 packed = L2V_PHYS;

		/* Prefer dense L2V packed flags; else on-demand resolve (PHYS). */
		ret = fmss_l2v_lookup_ex(target_lpn, &ce, &cau, &block, &page,
					 &packed);
		if (ret)
			ret = fmss_lpn_resolve(f, target_lpn, &ce, &cau,
					       &block, &page);
		if (!ret) {
			saved = page_chunks;
			page_chunks = 16;
			if (reset_every && f->pages_since_reset >= reset_every) {
				fmss_nand_reset(f);
				f->pages_since_reset = 0;
			}
			addr = fmss_ppn_addr(cau,
					     fmss_map_to_phys(cau, block, packed),
					     page, 0);
			ret = fmss_page_read(f, ce, addr);
			f->pages_since_reset++;
			page_chunks = saved;
			if (!ret) {
				poff = sector * FMSS_SECTOR_LEN;
				if (poff + FMSS_SECTOR_LEN <= f->last_page_len)
					memcpy(secbuf, f->last_page + poff,
					       FMSS_SECTOR_LEN);
				else
					ret = -ERANGE;
			}
		}
	}
	mutex_unlock(&f->lock);
	if (ret) {
		dev_warn(dev, "lpn_read %u: resolve/read failed (%d)\n",
			 target_lpn, ret);
		kfree(secbuf);
		return ret;
	}

	sector_log_len = 0;
	for (i = 0; i < 128; i++) {
		sector_log_len += scnprintf(sector_log + sector_log_len,
					    sizeof(sector_log) - sector_log_len,
					    "%02x%s",
					    secbuf[i],
					    ((i + 1) % 16) ? " " : "\n");
	}

	nand_dev_info(dev,
		 "lpn=%u sector=%u head=%02x%02x%02x%02x\n",
		 target_lpn, sector, secbuf[0], secbuf[1], secbuf[2], secbuf[3]);
	kfree(secbuf);
	return count;
}
static DEVICE_ATTR_WO(lpn_read);

static ssize_t resolve_log_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	if (!resolve_log_len)
		return sysfs_emit(buf, "(no resolve yet)\n");
	return sysfs_emit(buf, "%s", resolve_log);
}
static DEVICE_ATTR_RO(resolve_log);

static ssize_t read_sector_dense_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int lba;
	u8 *secbuf;
	int ret;

	if (!f)
		return -ENODEV;
	if (kstrtouint(buf, 0, &lba))
		return -EINVAL;
	secbuf = kmalloc(FMSS_SECTOR_LEN, GFP_KERNEL);
	if (!secbuf)
		return -ENOMEM;
	ret = nand_ftl_read_sector(lba, secbuf);
	dev_info(dev, "read_sector_dense LBA=%u ret=%d %s",
		 lba, ret, resolve_log);
	kfree(secbuf);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(read_sector_dense);

static ssize_t read_sector_slow_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int lba, lpn, sec, ce, cau, block, page, pblock, saved;
	u8 *secbuf;
	u32 addr, packed = L2V_PHYS;
	int ret;

	if (!f)
		return -ENODEV;
	if (kstrtouint(buf, 0, &lba))
		return -EINVAL;
	secbuf = kmalloc(FMSS_SECTOR_LEN, GFP_KERNEL);
	if (!secbuf)
		return -ENOMEM;

	/* Try dense first. */
	ret = nand_ftl_read_sector(lba, secbuf);
	if (!ret) {
		dev_info(dev, "read_sector_slow LBA=%u via dense OK\n", lba);
		kfree(secbuf);
		return count;
	}

	lpn = lba / NAND_FTL_SECTORS_PER_LPN;
	sec = lba % NAND_FTL_SECTORS_PER_LPN;
	mutex_lock(&f->lock);
	ret = fmss_l2v_lookup_ex(lpn, &ce, &cau, &block, &page, &packed);
	if (ret) {
		ret = fmss_lpn_resolve(f, lpn, &ce, &cau, &block, &page);
		packed = L2V_PHYS;
	}
	if (!ret) {
		pblock = fmss_map_to_phys(cau, block, packed);
		saved = page_chunks;
		page_chunks = 16;
		addr = fmss_ppn_addr(cau, pblock, page, 0);
		ret = fmss_page_read(f, ce, addr);
		page_chunks = saved;
		if (!ret) {
			memcpy(secbuf, f->last_page + sec * FMSS_SECTOR_LEN,
			       FMSS_SECTOR_LEN);
			resolve_log_len = scnprintf(
				resolve_log, sizeof(resolve_log),
				"slow LBA=%u via on-demand lpn_resolve phys=%d ce=%u cau=%u blk=%u→%u pg=%u sec=%u head=%02x%02x%02x%02x\n",
				lba, !!(packed & L2V_PHYS), ce, cau, block,
				pblock, page, sec,
				secbuf[0], secbuf[1], secbuf[2], secbuf[3]);
		}
	}
	mutex_unlock(&f->lock);
	dev_info(dev, "read_sector_slow LBA=%u ret=%d %s", lba, ret, resolve_log);
	kfree(secbuf);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(read_sector_slow);

static ssize_t read_sector_phys_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, block, page, sec = 0, saved;
	u8 *secbuf;
	u32 addr;
	int nf, ret;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u %u %u %u", &ce, &cau, &block, &page, &sec);
	if (nf < 4)
		return -EINVAL;
	if (sec > 3)
		return -EINVAL;
	secbuf = kmalloc(FMSS_SECTOR_LEN, GFP_KERNEL);
	if (!secbuf)
		return -ENOMEM;
	mutex_lock(&f->lock);
	saved = page_chunks;
	page_chunks = 16;
	addr = fmss_ppn_addr(cau, block, page, 0);
	ret = fmss_page_read(f, ce, addr);
	page_chunks = saved;
	if (!ret) {
		memcpy(secbuf, f->last_page + sec * FMSS_SECTOR_LEN,
		       FMSS_SECTOR_LEN);
		resolve_log_len = scnprintf(
			resolve_log, sizeof(resolve_log),
			"phys ce=%u cau=%u blk=%u pg=%u sec=%u head=%02x%02x%02x%02x\n",
			ce, cau, block, page, sec,
			secbuf[0], secbuf[1], secbuf[2], secbuf[3]);
		memcpy(f->last_page, secbuf, FMSS_SECTOR_LEN);
		sector_log_len = 0;
	}
	mutex_unlock(&f->lock);
	dev_info(dev, "read_sector_phys ret=%d %s", ret, resolve_log);
	kfree(secbuf);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(read_sector_phys);

static char grep_log[4096];
static unsigned int grep_log_len;

static ssize_t grep_log_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	if (!grep_log_len)
		return sysfs_emit(buf, "(no ftl_grep yet)\n");
	memcpy(buf, grep_log, min(grep_log_len, PAGE_SIZE));
	return min(grep_log_len, PAGE_SIZE);
}
static DEVICE_ATTR_RO(grep_log);

/*
 * Walk FTL superblocks and search page data for a short ASCII needle.
 * Usage: echo "START N_BLOCKS NEEDLE" > ftl_grep
 * echo "32 24 N31OS" > ftl_grep (defaults: start=32, n=24, N31OS)
 * Caps N_BLOCKS at FMSS_GREP_MAX_BLOCKS per call to avoid RetailOS watchdog.
 */
static ssize_t ftl_grep_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, b, p, saved, start = 32, nblocks = 16;
	unsigned int pages_done = 0, hits = 0;
	char needle[48] = "N31OS";
	size_t nlen = 5;
	int ret, nf;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u %47s", &start, &nblocks, needle);
	if (nf >= 3)
		nlen = strnlen(needle, sizeof(needle) - 1);
	else if (nf == 2)
		; /* start + nblocks, default needle */
	else if (nf == 1 && start == 1)
		start = 32;
	if (nblocks == 0)
		nblocks = 16;
	if (nblocks > grep_max_blocks)
		nblocks = grep_max_blocks;
	if (start >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
		start = 32;

	grep_log_len = 0;
	mutex_lock(&f->lock);
	saved = page_chunks;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (b = start; b < start + nblocks; b++) {
				if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
					break;
				if (reset_every && f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				/*
				 * Avoid require BTOC page 127 — Apple FAT clusters
				 * live in data pages even when BTOC looks blank.
				 * (Root cause: old code skipped whole superblocks.)
				 */
				for (p = 0; p < FMSS_BTOC_PAGE; p++) {
					unsigned int off = 0, show, flags;
					u32 lpn = ~0u;

					ret = fmss_read_lpn_page(f, ce, cau, b, p,
								 NULL, 0);
					pages_done++;
					if (ret)
						continue;
					flags = fmss_find_enc(f->last_page,
							      f->last_page_len,
							      needle, nlen,
							      &off);
					if (!flags)
						continue;
					hits++;
					nand_dev_info(dev,
						 "grep hit ce=%u cau=%u blk=%u pg=%u off=%u enc=%s\n",
						 ce, cau, b, p, off,
						 fmss_match_enc_name(flags));
					if (grep_log_len < sizeof(grep_log) - 160) {
						show = min(240u,
							   f->last_page_len - off);
						grep_log_len += scnprintf(
							grep_log + grep_log_len,
							sizeof(grep_log) - grep_log_len,
							"ce=%u cau=%u blk=%u pg=%u off=%u enc=%s\n",
							ce, cau, b, p, off,
							fmss_match_enc_name(flags));
						if (flags & FMSS_MATCH_ASCII)
							grep_log_len += scnprintf(
								grep_log + grep_log_len,
								sizeof(grep_log) - grep_log_len,
								"%.*s\n\n",
								show,
								f->last_page + off);
						else
							grep_log_len += scnprintf(
								grep_log + grep_log_len,
								sizeof(grep_log) - grep_log_len,
								"(wide/bswap match, %u bytes from off)\n\n",
								show);
					}
					(void)lpn;
				}
			}
		}
	}
	page_chunks = saved;
	mutex_unlock(&f->lock);
	if (!grep_log_len)
		grep_log_len = scnprintf(grep_log, sizeof(grep_log),
					 "NO HIT needle=%s pages=%u (tried ascii/utf16le/utf16be/bswap16)\n",
					 needle, pages_done);
	nand_dev_info(dev, "ftl_grep start=%u n=%u needle=%s pages=%u hits=%u\n",
		 start, nblocks, needle, pages_done, hits);
	return count;
}
static DEVICE_ATTR_WO(ftl_grep);

/*
 * Dump ASCII from a specific FTL page offset (after ftl_grep locates a file).
 * Usage: echo "CE CAU BLK PG OFF LEN" > ftl_ascii (LEN default 512, max 2048)
 */
static ssize_t readme_read_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, b, p, saved, start = 32, nblocks = 48;
	unsigned int pages_done = 0;
	const char *needle = "N31OS boot files on the RetailOS FAT volume";
	size_t nlen = 43;
	int ret, nf, found = 0;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u", &start, &nblocks);
	if (nf == 1 && start == 1)
		start = 32;
	if (nblocks == 0)
		nblocks = 48;
	if (nblocks > grep_max_blocks)
		nblocks = grep_max_blocks;

	grep_log_len = 0;
	mutex_lock(&f->lock);
	saved = page_chunks;
	/* CE0 first — matches disk-mode primary LUN behaviour. */
	for (ce = 0; ce < FMSS_NUM_CE && !found; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU && !found; cau++) {
			for (b = start; b < start + nblocks && !found; b++) {
				if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
					break;
				for (p = 0; p < FMSS_BTOC_PAGE && !found; p++) {
					unsigned int off, show, dump;

					if (reset_every &&
					    f->pages_since_reset >= reset_every) {
						fmss_nand_reset(f);
						f->pages_since_reset = 0;
					}
					ret = fmss_read_lpn_page(f, ce, cau, b, p,
								 NULL, 0);
					pages_done++;
					if (ret)
						continue;
					if (!fmss_find(f->last_page,
						       f->last_page_len,
						       needle, nlen))
						continue;
					off = 0;
					while (off + nlen <= f->last_page_len) {
						if (!memcmp(f->last_page + off,
							    needle, nlen))
							break;
						off++;
					}
					found = 1;
					dump = min(1024u, f->last_page_len - off);
					grep_log_len = scnprintf(
						grep_log, sizeof(grep_log),
						"OK ce=%u cau=%u blk=%u pg=%u off=%u len=%u pages=%u\n%.*s\n",
						ce, cau, b, p, off, dump, pages_done,
						dump, f->last_page + off);
					nand_dev_info(dev, "readme_read FOUND ce=%u cau=%u blk=%u pg=%u off=%u\n",
						   ce, cau, b, p, off);
				}
			}
		}
	}
	page_chunks = saved;
	mutex_unlock(&f->lock);
	if (!found) {
		dev_warn(dev, "readme_read: not found start=%u n=%u pages=%u (stage D:\\n31os via install-n31os-disk.ps1?)\n",
			 start, nblocks, pages_done);
		grep_log_len = scnprintf(grep_log, sizeof(grep_log),
					 "NOT FOUND (scanned %u pages from blk %u)\n",
					 pages_done, start);
		return -ENOENT;
	}
	return count;
}
static DEVICE_ATTR_WO(readme_read);

/*
 * Locate Apple FAT32 boot sector by scanning FTL pages for EB3C90 *UOKJIHC.
 * PIO only (fast). Usage: echo 1 > boot_read or echo "START N_BLOCKS" > boot_read
 * Result in grep_log + sector_hex (512 B boot sector).
 */
static int fmss_read_ftl_page_pio(struct nand_s5l8740 *f, unsigned int ce,
				  unsigned int cau, unsigned int block,
				  unsigned int page)
{
	unsigned int pblock, saved;
	u32 addr;
	int ret;

	/* Physical block from boot/carve scan — never VFL-remap. */
	pblock = fmss_map_to_phys(cau, block, L2V_PHYS);
	saved = page_chunks;
	page_chunks = 16;
	if (reset_every && f->pages_since_reset >= reset_every) {
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}
	addr = fmss_ppn_addr(cau, pblock, page, 0);
	ret = fmss_page_read(f, ce, addr);
	f->pages_since_reset++;
	page_chunks = saved;
	return ret;
}

static ssize_t boot_read_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, b, saved, start = 32, nblocks = 32;
	unsigned int fat_off = 0, sector_off = 0;
	unsigned int pages_done = 0;
	int ret, nf, found = 0;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %u", &start, &nblocks);
	if (nf == 1 && start == 1)
		start = 32;
	if (nblocks == 0)
		nblocks = 32;
	if (nblocks > grep_max_blocks)
		nblocks = grep_max_blocks;

	grep_log_len = 0;
	sector_log_len = 0;
	mutex_lock(&f->lock);
	saved = page_chunks;
	page_chunks = 16;

	for (ce = 0; ce < FMSS_NUM_CE && !found; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU && !found; cau++) {
			for (b = start; b < start + nblocks && !found; b++) {
				unsigned int p;

				if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
					break;
				for (p = 0; p < FMSS_BTOC_PAGE && !found; p++) {
					ret = fmss_read_ftl_page_pio(f, ce, cau, b, p);
					pages_done++;
					if (ret)
						continue;
					if (fmss_page_has_fat_boot(f->last_page,
								   f->last_page_len,
								   &fat_off)) {
						sector_off = p;
						found = 1;
					}
				}
			}
		}
	}

	page_chunks = saved;
	if (found) {
		unsigned int i, dump = min(512u, f->last_page_len - fat_off);

		for (i = 0; i < dump; i++) {
			sector_log_len += scnprintf(sector_log + sector_log_len,
						    sizeof(sector_log) - sector_log_len,
						    "%02x%s",
						    f->last_page[fat_off + i],
						    ((i + 1) % 16) ? " " : "\n");
		}
		grep_log_len = scnprintf(
			grep_log, sizeof(grep_log),
			"OK boot ce=%u cau=%u blk=%u pg=%u off=%u pages=%u\nOEM=%.8s vol=%.11s\n",
			ce, cau, b, sector_off, fat_off, pages_done,
			f->last_page + fat_off + 3,
			f->last_page + fat_off + 0x2b);
	} else {
		dev_warn(dev, "boot_read: no Apple FAT boot start=%u n=%u pages=%u\n",
			 start, nblocks, pages_done);
		grep_log_len = scnprintf(grep_log, sizeof(grep_log),
					 "NOT FOUND (scanned %u pages from blk %u)\n",
					 pages_done, start);
	}
	mutex_unlock(&f->lock);
	return found ? count : -ENOENT;
}
static DEVICE_ATTR_WO(boot_read);

static ssize_t ftl_ascii_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, block, page, off = 0, len = 512;
	int ret;

	if (!f)
		return -ENODEV;
	if (sscanf(buf, "%u %u %u %u %u %u", &ce, &cau, &block, &page, &off, &len) < 4)
		return -EINVAL;
	if (len > 2048)
		len = 2048;

	mutex_lock(&f->lock);
	ret = fmss_read_lpn_page(f, ce, cau, block, page, NULL, 0);
	if (ret) {
		mutex_unlock(&f->lock);
		return ret;
	}
	if (off >= f->last_page_len) {
		mutex_unlock(&f->lock);
		return -ERANGE;
	}
	if (off + len > f->last_page_len)
		len = f->last_page_len - off;

	grep_log_len = 0;
	grep_log_len += scnprintf(grep_log, sizeof(grep_log),
				  "ce=%u cau=%u blk=%u pg=%u off=%u len=%u\n",
				  ce, cau, block, page, off, len);
	grep_log_len += scnprintf(grep_log + grep_log_len,
				  sizeof(grep_log) - grep_log_len,
				  "%.*s\n",
				  len, f->last_page + off);
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "ftl_ascii ce=%u cau=%u blk=%u pg=%u off=%u len=%u\n",
		 ce, cau, block, page, off, len);
	return count;
}
static DEVICE_ATTR_WO(ftl_ascii);

/*
 * OSOS 4EB7E4 / Sogeti PPN-VFL: walk last blocks of each CAU, SLC page 0.
 * Without DMA meta we keep any non-blank data page and dump the 512-byte
 * VFLCxt header plus the u32 table at +256.
 */
static ssize_t vfl_scan_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, i, saved_chunks;
	unsigned int start = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;
	int hits = 0, xrmw = 0;
	u32 addr, t0, t1, t2, t3;
	int ret;

	if (!f)
		return -ENODEV;
	/* echo 1 just triggers. A value in [32, 2087] is the first CAU block. */
	if (buf[0] && buf[0] != '\n') {
		if (kstrtouint(buf, 0, &start))
			return -EINVAL;
		if (start < 32 || start >= FMSS_BLOCKS_PER_CAU)
			start = FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL;
	}

	mutex_lock(&f->lock);
	saved_chunks = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (i = FMSS_BLOCKS_PER_CAU - 1; i >= start; i--) {
				if (reset_every && f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				addr = fmss_ppn_addr(cau, i, 0, 1);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (ret)
					continue;
				if (fmss_page_blankish(f->last_page, 512))
					continue;
				hits++;
				memcpy(&t0, f->last_page + 256, 4);
				memcpy(&t1, f->last_page + 260, 4);
				memcpy(&t2, f->last_page + 264, 4);
				memcpy(&t3, f->last_page + 268, 4);
				if (f->last_page[0] == 'x' && f->last_page[1] == 'r' &&
				    f->last_page[2] == 'm' && f->last_page[3] == 'w')
					xrmw++;
				if (f->last_page[0] == 'w' && f->last_page[1] == 'r' &&
				    f->last_page[2] == 'm' && f->last_page[3] == 'x')
					xrmw++;
				fmss_vfl_ingest(f, cau, i, f->last_page);
				nand_dev_info(dev,
					 "vfl ce=%u cau=%u blk=%u addr=0x%08x %02x %02x %02x %02x %02x %02x %02x %02x +256 %08x %08x %08x %08x\n",
					 ce, cau, i, addr,
					 f->last_page[0], f->last_page[1],
					 f->last_page[2], f->last_page[3],
					 f->last_page[4], f->last_page[5],
					 f->last_page[6], f->last_page[7],
					 le32_to_cpu(t0), le32_to_cpu(t1),
					 le32_to_cpu(t2), le32_to_cpu(t3));
			}
		}
	}
	page_chunks = saved_chunks;
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "vfl_scan start_blk=%u hits=%d xrmw=%d\n", start, hits, xrmw);
	return count;
}
static DEVICE_ATTR_WO(vfl_scan);

static ssize_t vfl_map_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	unsigned int i, n = 0;

	if (!vfl_map_count)
		return sysfs_emit(buf, "(empty — run vfl_scan or vfl_build)\n");
	for (i = 0; i < vfl_map_count && i < 32; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "cau=%u virt=%u phys=%u\n",
			       vfl_map[i].cau, vfl_map[i].virt,
			       vfl_map[i].phys);
	if (vfl_map_count > 32)
		n += scnprintf(buf + n, PAGE_SIZE - n, "... +%u more\n",
			       vfl_map_count - 32);
	return n;
}
static DEVICE_ATTR_RO(vfl_map);

/*
 * Walk VFL tail (SLC page 0) on each CAU and ingest wrmx/xrmw remap tables.
 * Usage: echo 1 > vfl_build (last vfl_build_blocks blocks, default 32)
 * echo "START COUNT" > vfl_build (COUNT capped at FMSS_VFL_TAIL)
 */
static ssize_t vfl_build_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, i, saved, start, nblocks, scanned = 0;
	int ingested = 0, ret;

	if (!f)
		return -ENODEV;
	nblocks = vfl_build_blocks;
	if (!nblocks || nblocks > FMSS_VFL_TAIL)
		nblocks = 32;
	start = FMSS_BLOCKS_PER_CAU - nblocks;
	if (buf[0] && buf[0] != '\n') {
		unsigned int a, b = 0;
		int nf = sscanf(buf, "%u %u", &a, &b);

		if (nf < 1)
			return -EINVAL;
		if (nf == 1 && a == 1)
			; /* echo 1 > vfl_build — tail defaults */
		else {
			start = a;
			if (nf >= 2 && b)
				nblocks = b;
			if (nblocks > FMSS_VFL_TAIL)
				nblocks = FMSS_VFL_TAIL;
			if (start >= FMSS_BLOCKS_PER_CAU)
				return -EINVAL;
		}
	}

	mutex_lock(&f->lock);
	vfl_map_count = 0;
	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (i = 0; i < nblocks; i++) {
				unsigned int blk = start + i;
				u32 addr;

				if (blk >= FMSS_BLOCKS_PER_CAU)
					break;
				if (reset_every && f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				addr = fmss_ppn_addr(cau, blk, 0, 1);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				scanned++;
				if (ret || fmss_page_blankish(f->last_page, 512))
					continue;
				ingested += fmss_vfl_ingest(f, cau, blk, f->last_page);
			}
		}
	}
	page_chunks = saved;
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "vfl_build start=%u n=%u scanned=%u ingested=%d map=%u\n",
		 start, nblocks, scanned, ingested, vfl_map_count);
	return count;
}
static DEVICE_ATTR_WO(vfl_build);

static char btoc_last[PAGE_SIZE];
static unsigned int btoc_last_len;

static ssize_t btoc_log_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	if (!btoc_last_len)
		return sysfs_emit(buf, "(no scan yet)\n");
	memcpy(buf, btoc_last, btoc_last_len);
	return btoc_last_len;
}
static DEVICE_ATTR_RO(btoc_log);

/*
 * Sogeti/YaFTL BTOC: last page of a user superblock lists LPNs for pages 0..n-2.
 * N31 page 127 is a BE u32 array (we saw 11,12,13,... on cau1 blk 64).
 * Find the block whose BTOC[0]==0 — that page 0 is logical page 0 (FAT boot).
 */
static ssize_t btoc_scan_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, i, saved, start = 0, n = FMSS_BLOCKS_PER_CAU;
	int hits = 0, lpn0 = 0;
	u32 addr, a, b;
	int ret;

	if (!f)
		return -ENODEV;
	btoc_last_len = 0;
	if (buf[0] && buf[0] != '\n') {
		if (sscanf(buf, "%u %u", &start, &n) < 1)
			return -EINVAL;
		if (n == 0 || n > FMSS_BLOCKS_PER_CAU)
			n = FMSS_BLOCKS_PER_CAU;
		if (start >= FMSS_BLOCKS_PER_CAU)
			return -EINVAL;
		if (start + n > FMSS_BLOCKS_PER_CAU)
			n = FMSS_BLOCKS_PER_CAU - start;
	}
	mutex_lock(&f->lock);
	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (i = start; i < start + n; i++) {
				if (reset_every && f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				addr = fmss_ppn_addr(cau, i, 127, 0);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (ret || fmss_page_blankish(f->last_page, 16))
					continue;
				a = ((u32)f->last_page[0] << 24) | ((u32)f->last_page[1] << 16) |
				    ((u32)f->last_page[2] << 8) | f->last_page[3];
				b = ((u32)f->last_page[4] << 24) | ((u32)f->last_page[5] << 16) |
				    ((u32)f->last_page[6] << 8) | f->last_page[7];
				hits++;
				if (a == 0)
					lpn0++;
				if (a < 0x100000 && (b == a + 1 || a == 0)) {
					nand_dev_info(dev,
						 "btoc ce=%u cau=%u blk=%u addr=0x%08x lpn0=%u lpn1=%u %02x %02x %02x %02x\n",
						 ce, cau, i, addr, a, b,
						 f->last_page[0], f->last_page[1],
						 f->last_page[2], f->last_page[3]);
					if (btoc_last_len < PAGE_SIZE - 80)
						btoc_last_len += scnprintf(
							btoc_last + btoc_last_len,
							PAGE_SIZE - btoc_last_len,
							"ce=%u cau=%u blk=%u lpn=%u,%u\n",
							ce, cau, i, a, b);
				}
			}
		}
	}
	page_chunks = saved;
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "btoc_scan start=%u n=%u hits=%d lpn0=%d\n", start, n, hits, lpn0);
	return count;
}
static DEVICE_ATTR_WO(btoc_scan);

static ssize_t fat_scan_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, i, saved, start = 0, n = FMSS_BLOCKS_PER_CAU;
	int hits = 0;
	u32 addr;
	int ret;

	if (!f)
		return -ENODEV;
	if (buf[0] && buf[0] != '\n' && sscanf(buf, "%u %u", &start, &n) >= 1) {
		if (n == 0 || start + n > FMSS_BLOCKS_PER_CAU)
			n = FMSS_BLOCKS_PER_CAU - start;
	}
	btoc_last_len = 0;
	mutex_lock(&f->lock);
	saved = page_chunks;
	page_chunks = 1;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (i = start; i < start + n; i++) {
				if (reset_every && f->pages_since_reset >= reset_every) {
					fmss_nand_reset(f);
					f->pages_since_reset = 0;
				}
				addr = fmss_ppn_addr(cau, i, 0, 0);
				ret = fmss_page_read(f, ce, addr);
				f->pages_since_reset++;
				if (ret)
					continue;
				if (fmss_find(f->last_page, 1024, "MSDOS", 5) ||
				    fmss_find(f->last_page, 1024, "UOKJIHC", 7) ||
				    fmss_find(f->last_page, 1024, "AISPOD", 6) ||
				    fmss_find(f->last_page, 1024, "N31OS", 5) ||
				    fmss_find(f->last_page, 1024, "FAT32", 5) ||
				    fmss_find(f->last_page, 1024, "EXFAT", 5) ||
				    fmss_find(f->last_page, 1024, "iPod_Control", 12) ||
				    (f->last_page_len >= 512 &&
				     fmss_apple_fat_boot(f->last_page))) {
					hits++;
					nand_dev_info(dev,
						 "fat ce=%u cau=%u blk=%u addr=0x%08x %02x %02x %02x %02x %02x %02x %02x %02x\n",
						 ce, cau, i, addr,
						 f->last_page[0], f->last_page[1],
						 f->last_page[2], f->last_page[3],
						 f->last_page[4], f->last_page[5],
						 f->last_page[6], f->last_page[7]);
					if (btoc_last_len < PAGE_SIZE - 80)
						btoc_last_len += scnprintf(
							btoc_last + btoc_last_len,
							PAGE_SIZE - btoc_last_len,
							"fat ce=%u cau=%u blk=%u p0 %02x %02x %02x %02x\n",
							ce, cau, i,
							f->last_page[0], f->last_page[1],
							f->last_page[2], f->last_page[3]);
				}
			}
		}
	}
	page_chunks = saved;
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "fat_scan start=%u n=%u hits=%d\n", start, n, hits);
	return count;
}
static DEVICE_ATTR_WO(fat_scan);

/*
 * Scan FTL user blocks for a FAT12/16/32 boot sector (0xEB/0xE9 + 0xAA55 @ +510).
 * Usage: echo "START N_BLOCKS" > fat_boot_scan
 */
static ssize_t fat_boot_scan_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, cau, b, p, saved, start = 32, nblocks = 24;
	int hits = 0;

	if (!f)
		return -ENODEV;
	if (buf[0] && buf[0] != '\n' && sscanf(buf, "%u %u", &start, &nblocks) < 1)
		return -EINVAL;
	if (nblocks > grep_max_blocks)
		nblocks = grep_max_blocks;
	grep_log_len = 0;
	mutex_lock(&f->lock);
	saved = page_chunks;
	for (ce = 0; ce < FMSS_NUM_CE; ce++) {
		for (cau = 0; cau < FMSS_NUM_CAU; cau++) {
			for (b = start; b < start + nblocks; b++) {
				if (b >= FMSS_BLOCKS_PER_CAU - FMSS_VFL_TAIL)
					break;
				for (p = 0; p < FMSS_BTOC_PAGE; p++) {
					unsigned int off;

					if (fmss_read_lpn_page(f, ce, cau, b, p, NULL, 0))
						continue;
					for (off = 0; off + 512 <= f->last_page_len;
					     off += 512) {
						const u8 *s = f->last_page + off;

						if (s[0] != 0xeb && s[0] != 0xe9)
							continue;
						if (s[510] != 0x55 || s[511] != 0xaa)
							continue;
						if (!fmss_apple_fat_boot(s) &&
						    !fmss_find(f->last_page + off, 512,
							       "FAT32", 5))
							continue;
						hits++;
						nand_dev_info(dev,
							 "fat_boot ce=%u cau=%u blk=%u pg=%u off=%u oem=%.8s\n",
							 ce, cau, b, p, off, s + 3);
						if (grep_log_len < sizeof(grep_log) - 96)
							grep_log_len += scnprintf(
								grep_log + grep_log_len,
								sizeof(grep_log) - grep_log_len,
								"ce=%u cau=%u blk=%u pg=%u off=%u oem=%.8s\n",
								ce, cau, b, p, off,
								s + 3);
					}
				}
			}
		}
	}
	page_chunks = saved;
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "fat_boot_scan start=%u n=%u hits=%d\n", start, nblocks, hits);
	return count;
}
static DEVICE_ATTR_WO(fat_boot_scan);

static ssize_t scan_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, start, n, i;
	u32 addr;
	int nonempty = 0;

	if (!f)
		return -ENODEV;
	if (sscanf(buf, "%u %u %u", &ce, &start, &n) != 3)
		return -EINVAL;
	if (ce > 7 || n > 64)
		return -EINVAL;
	mutex_lock(&f->lock);
	for (i = 0; i < n; i++) {
		if (reset_every && f->pages_since_reset >= reset_every) {
			fmss_nand_reset(f);
			f->pages_since_reset = 0;
		}
		addr = (start + i) * 128u;
		if (fmss_page_read(f, ce, addr)) {
			nand_dev_info(dev, "scan ce=%u blk=%u FAIL\n", ce, start + i);
			f->pages_since_reset++;
			continue;
		}
		f->pages_since_reset++;
		if (f->last_page[0] != 0xff && f->last_page[0] != 0x00) {
			nonempty++;
			nand_dev_info(dev,
				 "scan ce=%u blk=%u p0 %02x %02x %02x %02x %02x %02x %02x %02x\n",
				 ce, start + i,
				 f->last_page[0], f->last_page[1], f->last_page[2],
				 f->last_page[3], f->last_page[4], f->last_page[5],
				 f->last_page[6], f->last_page[7]);
		}
	}
	mutex_unlock(&f->lock);
	nand_dev_info(dev, "scan ce=%u start=%u n=%u nonempty_head=%d\n",
		 ce, start, n, nonempty);
	return count;
}
static DEVICE_ATTR_WO(scan);

static ssize_t param_hex_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct nand_s5l8740 *f = nand_dev;
	int i, n = 0;

	if (!f)
		return -ENODEV;
	for (i = 0; i < 128; i++) {
		n += scnprintf(buf + n, PAGE_SIZE - n, "%02x%s",
			       f->last_param[i], ((i + 1) % 16) ? " " : "\n");
	}
	return n;
}
static DEVICE_ATTR_RO(param_hex);

static ssize_t param_info_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct nand_s5l8740 *f = nand_dev;
	const u8 *p;

	if (!f)
		return -ENODEV;
	p = f->last_param;
	return sysfs_emit(buf,
			  "ce=%d ret=%d\n"
			  "caus_per_channel=%u cau_bits=%u\n"
			  "blocks_per_cau=%u block_bits=%u\n"
			  "pages_per_block=%u pages_per_block_slc=%u\n"
			  "page_address_bits=%u bits_per_cell_addr=%u default_bits_per_cell=%u\n"
			  "page_size=%u\n",
			  f->last_param_ce, f->last_param_ret,
			  fmss_le32(p, 16), fmss_le32(p, 20),
			  fmss_le32(p, 24), fmss_le32(p, 28),
			  fmss_le32(p, 32), fmss_le32(p, 36),
			  fmss_le32(p, 40), fmss_le32(p, 44), fmss_le32(p, 48),
			  fmss_le32(p, 52));
}
static DEVICE_ATTR_RO(param_info);

static ssize_t param_read_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce = 0;
	int ret;

	if (!f)
		return -ENODEV;
	if (buf[0] && kstrtouint(buf, 0, &ce))
		return -EINVAL;
	mutex_lock(&f->lock);
	ret = fmss_param_read(f, ce);
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(param_read);

static ssize_t nand_reset_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	int ret;

	if (!f)
		return -ENODEV;
	mutex_lock(&f->lock);
	ret = fmss_nand_reset(f);
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(nand_reset);

static ssize_t set_feature_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, feat, val;
	int ret;

	if (!f)
		return -ENODEV;
	if (sscanf(buf, "%u %i %i", &ce, &feat, &val) < 3)
		return -EINVAL;
	mutex_lock(&f->lock);
	ret = fmss_set_feature(f, ce, (u16)feat, val);
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(set_feature);

static ssize_t get_feature_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int ce, feat, len = 16;
	int nf, ret;

	if (!f)
		return -ENODEV;
	nf = sscanf(buf, "%u %i %u", &ce, &feat, &len);
	if (nf < 2)
		return -EINVAL;
	if (len < 4)
		len = 4;
	if (len > 16)
		len = 16;
	mutex_lock(&f->lock);
	last_feat_ce = (int)ce;
	last_feat_id = (u16)feat;
	ret = fmss_get_feature(f, ce, (u16)feat, last_feat, len);
	last_feat_ret = ret;
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}

static ssize_t get_feature_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return sysfs_emit(buf,
			  "ce=%d feat=0x%04x ret=%d %02x %02x %02x %02x %02x %02x %02x %02x\n",
			  last_feat_ce, last_feat_id, last_feat_ret,
			  last_feat[0], last_feat[1], last_feat[2], last_feat[3],
			  last_feat[4], last_feat[5], last_feat[6], last_feat[7]);
}
static DEVICE_ATTR_RW(get_feature);

static ssize_t page_data_read(struct file *filp, struct kobject *kobj,
			     struct bin_attribute *attr, char *buf,
			     loff_t off, size_t count)
{
	struct nand_s5l8740 *f = nand_dev;

	if (!f)
		return -ENODEV;
	if (off >= FMSS_PAGE_LEN)
		return 0;
	if (off + count > FMSS_PAGE_LEN)
		count = FMSS_PAGE_LEN - off;
	memcpy(buf, f->last_page + off, count);
	return count;
}
static BIN_ATTR_RO(page_data, FMSS_PAGE_LEN);

static struct attribute *fmss_attrs[] = {
	&dev_attr_regs.attr,
	&dev_attr_id.attr,
	&dev_attr_read_id.attr,
	&dev_attr_page_read.attr,
	&dev_attr_dma_read.attr,
	&dev_attr_cs_canary_read.attr,
	&dev_attr_cs_canary_matrix.attr,
	&dev_attr_cs_phys_read.attr,
	&dev_attr_cs_phys_last.attr,
	&dev_attr_lba_weave_scan.attr,
	&dev_attr_seq_kick.attr,
	&dev_attr_scan.attr,
	&dev_attr_vfl_scan.attr,
	&dev_attr_vfl_dump.attr,
	&dev_attr_vfl_build.attr,
	&dev_attr_vfl_map.attr,
	&dev_attr_vfl_log.attr,
	&dev_attr_lpn_build.attr,
	&dev_attr_l2v_build.attr,
	&dev_attr_l2v_status.attr,
	&dev_attr_whimory_mount.attr,
	&dev_attr_whimory_status.attr,
	&dev_attr_lpn_read.attr,
	&dev_attr_read_sector_dense.attr,
	&dev_attr_read_sector_slow.attr,
	&dev_attr_read_sector_phys.attr,
	&dev_attr_resolve_log.attr,
	&dev_attr_ftl_grep.attr,
	&dev_attr_readme_read.attr,
	&dev_attr_boot_read.attr,
	&dev_attr_ftl_ascii.attr,
	&dev_attr_grep_log.attr,
	&dev_attr_lpn_index.attr,
	&dev_attr_sector_hex.attr,
	&dev_attr_btoc_scan.attr,
	&dev_attr_btoc_log.attr,
	&dev_attr_fat_scan.attr,
	&dev_attr_fat_boot_scan.attr,
	&dev_attr_page_status.attr,
	&dev_attr_page_hex.attr,
	&dev_attr_spare_hex.attr,
	&dev_attr_parity_hex.attr,
	&dev_attr_param_read.attr,
	&dev_attr_param_hex.attr,
	&dev_attr_param_info.attr,
	&dev_attr_nand_reset.attr,
	&dev_attr_set_feature.attr,
	&dev_attr_get_feature.attr,
	NULL,
};

static struct bin_attribute *fmss_bin_attrs[] = {
	&bin_attr_page_data,
	NULL,
};

static const struct attribute_group nand_group = {
	.attrs = fmss_attrs,
	.bin_attrs = fmss_bin_attrs,
};
static const struct attribute_group *nand_groups[] = { &nand_group, NULL };

static int fmss_probe(struct platform_device *pdev)
{
	struct nand_s5l8740 *f;

	f = devm_kzalloc(&pdev->dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	f->base = devm_ioremap(&pdev->dev, FMSS_PHYS, FMSS_SIZE);
	if (!f->base)
		return -ENOMEM;
	mutex_init(&f->lock);
	f->last_ce = -1;
	f->last_page_ce = -1;
	f->last_page_ret = -1;
	f->last_param_ce = -1;
	f->last_param_ret = -1;
	fmss_dma_setup(f, &pdev->dev);
	nand_dev = f;
	platform_set_drvdata(pdev, f);
	dev_info(&pdev->dev,
		 "FMSS peek FMCTRL0=0x%08x NANDSTAT=0x%08x quiet=%d (read-only until read_id/page_read)\n",
		 readl(f->base + FMCTRL0), readl(f->base + NANDSTAT), quiet);
	return 0;
}

/*
 * The controller has its own DMA. Handing the machine over with a
 * transfer in flight lets it complete into memory the next kernel owns,
 * and on the write side it can leave a page half-programmed -- so this
 * is the one shutdown here that protects the flash rather than just RAM.
 */
static void fmss_shutdown(struct platform_device *pdev)
{
	struct nand_s5l8740 *f = platform_get_drvdata(pdev);

	if (f)
		fmss_dma_teardown(f);
}

static void fmss_remove(struct platform_device *pdev)
{
	struct nand_s5l8740 *f = platform_get_drvdata(pdev);

	nand_dev = NULL;
	fmss_l2v_free();
	fmss_early_lba_free();
	boot_carve_valid = false;
	root_dir_valid = false;
	if (f)
		fmss_dma_teardown(f);
}

static struct platform_driver nand_driver = {
	.probe = fmss_probe,
	.remove = fmss_remove,
	.shutdown = fmss_shutdown,
	.driver = {
		.name = "s5l8740-nand",
		.dev_groups = nand_groups,
	},
};

static struct platform_device *nand_pdev;

static int __init nand_s5l8740_init(void)
{
	int ret;

	ret = platform_driver_register(&nand_driver);
	if (ret)
		return ret;
	nand_pdev = platform_device_register_simple("s5l8740-nand", -1, NULL, 0);
	if (IS_ERR(nand_pdev)) {
		platform_driver_unregister(&nand_driver);
		return PTR_ERR(nand_pdev);
	}
	return 0;
}

static void __exit nand_s5l8740_exit(void)
{
	platform_device_unregister(nand_pdev);
	platform_driver_unregister(&nand_driver);
}

/* --- Exported read-only FTL sector API (ftl-s5l8740.ko) --- */

bool nand_ftl_present(void)
{
	return nand_dev != NULL;
}
EXPORT_SYMBOL_GPL(nand_ftl_present);

struct device *nand_ftl_device(void)
{
	return nand_dev ? nand_dev->dev : NULL;
}
EXPORT_SYMBOL_GPL(nand_ftl_device);

unsigned int nand_ftl_lpn_count(void)
{
	return l2v_mapped ? l2v_mapped : lpn_index_count;
}
EXPORT_SYMBOL_GPL(nand_ftl_lpn_count);

int nand_ftl_build_map(unsigned int max_lpn)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int nblocks;
	int ret;

	if (!f)
		return -ENODEV;
	nblocks = l2v_scan_blocks ? l2v_scan_blocks : FMSS_L2V_DEFAULT_BLOCKS;
	mutex_lock(&f->lock);
	ret = fmss_l2v_build(f, max_lpn, 0, nblocks);
	mutex_unlock(&f->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(nand_ftl_build_map);

int nand_ftl_read_sector(u64 logical_sector, void *buf)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int lpn, sec, ce, cau, block, page, pblock, off, saved;
	u32 addr, packed;
	int ret;
	int (*hook)(u64 lba, void *buf);

	if (!buf)
		return -ENODEV;

	/*
	 * Whimory registers the real LBA reader after FTL_Open. Call it
	 * without the FMSS mutex — the FIL page_read wrapper takes that lock.
	 */
	hook = READ_ONCE(nand_ftl_read_hook);
	if (hook)
		return hook(logical_sector, buf);

	if (!f)
		return -ENODEV;

	mutex_lock(&f->lock);

	/* Prefer full LBA map (BTE / BTOC sector fills / carve / META). */
	if (logical_sector < fmss_lba_map_cap() &&
	    !fmss_lba_lookup((unsigned int)logical_sector, &ce, &cau,
			     &block, &page, &sec, &packed)) {
		pblock = fmss_map_to_phys(cau, block, packed);
		saved = page_chunks;
		page_chunks = 16;
		if (reset_every && f->pages_since_reset >= reset_every) {
			fmss_nand_reset(f);
			f->pages_since_reset = 0;
		}
		addr = fmss_ppn_addr(cau, pblock, page, 0);
		ret = fmss_page_read(f, ce, addr);
		f->pages_since_reset++;
		page_chunks = saved;
		if (!ret) {
			if (sec == L2V_SEC_FROM_LBA)
				sec = (unsigned int)logical_sector %
				      FMSS_VBAS_PER_PAGE;
			/* Pass 2: refuse stale map if META LBA disagrees. */
			if (f->last_spare_len >= 16 * (sec + 1)) {
				const u8 *m = f->last_spare + sec * 16;

				if (m[0] == 0x01) {
					u32 mlba = get_unaligned_le32(m + 8);

					if (mlba != (u32)logical_sector)
						ret = -EIO;
				}
			}
			if (!ret) {
				off = sec * FMSS_SECTOR_LEN;
				if (off + FMSS_SECTOR_LEN <= f->last_page_len)
					memcpy(buf, f->last_page + off,
					       FMSS_SECTOR_LEN);
				else
					ret = -ERANGE;
			}
			resolve_log_len = scnprintf(
				resolve_log, sizeof(resolve_log),
				"dense LBA=%llu src=lba_map phys=%d ce=%u cau=%u blk=%u→%u pg=%u sec=%u ret=%d head=%02x%02x%02x%02x\n",
				(unsigned long long)logical_sector,
				!!(packed & L2V_PHYS), ce, cau, block, pblock,
				page, sec, ret,
				ret ? 0 : ((u8 *)buf)[0],
				ret ? 0 : ((u8 *)buf)[1],
				ret ? 0 : ((u8 *)buf)[2],
				ret ? 0 : ((u8 *)buf)[3]);
			if (!ret) {
				mutex_unlock(&f->lock);
				return 0;
			}
			/* Mapped read failed / META mismatch — fall through to LPN. */
		} else {
			resolve_log_len = scnprintf(
				resolve_log, sizeof(resolve_log),
				"dense LBA=%llu src=lba_map PHYS-fail ret=%d; try l2v_lpn\n",
				(unsigned long long)logical_sector, ret);
			/* Fall through to LPN path. */
		}
	}

	lpn = (unsigned int)(logical_sector / NAND_FTL_SECTORS_PER_LPN);
	sec = (unsigned int)(logical_sector % NAND_FTL_SECTORS_PER_LPN);

	ret = nand_ftl_read_lpn_locked(f, lpn, sec, buf);
	if (!ret)
		resolve_log_len = scnprintf(
			resolve_log, sizeof(resolve_log),
			"dense LBA=%llu src=l2v_lpn lpn=%u sec=%u ret=0 head=%02x%02x%02x%02x\n",
			(unsigned long long)logical_sector, lpn, sec,
			((u8 *)buf)[0], ((u8 *)buf)[1],
			((u8 *)buf)[2], ((u8 *)buf)[3]);
	else
		resolve_log_len = scnprintf(
			resolve_log, sizeof(resolve_log),
			"dense LBA=%llu src=l2v_lpn lpn=%u sec=%u ret=%d\n",
			(unsigned long long)logical_sector, lpn, sec, ret);
	mutex_unlock(&f->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(nand_ftl_read_sector);

int s5l8740_nand_available(void)
{
	return nand_dev != NULL;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_available);

int s5l8740_nand_meta_transport_ok(void)
{
	struct nand_s5l8740 *f = nand_dev;

	/*
	 * Disk registration still requires meta_dma_read=1.
	 * Early CS phys helpers use s5l8740_nand_cs_phys_read() instead —
	 * glass-proven span4/rec4112, but FTL must not auto-open yet.
	 */
	return f && f->dma_ok && meta_dma_read;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_meta_transport_ok);

int s5l8740_nand_hw_init(void)
{
	struct nand_s5l8740 *f = nand_dev;
	int ret;

	if (!f)
		return -ENODEV;
	mutex_lock(&f->lock);
	ret = fmss_nand_reset(f);
	if (!ret)
		(void)fmss_param_read(f, 0);
	mutex_unlock(&f->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_hw_init);

int s5l8740_nand_query_geometry(struct s5l8740_nand_geom *g)
{
	struct nand_s5l8740 *f = nand_dev;
	const u8 *p;
	u32 caus, cau_bits, blocks, block_bits, pages, pages_slc;
	u32 page_bits, page_size;

	if (!g)
		return -EINVAL;
	if (!f)
		return -ENODEV;

	memset(g, 0, sizeof(*g));
	g->num_ce = FMSS_NUM_CE;
	g->num_cau = FMSS_NUM_CAU;
	g->blocks_per_cau = FMSS_BLOCKS_PER_CAU;
	g->pages_per_block = 128;
	g->pages_per_block_slc = 128;
	g->page_size = FMSS_PAGE_LEN;
	g->vfl_tail = FMSS_VFL_TAIL;
	g->page_bits = FMSS_PAGE_BITS;
	g->block_bits = FMSS_BLOCK_BITS;
	g->cau_bits = FMSS_CAU_BITS;
	g->caus_per_channel = FMSS_NUM_CAU;

	mutex_lock(&f->lock);
	if (f->last_param_ret != 0)
		(void)fmss_param_read(f, 0);
	p = f->last_param;
	if (f->last_param_ret == 0) {
		caus = fmss_le32(p, 16);
		cau_bits = fmss_le32(p, 20);
		blocks = fmss_le32(p, 24);
		block_bits = fmss_le32(p, 28);
		pages = fmss_le32(p, 32);
		pages_slc = fmss_le32(p, 36);
		page_bits = fmss_le32(p, 40);
		page_size = fmss_le32(p, 52);
		if (caus && caus <= 4)
			g->caus_per_channel = caus;
		if (cau_bits && cau_bits <= 4)
			g->cau_bits = cau_bits;
		if (blocks && blocks <= 8192)
			g->blocks_per_cau = blocks;
		if (block_bits && block_bits <= 16)
			g->block_bits = block_bits;
		if (pages && pages <= 256)
			g->pages_per_block = pages;
		if (pages_slc && pages_slc <= 256)
			g->pages_per_block_slc = pages_slc;
		if (page_bits && page_bits <= 16)
			g->page_bits = page_bits;
		if (page_size == 4096 || page_size == 8192 ||
		    page_size == 16384)
			g->page_size = page_size;
		g->from_param_page = true;
	}
	mutex_unlock(&f->lock);

	/*
	 * FIL GetInfo (vtable +80,:
	 * 101 — NAND present / signature +0x34 geometry (WhimoryBoot.c:169,260)
	 * 0 → "No NAND device found". Compared to sig[+0x34].
	 * Value stored at format is blocks_per_cau → 0x8D102CC
	 * is the first geometry word copied from the param page).
	 * 104 — BUF_Init data bytes first arg) = physical page size
	 * 105 — BUF_Init meta bytes second arg) = 16
	 * 135 — stored at 0x8D0CE2C and unused after GetInfo
	 */
	g->dev_id = g->blocks_per_cau;
	g->geom_104 = g->page_size;
	g->geom_105 = 16;
	g->geom_135 = g->page_size >> 12;
	if (!g->dev_id)
		return -ENODEV;
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_query_geometry);

u32 s5l8740_nand_fil_get_info(u32 selector)
{
	struct s5l8740_nand_geom g;

	if (s5l8740_nand_query_geometry(&g))
		return 0;
	switch (selector) {
	case 101:
		return g.dev_id;
	case 104:
		return g.geom_104;
	case 105:
		return g.geom_105;
	case 135:
		return g.geom_135;
	default:
		return 0;
	}
}
EXPORT_SYMBOL_GPL(s5l8740_nand_fil_get_info);

static int fmss_dma_page_read_records(struct nand_s5l8740 *f,
				      unsigned int ce, u32 addr,
				      unsigned int slot,
				      unsigned int span)
{
	unsigned int saved_slot = dma_slot;
	unsigned int saved_nsect = dma_nsect;
	unsigned int saved_rec = dma_rec;
	int ret;

	if (slot > 3)
		return -EINVAL;
	if (!span || span > 4 || slot + span > 4)
		return -EINVAL;

	dma_slot = slot;
	dma_nsect = span;
	dma_rec = FMSS_PPN_REC; /* 4096 data + 16 meta — glass ABI */

	if (meta_dma_reset_before)
		fmss_nand_reset(f);

	ret = fmss_dma_page_read(f, ce, addr);

	dma_rec = saved_rec;
	dma_nsect = saved_nsect;
	dma_slot = saved_slot;
	return ret;
}

void s5l8740_nand_meta_decode(const u8 *m16,
			      struct s5l8740_meta_decoded *out)
{
	unsigned int i;
	bool blank = true;

	if (!out)
		return;
	memset(out, 0, sizeof(*out));
	if (!m16)
		return;

	out->type = m16[0];
	out->flags = m16[1];
	out->weave = (u64)m16[2] | ((u64)m16[3] << 8) |
		     ((u64)m16[4] << 16) | ((u64)m16[5] << 24) |
		     ((u64)m16[6] << 32) | ((u64)m16[7] << 40);
	out->lba = (u32)m16[8] | ((u32)m16[9] << 8) |
		   ((u32)m16[10] << 16) | ((u32)m16[11] << 24);

	for (i = 0; i < N31_META_SLOT_SIZE; i++) {
		if (m16[i] != 0x00 && m16[i] != 0xff) {
			blank = false;
			break;
		}
	}
	out->blank = blank;
	out->valid = !blank;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_meta_decode);

void s5l8740_nand_meta_decode_legacy(const u8 *m16,
				     struct s5l8740_nand_slot_meta *out)
{
	struct s5l8740_meta_decoded d;

	if (!out)
		return;
	memset(out, 0, sizeof(*out));
	s5l8740_nand_meta_decode(m16, &d);
	out->type = d.type;
	out->flags = d.flags;
	out->weave48 = d.weave;
	out->lba = d.lba;
	if (m16)
		memcpy(out->aux, m16 + 12, 4);
	out->data_like = n31_meta_is_data_record(&d);
}
EXPORT_SYMBOL_GPL(s5l8740_nand_meta_decode_legacy);

int s5l8740_nand_meta_pick_lba(const struct s5l8740_cs_page *page,
			       u32 fmss_lba)
{
	int best = -ENOENT;
	u64 best_weave = 0;
	unsigned int s;

	if (!page)
		return -EINVAL;

	for (s = 0; s < N31_DATA_SLOTS; s++) {
		const struct s5l8740_meta_decoded *cur = &page->meta[s];

		if (!n31_meta_is_data_record(cur) || cur->lba != fmss_lba)
			continue;
		if (best < 0 || n31_weave_newer(cur->weave, best_weave)) {
			if (best >= 0 && page->meta[best].weave > cur->weave)
				pr_info("s5l8740-nand: weave moved backward "
					"fmss_lba=%u slot%u->%u "
					"%012llx -> %012llx\n",
					fmss_lba, best, s,
					(unsigned long long)page->meta[best].weave,
					(unsigned long long)cur->weave);
			best = (int)s;
			best_weave = cur->weave;
		} else if (best >= 0 && cur->weave < best_weave) {
			pr_info("s5l8740-nand: weave older skipped "
				"fmss_lba=%u slot=%u weave=%012llx "
				"best=%012llx\n",
				fmss_lba, s,
				(unsigned long long)cur->weave,
				(unsigned long long)best_weave);
		}
	}
	return best;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_meta_pick_lba);

/*
 * FTL map CS physical read: always slot0/span4/rec4112.
 * Fills struct s5l8740_cs_page. No lba_map ingest.
 */
/*
 * Read only the first record of a page.
 *
 * A page is four 4096+16 records and the full read moves 16448 bytes.
 * The FTL classify sweep looks at 64 bytes of slot-0 data and slot-0's
 * 16-byte meta to decide "is this block empty", and seventy percent of
 * the volume answers yes -- so most of that sweep was transferring 16 KB
 * to read 80 bytes. Span 1 moves 4112 bytes instead.
 *
 * Only the empty test is safe on one record. Anything else classify asks
 * -- CXT in particular -- inspects all four slots' meta, so the caller
 * must fall back to the full read whenever the answer is not "empty".
 * Slots 1..3 are left as 0xff here so a caller that ignores that rule
 * sees erased meta rather than stale data from the previous page.
 */
int s5l8740_nand_cs_phys_read_slot0(u8 ce, u8 cau, u16 block, u8 page,
				    struct s5l8740_cs_page *out)
{
	return s5l8740_nand_cs_phys_read_span(ce, cau, block, page, out, 1);
}
EXPORT_SYMBOL_GPL(s5l8740_nand_cs_phys_read_slot0);

int s5l8740_nand_cs_phys_read(u8 ce, u8 cau, u16 block, u8 page,
			      struct s5l8740_cs_page *out)
{
	return s5l8740_nand_cs_phys_read_span(ce, cau, block, page, out, 4);
}

int s5l8740_nand_cs_phys_read_span(u8 ce, u8 cau, u16 block, u8 page,
				   struct s5l8740_cs_page *out, unsigned int span)
{
	return s5l8740_nand_cs_phys_read_slc(ce, cau, block, page, 0, out,
					     span);
}

/*
 * The same read, on a nominated SLC plane.
 *
 * Everything above reads slc=0 because that is where the user area lives.
 * The FPart region at the tail of each CAU does not: it is SLC, and locating
 * it means trying slc=1 before slc=0.
 *
 * That scan used to go through s5l8740_nand_page_read(), which refuses any
 * request for meta unless meta_dma_read is set -- and it is deliberately not
 * set, because a permanent live CS kick reboots the device. So every FPart
 * read returned -EOPNOTSUPP before touching the NAND: 512 reads, 512
 * failures, sixty milliseconds, and a "signature not found" verdict about a
 * region nobody had actually looked at.
 */
int s5l8740_nand_cs_phys_read_slc(u8 ce, u8 cau, u16 block, u8 page, u8 slc,
				  struct s5l8740_cs_page *out, unsigned int span)
{
	struct nand_s5l8740 *f = nand_dev;
	u32 addr;
	bool saved_armed;
	int ret;
	u64 t0, t1, t2;
	unsigned int s;

	if (!f || !out)
		return -ENODEV;
	if (!f->dma_ok)
		return -ENODEV;
	if (ce >= FMSS_NUM_CE || cau >= FMSS_NUM_CAU ||
	    block >= FMSS_BLOCKS_PER_CAU || page > FMSS_BTOC_PAGE)
		return -EINVAL;
	if (dma_dry)
		return -EAGAIN;
	if (!dma_armed)
		return -EPERM;

	memset(out, 0, sizeof(*out));
	addr = fmss_ppn_addr(cau, block, page, slc);

	mutex_lock(&f->lock);
	if (cs_reset_every && f->pages_since_reset >= cs_reset_every) {
		/*
		 * Clearing the counter is the half that was missing. Without
		 * it the threshold latches and every read after the first
		 * reset resets again, which turns a periodic safeguard into a
		 * per-read cost.
		 */
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}
	saved_armed = dma_armed;
	/* One-shot friendly: re-arm for this kick; disarm after if one_shot. */
	dma_armed = true;
	dma_skip_ingest = true;
	t0 = ktime_get_ns();
	ret = fmss_dma_page_read_records(f, ce, addr, 0, span);
	t1 = ktime_get_ns();
	dma_skip_ingest = false;
	if (!dma_one_shot)
		dma_armed = saved_armed;
	f->pages_since_reset++;

	if (!ret) {
		size_t dn = f->last_page_len;
		size_t mn = f->last_spare_len;

		if (dn > FMSS_PAGE_LEN)
			dn = FMSS_PAGE_LEN;
		if (mn > S5L8740_NAND_META_SIZE)
			mn = S5L8740_NAND_META_SIZE;

		for (s = 0; s < N31_DATA_SLOTS; s++) {
			size_t doff = (size_t)s * N31_DATA_SLOT_SIZE;
			size_t moff = (size_t)s * N31_META_SLOT_SIZE;

			memset(out->data[s], 0xff, N31_DATA_SLOT_SIZE);
			if (dn > doff) {
				size_t n = N31_DATA_SLOT_SIZE;

				if (dn - doff < n)
					n = dn - doff;
				memcpy(out->data[s], f->last_page + doff, n);
			}
			memset(out->meta_raw[s], 0xff, N31_META_SLOT_SIZE);
			if (mn > moff) {
				size_t n = N31_META_SLOT_SIZE;

				if (mn - moff < n)
					n = mn - moff;
				memcpy(out->meta_raw[s], f->last_spare + moff, n);
			}
			s5l8740_nand_meta_decode(out->meta_raw[s], &out->meta[s]);
		}
	}

	/* Where the wall time goes; also the last line the host sees over
	 * /proc/kmsg if the glass dies mid-recover.
	 */
	t2 = ktime_get_ns();
	cs_ns_kick += t1 - t0;
	cs_ns_copy += t2 - t1;
	cs_reads_total++;
	if (cs_heartbeat && (cs_reads_total & 0x3ff) == 0)
		pr_info("s5l8740-nand: cs_phys_read n=%u since_reset=%u "
			"kick_us=%llu copy_us=%llu NANDSTAT=%08x\n",
			cs_reads_total, f->pages_since_reset,
			div_u64(cs_ns_kick, 1000 * cs_reads_total),
			div_u64(cs_ns_copy, 1000 * cs_reads_total),
			readl(f->base + NANDSTAT));
	mutex_unlock(&f->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_cs_phys_read_slc);
EXPORT_SYMBOL_GPL(s5l8740_nand_cs_phys_read);

/* Batch CS sessions for map build: keep armed across many phys reads. */
static struct {
	bool saved;
	bool dry;
	bool armed;
	bool one_shot;
} fmss_dma_batch;

int s5l8740_nand_dma_session_begin(void)
{
	if (fmss_dma_batch.saved)
		return -EBUSY;
	fmss_dma_batch.dry = dma_dry;
	fmss_dma_batch.armed = dma_armed;
	fmss_dma_batch.one_shot = dma_one_shot;
	fmss_dma_batch.saved = true;
	dma_dry = false;
	dma_one_shot = false;
	dma_armed = true;
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_dma_session_begin);

void s5l8740_nand_dma_session_end(void)
{
	if (!fmss_dma_batch.saved)
		return;
	dma_dry = fmss_dma_batch.dry;
	dma_armed = fmss_dma_batch.armed;
	dma_one_shot = fmss_dma_batch.one_shot;
	fmss_dma_batch.saved = false;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_dma_session_end);

int s5l8740_nand_page_read(unsigned int ce, unsigned int cau,
			   unsigned int block, unsigned int page,
			   unsigned int slc, unsigned int chunks,
			   void *data, size_t data_len,
			   void *meta, size_t meta_len)
{
	struct nand_s5l8740 *f = nand_dev;
	unsigned int saved;
	u32 addr;
	int ret;

	if (!f)
		return -ENODEV;
	if (ce >= FMSS_NUM_CE || cau >= FMSS_NUM_CAU ||
	    block >= FMSS_BLOCKS_PER_CAU || page > FMSS_BTOC_PAGE)
		return -EINVAL;
	if (!chunks || chunks > FMSS_MAX_CHUNKS)
		chunks = FMSS_MAX_CHUNKS;

	mutex_lock(&f->lock);

	if (reset_every && f->pages_since_reset >= reset_every) {
		fmss_nand_reset(f);
		f->pages_since_reset = 0;
	}

	saved = page_chunks;
	page_chunks = chunks;
	addr = fmss_ppn_addr(cau, block, page, slc);

	if (meta && meta_len) {
		/*
		 * Real metadata: always full-page CS span4/rec4112, then
		 * software-slice. Never invent PIO spare as Whimory meta.
		 */
		if (!meta_dma_read || !f->dma_ok) {
			ret = -EOPNOTSUPP;
			goto out_restore;
		}
		if (dma_dry) {
			ret = -EAGAIN;
			goto out_restore;
		}
		if (!dma_armed) {
			ret = -EPERM;
			goto out_restore;
		}
		dma_armed = true;
		ret = fmss_dma_page_read_records(f, ce, addr, 0, 4);
	} else {
		ret = fmss_page_read(f, ce, addr);
	}

	f->pages_since_reset++;

	if (!ret) {
		if (data && data_len) {
			size_t have = f->last_page_len;

			if (have > FMSS_PAGE_LEN)
				have = FMSS_PAGE_LEN;
			if (data_len > have)
				data_len = have;
			memcpy(data, f->last_page, data_len);
		}

		if (meta && meta_len) {
			size_t have = S5L8740_NAND_META_SIZE;

			memset(meta, 0xff, meta_len);
			if (have > f->last_spare_len)
				have = f->last_spare_len;
			if (have > meta_len)
				have = meta_len;
			memcpy(meta, f->last_spare, have);
		}
	}

out_restore:
	page_chunks = saved;
	mutex_unlock(&f->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_page_read);

int s5l8740_nand_reset(void)
{
	struct nand_s5l8740 *f = nand_dev;
	int ret;

	if (!f)
		return -ENODEV;
	mutex_lock(&f->lock);
	ret = fmss_nand_reset(f);
	mutex_unlock(&f->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s5l8740_nand_reset);

void s5l8740_nand_register_ftl_read(int (*fn)(u64 lba, void *buf))
{
	WRITE_ONCE(nand_ftl_read_hook, fn);
}
EXPORT_SYMBOL_GPL(s5l8740_nand_register_ftl_read);

module_init(nand_s5l8740_init);
module_exit(nand_s5l8740_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5L8740 NAND/FMSS controller (Whimory FIL, read-only)");
MODULE_AUTHOR("n31");
