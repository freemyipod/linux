// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 Whimory FTL — read-only block path (N31).
 *
 * Layers: NAND FIL → FPart → VFL → FTL/L2V → optional /dev/s5l8740-ftl.
 * The CS-map front-end in ftl-s5l8740-csmap.c is the preferred RO disk path.
 * This module retains the classic Whimory open/boot helpers.
 *
 * The block device is registered only after FAT-critical validation succeeds.
 * Empty or inconsistent maps never expose a disk.
 */
#include <linux/bio.h>
#include <linux/bitops.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/stdarg.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "whimory-s5l8740.h"
#include "ftl-s5l8740-csmap.h"

#define FTL_DISK_NAME		"s5l8740-ftl"
#define FTL_IPOD_NAME		"s5l8740-ipod"

#define WHIMORY_ORACLE_SIG	"apple/n31-whimory-sig.bin"
#define WHIMORY_ORACLE_ROOT	"apple/n31-whimory-l2v-root.bin"
#define WHIMORY_ORACLE_NODES	"apple/n31-whimory-l2v-nodes.bin"
#define WHIMORY_ORACLE_GLOBALS	"apple/n31-whimory-l2v-globals.bin"

#define WHIMORY_SPECIAL_LBA	0xFFFF0000u

static bool import_l2v_oracle;
module_param(import_l2v_oracle, bool, 0644);
MODULE_PARM_DESC(import_l2v_oracle,
		 "Load L2V root/nodes/globals from /lib/firmware/apple/");

/*
 * Rebuild every open superblock we find, up to a backstop.
 *
 * This was 16, which was a bring-up number and never lifted. It is not a
 * tuning knob: an open superblock holds writes that happened after the
 * checkpoint, so a cap on how many get rebuilt is a cap on how much recent
 * data ends up in the map. With 1619 open superblocks on this volume it was
 * rebuilding one percent of them and silently dropping the rest, which is a
 * second and entirely separate cause of the missing-recent-files symptom
 * that the page-0 weave bug caused.
 *
 * 4096 is above the number of blocks a CAU has, so in practice this is "all
 * of them" -- it stays a finite number only so a corrupt classify cannot turn
 * into an unbounded loop. The cost is real and is reported: each rebuild
 * reads pages from page 0 until it finds a blank one, so a genuinely open
 * block stops early and a block that is actually full reads all 127.
 */
/*
 * Rebuild a closed superblock whose BTOC has no BTE array.
 *
 * Off, and the measurement that turned it off is worth keeping. Enabling it
 * answered the question it was built for -- the 70 such superblocks report
 * 127 pages/sb, so they are full sealed blocks and not empty ones, exactly
 * as the decomp of sub_567E3C predicted. But the second number says the
 * work is unnecessary and the third says it is harmful:
 *
 *   btoc_fallback sbs=70 pages=8890 hits=35560 (127 pages/sb)
 *   mapped_lbas   949733 -> 949733
 *
 * 35560 mappings applied and not one new LBA. They did not add anything;
 * they replaced existing mappings one for one with different VBAs. Reads
 * then came back holding the wrong logical block:
 *
 *   sftl lba mismatch want=0x8042b meta=0x7fed0 type=01
 *   sftl lba mismatch want=0x7483f meta=0x7482c type=01
 *
 * with scattered deltas, which is what overwriting a correct map with a
 * differently-derived one looks like. The build immediately before this
 * read every file on the volume with no error.
 *
 * The reason is the same one that made the uncapped open rebuild
 * destructive: on this volume no superblock is newer than the checkpoint
 * (weave newer=0 older=2182), so per-page meta has nothing to add over the
 * CXT and every override is a regression. The CXT already maps everything
 * those blocks hold, which is precisely why mapped_lbas did not move.
 *
 * Kept as a switch rather than deleted because the reasoning is
 * volume-specific: a device whose checkpoint is genuinely behind its data
 * would need this, and would show it by mapped_lbas rising when it runs.
 */
/*
 * Dump the raw CXT tree records covering a window of logical space.
 *
 * The mismatch narrowed to the logical cursor in whimory_cxt_parse_tree:
 * the runs land in the right VBAs and carry the wrong LBA labels. The
 * cursor is checked against each record's declared start LBA and never
 * disagreed, so whatever goes wrong is internal to a record and cancels out
 * by the end of it -- which cannot be reasoned about from summary counters.
 *
 * cxt_dump_lba names the window, cxt_dump_len its size. Every pair whose
 * logical extent touches it is printed with the cursor value it was given,
 * so the pair whose span does not match the run it describes is visible
 * directly.
 */
/*
 * Trace the first records of every CXT superblock walk.
 *
 * Two readings are left for the two newest checkpoints, whose records are
 * eight bytes of 0xff followed by file content, and they need opposite
 * fixes:
 *
 *   the walk is landing on the wrong page  -> an addressing bug here
 *   a record can be a header-only terminator followed by pre-erase
 *   contents                               -> accept it and keep walking
 *
 * The discriminator is whether the meta and the data come from the same
 * place. Printing each slot's own meta beside its own data head, with the
 * page's four metas alongside, settles it: a page whose slots all carry
 * SFTL_CXT meta while their data is XML is a page we should not be reading,
 * whereas a CXT page whose later slots hold stale content is a terminator.
 */
/*
 * Replay this virtual block even if the checkpoint says it is covered.
 *
 * The last hypothesis for the handful of files that will not read. Their
 * mapping comes from the checkpoint, is structurally sound, and points at a
 * page holding much older data -- so either the checkpoint entry is stale
 * and the correction lives in per-page meta that the skip threw away, or it
 * is not stale and the fault is elsewhere. Forcing one block through the
 * replay separates those without changing the rule for anything else.
 *
 * 0 disables. This is a test instrument, not a fix: if it works the fix is
 * to make the skip rule stop being wrong about blocks like it, not to keep
 * a hardcoded exception.
 */
static unsigned int force_replay_vblock;
module_param(force_replay_vblock, uint, 0644);
MODULE_PARM_DESC(force_replay_vblock,
		 "replay this vblock even when the CXT covers it (0 = off)");

static unsigned int cxt_trace_sb;
module_param(cxt_trace_sb, uint, 0644);
MODULE_PARM_DESC(cxt_trace_sb, "trace this many leading records of each CXT superblock");

static unsigned int cxt_dump_lba;
module_param(cxt_dump_lba, uint, 0644);
MODULE_PARM_DESC(cxt_dump_lba, "dump CXT tree pairs covering this LBA (0 = off)");

static unsigned int cxt_dump_len = 16384;
module_param(cxt_dump_len, uint, 0644);
MODULE_PARM_DESC(cxt_dump_len, "size of the cxt_dump_lba window");

static unsigned int cxt_dump_max = 96;
module_param(cxt_dump_max, uint, 0644);
MODULE_PARM_DESC(cxt_dump_max, "cap on dumped pairs");

static bool btoc_meta_fallback;
module_param(btoc_meta_fallback, bool, 0644);
MODULE_PARM_DESC(btoc_meta_fallback,
		 "Rebuild BTOC-less closed superblocks from per-page meta "
		 "(default N; overrides the CXT and corrupts the map when "
		 "the CXT is newer, which it is on N31)");

static unsigned int max_open_sbs = 4096;
module_param(max_open_sbs, uint, 0644);
MODULE_PARM_DESC(max_open_sbs,
		 "Max open superblocks to META-rebuild (0 = all; default 4096)");

/*
 * Scan every block. 256 was a bring-up limit that was never lifted, and
 * it quietly disabled the CXT fast path: the snapshot blocks sit above
 * that mark, so classify never saw one and every boot fell back to
 * replaying the whole device. Measured on this unit, 256 against 0:
 *
 *   replayed superblocks   624   ->      0
 *   skipped_by_cxt           0   ->   2274
 *   cxt_seeded               0   -> 239525
 *   classified_cxt           0   ->      4
 *   mapped_ranges         1933   -> 200000
 *
 * so it was not only slow, it was building a far less complete map.
 * Classifying more blocks costs less than replaying 624 superblocks.
 */
static unsigned int scan_blocks;
module_param(scan_blocks, uint, 0644);
MODULE_PARM_DESC(scan_blocks,
		 "User blocks per CE/CAU to classify (0 = all, the default)");

/*
 * BTOC meta-confirm re-reads every candidate data page over CS. Full-SB
 * confirm across closed BTOCs wedges USB (~2min then reset). Cap confirms;
 * open-META rebuild remains the bulk authority. 0 = unlimited (raise only
 * after correctness passes; stage with scan_blocks).
 */
static bool btoc_meta_confirm = true;
module_param(btoc_meta_confirm, bool, 0644);
MODULE_PARM_DESC(btoc_meta_confirm,
		 "CS-read data pages to take meta_lba as L2V key (default Y)");

/*
 * BTOC confirmation budget.
 *
 * This was 512, and on a full volume that is not a budget, it is a
 * truncation. Measured on an N31 with the same flash contents:
 *
 *              512          65536
 *   pages_valid      6            312
 *   l2v_updates   2048          97820
 *   confirm_capped 23943             0
 *   mapped_lbas   7468          77553
 *   mapped_ranges 1933          20596
 *
 * Every one of the extra confirmations was good -- meta_mismatch stayed
 * at 0 -- so the old default was discarding about nine tenths of the
 * mapping and reporting success while doing it. That is the same shape
 * of bug as the old max_range_nodes ceiling: the statistic sits exactly
 * on the limit, which means the limit chose the answer.
 *
 * This was reverted to 512 once, after a single run where the richer map
 * flipped BPB selection to the other candidate (49285) and the volume
 * would not mount. That revert was wrong. It let one observation override
 * a measurement, and the cost is not subtle: at 512 the volume mounts but
 * only about a tenth of it is mapped, so most files simply are not there.
 * On the boot that settled it, 65536 bound to 49279 and mounted with Apps,
 * iPod_Control and n31os all present.
 *
 * If BPB selection ever does pick an unmountable candidate again, fix the
 * selection -- it should prefer a candidate whose FAT actually reads --
 * rather than starving the map to steer it.
 *
 * 24455 confirm pages were needed here, so 65536 leaves headroom for a
 * fuller volume without being unbounded. Raise it (or set 0) if
 * btoc_confirm_capped is ever non-zero -- that value being non-zero is
 * the signal that recovery is being cut short.
 */
/*
 * Read page 127 only when page 0 leaves the question open.
 *
 * This is a large IO saving and it is why classify dropped from ~57s to
 * ~26s. It is also the kind of optimisation that can quietly change what
 * gets classified, so it is switchable: set btoc_page_lazy=0 to go back
 * to reading page 127 for every block and compare the classify totals.
 * If the two disagree, the laziness is wrong, not the flash.
 */
/*
 * Probe blocks for "empty" with a one-record read before doing the full
 * four-record page read. On by default: it is a strict reduction in NAND
 * traffic for the majority case and changes no classification, because a
 * negative probe falls through to exactly the old path.
 */
static bool fast_empty_probe = true;
module_param(fast_empty_probe, bool, 0644);

static bool batch_classify = true;
module_param(batch_classify, bool, 0644);
MODULE_PARM_DESC(batch_classify,
		 "classify page-0 scan reads 16 blocks per sequencer kick (default on)");
MODULE_PARM_DESC(fast_empty_probe,
		 "1=one-record empty probe before the full page read (default)");

static bool btoc_page_lazy = true;
module_param(btoc_page_lazy, bool, 0644);

static unsigned int read_prefetch_pages = WHIMORY_RC_SLOTS;
module_param(read_prefetch_pages, uint, 0644);
MODULE_PARM_DESC(read_prefetch_pages,
		 "physical pages held in the sequential read-ahead window, 0 to disable (default 8)");

/*
 * How far ahead to walk the VBAs when filling the window. 32 VBAs is one
 * 128 KiB readahead request, which at four LBAs per page is the eight pages
 * the window holds.
 */
static unsigned int read_prefetch_vbas = 32;
module_param(read_prefetch_vbas, uint, 0644);

/* Window behaviour, read-only. Guessing at this cost a 4x regression once. */
static unsigned int rc_fills, rc_hits, rc_misses, rc_fails, rc_pages;
module_param(rc_fills, uint, 0444);
module_param(rc_hits, uint, 0444);
module_param(rc_misses, uint, 0444);
module_param(rc_fails, uint, 0444);
module_param(rc_pages, uint, 0444);
MODULE_PARM_DESC(read_prefetch_vbas,
		 "VBAs to look ahead when filling the read-ahead window (default 32)");
MODULE_PARM_DESC(btoc_page_lazy,
		 "1=read page 127 only when page 0 is inconclusive (default); 0=always read it");

static unsigned int btoc_confirm_max = 65536;
module_param(btoc_confirm_max, uint, 0644);
MODULE_PARM_DESC(btoc_confirm_max,
		 "Max BTOC CS page confirms per recover (default 65536; 0=unlimited). Non-zero btoc_confirm_capped means this is too low");
/* Alias name from bring-up notes. */
module_param_named(btoc_confirm_pages_cap, btoc_confirm_max, uint, 0644);

/* Soft wall-clock budget for BTOC confirms (0 = ignore). */
static unsigned int recover_budget_ms;
module_param(recover_budget_ms, uint, 0644);
MODULE_PARM_DESC(recover_budget_ms,
		 "Stop BTOC confirms after this many ms (0=off)");

/* Keep RNDIS/USB alive during long recover (default 2ms every 4 blocks). */
/*
 * No sleep by default. This existed to keep RNDIS and the watchdog alive
 * during a long scan, but cond_resched() alone does that under
 * CONFIG_PREEMPT, and at 2 ms every fourth block the sleeps alone cost
 * several seconds of a mount that should take a few.
 */
static unsigned int recover_yield_us;
module_param(recover_yield_us, uint, 0644);
MODULE_PARM_DESC(recover_yield_us,
		 "usleep between classify blocks to keep USB alive (0=off; default 2000)");

/*
 * Interval-map node budget. N31 glass has ~55 MiB of RAM total; each
 * whimory_range is a kmalloc-64 slab object. Running out mid-recover is an
 * OOM panic (panic=-1 → reboot to RetailOS), which costs a DFU cycle and
 * loses the log. Stop adding mappings at the budget and report instead.
 */
/*
 * 200000 was below what this device actually needs, and the failure is
 * silent in the worst way: the map fills to exactly the ceiling and the
 * rest of the volume is simply absent.
 *
 * Measured here, with the CXT fast path working:
 *
 *   CXT_SEED extents=239525
 *   mapped_ranges=200000  range_budget_stop=39512
 *
 * so 39512 ranges were dropped on the floor. Reads past them return
 * UNMAPPED, FAT cannot fetch its directory blocks, and the mount comes
 * up with "invalid cluster chain" and missing folders -- which looks
 * like a corrupt disk rather than a driver that stopped writing down
 * where things are. L2V packing then fails with -12 and it falls back
 * to the truncated interval map.
 *
 * A whimory_range is about 32 bytes, so this ceiling is roughly 16 MB
 * if it were ever reached. It is a ceiling rather than an allocation:
 * nodes are only created for extents that exist, so a device with
 * fewer extents pays nothing for the headroom. Set well above this
 * unit's 239525 deliberately, so a fuller or more fragmented volume
 * on another device does not hit the same silent truncation.
 * Set 0 for unlimited.
 */
static unsigned int max_range_nodes = 500000;
module_param(max_range_nodes, uint, 0644);
MODULE_PARM_DESC(max_range_nodes,
		 "Interval-map node ceiling; stop mapping past it (0=unlimited)");

/*
 * The SFTL context is the authoritative FTL snapshot and only turns up
 * once scan_blocks reaches the high blocks that hold it (~1960 on N31).
 * It maps 600k+ LBAs in a few thousand coalesced ranges, but its VBAs do
 * not yet resolve (sftl lba mismatch → no BPB in L2V). Off skips both the
 * CXT load and the weave filter that suppresses replay of older SBs, so
 * the brute-force BTOC/open rebuild still yields a mountable disk.
 */
/*
 * Open-SB rebuild assigns each VBA its own weave, so adjacent LBAs landing
 * in adjacent VBAs never merged and the map cost one 64-byte rb node per
 * LBA — 46 MiB to replay every open SB on a 55 MiB device. Merging
 * neighbours that are contiguous in both LBA and VBA and keeping the OLDER
 * weave collapses sequentially written data; keeping the older weave means
 * a later claim is never wrongly rejected as stale, it just splits the
 * range again. 0 restores exact per-weave ranges.
 */
/*
 * Closed superblocks currently cost the same 127 CS reads as open ones:
 * the BTOC is used only to pick pages to re-read, and the per-slot meta is
 * taken as the L2V key. Before trusting the BTOC records outright, measure
 * whether they actually predict that metadata.
 */
static unsigned int btoc_verify_sbs;
module_param(btoc_verify_sbs, uint, 0644);
MODULE_PARM_DESC(btoc_verify_sbs,
		 "Closed SBs to probe for BTOC-vs-meta agreement (0=off)");

static unsigned int btoc_verify_pages = 6;
module_param(btoc_verify_pages, uint, 0644);
MODULE_PARM_DESC(btoc_verify_pages,
		 "Pages sampled per verified BTOC (default 6)");

/*
 * Console logging is not free on this target: every dev_info goes out the
 * serial console and measurably lengthens recover. Quiet by default; the
 * deep bring-up dumps come back with diag=1.
 */
static bool ftl_diag;
module_param_named(diag, ftl_diag, bool, 0644);
MODULE_PARM_DESC(diag,
		 "Per-LBA/BTOC/VBA diagnostic dumps (default N)");

static bool ftl_progress = true;
module_param_named(progress, ftl_progress, bool, 0644);
MODULE_PARM_DESC(progress, "Periodic recover progress lines (default Y)");

/*
 * 5 s suited a console log but is far too coarse to drive a progress
 * bar, which needs to move several times a second or it reads as hung.
 * This gates the sysfs counters as well as the log lines, so it is now
 * a UI refresh rate rather than a logging interval.
 */
static unsigned int progress_ms = 500;
module_param(progress_ms, uint, 0644);
MODULE_PARM_DESC(progress_ms, "Minimum ms between progress updates");

/* Rate-limit: emit at most this many of a repeating diagnostic. */
static unsigned int diag_max_lines = 3;
module_param(diag_max_lines, uint, 0644);

/*
 * Log the parts of each FPart system object that carry something.
 * Off: this is for reverse engineering the objects, not for boots.
 */
static bool fpart_dump;
module_param(fpart_dump, bool, 0644);
MODULE_PARM_DESC(fpart_dump,
		 "Dump non-fill rows of the FPart system objects (default N)");
MODULE_PARM_DESC(diag_max_lines, "Cap on repeated read-miss/winner lines");

/* True once per progress_ms window; keeps hot loops from flooding. */
static bool ftl_progress_due(struct whimory *w)
{
	unsigned long now = jiffies;

	if (!ftl_progress)
		return false;
	if (w->progress_jiffies &&
	    time_before(now, w->progress_jiffies + msecs_to_jiffies(progress_ms)))
		return false;
	w->progress_jiffies = now;
	return true;
}

static void ftl_progress_set(struct whimory *w, const char *phase,
			     unsigned int cur, unsigned int total)
{
	w->prog_phase = phase;
	w->prog_cur = cur;
	w->prog_total = total;
}

static bool range_coalesce = true;
module_param(range_coalesce, bool, 0644);
MODULE_PARM_DESC(range_coalesce,
		 "Merge LBA/VBA-contiguous ranges across weaves (default Y)");

static bool use_cxt = true;
module_param(use_cxt, bool, 0644);

/*
 * Mount from the checkpoint alone, as RetailOS does.
 *
 * s_cxt_load.c inserts every (vba, span) pair the context carries unless
 * the pair's superblock is set in diff->sbFilter:
 *
 *	sb = vba_to_sb(vba);
 *	if (vba >= invalid_vba || !sbFilter_test(sb))
 *		l2v_insert(lba, span, vba);
 *
 * and s_cxt_diff.c:405 allocates that filter with numRecords = 0 at init.
 * Nothing populates the record array during a load -- the diff machinery
 * belongs to the running FTL, where it tracks superblocks dirtied since the
 * last checkpoint so the next one can be incremental. At mount the filter
 * is empty, every pair is inserted, and no superblock is replayed.
 *
 * This driver keeps one safety net stock does not need: a superblock whose
 * newest page is at or after the checkpoint weave was written after the
 * checkpoint, so the checkpoint cannot describe it and it is replayed.
 * On a cleanly unmounted volume that set is empty and the whole replay
 * disappears.
 *
 * Clearing this restores the per-superblock weave test, which replays
 * anything whose bound did not come from page 127.
 */
/*
 * Scan superblock meta the way RetailOS does: first and last page queued
 * together, 256 records per sequencer kick, page data discarded.
 */
static bool stock_scan = true;
module_param(stock_scan, bool, 0644);
MODULE_PARM_DESC(stock_scan,
		 "classify reads first+last page meta in 256-record batches (default Y)");

static bool cxt_fast = true;
module_param(cxt_fast, bool, 0644);
MODULE_PARM_DESC(cxt_fast,
		 "mount from the checkpoint and replay only superblocks newer than it (default Y)");
MODULE_PARM_DESC(use_cxt,
		 "Load the SFTL CXT snapshot during recover (default Y)");

/*
 * The checkpoint is not an optimisation, it is the mount.
 *
 * Recovering from the checkpoint takes about eight seconds on this device;
 * rebuilding the same map by replaying every superblock takes about eight
 * minutes and reads roughly two hundred thousand pages to arrive at the
 * same answer. The full replay exists as an oracle -- it is built from
 * per-page metadata, so it is what a suspect checkpoint gets checked
 * against -- and as the thing to reach for when the checkpoint decode is
 * under suspicion. It is not a boot path.
 *
 * With this set, a checkpoint that will not load is an error and the
 * recover stops there. Clearing it restores the old silent fallback.
 */
static bool require_cxt = true;
module_param(require_cxt, bool, 0644);
MODULE_PARM_DESC(require_cxt,
		 "Fail the recover instead of full-replaying when the CXT will not load (default Y)");

/*
 * A full rebuild used to be re-runnable at any time, so a stray write to
 * ftl_sftl_recover could tear down a live map underneath a mounted disk.
 */
enum whimory_recovery_state {
	RECOVERY_NONE = 0,
	RECOVERY_RUNNING,
	RECOVERY_VALID,
	RECOVERY_FAILED,
};

static enum whimory_recovery_state recovery_state;
static u32 recovery_params_key;

static bool recover_force;
module_param(recover_force, bool, 0644);
MODULE_PARM_DESC(recover_force,
		 "Allow rebuild when a map is already valid/bound (default N)");

/*
 * Check each CXT extent against the page it points at before seeding it.
 * Off is faster and reproduces the old behaviour, at the cost of
 * trusting a snapshot that may predate the writes which moved its data.
 */
/*
 * Re-reading each extent's page meta to confirm the checkpoint is not
 * something RetailOS does: s_cxt_load.c inserts what the context says and
 * moves on. It costs one page read per extent -- 6276 on this volume --
 * and is kept only as a diagnostic.
 */
static bool cxt_meta_confirm;
module_param(cxt_meta_confirm, bool, 0644);
MODULE_PARM_DESC(cxt_meta_confirm,
		 "Confirm CXT extents against page metadata before seeding (default Y)");

static unsigned int cxt_confirm_max;
module_param(cxt_confirm_max, uint, 0644);
MODULE_PARM_DESC(cxt_confirm_max,
		 "Cap on CXT confirmation page reads (0 = no cap)");

/* Changing any of these is a different map, so a repeat is not a no-op. */
static u32 whimory_recover_key(void)
{
	return scan_blocks * 1000003u + max_open_sbs * 10007u +
	       btoc_confirm_max * 101u + (use_cxt ? 2u : 0u) + (cxt_meta_confirm ? 4u : 0u) +
	       (range_coalesce ? 1u : 0u) + max_range_nodes;
}

const char *whimory_recovery_state_name(void)
{
	switch (recovery_state) {
	case RECOVERY_RUNNING:
		return "running";
	case RECOVERY_VALID:
		return "valid";
	case RECOVERY_FAILED:
		return "failed";
	default:
		return "none";
	}
}
EXPORT_SYMBOL_GPL(whimory_recovery_state_name);

static bool audit_lba_winners;
module_param(audit_lba_winners, bool, 0644);
MODULE_PARM_DESC(audit_lba_winners,
		 "Log duplicate-LBA winner decisions for critical fmss LBAs");

/* Dump every winner decision for this absolute fmss_lba (0 = off). */
static unsigned int l2v_trace_lba;
module_param(l2v_trace_lba, uint, 0644);
MODULE_PARM_DESC(l2v_trace_lba,
		 "Log L2V winner old/new for this fmss_lba (0=off)");

/* Apply BTOC FFFF0001 LIST unmaps during recover (default Y). */
static bool btoc_apply_list = true;
module_param(btoc_apply_list, bool, 0644);
MODULE_PARM_DESC(btoc_apply_list,
		 "Apply BTOC LIST (FFFF0001) unmap payloads during recover");

static bool vba_page_dump;
module_param(vba_page_dump, bool, 0644);
MODULE_PARM_DESC(vba_page_dump,
		 "Emit VBA_DIAG sibling slot dump on critical reads (default N)");

/*
 * Off by default: 508 page reads that nothing acts on.
 *
 * This full-scans closed superblocks after the map is already built, page 0
 * to 126, and hands every page to whimory_note_meta0() -- which increments
 * meta0_hits and, unless diag is set, does nothing else. At four
 * superblocks that is 4 x 127 reads and about 0.8 seconds added to every
 * mount, to produce one number in RECOVERY_STATS.
 *
 * It was worth having while the question was "does anything on this volume
 * carry meta lba 0", and it is worth having again the next time that comes
 * up. It is not worth having on the boot path. Set it to 4 to get it back.
 */
static unsigned int meta0_scan_sbs;
module_param(meta0_scan_sbs, uint, 0644);
MODULE_PARM_DESC(meta0_scan_sbs,
		 "Closed SBs to full-scan for META lba=0 after recover (default 0 = off)");

static bool allow_sigless_debug;
module_param(allow_sigless_debug, bool, 0644);
MODULE_PARM_DESC(allow_sigless_debug,
		 "If true, classify/recover without xrmw (default N — NAND wedge / fake META)");

static unsigned int sig_scan_blocks;
module_param(sig_scan_blocks, uint, 0644);
MODULE_PARM_DESC(sig_scan_blocks,
		 "FPart assignment scan: tail blocks (0 = vfl_tail)");

static unsigned int fpart_assign_pages = 1;
module_param(fpart_assign_pages, uint, 0644);
MODULE_PARM_DESC(fpart_assign_pages,
		 "Pages per tail block to scan for META 0x30 assignment (default 1 = page0)");

static bool sig_brute_scan;
module_param(sig_brute_scan, bool, 0644);
MODULE_PARM_DESC(sig_brute_scan,
		 "META 0x30 page0 brute (default N — PIO spare is not Sogeti)");

static bool payload_magic_scan;
module_param(payload_magic_scan, bool, 0644);
MODULE_PARM_DESC(payload_magic_scan,
		 "Debug-only data xrmw/wrmx hunt (default N — FPart uses META 0x30)");

/* Kept so existing insmod lines do not fail. Recovery always runs at probe. */
static bool ftl_auto_map __maybe_unused;
module_param(ftl_auto_map, bool, 0644);
MODULE_PARM_DESC(ftl_auto_map, "ignored; Whimory always recovers at insmod");

static bool fpart_auto_scan __maybe_unused;
module_param(fpart_auto_scan, bool, 0644);
MODULE_PARM_DESC(fpart_auto_scan, "ignored; host slices created after LBA0");

static struct whimory *whimory_dev;
static struct platform_device *ftl_pdev;

static void whimory_l2v_find_frag(struct whimory *w);
static void whimory_l2v_cache_flush(struct whimory *w);
static void whimory_l2v_free_tree(struct whimory *w, u32 node_idx, u32 root_idx);
static int whimory_l2v_update_packed(struct whimory *w, u32 ridx, u32 off,
				     u32 span, u32 vba);
static int n31_sftl_read_lba(struct whimory *w, u32 lba, void *buf,
			     bool allow_blank);
static bool whimory_audit_fmss_lba(u32 fmss_lba);
static void whimory_note_payload_strings(struct whimory *w, const u8 *data,
					 unsigned int len);
static void whimory_dump_vba_page(struct whimory *w, u32 vba, u32 fmss_lba);

static u64 whimory_weave48(const u8 *m)
{
	return (u64)get_unaligned_le16(m + 2) |
	       ((u64)get_unaligned_le32(m + 4) << 16);
}

/*
 * CS span4/rec4112 page read — real 4× META (glass-proven). Used for
 * classify / BTOC / open-SB / CXT / VBA reads when meta_dma_read=0.
 */
static int whimory_cs_read_page(struct whimory *w, unsigned int ce,
				unsigned int cau, unsigned int block,
				unsigned int page, void *data, size_t data_len,
				void *meta, size_t meta_len)
{
	struct s5l8740_cs_page *csp;
	unsigned int s;
	int ret;

	if (!w || !data || data_len < S5L8740_NAND_PAGE_SIZE)
		return -EINVAL;
	csp = w->sftl.cs_page;
	if (!csp)
		return -ENOMEM;

	ret = s5l8740_nand_cs_phys_read((u8)ce, (u8)cau, (u16)block, (u8)page,
					csp);
	if (ret)
		return ret;

	for (s = 0; s < S5L8740_NAND_SLOTS_PER_PAGE; s++)
		memcpy((u8 *)data + s * S5L8740_NAND_SLOT_DATA,
		       csp->data[s], S5L8740_NAND_SLOT_DATA);

	if (meta && meta_len) {
		size_t copy = min_t(size_t, meta_len, S5L8740_NAND_META_SIZE);

		memset(meta, 0xff, meta_len);
		for (s = 0; s < S5L8740_NAND_SLOTS_PER_PAGE &&
			     (s + 1) * WHIMORY_META_SIZE <= copy; s++)
			memcpy((u8 *)meta + s * WHIMORY_META_SIZE,
			       csp->meta_raw[s], WHIMORY_META_SIZE);
	}
	return 0;
}

static bool whimory_meta_any_btoc(const u8 *meta64)
{
	unsigned int s;

	if (!meta64)
		return false;
	for (s = 0; s < WHIMORY_VBAS_PER_PAGE; s++) {
		if (meta64[s * WHIMORY_META_SIZE] == WHIMORY_META_TYPE_BTOC)
			return true;
	}
	return false;
}

static bool whimory_meta_slot0_or_any_cxt(const u8 *meta64)
{
	unsigned int s;

	if (!meta64)
		return false;
	if (meta64[0] == WHIMORY_META_TYPE_SFTL_CXT)
		return true;
	for (s = 1; s < WHIMORY_VBAS_PER_PAGE; s++) {
		if (meta64[s * WHIMORY_META_SIZE] == WHIMORY_META_TYPE_SFTL_CXT)
			return true;
	}
	return false;
}

static bool whimory_page_blank(const u8 *p, unsigned int n)
{
	unsigned int i;
	u8 all_ff = 0xff, all_00 = 0;

	if (!p || !n)
		return true;
	for (i = 0; i < n; i++) {
		all_ff &= p[i];
		all_00 |= p[i];
	}
	return all_ff == 0xff || all_00 == 0;
}

static bool whimory_meta_erased(const u8 *m, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (m[i] != 0xff)
			return false;
	}
	return true;
}

/*
 * Cheap "is this block empty" probe.
 *
 * classify visits every block and, for seventy percent of them, only needs
 * to answer "empty?" -- 64 bytes of slot-0 data and slot-0's 16-byte meta.
 * The full page read moves 16448 bytes to obtain those 80. This reads one
 * record, 4112 bytes.
 *
 * Deliberately narrow: it answers empty/not-empty and nothing else. Every
 * other question classify asks, CXT detection above all, reads all four
 * slots' meta, so a "no" here must be followed by the full read. Returning
 * a bool rather than filling meta0 makes that impossible to get wrong by
 * accident.
 */
/*
 * Read slot 0 of a page: 4112 bytes instead of 16448.
 *
 * The controller takes a column offset and length -- col_len packs the
 * length in the low half and the start column in the high half -- so a
 * one-record read is a genuine short transfer, not a full page quietly
 * discarded. Slot 0 carries the first 4096 data bytes and its own 16-byte
 * meta, which between them settle almost every question classify asks.
 *
 * The exception is whimory_meta_slot0_or_any_cxt(), which scans all four
 * slots, so a meta this read cannot classify must escalate to the full
 * page. That is rare: on this volume the meta type histogram is 5492
 * erased, 2268 data, 2 data2, 2 at 0x4b and 76 zero, so slot 0 answers for
 * all but the last hundred or so.
 *
 * Returns 0 with data and meta filled, or negative on a read error.
 */
static int whimory_cs_read_slot0(struct whimory *w, unsigned int ce,
				 unsigned int cau, unsigned int block,
				 unsigned int page, const u8 **data,
				 u8 *meta0)
{
	struct s5l8740_cs_page *csp = w->sftl.cs_page;
	unsigned int i;
	int ret;

	if (!csp)
		return -ENOMEM;
	ret = s5l8740_nand_cs_phys_read_slot0((u8)ce, (u8)cau, (u16)block,
					      (u8)page, csp);
	if (ret)
		return ret;
	for (i = 0; i < WHIMORY_META_SIZE; i++)
		meta0[i] = csp->meta_raw[0][i];
	*data = csp->data[0];
	return 0;
}

/*
 * Batched page-0 prefetch for the classify scan.
 *
 * Classify reads page 0 of every block on the volume -- about 7840 of them
 * -- and does nothing between one read and the next that depends on the
 * previous answer. That makes it the one pass where the per-kick cost is
 * pure waste: measured on the device at kick_us=1536 against a NAND tR of
 * 60-80 us, roughly 1.45 ms of every 1.54 ms read is the sequencer being
 * set up and torn down for a single page.
 *
 * So set it up once for sixteen. The window slides forward through the
 * block range and every read the loop asks for is served from it, including
 * the full 4 KiB slot the escalation path wants -- there is no point
 * batching the blank test and then reading the page again for the blocks
 * that are not blank.
 *
 * Refill failure is not fatal. batch_valid goes false and the caller falls
 * back to the single-page path for that window, which is what the scan did
 * before any of this existed.
 */
static void whimory_prefetch_reset(struct whimory *w)
{
	w->sftl.pf_valid = false;
	w->sftl.pf_count = 0;
}

static int whimory_prefetch_slot0(struct whimory *w, unsigned int ce,
				  unsigned int cau, unsigned int block,
				  unsigned int nscan, const u8 **data,
				  u8 *meta0)
{
	struct whimory_sftl *s = &w->sftl;
	u16 blocks[WHIMORY_PF_SLOTS];
	unsigned int i, n, idx;
	int ret;

	if (!batch_classify || !s->pf_data)
		goto fallback;

	if (!(s->pf_valid && ce == s->pf_ce && cau == s->pf_cau &&
	      block >= s->pf_first && block < s->pf_first + s->pf_count)) {
		n = nscan - block;
		if (n > WHIMORY_PF_SLOTS)
			n = WHIMORY_PF_SLOTS;
		if (n < 1)
			goto fallback;
		for (i = 0; i < n; i++)
			blocks[i] = (u16)(block + i);
		ret = s5l8740_nand_cs_read_meta_batch((u8)ce, (u8)cau, blocks,
						      0, n, s->pf_meta,
						      s->pf_data,
						      S5L8740_NAND_SLOT_DATA);
		if (ret) {
			/*
			 * Say so once per mount rather than per window. A
			 * batch that cannot run is a speed regression, not a
			 * correctness one, and the fallback below is the
			 * path the scan used before batching existed.
			 */
			if (!s->pf_failed) {
				s->pf_failed = true;
				dev_warn(w->dev,
					 "SFTL batch prefetch failed (%d), classify falls back to single-page reads\n",
					 ret);
			}
			whimory_prefetch_reset(w);
			goto fallback;
		}
		s->pf_ce = ce;
		s->pf_cau = cau;
		s->pf_first = block;
		s->pf_count = n;
		s->pf_valid = true;
		s->pf_kicks++;

	/*
	 * Prove the descriptor walk once, on the first window of the mount.
	 *
	 * The list format is read out of the 4EDDDC decomp, and the one thing
	 * the decomp does not answer is whether the CS microcode blob we load
	 * walks a multi-descriptor list or executes the first pair and stops
	 * at the terminator. If it stops, every entry past index 0 is stale
	 * buffer rather than the block that was asked for -- and a classify
	 * pass that believes it would mismarks most of the volume.
	 *
	 * So the second entry gets read again the slow way and compared. One
	 * extra page read per mount buys the right to default this on; a
	 * mismatch turns batching off for the rest of the mount and the scan
	 * carries on down the path it used before.
	 */
	if (!s->pf_checked && n >= 2) {
		const u8 *vd = NULL;
		u8 vm[WHIMORY_META_SIZE];

		s->pf_checked = true;
		/*
		 * Only a read that succeeds and disagrees is evidence. A
		 * verification read that fails outright says nothing about
		 * the batch -- dma_one_shot disarms CS after a single read,
		 * so a bare -EPERM here is routine and it comes back as an
		 * all-zero meta, which is indistinguishable from a real
		 * disagreement if you only compare bytes. Treating that as a
		 * mismatch would switch batching off on every mount.
		 */
		if (!whimory_cs_read_slot0(w, ce, cau, block + 1, 0, &vd, vm) &&
		    (memcmp(vm, s->pf_meta + S5L8740_NAND_BATCH_META_SIZE,
			    WHIMORY_META_SIZE) ||
		     memcmp(vd, s->pf_data + S5L8740_NAND_SLOT_DATA, 64))) {
			dev_warn(w->dev,
				 "SFTL batch prefetch does not match single-page reads at blk=%u -- the CS blob is not walking the descriptor list; batching off\n",
				 block + 1);
			batch_classify = false;
			whimory_prefetch_reset(w);
			goto fallback;
		}
		dev_info(w->dev,
			 "SFTL batch prefetch verified against single-page reads, %u blocks per kick\n",
			 n);
	}

	}

	idx = block - s->pf_first;
	memcpy(meta0, s->pf_meta + idx * S5L8740_NAND_BATCH_META_SIZE,
	       WHIMORY_META_SIZE);
	*data = s->pf_data + (size_t)idx * S5L8740_NAND_SLOT_DATA;
	s->pf_hits++;
	return 0;

fallback:
	return whimory_cs_read_slot0(w, ce, cau, block, 0, data, meta0);
}


/*
 * Can slot 0 alone classify this block?
 *
 * The only thing classify needs the other three slots for is
 * whimory_meta_slot0_or_any_cxt(), which exists to catch a partially
 * written CXT page whose slot 0 never got its meta. So the question is
 * narrower than it looks: does this slot-0 type rule out a CXT hiding in
 * slots 1..3?
 *
 * Plain user data and a slot-0 CXT do. A page is written slot 0 first with
 * one kind of content, so DATA in slot 0 means the page is user data, and
 * SFTL_CXT in slot 0 answers the CXT question outright.
 *
 * Everything else escalates, deliberately -- an erased or zero or
 * unrecognised slot-0 meta is exactly the partial-write shape that
 * _or_any_cxt() was written for, and misreading one of those loses a
 * checkpoint. On this volume that leaves about eighty blocks paying for a
 * full read against roughly 2270 settling from slot 0.
 */
static bool whimory_meta0_is_conclusive(const u8 *meta0)
{
	switch (meta0[0]) {
	case WHIMORY_META_TYPE_DATA:
	case WHIMORY_META_TYPE_DATA2:
	case WHIMORY_META_TYPE_SFTL_CXT:
		return true;
	default:
		return false;
	}
}


static bool whimory_meta_is_user_data(const struct whimory_meta *m)
{
	return m->type == WHIMORY_META_TYPE_DATA ||
	       m->type == WHIMORY_META_TYPE_DATA2;
}

static bool whimory_meta_is_cxt_base(const u8 *m, u32 vba_ofs)
{
	return m[0] == WHIMORY_META_TYPE_SFTL_CXT &&
	       m[1] == WHIMORY_CXT_TAG_BASE &&
	       vba_ofs == 0;
}

static bool whimory_meta_is_data_raw(const u8 *m)
{
	return m[0] == WHIMORY_META_TYPE_DATA ||
	       m[0] == WHIMORY_META_TYPE_DATA2;
}

/*
 * Does this metadata belong to a block that is part of a superblock?
 *
 * Membership from one record, for the blocks above user_blocks that the
 * classify loop does not enumerate but that VBAs still name. Data, a BTOC
 * and a checkpoint record all say the FTL allocated this block to a
 * superblock; an erased record says it did not, and so does the all-zero
 * type 00 shape that 76 blocks on this unit carry at both page 0 and page
 * 127.
 */
static bool whimory_meta_is_member(const u8 *m)
{
	if (whimory_meta_erased(m, WHIMORY_META_SIZE))
		return false;
	/*
	 * One 16-byte record, not a 64-byte page spare -- the batched plane
	 * scan returns a single meta per (block, page), so the four-slot
	 * helpers cannot be used here without reading past the end of it.
	 */
	return whimory_meta_is_data_raw(m) ||
	       m[0] == WHIMORY_META_TYPE_BTOC ||
	       m[0] == WHIMORY_META_TYPE_SFTL_CXT;
}

static bool whimory_special_lba(u32 lba)
{
	return (lba & 0xFFFF0000u) == WHIMORY_SPECIAL_LBA ||
	       lba == WHIMORY_LBA_BLANK || lba == WHIMORY_LBA_DELETED;
}

static u32 whimory_vfl_phys(struct whimory *w, u32 cau, u32 virt)
{
	/*
	 * The N31 VFL is an identity map over blocks_per_cau: physical
	 * block number equals virtual block number.
	 *
	 * The u16 table at CXT +0x200 looks like a remap table but is not
	 * one — it is the VFL context copy journal, where each value is
	 * index | (generation << 15) and 0xC070 marks a free slot. A failed
	 * user block keeps its VBN and moves to another CAU instead, which
	 * the bank bitmap records.
	 */
	if (cau >= w->geom.num_cau || !w->vfl.remap[cau])
		return virt;
	if (virt >= w->geom.blocks_per_cau)
		return virt;
	return w->vfl.remap[cau][virt];
}

/*
 * Which CAUs carry this virtual block, for the CAU-substitution rule.
 *
 * This is not superblock bank membership, and reading it as if it were is
 * what let the address decode assume four banks everywhere. Its domain is
 * CAUs -- num_cau entries, no CE in it at all -- and it answers only "if
 * this block failed on its own CAU, which CAU holds it now". vfl.bank_mask
 * behind it is still a stub: allocated all-ones in n31_vfl_init() and never
 * filled from flash, which makes whimory_vfl_bank() the identity and is why
 * nothing has ever depended on it.
 *
 * The count that address arithmetic needs -- how many (ce, cau) banks a
 * superblock spans -- is sftl.sb_bank_mask, built by classify. See
 * whimory_sb_banks().
 */
static u32 whimory_vfl_banks_in_vbn(struct whimory *w, u32 vbn, u8 *out,
				    u32 out_max)
{
	u8 mask;
	u32 n = 0, b;

	if (w->vfl.cached_vbn == (u16)vbn && w->vfl.cached_n) {
		n = min_t(u32, w->vfl.cached_n, out_max);
		if (out)
			memcpy(out, w->vfl.cached_banks, n);
		return w->vfl.cached_n;
	}
	mask = 0;
	if (w->vfl.bank_mask && vbn < w->geom.blocks_per_cau) {
		u32 stride = w->vfl.bank_stride ? w->vfl.bank_stride : 1;
		const u8 *row = w->vfl.bank_mask + stride * vbn;
		u32 b;

		for (b = 0; b < w->geom.num_cau && b < 8; b++) {
			u32 bi = b >> 3;

			if (bi < stride && (row[bi] & (1u << (b & 7))))
				mask |= (u8)(1u << b);
		}
	}
	if (!mask)
		mask = (1u << w->geom.num_cau) - 1;
	for (b = 0; b < w->geom.num_cau && b < 8; b++) {
		if (!(mask & (1u << b)))
			continue;
		if (out && n < out_max)
			out[n] = (u8)b;
		if (n < S5L8740_NAND_MAX_CAU)
			w->vfl.cached_banks[n] = (u8)b;
		n++;
	}
	w->vfl.cached_vbn = (u16)vbn;
	w->vfl.cached_n = (u8)n;
	return n;
}

static u32 whimory_vfl_bank(struct whimory *w, u32 cau, u32 vblock)
{
	u8 banks[S5L8740_NAND_MAX_CAU];
	u32 n, i;

	n = whimory_vfl_banks_in_vbn(w, vblock, banks, ARRAY_SIZE(banks));
	if (!n)
		return cau;
	for (i = 0; i < n; i++) {
		if (banks[i] == (u8)cau)
			return cau;
	}
	return banks[0];
}

static u32 whimory_vfl_virt(struct whimory *w, u32 cau, u32 phys)
{
	u32 i, n;

	if (cau >= w->geom.num_cau || !w->vfl.remap[cau])
		return phys;
	n = w->geom.blocks_per_cau;
	for (i = 0; i < n; i++) {
		if (w->vfl.remap[cau][i] == phys)
			return i;
	}
	return phys;
}

/*
 * Removed with the move to the native VBA space: nothing builds a VBA from
 * a bank-major superblock index any more. whimory_sb_ofs_to_vba() is the
 * replacement and converts at the boundary instead.
 */

/*
 * Resolve a (ce, cau, vblock) triple to the hardware that holds it.
 *
 * Two steps, and they have to happen in this order: whimory_vfl_bank()
 * answers which CAU actually carries this virtual block, and only then does
 * whimory_vfl_phys() translate the block number within that CAU.
 *
 * n31_vfl_read_vba() and whimory_l2v_search_phys() did both. The BTOC
 * confirm pass and the checkpoint walk did only the second, with the
 * unremapped CAU -- so if the bank bitmap were ever populated, the confirm
 * pass would key the L2V on metadata read from one plane while every later
 * read fetched another, a self-inflicted lba mismatch across the whole map.
 * Both helpers are the identity while vfl_remap_mode=off, which is why it
 * has never bitten; one helper means it cannot start.
 */
static void whimory_vfl_resolve(struct whimory *w, u32 vblock, u32 *cau,
				u32 *pblock)
{
	*cau = whimory_vfl_bank(w, *cau, vblock);
	*pblock = whimory_vfl_phys(w, *cau, vblock);
}

/*
 * The number of banks a superblock may have, and the VBA stride of one
 * virtual block.
 *
 * The stride is the *maximum* -- max_banks * pages_per_sb * vbas_per_page,
 * 2048 here -- not the number a given superblock uses. s_vfl.c builds an
 * address as
 *
 *	(vbn * max_banks * pages_per_sb + page * nbanks + bank_ofs)
 *		* vbas_per_page + slot
 *
 * (sub_4EAD34 forward, sub_4EAE40 back), so every virtual block owns the
 * same 2048-address window and a short superblock simply leaves the tail of
 * its window unused. Confirmed on this unit: CXT extents land exactly on
 * vbn * 2048 boundaries -- vba 0xa0000 is vblock 320 page 0, 0x9d800 is
 * vblock 315 page 0.
 */
static u32 whimory_max_banks(const struct whimory *w)
{
	u32 n = w->geom.num_ce * w->geom.num_cau;

	return n ? n : 1;
}

static u32 whimory_vbas_per_vblock(const struct whimory *w)
{
	return whimory_max_banks(w) * w->sftl.pages_per_sb *
	       w->sftl.vbas_per_page;
}

/*
 * The banks this virtual block spans, and how many.
 *
 * An empty mask means classify never reached this block -- the region
 * above user_blocks, or a run cut short by scan_blocks. Those fall back to
 * every bank, which is what this driver assumed everywhere before the map
 * existed, so an unscanned region behaves exactly as it used to.
 */
static u32 whimory_sb_banks(const struct whimory *w, u32 vblock, u8 *mask_out)
{
	u32 maxb = whimory_max_banks(w);
	u8 mask = 0;

	if (w->sftl.sb_bank_mask && vblock < w->sftl.sb_bank_blocks)
		mask = w->sftl.sb_bank_mask[vblock];
	if (!mask)
		mask = (u8)((1u << maxb) - 1);
	if (mask_out)
		*mask_out = mask;
	return hweight8(mask);
}

/* The bank sitting at position idx of a mask, counting from bit 0. */
static u32 whimory_bank_at(u8 mask, u32 idx)
{
	u32 b;

	for (b = 0; b < 8; b++) {
		if (!(mask & (1u << b)))
			continue;
		if (!idx)
			return b;
		idx--;
	}
	return 0;
}

/*
 * Record that (ce, cau) carries this virtual block.
 *
 * Called from classify for every block that came back as a real superblock
 * member -- closed, open, or CXT. Empty blocks and blocks nothing could
 * recognise deliberately do not call this: on this unit the 76 blocks whose
 * page 0 and page 127 both read as type 00 are exactly the banks their
 * superblocks do not span, and they are what the classify histogram has
 * been reporting as "unknown" all along.
 */
static void whimory_sb_bank_note(struct whimory *w, u32 vblock, u32 ce,
				 u32 cau)
{
	u32 bank = ce * w->geom.num_cau + cau;

	if (!w->sftl.sb_bank_mask || vblock >= w->sftl.sb_bank_blocks ||
	    bank >= 8)
		return;
	w->sftl.sb_bank_mask[vblock] |= (u8)(1u << bank);
}

/* Where a bank sits in a mask, or -1 when it is not a member. */
static int whimory_bank_index(u8 mask, u32 bank)
{
	u32 b, n = 0;

	for (b = 0; b < 8 && b < bank; b++)
		if (mask & (1u << b))
			n++;
	if (bank >= 8 || !(mask & (1u << bank)))
		return -1;
	return (int)n;
}

static u32 s_g_vba_to_sb(const struct whimory *w, u32 vba)
{
	u32 per_vb = whimory_vbas_per_vblock(w);

	/*
	 * Divide by the per-virtual-block stride, not by sftl.vbas_per_sb.
	 * vbas_per_sb is 512, the per-plane count the BTOC parsers size
	 * their arrays with; VBAs are packed over all four banks, so using
	 * it here reported a superblock index four times too large in every
	 * VBA_DIAG line.
	 */
	if (!per_vb)
		return 0;
	return vba / per_vb;
}

static u32 s_g_vba_to_ofs(const struct whimory *w, u32 vba)
{
	u32 per_vb = whimory_vbas_per_vblock(w);

	if (!per_vb)
		return 0;
	return vba % per_vb;
}

static u32 whimory_sb_index(const struct whimory *w, u32 ce, u32 cau,
			    u32 vblock)
{
	return (ce * w->geom.num_cau + cau) * w->sftl.user_blocks + vblock;
}

/*
 * The VBA space is the FTL's native one.
 *
 * Apple treats a superblock as the same virtual block across every
 * (ce, cau) plane, so the plane index sits between the page and the slot:
 *
 *   vba = vblock * (pages_per_sb * planes * vbas_per_page)
 *       + page   * (planes * vbas_per_page)
 *       + plane  * vbas_per_page
 *       + slot
 *
 * This used to be bank-major -- every (ce, cau, vblock) triple got its own
 * superblock index -- which meant CXT VBAs had to be translated on the way
 * in, and a run of consecutive CXT VBAs was only contiguous here within one
 * 4-slot group, because the next group belonged to a different plane. The
 * cost of that was not subtle: the CXT seed produced 236675 ranges for
 * 938395 LBAs, just under four LBAs per range, when the same data in the
 * native space is a few thousand contiguous runs.
 *
 * Matching the native layout removes the translation entirely and lets
 * extents stay whole, which is what the range budget was fighting.
 */
/*
 * How many virtual blocks a VBA may name.
 *
 * Not user_blocks. user_blocks is blocks_per_cau minus the VFL tail -- 1960
 * of 2088 here -- and it is the right number for "how much space may be
 * allocated to the user". It is the wrong number for "which blocks may a
 * stored VBA refer to", and using it as the range check silently discarded
 * 81 CXT records:
 *
 *   CXT_XLATE_FAIL vba=0x003e3883 (vblk=1991 pg=8 plane=0 slot=3)
 *                  lba=841408 span=256 user_blocks=1960
 *
 * Those are not corrupt entries. They cluster in vblocks 1987..1991, their
 * spans are large and ordinary (93, 116, 125, 128, 256, 384), and their VBAs
 * run contiguously through the plane interleave exactly as the arithmetic
 * predicts -- 0x3e3883 + 256 lands on 0x3e3983 and the next record begins at
 * 0x3e3984. That is the FTL telling us, correctly, where it put roughly two
 * thousand LBAs around 839k-842k. We were throwing them away.
 *
 * The VFL is an identity map over blocks_per_cau, so any block below that is
 * addressable and a VBA naming one is legitimate.
 */
static u32 whimory_vba_blocks(const struct whimory *w)
{
	if (w->geom.blocks_per_cau)
		return w->geom.blocks_per_cau;
	return w->sftl.user_blocks;
}

static u32 whimory_pack_vba(const struct whimory *w, u32 ce, u32 cau,
			    u32 vblock, u32 page, u32 slot)
{
	u32 bank = ce * w->geom.num_cau + cau;
	u32 per_vb = whimory_vbas_per_vblock(w);
	u8 mask;
	u32 nbanks = whimory_sb_banks(w, vblock, &mask);
	int ofs = whimory_bank_index(mask, bank);

	/*
	 * A bank the map does not list as a member of this superblock.
	 *
	 * That is either a block classify could not read or a genuine
	 * derivation error, and either way the honest answer is the layout
	 * this driver used before the map existed: all banks, dense. It
	 * keeps pack and unpack mutual inverses, which is what the replay
	 * paths depend on -- they store an address and read it straight
	 * back, so a wrong bank count cancels for them and only the
	 * FTL-authored addresses (CXT extents, BTOC arrays) can go wrong.
	 */
	if (ofs < 0) {
		nbanks = whimory_max_banks(w);
		ofs = (int)bank;
	}

	return vblock * per_vb +
	       (page * nbanks + (u32)ofs) * w->sftl.vbas_per_page + slot;
}

/*
 * Build a VBA from a bank-major superblock index and an in-superblock
 * offset. The replay paths still enumerate one (ce, cau, vblock) at a
 * time, which is a bank-major idea; this converts at the boundary so the
 * stored VBA is native.
 */
static u32 whimory_sb_ofs_to_vba(const struct whimory *w, u32 sb_idx, u32 ofs)
{
	u32 per_ce = w->geom.num_cau * w->sftl.user_blocks;
	u32 ce, cau, vblock, rem;

	if (!per_ce || !w->sftl.user_blocks || !w->sftl.vbas_per_page)
		return 0;
	ce = sb_idx / per_ce;
	rem = sb_idx % per_ce;
	cau = rem / w->sftl.user_blocks;
	vblock = rem % w->sftl.user_blocks;

	return whimory_pack_vba(w, ce, cau, vblock,
				ofs / w->sftl.vbas_per_page,
				ofs % w->sftl.vbas_per_page);
}

/* The virtual block a bank-major superblock index names. */
static u32 whimory_cxt_sb_vblock(const struct whimory *w, u32 sb_idx)
{
	u32 per_ce = w->geom.num_cau * w->sftl.user_blocks;

	if (!per_ce || !w->sftl.user_blocks)
		return 0;
	return (sb_idx % per_ce) % w->sftl.user_blocks;
}


static int whimory_unpack_vba(const struct whimory *w, u32 vba,
			      u32 *ce, u32 *cau, u32 *vblock,
			      u32 *page, u32 *slot)
{
	if (!w->sftl.vbas_per_sb || !w->sftl.vbas_per_page ||
	    !w->sftl.user_blocks)
		return -EINVAL;
	/* Exact inverse of whimory_pack_vba(); see the layout there. */
	{
		u32 per_vb = whimory_vbas_per_vblock(w);
		u32 rem, unit, nbanks, bank;
		u8 mask;

		if (!per_vb)
			return -EINVAL;

		*vblock = vba / per_vb;
		rem = vba % per_vb;
		*slot = rem % w->sftl.vbas_per_page;
		unit = rem / w->sftl.vbas_per_page;

		nbanks = whimory_sb_banks(w, *vblock, &mask);
		*page = unit / nbanks;
		bank = whimory_bank_at(mask, unit % nbanks);
		*ce = bank / w->geom.num_cau;
		*cau = bank % w->geom.num_cau;
	}

	if (*ce >= w->geom.num_ce || *cau >= w->geom.num_cau)
		return -ERANGE;
	if (*vblock >= whimory_vba_blocks(w))
		return -ERANGE;
	if (*page >= w->sftl.pages_per_sb)
		return -ERANGE;
	return 0;
}

static void whimory_set_status(struct whimory *w, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(w->status, sizeof(w->status), fmt, ap);
	va_end(ap);
}

/* ------------------------------------------------------------------ */
/* Interval map: weave-order LBA→VBA, then packed into the L2V tree. */
/* ------------------------------------------------------------------ */

static struct whimory_range *whimory_range_find(struct rb_root *root, u32 lba)
{
	struct rb_node *n = root->rb_node;

	while (n) {
		struct whimory_range *r = rb_entry(n, struct whimory_range, rb);

		if (lba < r->start)
			n = n->rb_left;
		else if (lba >= r->start + r->len)
			n = n->rb_right;
		else
			return r;
	}
	return NULL;
}

/*
 * Leftmost range that can overlap [lba, ...) — i.e. the first with
 * start + len > lba. Ranges are disjoint and keyed by start, so r_end is
 * monotonic in tree order and the predicate is a valid binary search.
 *
 * Callers used to walk from rb_first(), which made every L2V update
 * O(ranges) and the whole recover O(ranges^2). At wide scan_blocks that
 * spins the CPU long enough to starve RNDIS and trip the watchdog.
 */
static struct whimory_range *whimory_range_lower(struct rb_root *root, u32 lba)
{
	struct rb_node *n = root->rb_node;
	struct whimory_range *best = NULL;

	while (n) {
		struct whimory_range *r = rb_entry(n, struct whimory_range, rb);

		if (r->start + r->len > lba) {
			best = r;
			n = n->rb_left;
		} else {
			n = n->rb_right;
		}
	}
	return best;
}

static int whimory_range_link(struct rb_root *root, struct whimory_range *n)
{
	struct rb_node **link = &root->rb_node, *parent = NULL;

	while (*link) {
		struct whimory_range *r = rb_entry(*link, struct whimory_range,
						   rb);

		parent = *link;
		if (n->start < r->start)
			link = &(*link)->rb_left;
		else
			link = &(*link)->rb_right;
	}
	rb_link_node(&n->rb, parent, link);
	rb_insert_color(&n->rb, root);
	return 0;
}

static int whimory_range_split(struct whimory *w, struct whimory_range *r,
			       u32 at)
{
	struct whimory_range *right;
	u32 left_len;

	if (at <= r->start || at >= r->start + r->len)
		return 0;
	right = kzalloc(sizeof(*right), GFP_KERNEL);
	if (!right)
		return -ENOMEM;
	left_len = at - r->start;
	right->start = at;
	right->len = r->len - left_len;
	right->vba = r->vba + left_len;
	right->weave = r->weave;
	right->src = r->src;
	r->len = left_len;
	whimory_range_link(&w->ranges, right);
	w->sftl.range_nodes++;
	w->sftl.map_gen++;
	return 0;
}

static void whimory_range_erase(struct whimory *w, struct whimory_range *r)
{
	rb_erase(&r->rb, &w->ranges);
	kfree(r);
	w->sftl.map_gen++;
	if (w->sftl.range_nodes)
		w->sftl.range_nodes--;
}

static int whimory_range_insert_new(struct whimory *w, u32 start, u32 len,
				    u32 vba)
{
	struct whimory_range *n;

	if (!len)
		return 0;
	/*
	 * Backstop. whimory_range_update() checks the budget before it
	 * splits or erases anything, so reaching here means a caller went
	 * around it. Returning 0 -- success, having inserted nothing -- is
	 * what turned working mappings into holes; say -ENOSPC instead so a
	 * new caller fails loudly rather than silently truncating the map.
	 */
	if (max_range_nodes && w->sftl.range_nodes >= max_range_nodes) {
		w->sftl.range_budget_stop++;
		return -ENOSPC;
	}
	n = kzalloc(sizeof(*n), GFP_KERNEL);
	if (!n)
		return -ENOMEM;
	n->start = start;
	n->len = len;
	n->vba = vba;
	n->weave = w->sftl.claim_weave;
	n->src = w->sftl.claim_source;
	whimory_range_link(&w->ranges, n);
	w->sftl.range_nodes++;
	w->sftl.map_gen++;
	return 0;
}

/* Merge r with either neighbour when both LBA and VBA stay contiguous. */
static bool whimory_range_joinable(const struct whimory_range *a,
				   const struct whimory_range *b)
{
	if (a->start + a->len != b->start || a->vba + a->len != b->vba)
		return false;
	return range_coalesce || a->weave == b->weave;
}

static void whimory_range_coalesce_at(struct whimory *w, u32 start)
{
	struct whimory_range *r, *prev, *next;
	struct rb_node *p, *q;

	r = whimory_range_find(&w->ranges, start);
	if (!r)
		return;
	p = rb_prev(&r->rb);
	if (p) {
		prev = rb_entry(p, struct whimory_range, rb);
		if (whimory_range_joinable(prev, r)) {
			prev->len += r->len;
			if (r->weave < prev->weave)
				prev->weave = r->weave;
			whimory_range_erase(w, r);
			r = prev;
		}
	}
	q = rb_next(&r->rb);
	if (q) {
		next = rb_entry(q, struct whimory_range, rb);
		if (whimory_range_joinable(r, next)) {
			r->len += next->len;
			if (next->weave < r->weave)
				r->weave = next->weave;
			whimory_range_erase(w, next);
		}
	}
}

/*
 * Apply as much of [lba, lba+span) as this claim is entitled to.
 *
 * Returns the number of LBAs consumed -- always at least one, so the caller
 * cannot spin -- or a negative errno. *applied says whether that run was
 * written to the map or passed over.
 *
 * TWO THINGS THIS USED TO GET WRONG
 *
 * It rejected the whole span the moment any part of it was covered by a
 * newer weave: "return 1" from inside the scan, with the comment "stale --
 * do not touch packed L2V". A BTOC or CXT run of 256 LBAs where one LBA had
 * been superseded therefore lost all 256, and the 255 with no competing
 * claim kept whatever was there before, which was frequently nothing. The
 * run is now split at the newer range instead: the part in front of it is
 * applied, the overlap is skipped, and the caller comes back for the rest.
 *
 * And it erased before it knew whether it could insert. The split and erase
 * ran first, then whimory_range_insert_new() returned 0 -- success --
 * without inserting anything once max_range_nodes was reached. Past the
 * ceiling every update turned a working mapping into a hole and reported
 * success while doing it. The budget is checked up front now, before
 * anything is destroyed, and a claim that cannot be recorded is passed over
 * intact rather than half-applied.
 */
static int whimory_range_update(struct whimory *w, u32 lba, u32 span, u32 vba,
				bool *applied)
{
	bool is_unmap = vba >= w->l2v.invalid_vba;
	struct whimory_range *blocker = NULL;
	struct whimory_range *hit;
	struct rb_node *node, *next;
	u32 run, end;
	int ret;

	*applied = false;
	if (!span)
		return 0;
	if (whimory_special_lba(lba))
		return span;

	/* First range in the way that a newer writer already owns. */
	{
		struct whimory_range *first = whimory_range_lower(&w->ranges,
								  lba);
		struct rb_node *n = first ? &first->rb : NULL;

		while (n) {
			struct whimory_range *r = rb_entry(n,
							   struct whimory_range,
							   rb);

			if (r->start >= lba + span)
				break;
			if (r->weave > w->sftl.claim_weave) {
				blocker = r;
				break;
			}
			n = rb_next(n);
		}
	}

	if (blocker && blocker->start <= lba) {
		/*
		 * The head of the run is owned by something newer. Skip
		 * exactly the overlap and let the caller retry past it --
		 * the tail may be entirely unclaimed.
		 */
		u32 blocker_end = blocker->start + blocker->len;

		w->sftl.stale_mapping_rejected++;
		run = min(lba + span, blocker_end) - lba;
		return (int)run;
	}

	run = blocker ? blocker->start - lba : span;
	end = lba + run;
	if (WARN_ON_ONCE(end < lba))		/* would wrap; cannot with a
						 * non-special lba and
						 * run <= WHIMORY_L2V_ROOT_SPAN */
		return (int)run;

	/*
	 * Budget first, and only for claims that add a node. An unmap only
	 * erases, so it is net-negative on the node count and is always
	 * allowed through -- refusing it would strand a stale mapping.
	 *
	 * The two splits below each allocate, and they run before the
	 * insert. Checking for bare equality would let a claim past the gate
	 * with one node of headroom, split twice, and then fail the insert
	 * having already erased -- which is the exact hole this check exists
	 * to prevent, just moved to the boundary. Reserve the splits.
	 */
	if (!is_unmap && max_range_nodes &&
	    w->sftl.range_nodes + 2 >= max_range_nodes) {
		w->sftl.range_budget_stop++;
		return (int)run;
	}

	hit = whimory_range_find(&w->ranges, lba);
	if (hit) {
		ret = whimory_range_split(w, hit, lba);
		if (ret)
			return ret;
	}
	hit = whimory_range_find(&w->ranges, end - 1);
	if (hit && hit->start < end) {
		ret = whimory_range_split(w, hit, end);
		if (ret)
			return ret;
	}

	hit = whimory_range_lower(&w->ranges, lba);
	node = hit ? &hit->rb : NULL;
	while (node) {
		struct whimory_range *r = rb_entry(node, struct whimory_range,
						   rb);

		next = rb_next(node);
		if (r->start >= end)
			break;
		if (r->start >= lba && r->start + r->len <= end)
			whimory_range_erase(w, r);
		node = next;
		cond_resched();
	}

	/* True unmap: erase only; do not insert invalid_vba placeholders. */
	if (is_unmap) {
		whimory_range_coalesce_at(w, lba);
		*applied = true;
		return (int)run;
	}

	ret = whimory_range_insert_new(w, lba, run, vba);
	if (ret)
		return ret;
	whimory_range_coalesce_at(w, lba);
	*applied = true;
	return (int)run;
}

/*
 * L2V_Update: split at 0x8000 root boundaries, then insert.
 * The interval map is the RO observable of the live tree.
 * Returns 0 on success (including stale-skip of a chunk), <0 on OOM/error.
 */
static int whimory_l2v_update(struct whimory *w, u32 lba, u32 span, u32 vba)
{
	bool is_unmap = vba >= w->l2v.invalid_vba;

	while (span) {
		u32 chunk = WHIMORY_L2V_ROOT_SPAN -
			    (lba & (WHIMORY_L2V_ROOT_SPAN - 1));
		bool applied;
		u32 done;
		int ret;

		if (chunk > span)
			chunk = span;
		/*
		 * range_update consumes as much of the chunk as this claim
		 * is entitled to, which may be less than all of it when a
		 * newer weave owns part of the range. It always consumes at
		 * least one LBA, so this loop always advances.
		 */
		ret = whimory_range_update(w, lba, chunk, vba, &applied);
		if (ret < 0)
			return ret;
		done = (u32)ret;
		if (WARN_ON_ONCE(!done || done > chunk))
			return -EINVAL;
		/*
		 * A chunk can now take several passes -- one per newer range
		 * standing in the way -- so this loop is no longer bounded by
		 * a couple of iterations. Up to WHIMORY_L2V_ROOT_SPAN of
		 * them, if the tree happens to alternate.
		 */
		cond_resched();
		if (!applied) {
			/* Skipped: leave the packed L2V alone for this run. */
			span -= done;
			lba += done;
			if (!is_unmap)
				vba += done;
			continue;
		}
		chunk = done;

		w->sftl.l2v_update_calls++;
		if (is_unmap)
			w->sftl.l2v_unmap_calls++;

		if (w->l2v.root && w->l2v.num_roots) {
			u32 ridx = lba >> 15;
			u8 *rec;
			u16 ver, node_idx;

			if (ridx < w->l2v.num_roots) {
				rec = w->l2v.root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
				ver = get_unaligned_le16(rec + 4);
				if (ver == 0xffff)
					ver = 0;
				put_unaligned_le16(ver + 1, rec + 4);
				/*
				 * whole-root unmap (off=0, span=0x8000,
				 * vba=invalid) frees the tree.
				 */
				if (!(lba & 0x7fff) &&
				    chunk == WHIMORY_L2V_ROOT_SPAN &&
				    is_unmap) {
					node_idx = get_unaligned_le16(rec);
					if (node_idx != WHIMORY_L2V_INVALID_ROOT)
						whimory_l2v_free_tree(w,
								      node_idx,
								      ridx);
					put_unaligned_le16(
						WHIMORY_L2V_INVALID_ROOT, rec);
				}
			}
			w->l2v.updates++;
			w->l2v.gen++;
			if (w->l2v.updates >= WHIMORY_L2V_UPDATE_REPACK)
				w->l2v.updates = 0;
		}
		/*
		 * whimory_l2v_update_packed() repacks a whole root, and
		 * collecting a root walks the interval map. Doing that per
		 * update makes replay O(updates x ranges) — the tail of a full
		 * recover crawled to ~14 s per superblock. The interval map is
		 * the lookup authority; pack once at the end instead.
		 */
		if (w->l2v.root && w->l2v.num_roots && !w->l2v_defer_pack) {
			u32 ridx = lba >> 15;
			bool whole_unmap = !(lba & 0x7fff) &&
				chunk == WHIMORY_L2V_ROOT_SPAN &&
				is_unmap;

			if (ridx < w->l2v.num_roots && !whole_unmap) {
				ret = whimory_l2v_update_packed(w, ridx,
						lba & 0x7fff, chunk, vba);
				if (ret)
					dev_dbg(w->dev,
						"L2V packed update r=%u %d\n",
						ridx, ret);
			}
		}
		span -= chunk;
		lba += chunk;
		if (vba < w->l2v.invalid_vba)
			vba += chunk;
	}
	return 0;
}

static void whimory_range_free(struct whimory *w)
{
	struct rb_node *n;

	while ((n = rb_first(&w->ranges))) {
		struct whimory_range *r = rb_entry(n, struct whimory_range, rb);

		whimory_range_erase(w, r);
	}
	w->ranges = RB_ROOT;
	w->sftl.range_nodes = 0;
	w->sftl.map_gen++;
	whimory_l2v_cache_flush(w);
}

/* ------------------------------------------------------------------ */
/* L2V init / lookup / tree pack , */
/* ------------------------------------------------------------------ */

static void whimory_l2v_free(struct whimory *w)
{
	kvfree(w->l2v.root);
	kvfree(w->l2v.nodes);
	kvfree(w->l2v.leaf_scratch);
	w->l2v.root = NULL;
	w->l2v.nodes = NULL;
	w->l2v.leaf_scratch = NULL;
	w->l2v.num_roots = 0;
	w->l2v.nodepool_bytes = 0;
	w->l2v.free_head = WHIMORY_L2V_INVALID_ROOT;
	w->l2v.free_count = 0;
}

/* L2V_Mem.c— intrusive free list in node[0]. */
static void whimory_l2v_mem_free(struct whimory_l2v *l2v, u32 idx)
{
	u8 *node;
	u32 n = l2v->nodepool_bytes / WHIMORY_L2V_NODE_SIZE;

	if (!l2v->nodes || idx >= n)
		return;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	put_unaligned_le32(l2v->free_head, node);
	l2v->free_head = idx;
	l2v->free_count++;
}

static void whimory_l2v_mem_reset(struct whimory_l2v *l2v)
{
	u32 n = l2v->nodepool_bytes / WHIMORY_L2V_NODE_SIZE;
	s32 j;

	l2v->free_head = WHIMORY_L2V_INVALID_ROOT;
	l2v->free_count = 0;
	l2v->nodes_used = 0;
	if (!l2v->nodes || !n)
		return;
	for (j = (s32)n - 1; j >= 0; j--)
		whimory_l2v_mem_free(l2v, (u32)j);
}

static u32 whimory_l2v_alloc_node(struct whimory_l2v *l2v)
{
	u32 idx, next;
	u8 *node;
	u32 n = l2v->nodepool_bytes / WHIMORY_L2V_NODE_SIZE;

	idx = l2v->free_head;
	if (idx == WHIMORY_L2V_INVALID_ROOT || !l2v->free_count || idx >= n)
		return WHIMORY_L2V_INVALID_ROOT;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	next = get_unaligned_le32(node);
	l2v->free_head = next;
	l2v->free_count--;
	memset(node, 0, WHIMORY_L2V_NODE_SIZE);
	if (idx + 1 > l2v->nodes_used)
		l2v->nodes_used = idx + 1;
	return idx;
}

static int whimory_l2v_init(struct whimory *w, u32 max_lba,
			    u32 vba_factor_a, u32 vba_factor_b,
			    u32 nodepool_bytes)
{
	struct whimory_l2v *l2v = &w->l2v;
	u64 prod;

	whimory_l2v_free(w);

	if (nodepool_bytes < WHIMORY_MIN_NODEPOOL_BYTES)
		nodepool_bytes = WHIMORY_MIN_NODEPOOL_BYTES;

	prod = 2ull * vba_factor_a * vba_factor_b;
	if (prod < 2)
		return -EINVAL;
	l2v->bits_vba = fls64(prod - 1) - 1;
	if (!l2v->bits_vba || l2v->bits_vba > 30)
		return -EINVAL;
	l2v->spanbits_vba = 30 - l2v->bits_vba;

	prod = 2ull * (nodepool_bytes >> 6);
	if (prod < 2)
		return -EINVAL;
	l2v->bits_nodeidx = fls64(prod - 1) - 1;
	if (!l2v->bits_nodeidx || l2v->bits_nodeidx > 30)
		return -EINVAL;
	l2v->spanbits_nodeidx = 30 - l2v->bits_nodeidx;

	l2v->sentinel_vba = (1u << l2v->bits_vba) - 1;
	l2v->invalid_vba = l2v->sentinel_vba;
	l2v->num_roots = (max_lba >> 15) + 1;
	l2v->nodepool_bytes = nodepool_bytes;
	l2v->nodes_used = 0;
	l2v->updates = 0;
	l2v->gen = 0;
	l2v->frag_count = 0;
	l2v->frag_max = 0;
	l2v->free_head = WHIMORY_L2V_INVALID_ROOT;
	l2v->free_count = 0;

	l2v->root = kvzalloc(WHIMORY_L2V_ROOT_REC_SIZE * l2v->num_roots,
			     GFP_KERNEL);
	if (!l2v->root)
		return -ENOMEM;
	l2v->nodes = kvzalloc(nodepool_bytes, GFP_KERNEL);
	if (!l2v->nodes) {
		kvfree(l2v->root);
		l2v->root = NULL;
		return -ENOMEM;
	}
	l2v->leaf_scratch = kvcalloc(WHIMORY_L2V_ROOT_SPAN,
				     sizeof(*l2v->leaf_scratch), GFP_KERNEL);
	if (!l2v->leaf_scratch) {
		kvfree(l2v->nodes);
		kvfree(l2v->root);
		l2v->nodes = NULL;
		l2v->root = NULL;
		return -ENOMEM;
	}
	memset(l2v->root, 0xff, WHIMORY_L2V_ROOT_REC_SIZE * l2v->num_roots);
	memset(l2v->nodes, 0, nodepool_bytes);
	whimory_l2v_mem_reset(l2v);

	dev_info(w->dev,
		 "L2V init roots=%u nodepool=0x%x bits_vba=%u/%u bits_node=%u/%u invalid=0x%x\n",
		 l2v->num_roots, l2v->nodepool_bytes,
		 l2v->bits_vba, l2v->spanbits_vba,
		 l2v->bits_nodeidx, l2v->spanbits_nodeidx,
		 l2v->invalid_vba);
	return 0;
}

static int whimory_l2v_encode(struct whimory_l2v *l2v, u8 *node,
			      u32 *front, u32 *back, bool is_node,
			      u32 value, u32 span)
{
	u8 value_bits = is_node ? l2v->bits_nodeidx : l2v->bits_vba;
	u8 span_bits = is_node ? l2v->spanbits_nodeidx : l2v->spanbits_vba;
	u32 span_m1, span_mask, value_mask, e, need;
	bool has_ext;

	if (!span || !value_bits)
		return -EINVAL;
	span_m1 = span - 1;
	span_mask = span_bits ? ((1u << span_bits) - 1) : 0;
	value_mask = (1u << value_bits) - 1;
	if (value > value_mask)
		return -EINVAL;
	has_ext = span_m1 > span_mask;
	if (has_ext && span_bits && (span_m1 >> span_bits) > 0xffffu)
		return -E2BIG;
	need = 4 + (has_ext ? 2 : 0);
	if (*front + need > *back)
		return -ENOSPC;
	e = (is_node ? 1u : 0u) | (has_ext ? 2u : 0u);
	e |= (value & value_mask) << 2;
	e |= (span_m1 & span_mask) << (value_bits + 2);
	put_unaligned_le32(e, node + *front);
	*front += 4;
	if (has_ext) {
		*back -= 2;
		put_unaligned_le16((u16)(span_m1 >> span_bits),
				   node + *back);
	}
	return 0;
}

static u32 whimory_leaf_span_sum(const struct whimory_leaf *l, u32 n)
{
	u32 i, s = 0;

	for (i = 0; i < n; i++)
		s += l[i].span;
	return s;
}

static int whimory_l2v_pack_leaves(struct whimory *w,
				   const struct whimory_leaf *leaves, u32 n,
				   u32 *idx_out);

static int whimory_l2v_pack_parent(struct whimory *w, u32 left, u32 span_l,
				   u32 right, u32 span_r, u32 *idx_out)
{
	struct whimory_l2v *l2v = &w->l2v;
	u8 *node;
	u32 idx, front, back;
	int ret;

	idx = whimory_l2v_alloc_node(l2v);
	if (idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOMEM;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	front = 0;
	back = WHIMORY_L2V_NODE_SIZE;
	ret = whimory_l2v_encode(l2v, node, &front, &back, true, left, span_l);
	if (!ret)
		ret = whimory_l2v_encode(l2v, node, &front, &back, true,
					 right, span_r);
	if (ret) {
		memset(node, 0xff, WHIMORY_L2V_NODE_SIZE);
		return ret;
	}
	if (front <= back - 4)
		put_unaligned_le32(0xffffffff, node + front);
	*idx_out = idx;
	return 0;
}

static int whimory_l2v_pack_leaves(struct whimory *w,
				   const struct whimory_leaf *leaves, u32 n,
				   u32 *idx_out)
{
	struct whimory_l2v *l2v = &w->l2v;
	u8 *node;
	u32 idx, front, back, i, mid, left, right, span_l, span_r;
	int ret;

	if (!n)
		return -EINVAL;

	idx = whimory_l2v_alloc_node(l2v);
	if (idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOMEM;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	front = 0;
	back = WHIMORY_L2V_NODE_SIZE;
	for (i = 0; i < n; i++) {
		ret = whimory_l2v_encode(l2v, node, &front, &back, false,
					 leaves[i].vba, leaves[i].span);
		if (ret)
			break;
	}
	if (!ret) {
		if (front <= back - 4)
			put_unaligned_le32(0xffffffff, node + front);
		*idx_out = idx;
		return 0;
	}
	memset(node, 0xff, WHIMORY_L2V_NODE_SIZE);
	if (l2v->nodes_used == idx + 1)
		l2v->nodes_used = idx;

	if (n == 1) {
		struct whimory_leaf half[2];
		u32 s0, s1;

		s0 = leaves[0].span / 2;
		s1 = leaves[0].span - s0;
		if (!s0 || !s1)
			return -EINVAL;
		half[0].vba = leaves[0].vba;
		half[0].span = s0;
		half[1].vba = leaves[0].vba + s0;
		half[1].span = s1;
		ret = whimory_l2v_pack_leaves(w, half, 1, &left);
		if (ret)
			return ret;
		ret = whimory_l2v_pack_leaves(w, half + 1, 1, &right);
		if (ret)
			return ret;
		return whimory_l2v_pack_parent(w, left, s0, right, s1, idx_out);
	}

	mid = n / 2;
	if (!mid)
		mid = 1;
	ret = whimory_l2v_pack_leaves(w, leaves, mid, &left);
	if (ret)
		return ret;
	ret = whimory_l2v_pack_leaves(w, leaves + mid, n - mid, &right);
	if (ret)
		return ret;
	span_l = whimory_leaf_span_sum(leaves, mid);
	span_r = whimory_leaf_span_sum(leaves + mid, n - mid);
	return whimory_l2v_pack_parent(w, left, span_l, right, span_r, idx_out);
}

static u32 whimory_l2v_collect_root(struct whimory *w, u32 ridx,
				    struct whimory_leaf *leaves)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 base = ridx * WHIMORY_L2V_ROOT_SPAN;
	u32 win_end = base + WHIMORY_L2V_ROOT_SPAN;
	u32 cursor = base, nleaf = 0;
	struct whimory_range *first = whimory_range_lower(&w->ranges, base);
	struct rb_node *n;

	for (n = first ? &first->rb : NULL; n; n = rb_next(n)) {
		struct whimory_range *rg = rb_entry(n, struct whimory_range, rb);
		u32 s, e, vba, span;

		if (rg->start >= win_end)
			break;
		e = rg->start + rg->len;
		if (e <= base)
			continue;
		s = max(rg->start, base);
		e = min(e, win_end);
		if (s >= e)
			continue;
		vba = rg->vba + (s - rg->start);
		if (s > cursor) {
			leaves[nleaf].vba = l2v->invalid_vba;
			leaves[nleaf].span = s - cursor;
			nleaf++;
		}
		span = e - s;
		leaves[nleaf].vba = vba;
		leaves[nleaf].span = span;
		nleaf++;
		cursor = e;
	}
	if (nleaf && cursor < win_end) {
		leaves[nleaf].vba = l2v->invalid_vba;
		leaves[nleaf].span = win_end - cursor;
		nleaf++;
	}
	return nleaf;
}

/* Discard this root's packed tree and rebuild it from the interval map. */
static int whimory_l2v_pack_root(struct whimory *w, u32 ridx)
{
	struct whimory_l2v *l2v = &w->l2v;
	struct whimory_leaf *leaves = l2v->leaf_scratch;
	u8 *rec;
	u16 ver, node_old;
	u32 nleaf, node_idx, used0;
	int ret;

	if (!leaves || ridx >= l2v->num_roots)
		return -EINVAL;
	rec = l2v->root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
	nleaf = whimory_l2v_collect_root(w, ridx, leaves);
	node_old = get_unaligned_le16(rec);
	if (node_old != WHIMORY_L2V_INVALID_ROOT)
		whimory_l2v_free_tree(w, node_old, ridx);
	if (!nleaf) {
		put_unaligned_le16(WHIMORY_L2V_INVALID_ROOT, rec);
		put_unaligned_le16(0, rec + 2);
		return 0;
	}
	w->sftl.l2v_repack_roots++;
	used0 = l2v->nodes_used;
	ret = whimory_l2v_pack_leaves(w, leaves, nleaf, &node_idx);
	if (ret) {
		put_unaligned_le16(WHIMORY_L2V_INVALID_ROOT, rec);
		put_unaligned_le16(0, rec + 2);
		return ret;
	}
	ver = get_unaligned_le16(rec + 4);
	if (ver == 0xffff)
		ver = 1;
	put_unaligned_le16((u16)node_idx, rec);
	put_unaligned_le16((u16)min_t(u32, l2v->nodes_used - used0, 0xffff),
			   rec + 2);
	put_unaligned_le16(ver, rec + 4);
	return 0;
}

/* First insert into an empty root: one node holding up to three leaves. */
static int whimory_l2v_grow_empty(struct whimory *w, u32 ridx, u32 off,
				  u32 span, u32 vba)
{
	struct whimory_l2v *l2v = &w->l2v;
	u8 *node, *rec;
	u32 idx, front = 0, back = WHIMORY_L2V_NODE_SIZE;
	u32 old = l2v->invalid_vba;
	int ret;

	if (off + span > WHIMORY_L2V_ROOT_SPAN)
		return -EINVAL;
	idx = whimory_l2v_alloc_node(l2v);
	if (idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOMEM;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	if (off) {
		ret = whimory_l2v_encode(l2v, node, &front, &back, false, old,
					 off);
		if (ret)
			goto fail;
	}
	ret = whimory_l2v_encode(l2v, node, &front, &back, false, vba, span);
	if (ret)
		goto fail;
	if (off + span < WHIMORY_L2V_ROOT_SPAN) {
		u32 tail = WHIMORY_L2V_ROOT_SPAN - off - span;
		u32 tail_vba = old;

		if (old < l2v->invalid_vba)
			tail_vba = old + off + span;
		ret = whimory_l2v_encode(l2v, node, &front, &back, false,
					 tail_vba, tail);
		if (ret)
			goto fail;
	}
	if (front < back)
		memset(node + front, 0xff, back - front);
	rec = l2v->root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
	put_unaligned_le16((u16)idx, rec);
	put_unaligned_le16(1, rec + 2);
	return 0;
fail:
	whimory_l2v_mem_free(l2v, idx);
	return ret;
}

static int whimory_l2v_update_packed(struct whimory *w, u32 ridx, u32 off,
				     u32 span, u32 vba)
{
	u16 node_idx;
	u8 *rec;

	if (!w->l2v.root || ridx >= w->l2v.num_roots || !span)
		return 0;
	rec = w->l2v.root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
	node_idx = get_unaligned_le16(rec);
	if (node_idx == WHIMORY_L2V_INVALID_ROOT)
		return whimory_l2v_grow_empty(w, ridx, off, span, vba);
	return whimory_l2v_pack_root(w, ridx);
}

static int whimory_l2v_build_from_ranges(struct whimory *w)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 ridx, mapped_roots = 0, mapped_lbas = 0;
	int ret = 0;
	struct rb_node *n;

	if (!l2v->root || !l2v->nodes || !l2v->leaf_scratch)
		return -ENODEV;

	{
		u16 *vers;
		u32 i;

		vers = kvmalloc_array(l2v->num_roots, sizeof(u16), GFP_KERNEL);
		if (!vers)
			return -ENOMEM;
		for (i = 0; i < l2v->num_roots; i++)
			vers[i] = get_unaligned_le16(
				l2v->root + i * WHIMORY_L2V_ROOT_REC_SIZE + 4);
		for (i = 0; i < l2v->num_roots; i++) {
			u8 *rec = l2v->root + i * WHIMORY_L2V_ROOT_REC_SIZE;

			put_unaligned_le16(WHIMORY_L2V_INVALID_ROOT, rec);
			put_unaligned_le16(0, rec + 2);
			put_unaligned_le16(vers[i], rec + 4);
		}
		kvfree(vers);
		whimory_l2v_mem_reset(l2v);
	}

	for (ridx = 0; ridx < l2v->num_roots; ridx++) {
		ret = whimory_l2v_pack_root(w, ridx);
		if (ret)
			break;
		if (get_unaligned_le16(l2v->root +
				       ridx * WHIMORY_L2V_ROOT_REC_SIZE) !=
		    WHIMORY_L2V_INVALID_ROOT)
			mapped_roots++;
	}
	for (n = rb_first(&w->ranges); n; n = rb_next(n)) {
		struct whimory_range *rg = rb_entry(n, struct whimory_range, rb);

		if (rg->vba < l2v->invalid_vba)
			mapped_lbas += rg->len;
	}
	w->sftl.mapped_roots = mapped_roots;
	w->sftl.mapped_lbas = mapped_lbas;
	if (ret)
		return ret;
	whimory_l2v_find_frag(w);
	if (l2v->free_count < WHIMORY_L2V_MIN_FREE)
		dev_warn(w->dev, "L2V free %u < %u after pack\n",
			 l2v->free_count, WHIMORY_L2V_MIN_FREE);
	dev_info(w->dev,
		 "L2V recovery OK mapped_roots=%u mapped_lbas=%u nodes_used=%u range_nodes=%u frag=%u/%u free=%u\n",
		 mapped_roots, mapped_lbas, l2v->nodes_used,
		 w->sftl.range_nodes, l2v->frag_count, l2v->frag_max,
		 l2v->free_count);
	return mapped_roots ? 0 : -ENOENT;
}

/* L2V_FindFrag.c— walk leaves, record fragment stats. */
static void whimory_l2v_find_frag_node(struct whimory *w, u32 node_idx,
				       u32 *count, u32 *maxspan, int depth)
{
	struct whimory_l2v *l2v = &w->l2v;
	const u8 *node;
	u32 front = 0, back = WHIMORY_L2V_NODE_SIZE;

	if (depth > WHIMORY_L2V_FINDFRAG_WIN ||
	    (node_idx + 1) * WHIMORY_L2V_NODE_SIZE >
	    l2v->nodepool_bytes)
		return;
	node = l2v->nodes + node_idx * WHIMORY_L2V_NODE_SIZE;
	while (front + 4 <= back) {
		u32 e = get_unaligned_le32(node + front);
		bool is_node, has_ext;
		u32 value_bits, span_bits, value, span_minus1, span;

		if (e == 0xffffffff)
			break;
		is_node = e & 1;
		has_ext = e & 2;
		value_bits = is_node ? l2v->bits_nodeidx : l2v->bits_vba;
		span_bits = is_node ? l2v->spanbits_nodeidx : l2v->spanbits_vba;
		value = (e >> 2) & ((1u << value_bits) - 1);
		span_minus1 = span_bits ?
			((e >> (value_bits + 2)) & ((1u << span_bits) - 1)) : 0;
		if (has_ext) {
			back -= 2;
			if (front + 4 > back)
				break;
			span_minus1 += (u32)get_unaligned_le16(node + back) <<
				       span_bits;
		}
		span = span_minus1 + 1;
		if (is_node)
			whimory_l2v_find_frag_node(w, value, count, maxspan,
						   depth + 1);
		else {
			(*count)++;
			if (span > *maxspan)
				*maxspan = span;
		}
		front += 4;
	}
}

static void whimory_l2v_free_tree(struct whimory *w, u32 node_idx, u32 root_idx)
{
	struct whimory_l2v *l2v = &w->l2v;
	const u8 *node;
	u32 front = 0, back = WHIMORY_L2V_NODE_SIZE;
	u8 *rec;

	if ((node_idx + 1) * WHIMORY_L2V_NODE_SIZE > l2v->nodepool_bytes)
		return;
	node = l2v->nodes + node_idx * WHIMORY_L2V_NODE_SIZE;
	while (front + 4 <= back) {
		u32 e = get_unaligned_le32(node + front);
		bool is_node, has_ext;
		u32 value_bits, value;

		if (e == 0xffffffff)
			break;
		is_node = e & 1;
		has_ext = e & 2;
		value_bits = is_node ? l2v->bits_nodeidx : l2v->bits_vba;
		value = (e >> 2) & ((1u << value_bits) - 1);
		if (has_ext) {
			back -= 2;
			if (front + 4 > back)
				break;
		}
		if (is_node)
			whimory_l2v_free_tree(w, value, root_idx);
		front += 4;
	}
	whimory_l2v_mem_free(l2v, node_idx);
	if (root_idx < l2v->num_roots) {
		rec = l2v->root + root_idx * WHIMORY_L2V_ROOT_REC_SIZE;
		{
			u16 n_nodes = get_unaligned_le16(rec + 2);

			if (n_nodes && n_nodes != 0xffff)
				put_unaligned_le16(n_nodes - 1, rec + 2);
		}
	}
}

static void whimory_l2v_find_frag(struct whimory *w)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 ridx, count = 0, maxspan = 0;

	if (!l2v->root || !l2v->nodes)
		return;
	for (ridx = 0; ridx < l2v->num_roots; ridx++) {
		u32 node_idx = get_unaligned_le16(
			l2v->root + ridx * WHIMORY_L2V_ROOT_REC_SIZE);

		if (node_idx == WHIMORY_L2V_INVALID_ROOT)
			continue;
		whimory_l2v_find_frag_node(w, node_idx, &count, &maxspan, 0);
	}
	l2v->frag_count = count;
	l2v->frag_max = maxspan;
}

static int whimory_l2v_lookup(struct whimory *w, u32 lba,
			      u32 *vba_out, u32 *span_out)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 root_idx = lba >> 15;
	u32 target = lba & 0x7fff;
	u32 consumed;
	u32 node_idx;
	int depth;

	if (!l2v->root || !l2v->nodes)
		return -ENODEV;
	if (root_idx >= l2v->num_roots)
		return -ERANGE;

	node_idx = get_unaligned_le16(l2v->root + 6 * root_idx);
	if (node_idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOENT;

	for (depth = 0; depth < 32; depth++) {
		const u8 *node;
		u32 front = 0;
		u32 back = WHIMORY_L2V_NODE_SIZE;

		if ((node_idx + 1) * WHIMORY_L2V_NODE_SIZE >
		    l2v->nodepool_bytes)
			return -EINVAL;
		node = l2v->nodes + node_idx * WHIMORY_L2V_NODE_SIZE;
		consumed = 0;

		while (front + 4 <= back) {
			u32 e = get_unaligned_le32(node + front);
			bool is_node, has_ext;
			u32 value_bits, span_bits, value_mask, value;
			u32 span_minus1, span;

			if (e == 0xffffffff)
				break;
			is_node = e & 1;
			has_ext = e & 2;
			if (is_node) {
				value_bits = l2v->bits_nodeidx;
				span_bits = l2v->spanbits_nodeidx;
			} else {
				value_bits = l2v->bits_vba;
				span_bits = l2v->spanbits_vba;
			}
			value_mask = value_bits ? ((1u << value_bits) - 1) : 0;
			value = (e >> 2) & value_mask;
			span_minus1 = e >> (value_bits + 2);
			if (span_bits)
				span_minus1 &= (1u << span_bits) - 1;
			else
				span_minus1 = 0;
			if (has_ext) {
				back -= 2;
				if (front + 4 > back)
					return -EINVAL;
				span_minus1 +=
					(u32)get_unaligned_le16(node + back) <<
					span_bits;
			}
			span = span_minus1 + 1;
			if (!span)
				return -EINVAL;
			if (target < consumed + span) {
				u32 delta = target - consumed;

				if (is_node) {
					node_idx = value;
					goto next_level;
				}
				if (value >= l2v->invalid_vba)
					return -ENOENT;
				*vba_out = value + delta;
				*span_out = span - delta;
				return 0;
			}
			consumed += span;
			front += 4;
		}
		return -EINVAL;
next_level:
		continue;
	}
	return -ELOOP;
}

/*
 * L2V_Search keeps a sequential hint; do the same here. VFAT walks a cluster
 * one 4 KiB sector at a time, so consecutive lookups land in the same extent
 * and the rbtree descent is pure overhead. Invalidated by map generation.
 */
static void whimory_l2v_cache_store(struct whimory *w,
				    const struct whimory_range *r)
{
	w->search_start = r->start;
	w->search_len = r->len;
	w->search_vba = r->vba;
	w->search_gen = w->sftl.map_gen;
	w->search_valid = true;
}

static void whimory_l2v_cache_flush(struct whimory *w)
{
	if (w)
		w->search_valid = false;
}

/* Prefer the interval map (L2V_Update result); packed tree is for Search. */
static int whimory_l2v_search(struct whimory *w, u32 lba,
			      u32 *vba_out, u32 *span_out)
{
	struct whimory_range *r;

	if (w->search_valid && w->search_gen == w->sftl.map_gen &&
	    lba >= w->search_start && lba - w->search_start < w->search_len) {
		u32 delta = lba - w->search_start;

		*vba_out = w->search_vba + delta;
		*span_out = w->search_len - delta;
		w->sftl.search_cache_hits++;
		return 0;
	}

	r = whimory_range_find(&w->ranges, lba);
	if (r) {
		u32 delta = lba - r->start;

		whimory_l2v_cache_store(w, r);
		w->sftl.search_cache_misses++;
		*vba_out = r->vba + delta;
		*span_out = r->len - delta;
		return 0;
	}
	w->sftl.search_cache_misses++;
	return whimory_l2v_lookup(w, lba, vba_out, span_out);
}

/* ------------------------------------------------------------------ */
/* FIL */
/* ------------------------------------------------------------------ */

static int whimory_fil_init(struct whimory *w)
{
	struct s5l8740_nand_geom g;
	int ret;

	ret = s5l8740_nand_hw_init();
	if (ret)
		return ret;
	ret = s5l8740_nand_query_geometry(&g);
	if (ret)
		return ret;
	if (!g.dev_id)
		return -ENODEV;

	w->geom.num_ce = g.num_ce;
	w->geom.num_cau = g.num_cau;
	w->geom.blocks_per_cau = g.blocks_per_cau;
	w->geom.pages_per_block = g.pages_per_block;
	w->geom.page_size = g.page_size;
	w->geom.vfl_tail = g.vfl_tail;
	w->geom.user_blocks = g.blocks_per_cau - g.vfl_tail;
	w->geom.dev_id = s5l8740_nand_fil_get_info(101);
	w->geom.geom_104 = s5l8740_nand_fil_get_info(104);
	w->geom.geom_105 = s5l8740_nand_fil_get_info(105);
	w->geom.geom_135 = s5l8740_nand_fil_get_info(135);
	if (!w->geom.dev_id)
		return -ENODEV;
	if (w->geom.geom_104 && w->geom.geom_104 != w->geom.page_size) {
		dev_err(w->dev,
			"FIL GetInfo(104)=%u != page_size=%u\n",
			w->geom.geom_104, w->geom.page_size);
		return -EINVAL;
	}
	if (w->geom.geom_105 && w->geom.geom_105 != WHIMORY_FIL_META_BYTES) {
		dev_err(w->dev, "FIL GetInfo(105)=%u != %u\n",
			w->geom.geom_105, WHIMORY_FIL_META_BYTES);
		return -EINVAL;
	}

	dev_info(w->dev,
		 "FIL_Init OK dev_id=%u g104=%u g105=%u g135=%u ce=%u cau=%u blocks=%u user=%u param=%d\n",
		 w->geom.dev_id, w->geom.geom_104, w->geom.geom_105,
		 w->geom.geom_135, w->geom.num_ce, w->geom.num_cau,
		 w->geom.blocks_per_cau, w->geom.user_blocks,
		 g.from_param_page);
	w->fil_ok = true;
	return 0;
}

/* ------------------------------------------------------------------ */
/* FPart — signature from media (or oracle firmware file) */
/* ------------------------------------------------------------------ */

/*
 *— FPart signature is NOT a user-page hunt.
 * OSOS: memset(sig, 0xA5, 0x600) then _fpart->op80(sig, 0x600, 0xC101).
 * READ ONLY — never AllocateSpecialBlock / WriteSpecial / erase.
 * Validate: magic 0x776d7278, ver<=6, +0x34 == FIL GetInfo(101).
 */
static void whimory_log_sig_fields(struct whimory *w, const u8 *s,
				   const char *why)
{
	u32 magic = whimory_sig32(s, 0x00);
	u32 ver = whimory_sig32(s, 0x08);
	u32 ftl_m = whimory_sig32(s, 0x0c);
	u32 ftl_n = whimory_sig32(s, 0x10);
	u32 vfl_m = whimory_sig32(s, 0x18);
	u32 vfl_n = whimory_sig32(s, 0x1c);
	u32 vfl_arg = whimory_sig32(s, 0x20);
	u32 fpt_m = whimory_sig32(s, 0x24);
	u32 fpt_n = whimory_sig32(s, 0x28);
	u32 fpt_a = whimory_sig32(s, 0x2c);
	u32 extra = whimory_sig32(s, 0x30);
	u32 geom = whimory_sig32(s, 0x34);
	u32 cfg = whimory_sig32(s, 0xb8);

	dev_info(w->dev,
		 "WHIMORY_SIG %s magic=%08x ver=%u ftl=%u.%u vfl=%u.%u fpart=%u.%u geom=%u num_ce=%u vfl_arg=%u fpart_arg=%u extra=%u cfg_b8=%u first32=%32ph\n",
		 why, magic, ver, ftl_m, ftl_n, vfl_m, vfl_n, fpt_m, fpt_n,
		 geom, w->geom.num_ce, vfl_arg, fpt_a, extra, cfg, s);
}

/* OSOSchecks — not the old ver>=1 / major<=16 heuristic. */
static int whimory_validate_signature(struct whimory *w, const u8 *sig)
{
	u32 magic = whimory_sig32(sig, 0x00);
	u32 ver = whimory_sig32(sig, 0x08);
	u32 geom = whimory_sig32(sig, 0x34);

	whimory_log_sig_fields(w, sig, "validate");
	if (magic != WHIMORY_SIG_MAGIC) {
		dev_info(w->dev,
			 "FPART_SIG_READ reject: magic=%08x want=776d7278\n",
			 magic);
		return -EINVAL;
	}
	if (ver > 6) {
		dev_info(w->dev,
			 "FPART_SIG_READ reject: version=%u > 6\n", ver);
		return -EINVAL;
	}
	/*
	 * The +0x34 field is a device count, not a block count.
	 *
	 * This compared it against FIL GetInfo(101) and rejected the
	 * signature on every boot of this unit: geom=2 against
	 * GetInfo(101)=2088. Those are not a mismatched pair of the same
	 * quantity, they are two different quantities -- GetInfo(101)
	 * returns blocks_per_cau, and the signature field holds 2, which is
	 * num_ce.
	 *
	 * OSOS settles it. The validator reads, with the signature buffer at
	 * 0x8D0C220:
	 *
	 *   if (MEMORY[0x8D0C220] != 0x776D7278)  "invalid magic"
	 *   if (MEMORY[0x8D0C244] != a1)          "FPart major ver"
	 *   if (MEMORY[0x8D0C254] != MEMORY[0x8D0CE20]) "Geometry does not match"
	 *   if (MEMORY[0x8D0C228] > 6)            "Device version unsupported"
	 *
	 * so +0x34 is compared against MEMORY[0x8D0CE20]. Earlier in the
	 * same function that word is the one tested as
	 *
	 *   if (!MEMORY[0x8D0CE20]) "[NAND] No NAND device found"
	 *
	 * which is a count of NAND devices. It cannot be a block count: a
	 * zero block count is not how firmware says no chip responded.
	 *
	 * Rejecting the signature is not a harmless miss. It is what left
	 * fpart_sig=0 vfl_ctx_hits=0 vfl_cxt_loc=0 on every boot, and
	 * without the FPart signature there is no VFL context, so recovery
	 * fell back to scanning all 1960 user blocks three times and
	 * inferring by weave what the context would have stated.
	 */
	if (geom != w->geom.num_ce) {
		dev_info(w->dev,
			 "FPART_SIG_READ reject: geom=%u != num_ce=%u\n",
			 geom, w->geom.num_ce);
		return -EINVAL;
	}
	return 0;
}

static void whimory_dump_sparse(struct whimory *w, const char *tag,
				const u8 *p, unsigned int len);

static int whimory_parse_signature(struct whimory *w, const u8 *s)
{
	int ret;

	ret = whimory_validate_signature(w, s);
	if (ret)
		return ret;
	memcpy(w->sig.raw, s, WHIMORY_SIG_SIZE);
	w->sig.version = whimory_sig32(s, 0x08);
	w->sig.ftl_major = whimory_sig32(s, 0x0c);
	w->sig.ftl_minor = whimory_sig32(s, 0x10);
	w->sig.vfl_major = whimory_sig32(s, 0x18);
	w->sig.vfl_minor = whimory_sig32(s, 0x1c);
	w->sig.fpart_major = whimory_sig32(s, 0x24);
	w->sig.fpart_minor = whimory_sig32(s, 0x28);
	w->sig.sig_geom = whimory_sig32(s, 0x34);
	w->sig.flags_or_open = whimory_sig32(s, 0x20);
	w->sig.fpart_arg = whimory_sig32(s, 0x2c);
	w->sig.extra_arg = whimory_sig32(s, 0x30);
	w->sig_ok = true;
	whimory_dump_sparse(w, "SIG", s, WHIMORY_SIG_SIZE);
	dev_info(w->dev,
		 "Whimory sig OK ver=%u fpart=%u.%u vfl=%u.%u ftl=%u.%u geom=%u vfl_arg=%u fpart_arg=%u extra=%u\n",
		 w->sig.version, w->sig.fpart_major, w->sig.fpart_minor,
		 w->sig.vfl_major, w->sig.vfl_minor,
		 w->sig.ftl_major, w->sig.ftl_minor, w->sig.sig_geom,
		 w->sig.flags_or_open, w->sig.fpart_arg, w->sig.extra_arg);
	return 0;
}

static int n31_fpart_init(struct whimory *w)
{
	if (!w->fil_ok)
		return -ENODEV;
	memset(w->fpart_ctx.table, 0xff, sizeof(w->fpart_ctx.table));
	w->fpart_ctx.count = 0;
	w->fpart_ctx.scanned = false;
	return 0;
}

static u32 n31_fpart_minor(struct whimory *w)
{
	return w->sig.fpart_minor;
}

static u16 fpart_num_banks(const struct whimory *w)
{
	return w->geom.num_ce * w->geom.num_cau;
}

static void fpart_bank_to_ce_cau(const struct whimory *w, u16 bank,
				 unsigned int *ce, unsigned int *cau)
{
	u16 ncau = w->geom.num_cau ? w->geom.num_cau : 1;

	*ce = bank / ncau;
	*cau = bank % ncau;
}

static bool fpart_type_class1(u16 type_word)
{
	return ((type_word >> 8) & FPART_SPECIAL_CLASS_MASK) ==
	       FPART_SPECIAL_CLASS;
}

static bool fpart_meta_special(const u8 *meta, u8 want_chunk, u16 *type_out);
static bool fpart_meta_is_assign(const u8 *meta, u16 *type_out);
static bool fpart_has_xrmw(const u8 *page);

/*
 *op=1 analogue. Special objects often live on SLC; try SLC
 * then MLC. Full 16 KiB data + 64B META; special uses first 16 META bytes.
 */
/*
 * Read one FPart page, trying SLC plane 1 then 0.
 *
 * This went through s5l8740_nand_page_read() until now, and never once
 * reached the NAND. That function refuses any request carrying a meta buffer
 * unless meta_dma_read is set, and meta_dma_read is deliberately off -- a
 * permanent live CS kick reboots the device, so the sanctioned path is a
 * temporary dma_session around cs_phys instead. Every FPart read therefore
 * returned -EOPNOTSUPP: the log said reads=512 fail=512 in sixty
 * milliseconds, which is far too fast to have been a NAND access at all, and
 * "sig=0, not a native open" was a verdict about a region nobody had looked
 * at.
 *
 * cs_phys is the same path classify uses, and it needs the caller to hold a
 * DMA session -- fpart_scan_region() opens one.
 */
static int fpart_fil_read_page(struct whimory *w, u16 bank, u32 block,
			       u32 page, struct s5l8740_cs_page *csp,
			       void *data, u8 *meta)
{
	unsigned int ce, cau, i, sl;
	int last = -EIO;
	const u8 slc_order[2] = { 1, 0 };

	fpart_bank_to_ce_cau(w, bank, &ce, &cau);
	if (ce >= w->geom.num_ce || cau >= w->geom.num_cau ||
	    block >= w->geom.blocks_per_cau ||
	    page >= w->geom.pages_per_block)
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		int ret;

		ret = s5l8740_nand_cs_phys_read_slc((u8)ce, (u8)cau, (u16)block,
						    (u8)page, slc_order[i],
						    csp, 4);
		if (ret)
			continue;

		/* Flatten the four records back into the flat page and the
		 * 64-byte meta the FPart parsers expect.
		 */
		for (sl = 0; sl < N31_DATA_SLOTS; sl++) {
			size_t doff = (size_t)sl * N31_DATA_SLOT_SIZE;

			if (doff + N31_DATA_SLOT_SIZE <= w->geom.page_size)
				memcpy((u8 *)data + doff, csp->data[sl],
				       N31_DATA_SLOT_SIZE);
			memcpy(meta + sl * WHIMORY_META_SIZE,
			       csp->meta_raw[sl], WHIMORY_META_SIZE);
		}

		last = 0;
		if (fpart_meta_special(meta, 0, NULL) ||
		    fpart_meta_is_assign(meta, NULL))
			return 0;
		if (payload_magic_scan && fpart_has_xrmw(data))
			return 0;
	}
	return last;
}

/*— 16-byte META copy. LE type_word at +2 (RE). */
static bool fpart_meta_special(const u8 *meta, u8 want_chunk, u16 *type_out)
{
	unsigned int slot;

	if (!meta)
		return false;
	for (slot = 0; slot < 4; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;

		if (m[0] != FPART_SPECIAL_TAG)
			continue;
		if (m[1] != want_chunk)
			continue;
		if (type_out)
			*type_out = get_unaligned_le16(m + 2);
		return true;
	}
	return false;
}

/*
 * Scanner: META tag 0x30 and class 1. Chunk-0 assignment pages use m[1]==0
 * with class in type_word[15:8]. Avoid treat payload magic as a hit.
 */
static bool fpart_meta_is_assign(const u8 *meta, u16 *type_out)
{
	unsigned int slot;

	if (!meta)
		return false;
	for (slot = 0; slot < 4; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;
		u16 tw;

		if (m[0] != FPART_SPECIAL_TAG)
			continue;
		tw = get_unaligned_le16(m + 2);
		if ((m[1] & FPART_SPECIAL_CLASS_MASK) == FPART_SPECIAL_CLASS ||
		    (m[1] == 0 && fpart_type_class1(tw))) {
			if (type_out)
				*type_out = tw;
			return true;
		}
	}
	return false;
}

static int fpart_meta_special_slot(const u8 *meta, u8 want_chunk, u16 *type_out)
{
	unsigned int slot;

	if (!meta)
		return -1;
	for (slot = 0; slot < 4; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;

		if (m[0] != FPART_SPECIAL_TAG)
			continue;
		if (m[1] != want_chunk)
			continue;
		if (type_out)
			*type_out = get_unaligned_le16(m + 2);
		return (int)slot;
	}
	return -1;
}

static bool fpart_meta_interesting(const u8 *m)
{
	return m && (m[0] == FPART_SPECIAL_TAG ||
		     m[0] == WHIMORY_META_TYPE_VFL_CXT ||
		     m[0] == WHIMORY_META_TYPE_SFTL_CXT ||
		     m[0] == WHIMORY_META_TYPE_BTOC);
}

static u32 fpart_word_at(const u8 *page, unsigned int off)
{
	return get_unaligned_le32(page + off);
}

static bool fpart_has_xrmw(const u8 *page)
{
	u32 a = fpart_word_at(page, 0);
	u32 b = fpart_word_at(page, FPART_SPECIAL_HDR);

	return a == WHIMORY_SIG_MAGIC || b == WHIMORY_SIG_MAGIC;
}

static bool fpart_has_wrmx(const u8 *page)
{
	u32 a = fpart_word_at(page, 0);
	u32 b = fpart_word_at(page, FPART_SPECIAL_HDR);

	return a == WHIMORY_SIG_MAGIC_WRMX || b == WHIMORY_SIG_MAGIC_WRMX;
}

/*
 * Cache insert: sorted by type_word, then bank, then block (OSOS
 * fpart_special_cache_add_pairs). Table size 0x2d0 / 6 = 120.
 */
static int fpart_cache_add(struct whimory *w, u16 bank, u16 block,
			   u16 type_word)
{
	struct fpart_special_entry *t = w->fpart_ctx.table;
	u16 n = w->fpart_ctx.count;
	u16 i, j;

	if (n >= FPART_SPECIAL_MAX_ENTRIES)
		return -ENOSPC;

	for (i = 0; i < n; i++) {
		if (t[i].type_word == type_word && t[i].bank == bank &&
		    t[i].block == block)
			return 0;
		if (t[i].type_word > type_word)
			break;
		if (t[i].type_word == type_word && t[i].bank > bank)
			break;
		if (t[i].type_word == type_word && t[i].bank == bank &&
		    t[i].block > block)
			break;
	}
	for (j = n; j > i; j--)
		t[j] = t[j - 1];
	t[i].bank = bank;
	t[i].block = block;
	t[i].type_word = type_word;
	w->fpart_ctx.count = n + 1;
	return 1;
}

/* Assignment page DATA: up to eight u16 bank, u16 block; bank==0xffff ends. */
static bool fpart_data_is_assignment(const u8 *data, u16 nbanks, u32 nblk)
{
	unsigned int i, valid = 0;

	for (i = 0; i < FPART_ASSIGN_MAX_PAIRS; i++) {
		u16 bank = get_unaligned_le16(data + i * 4);
		u16 block = get_unaligned_le16(data + i * 4 + 2);

		if (bank == 0xffff)
			return i == 0 || valid > 0;
		if (bank >= nbanks || block >= nblk)
			return false;
		valid++;
	}
	return valid > 0;
}

static unsigned int fpart_ingest_pairs(struct whimory *w, const u8 *data,
				       u16 type_word)
{
	unsigned int i, added = 0;
	u16 nbanks = fpart_num_banks(w);

	for (i = 0; i < FPART_ASSIGN_MAX_PAIRS; i++) {
		u16 bank = get_unaligned_le16(data + i * 4);
		u16 block = get_unaligned_le16(data + i * 4 + 2);

		if (bank == 0xffff)
			break;
		if (bank >= nbanks || block >= w->geom.blocks_per_cau) {
			dev_info(w->dev,
				 "FPART_ASSIGN skip pair bank=%u block=%u (banks=%u blocks=%u)\n",
				 bank, block, nbanks, w->geom.blocks_per_cau);
			continue;
		}
		if (fpart_cache_add(w, bank, block, type_word) > 0) {
			dev_info(w->dev,
				 "FPART_ASSIGN_ADD bank=%u block=%u type=0x%04x\n",
				 bank, block, type_word);
			added++;
		}
	}
	return added;
}

static bool fpart_find_in_cache(struct whimory *w, u16 *index, u16 type)
{
	u8 low = type & 0xff;
	u16 i;

	for (i = 0; i < w->fpart_ctx.count; i++) {
		if ((w->fpart_ctx.table[i].type_word & 0xff) == low) {
			*index = i;
			return true;
		}
	}
	return false;
}

/*
 * Say out loud every distinct FPART special type the scan actually found.
 *
 * We only act on three: 0xc101 SIGNATURE, 0xc104 VFL_CXT, 0xc105 CONFIG (which
 * carries SysCfg). Anything else is cached and then silently ignored, so a
 * type that exists on the medium has never been visible in a boot log.
 *
 * That matters right now because the Grape touch firmware has a second source.
 * sub_1A640 falls back to sub_201FC(0x67706677, ...) -- 'gpfw' big-endian --
 * when MEMORY[0x8A8FAA4] is zero, and that lookup goes through sub_683E0 ->
 * sub_26794 -> sub_261A4, a keyed record lookup whose sub_26794 carries the
 * string "APPLE_MDFW". Which medium backs it is NOT established from the
 * decomp yet; a NAND special area analogous to SysCfg is the working
 * hypothesis and this log is how the device answers it.
 *
 * Printed once per scan, unconditionally, because the whole point is to turn a
 * cold boot into evidence.
 */
static void fpart_log_special_types(struct whimory *w)
{
	u16 seen[16];
	unsigned int nseen = 0;
	u16 i, j;

	for (i = 0; i < w->fpart_ctx.count; i++) {
		u16 t = w->fpart_ctx.table[i].type_word;

		for (j = 0; j < nseen; j++)
			if (seen[j] == t)
				break;
		if (j < nseen)
			continue;
		if (nseen < ARRAY_SIZE(seen))
			seen[nseen++] = t;
	}

	for (j = 0; j < nseen; j++) {
		const char *known =
			seen[j] == FPART_TYPE_SIGNATURE ? " SIGNATURE" :
			seen[j] == FPART_TYPE_VFL_CXT   ? " VFL_CXT" :
			seen[j] == FPART_TYPE_CONFIG    ? " CONFIG/SysCfg" :
							  " UNCLAIMED";
		u16 n = 0;

		for (i = 0; i < w->fpart_ctx.count; i++)
			if (w->fpart_ctx.table[i].type_word == seen[j])
				n++;

		dev_info(w->dev,
			 "FPART_SPECIAL_TYPE 0x%04x copies=%u%s\n",
			 seen[j], n, known);
	}
	dev_info(w->dev, "FPART_SPECIAL_TYPES distinct=%u of %u copies\n",
		 nseen, w->fpart_ctx.count);
}

static u16 fpart_count_special_copies(struct whimory *w, u16 type_word)
{
	u8 low = type_word & 0xff;
	u16 n = 0, i;

	for (i = 0; i < w->fpart_ctx.count; i++) {
		if ((w->fpart_ctx.table[i].type_word & 0xff) == low)
			n++;
	}
	return n;
}

static int fpart_scan_region(struct whimory *w, u16 type,
			     u32 block_lo, u32 block_hi,
			     unsigned int page_lo, unsigned int page_hi,
			     bool *matched)
{
	u8 *page;
	u8 meta[S5L8740_NAND_META_SIZE];
	struct s5l8740_cs_page *csp;
	u16 bank, nbanks = fpart_num_banks(w);
	u32 b, p;
	int ret, reads = 0, tag30 = 0, xrmw = 0, wrmx = 0, fail = 0;
	unsigned int sample = 0;
	/*
	 * Heap, not stack. 256 u32 is a kilobyte, and with the page buffer
	 * pointers and the meta array around it this frame measured 1224
	 * bytes -- past the 1024-byte warning and a quarter of an 8 KiB
	 * kernel stack, on a function that also calls into the NAND driver.
	 */
	u32 *hist;
	int sess;

	page = kvmalloc(w->geom.page_size, GFP_KERNEL);
	if (!page)
		return -ENOMEM;
	hist = kcalloc(256, sizeof(*hist), GFP_KERNEL);
	if (!hist) {
		kvfree(page);
		return -ENOMEM;
	}

	csp = kvmalloc(sizeof(*csp), GFP_KERNEL);
	if (!csp) {
		kfree(hist);
		kvfree(page);
		return -ENOMEM;
	}

	/*
	 * Clamp to the SLC page count, not the MLC one.
	 *
	 * This region is SLC and an SLC block holds geom_105 pages, 16 on
	 * this part, against 128 in MLC. Sweeping to pages_per_block asks
	 * for pages 16..127, which do not exist there -- and the failures
	 * do not stay local: measured over blk[1960,2088), pages[0,127]
	 * gave 65536 reads, fail=0 and tag30=0, with 97 per cent of pages
	 * reading back as meta 0x00. Every object in the region vanished,
	 * including the five on page 0 that the same code finds every
	 * time when it stops at page 0. The same sweep bounded to
	 * pages[0,15] gives 8192 reads, fail=0, tag30=80 and entries=5:
	 * the five objects, each occupying all 16 SLC pages of its block.
	 *
	 * So an out-of-range SLC page does not merely fail, it takes the
	 * reads after it with it, and the result looks like a clean scan
	 * of an empty region rather than like an error.
	 */
	if (w->geom.geom_105 && page_hi >= w->geom.geom_105)
		page_hi = w->geom.geom_105 - 1;
	else if (page_hi >= w->geom.pages_per_block)
		page_hi = w->geom.pages_per_block - 1;

	/*
	 * Arm live CS for the sweep. Sessions nest and begin() always
	 * succeeds, so this one owns exactly its own end; an outer session
	 * simply keeps CS armed past it. The failure check below is kept for
	 * a future begin() that can fail.
	 */
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		dev_warn(w->dev, "FPART_SCAN no DMA session (%d)\n", sess);
		kfree(hist);
		kvfree(csp);
		kvfree(page);
		return sess;
	}

	s5l8740_nand_reset();

	for (bank = 0; bank < nbanks; bank++) {
		for (b = block_hi; b > block_lo; b--) {
			u32 blk = b - 1;

			for (p = page_lo; p <= page_hi; p++) {
				u16 type_word = 0;
				unsigned int ce, cau, pairs;
				u32 obj_len;
				bool special, magic;

				cond_resched();
				ret = fpart_fil_read_page(w, bank, blk, p,
							  csp, page, meta);
				reads++;
				if (ret) {
					fail++;
					continue;
				}
				hist[meta[0]]++;
				/*
				 * Gated, because printing is not free here.
				 *
				 * Each of these is a 112-byte hex dump down a
				 * slow console, and the timestamps put one at
				 * 85 to 95 ms. Two dozen of them across the
				 * scan is about two seconds added to every
				 * mount, spent describing pages the scan
				 * already summarises in FPART_SPECIAL_TYPE
				 * and the meta histogram. diag=1 brings them
				 * back.
				 */
				if (ftl_diag && sample < 12) {
					fpart_bank_to_ce_cau(w, bank, &ce, &cau);
					dev_info(w->dev,
						 "FPART_META_SAMPLE n=%u bank=%u ce=%u cau=%u blk=%u pg=%u meta=%16ph data00=%32ph data80=%32ph\n",
						 sample, bank, ce, cau, blk, p,
						 meta, page,
						 page + FPART_SPECIAL_HDR);
					sample++;
				}
				{
					unsigned int s, interesting = 0;

					for (s = 0; s < 4; s++)
						if (fpart_meta_interesting(meta +
									    s * WHIMORY_META_SIZE))
							interesting++;
					if (ftl_diag && interesting &&
					    w->fpart_ctx.slot_logs < 48) {
						fpart_bank_to_ce_cau(w, bank, &ce, &cau);
						for (s = 0; s < 4; s++) {
							const u8 *m = meta + s * WHIMORY_META_SIZE;
							const u8 *d = page + s * WHIMORY_LBA_SIZE;

							if (!fpart_meta_interesting(m) &&
							    s != 0)
								continue;
							dev_info(w->dev,
								 "FPART_SLOTS n=%u bank=%u ce=%u cau=%u blk=%u pg=%u slot=%u type=%02x chunk=%02x tw=0x%04x meta=%16ph data=%32ph\n",
								 w->fpart_ctx.slot_logs,
								 bank, ce, cau, blk, p, s,
								 m[0], m[1],
								 get_unaligned_le16(m + 2),
								 m, d);
						}
						w->fpart_ctx.slot_logs++;
					}
				}
				special = fpart_meta_is_assign(meta, &type_word);
				if (!special)
					special = fpart_meta_special(meta, 0,
								     &type_word);
				magic = fpart_has_xrmw(page);
				if (fpart_has_wrmx(page))
					wrmx++;
				if (magic)
					xrmw++;
				if (!special)
					continue;
				tag30++;
				if (ftl_diag) {
					fpart_bank_to_ce_cau(w, bank, &ce, &cau);
					dev_info(w->dev,
						 "FPART_ASSIGN_SCAN bank=%u ce=%u cau=%u block=%u page=%u slot=%d type_word=0x%04x blank=%d m0=%16ph m1=%16ph m2=%16ph m3=%16ph data00=%32ph data80=%32ph\n",
						 bank, ce, cau, blk, p,
						 fpart_meta_special_slot(meta, 0, NULL),
						 type_word,
						 whimory_page_blank(page, 256),
						 meta, meta + 16, meta + 32,
						 meta + 48,
						 page, page + FPART_SPECIAL_HDR);
				}
				if (whimory_page_blank(page, 256)) {
					dev_info(w->dev,
						 "FPART_ASSIGN skip blank data type_word=0x%04x\n",
						 type_word);
					continue;
				}
				if (fpart_data_is_assignment(page, nbanks,
							     w->geom.blocks_per_cau)) {
					pairs = fpart_ingest_pairs(w, page,
								   type_word);
					dev_info(w->dev,
						 "FPART_ASSIGN_PAGE type_word=0x%04x pairs=%u count=%u\n",
						 type_word, pairs,
						 w->fpart_ctx.count);
				} else {
					obj_len = get_unaligned_le32(page +
							FPART_SPECIAL_LEN_OFF);
					if (magic ||
					    (obj_len && obj_len != 0xffffffffu &&
					     obj_len < 0x100000u)) {
						if (fpart_cache_add(w, bank,
								    blk,
								    type_word) > 0)
							dev_info(w->dev,
								 "FPART_ASSIGN_ADD bank=%u block=%u type=0x%04x (object chunk0)\n",
								 bank, blk,
								 type_word);
					}
				}
				if ((type_word & 0xff) == (type & 0xff))
					*matched = true;
			}
		}
	}
	dev_info(w->dev,
		 "FPART_SCAN blk[%u,%u) pages[%u,%u] reads=%d fail=%d tag30=%d xrmw=%d wrmx=%d entries=%u matched=%d meta0_top=%02x/%u %02x/%u %02x/%u %02x/%u\n",
		 block_lo, block_hi, page_lo, page_hi, reads, fail, tag30, xrmw,
		 wrmx, w->fpart_ctx.count, *matched,
		 0xff, hist[0xff], 0x00, hist[0], 0x30, hist[0x30], 0x20,
		 hist[0x20]);
	fpart_log_special_types(w);
	if (!sess)
		s5l8740_nand_dma_session_end();
	kfree(hist);
	kvfree(csp);
	kvfree(page);
	return 0;
}

/*
 * fpart_locate_special_4EBBDC: cache by low byte, else scan tail assignment
 * pages (META 0x30 chunk 0). scanned=true after a full miss so we do not
 * rescan.op=4 bitmap is not ported — every tail block is read.
 */
static bool fpart_locate_special(struct whimory *w, u16 *index, u16 type)
{
	u32 tail, start, nblk = w->geom.blocks_per_cau;
	unsigned int npg = fpart_assign_pages ? fpart_assign_pages : 1;
	bool matched = false;

	if (fpart_find_in_cache(w, index, type))
		return true;
	if (w->fpart_ctx.scanned)
		return false;

	tail = sig_scan_blocks ? sig_scan_blocks : w->geom.vfl_tail;
	if (!tail)
		tail = 128;
	if (tail > nblk)
		tail = nblk;
	start = nblk - tail;

	dev_info(w->dev,
		 "FPART_LOCATE type=0x%04x tail=%u start=%u pages=0..%u banks=%u\n",
		 type, tail, start, npg - 1, fpart_num_banks(w));

	if (fpart_scan_region(w, type, start, nblk, 0, npg - 1, &matched))
		return false;

	if (!matched && sig_brute_scan && start) {
		dev_info(w->dev,
			 "FPART_LOCATE brute remaining blk[0,%u) (debug)\n",
			 start);
		fpart_scan_region(w, type, 0, start, 0, 0, &matched);
	}

	w->fpart_ctx.scanned = true;
	if (!fpart_find_in_cache(w, index, type)) {
		dev_info(w->dev,
			 "FPART_LOCATE type=0x%04x miss entries=%u\n",
			 type, w->fpart_ctx.count);
		return false;
	}
	return true;
}

/*
 * fpart_read_special_copy_4F1420.
 * Chunk 0: META 30 <chunk> type_word; object_len @+0x24, gen @+0x28,
 * payload @+0x80. Later chunks: payload @+0, dst off = page_size*chunk-128.
 */
static int fpart_read_special_copy(struct whimory *w, u8 *dst, u32 dst_len,
				   u16 entry_i, u32 *gen_out)
{
	struct fpart_special_entry *e;
	struct s5l8740_cs_page *csp;
	u8 *page;
	u8 meta[S5L8740_NAND_META_SIZE];
	u32 page_size, chunk_count = 1, copy_slots, chunk, slot;
	u32 object_len = 0, copy_len = 0, generation = 0;
	int ret = -ENOENT;
	int sess;

	if (entry_i >= w->fpart_ctx.count)
		return -EINVAL;
	e = &w->fpart_ctx.table[entry_i];
	page_size = w->geom.page_size;
	copy_slots = w->geom.pages_per_block;
	if (!page_size || !copy_slots)
		return -EINVAL;

	page = kvmalloc(page_size, GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	csp = kvmalloc(sizeof(*csp), GFP_KERNEL);
	if (!csp) {
		kvfree(page);
		return -ENOMEM;
	}

	/* Same as the scan: cs_phys needs live CS armed, and -EBUSY only
	 * means someone outside already armed it.
	 */
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		dev_warn(w->dev, "FPART_COPY no DMA session (%d)\n", sess);
		kvfree(csp);
		kvfree(page);
		return sess;
	}

	for (chunk = 0; chunk < chunk_count; chunk++) {
		bool got = false;

		for (slot = 0; slot < copy_slots; slot++) {
			u32 pg = chunk + slot * chunk_count;
			u16 meta_type = 0;

			if (pg >= w->geom.pages_per_block)
				break;
			cond_resched();
			ret = fpart_fil_read_page(w, e->bank, e->block, pg,
						  csp, page, meta);
			if (ret)
				continue;
			if (!fpart_meta_special(meta, chunk, &meta_type))
				continue;
			if (meta_type != e->type_word)
				continue;
			got = true;

			if (chunk == 0) {
				object_len = get_unaligned_le32(page +
								FPART_SPECIAL_LEN_OFF);
				generation = get_unaligned_le32(page +
								FPART_SPECIAL_GEN_OFF);
				if (!object_len || object_len == 0xffffffffu) {
					got = false;
					continue;
				}
				copy_len = min(object_len, dst_len ? dst_len :
					       object_len);
				chunk_count = DIV_ROUND_UP(copy_len +
							   FPART_SPECIAL_HDR,
							   page_size);
				if (!chunk_count)
					chunk_count = 1;
				copy_slots = w->geom.pages_per_block /
					     chunk_count;
				if (!copy_slots)
					copy_slots = 1;
				dev_info(w->dev,
					 "FPART_SPECIAL_COPY entry=%u bank=%u block=%u type_word=0x%04x chunk0 page=%u meta=%16ph object_len=0x%x gen=%u raw00=%32ph raw80=%32ph\n",
					 entry_i, e->bank, e->block,
					 e->type_word, pg, meta, object_len,
					 generation, page,
					 page + FPART_SPECIAL_HDR);
				if (dst && dst_len) {
					u32 n = min(page_size - FPART_SPECIAL_HDR,
						    copy_len);

					n = min(n, dst_len);
					memcpy(dst, page + FPART_SPECIAL_HDR, n);
				}
			} else if (dst && dst_len) {
				u32 dst_off = page_size * chunk - FPART_SPECIAL_HDR;
				u32 n;

				if (dst_off >= dst_len || dst_off >= copy_len)
					break;
				n = min(page_size, copy_len - dst_off);
				n = min(n, dst_len - dst_off);
				memcpy(dst + dst_off, page, n);
			}
			break;
		}
		if (!got) {
			ret = -ENOENT;
			goto out;
		}
	}
	if (gen_out)
		*gen_out = generation;
	if (dst && dst_len)
		dev_info(w->dev,
			 "FPART_SPECIAL_PAYLOAD entry=%u first32=%32ph\n",
			 entry_i, dst);
	ret = 0;
out:
	if (!sess)
		s5l8740_nand_dma_session_end();
	kvfree(csp);
	kvfree(page);
	return ret;
}

/* Newest generation among contiguous low-byte copies (4F12DC / 4EB428). */
static int fpart_read_special_by_index(struct whimory *w, u8 *dst, u32 len,
				       u16 index)
{
	u16 type_word, copies, i, best_i = 0;
	bool have = false;
	u32 best_gen = 0;

	if (index >= w->fpart_ctx.count)
		return -EINVAL;
	type_word = w->fpart_ctx.table[index].type_word;
	copies = fpart_count_special_copies(w, type_word);
	dev_info(w->dev,
		 "FPART_READ_SPECIAL type=0x%04x index=%u copies=%u class=%u\n",
		 type_word, index, copies, (type_word >> 8) &
		 FPART_SPECIAL_CLASS_MASK);

	for (i = 0; i < copies; i++) {
		u16 entry_i = index + i;
		u32 gen = 0;
		int ok;

		if (entry_i >= w->fpart_ctx.count)
			break;
		if ((w->fpart_ctx.table[entry_i].type_word & 0xff) !=
		    (type_word & 0xff))
			break;
		dev_info(w->dev,
			 "FPART_READ_SPECIAL copy=%u bank=%u block=%u type_word=0x%04x\n",
			 i, w->fpart_ctx.table[entry_i].bank,
			 w->fpart_ctx.table[entry_i].block,
			 w->fpart_ctx.table[entry_i].type_word);
		ok = fpart_read_special_copy(w, have ? NULL : dst,
					     have ? 0 : len, entry_i, &gen);
		if (ok)
			continue;
		if (!have) {
			have = true;
			best_gen = gen;
			best_i = entry_i;
			continue;
		}
		if (gen > best_gen) {
			if (!fpart_read_special_copy(w, dst, len, entry_i,
						     &gen)) {
				best_gen = gen;
				best_i = entry_i;
			} else if (fpart_read_special_copy(w, dst, len,
							   best_i, NULL)) {
				return -EIO;
			}
		}
	}
	if (!have)
		return -ENOENT;
	dev_info(w->dev,
		 "FPART_READ_SPECIAL selected entry=%u gen=%u\n",
		 best_i, best_gen);
	return 0;
}

/*
 * fpart_read_special_common_4EEB68 / vtable +80.
 * Do NOT expect xrmw at raw page offset 0 — payload is after the 0x80 header.
 */
static int whimory_fpart_read_special(struct whimory *w, u32 type, u8 *buf,
				      size_t len)
{
	u16 index = (u16)type;
	u16 type_word;

	if (!buf || !len || type > 0xffff)
		return -EINVAL;

	memset(buf, 0xa5, len);
	if (!fpart_locate_special(w, &index, (u16)type))
		return -ENOENT;

	type_word = w->fpart_ctx.table[index].type_word;
	dev_info(w->dev,
		 "FPART_LOCATE type=0x%04x index=%u entry bank=%u block=%u type_word=0x%04x class=%u\n",
		 type, index, w->fpart_ctx.table[index].bank,
		 w->fpart_ctx.table[index].block, type_word,
		 (type_word >> 8) & FPART_SPECIAL_CLASS_MASK);
	if (!fpart_type_class1(type_word))
		return -EINVAL;

	return fpart_read_special_by_index(w, buf, len, index);
}

static int n31_fpart_read_signature(struct whimory *w, u8 *buf, size_t len)
{
	return whimory_fpart_read_special(w, WHIMORY_SIG_TYPE, buf, len);
}

static const struct whimory_fpart_ops n31_ppn_fpart_ops = {
	.major = 0,
	.minor = n31_fpart_minor,
	.init = n31_fpart_init,
	.read_special = whimory_fpart_read_special,
	.read_signature = n31_fpart_read_signature,
};

static void whimory_dump256(struct whimory *w, const char *tag, const u8 *p)
{
	unsigned int i;

	for (i = 0; i < 256; i += 32)
		dev_info(w->dev, "%s +0x%02x: %32ph\n", tag, i, p + i);
}

static int whimory_payload_read_page(struct whimory *w, u16 bank, u32 block,
				     u32 page, void *data, unsigned int *slc_out)
{
	unsigned int ce, cau, i;
	const unsigned int slc_order[2] = { 1, 0 };
	int last = -EIO;

	fpart_bank_to_ce_cau(w, bank, &ce, &cau);
	if (ce >= w->geom.num_ce || cau >= w->geom.num_cau ||
	    block >= w->geom.blocks_per_cau ||
	    page >= w->geom.pages_per_block)
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		int ret;

		ret = s5l8740_nand_page_read(ce, cau, block, page, slc_order[i],
					     16, data, w->geom.page_size,
					     NULL, 0);
		if (ret) {
			last = ret;
			continue;
		}
		if (slc_out)
			*slc_out = slc_order[i];
		/* One successful FIL read is enough — do not MLC-retry blanks. */
		return 0;
	}
	return last;
}

static int whimory_payload_check_hit(struct whimory *w, u16 bank, u32 blk,
				     u32 pg, unsigned int slc, const u8 *page)
{
	unsigned int ce, cau, i;
	const unsigned int offs[2] = { 0, FPART_SPECIAL_HDR };

	fpart_bank_to_ce_cau(w, bank, &ce, &cau);
	for (i = 0; i < 2; i++) {
		unsigned int off = offs[i];
		u32 mag;
		const char *kind;

		if (off + 4 > w->geom.page_size)
			continue;
		mag = get_unaligned_le32(page + off);
		if (mag == WHIMORY_SIG_MAGIC)
			kind = "xrmw";
		else if (mag == WHIMORY_SIG_MAGIC_WRMX)
			kind = "wrmx";
		else
			continue;
		dev_info(w->dev,
			 "PAYLOAD_MAGIC hit kind=%s off=0x%x bank=%u ce=%u cau=%u blk=%u pg=%u slc=%u\n",
			 kind, off, bank, ce, cau, blk, pg, slc);
		whimory_dump256(w, "PAYLOAD_MAGIC data00", page);
		whimory_dump256(w, "PAYLOAD_MAGIC data80", page + FPART_SPECIAL_HDR);
		if (mag == WHIMORY_SIG_MAGIC &&
		    off + WHIMORY_SIG_SIZE <= w->geom.page_size &&
		    !whimory_parse_signature(w, page + off))
			dev_info(w->dev, "PAYLOAD_MAGIC parsed xrmw as signature\n");
		return 1;
	}
	return 0;
}

static int whimory_payload_scan_range(struct whimory *w, void *page,
				      u32 block_lo, u32 block_hi,
				      unsigned int page_lo, unsigned int page_hi,
				      const char *why, int *reads)
{
	u16 bank, nbanks = fpart_num_banks(w);
	u32 b, p;

	/*
	 * Clamp to the SLC page count, not the MLC one.
	 *
	 * This region is SLC and an SLC block holds geom_105 pages, 16 on
	 * this part, against 128 in MLC. Sweeping to pages_per_block asks
	 * for pages 16..127, which do not exist there -- and the failures
	 * do not stay local: measured over blk[1960,2088), pages[0,127]
	 * gave 65536 reads, fail=0 and tag30=0, with 97 per cent of pages
	 * reading back as meta 0x00. Every object in the region vanished,
	 * including the five on page 0 that the same code finds every
	 * time when it stops at page 0. The same sweep bounded to
	 * pages[0,15] gives 8192 reads, fail=0, tag30=80 and entries=5:
	 * the five objects, each occupying all 16 SLC pages of its block.
	 *
	 * So an out-of-range SLC page does not merely fail, it takes the
	 * reads after it with it, and the result looks like a clean scan
	 * of an empty region rather than like an error.
	 */
	if (w->geom.geom_105 && page_hi >= w->geom.geom_105)
		page_hi = w->geom.geom_105 - 1;
	else if (page_hi >= w->geom.pages_per_block)
		page_hi = w->geom.pages_per_block - 1;
	if (block_hi > w->geom.blocks_per_cau)
		block_hi = w->geom.blocks_per_cau;
	if (block_lo >= block_hi)
		return 0;

	dev_info(w->dev,
		 "PAYLOAD_SCAN %s blk[%u,%u) pages[%u,%u] banks=%u\n",
		 why, block_lo, block_hi, page_lo, page_hi, nbanks);

	for (bank = 0; bank < nbanks; bank++) {
		for (b = block_hi; b > block_lo; b--) {
			u32 blk = b - 1;

			for (p = page_lo; p <= page_hi; p++) {
				unsigned int slc = 0;
				int ret;

				cond_resched();
				ret = whimory_payload_read_page(w, bank, blk, p,
								page, &slc);
				(*reads)++;
				if (!(*reads % 512))
					dev_info(w->dev,
						 "PAYLOAD_SCAN %s progress reads=%d blk=%u pg=%u\n",
						 why, *reads, blk, p);
				if (ret)
					continue;
				if (whimory_payload_check_hit(w, bank, blk, p,
							      slc, page))
					return 1;
			}
		}
	}
	return 0;
}

/*
 * Data-only xrmw/wrmx hunt. PIO last_spare is not Sogeti META. Abort on
 * first hit. No classify / L2V.
 */
static int whimory_payload_magic_scan(struct whimory *w)
{
	void *page;
	int reads = 0, hit;
	u32 nblk = w->geom.blocks_per_cau;
	u32 user = w->geom.user_blocks;
	u32 around = 1461;

	if (!user || user > nblk)
		user = nblk > 128 ? nblk - 128 : nblk;
	if (around >= nblk)
		around = nblk / 2;

	page = kvmalloc(w->geom.page_size, GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	s5l8740_nand_reset();
	hit = whimory_payload_scan_range(w, page, user, nblk, 0,
					 w->geom.pages_per_block - 1,
					 "tail", &reads);
	if (!hit)
		hit = whimory_payload_scan_range(w, page, 0, user, 0, 0,
						 "user-pg0", &reads);
	if (!hit && around + 1 < nblk)
		hit = whimory_payload_scan_range(w, page, around, around + 1, 0,
						 w->geom.pages_per_block - 1,
						 "blk1461", &reads);
	dev_info(w->dev, "PAYLOAD_SCAN done hit=%d reads=%d sig=%d\n",
		 hit, reads, w->sig_ok);
	kvfree(page);
	return hit;
}

static int whimory_read_signature(struct whimory *w)
{
	int ret;

	w->fpart = &n31_ppn_fpart_ops;
	ret = w->fpart->init(w);
	if (ret)
		return ret;
	if (payload_magic_scan)
		whimory_payload_magic_scan(w);
	ret = w->fpart->read_signature(w, w->sig.raw, WHIMORY_SIG_SIZE);
	if (ret) {
		dev_warn(w->dev,
			 "FPart special 0xC101 miss (%d). sig=0 is not a native open.\n",
			 ret);
		if (!allow_sigless_debug) {
			dev_err(w->dev,
				"allow_sigless_debug=0: refusing VFL/FTL without signature\n");
			return ret;
		}
		dev_warn(w->dev,
			 "allow_sigless_debug=1: classify/recover anyway (research only)\n");
		return 0;
	}
	return whimory_parse_signature(w, w->sig.raw);
}

static u32 n31_vfl_minor(struct whimory *w)
{
	return w->sig.vfl_minor;
}

static u32 n31_sftl_minor(struct whimory *w)
{
	return w->sig.ftl_minor;
}

/* ------------------------------------------------------------------ */
/* VFL */
/* ------------------------------------------------------------------ */

static int n31_vfl_init(struct whimory *w)
{
	unsigned int cau, i, n, cxt_len;

	n = w->geom.blocks_per_cau;
	if (!n)
		return -EINVAL;
	cxt_len = 16;
	w->vfl.cxt_u16_len = cxt_len;
	w->vfl.bank_stride = 1;
	w->vfl.bank_mask = kvmalloc(n, GFP_KERNEL);
	if (!w->vfl.bank_mask)
		return -ENOMEM;
	memset(w->vfl.bank_mask, (1u << min_t(u32, w->geom.num_cau, 8)) - 1, n);
	w->vfl.cached_vbn = 0xffff;
	w->vfl.cached_n = 0;
	for (cau = 0; cau < w->geom.num_cau; cau++) {
		w->vfl.remap[cau] = kvmalloc_array(n, sizeof(u32), GFP_KERNEL);
		w->vfl.cxt_u16[cau] = kvmalloc_array(cxt_len, sizeof(u16),
						     GFP_KERNEL);
		if (!w->vfl.remap[cau] || !w->vfl.cxt_u16[cau])
			return -ENOMEM;
		for (i = 0; i < n; i++)
			w->vfl.remap[cau][i] = i;
		for (i = 0; i < cxt_len; i++)
			w->vfl.cxt_u16[cau][i] = 0xffff;
		w->vfl.ctx_block[cau] = ~0u;
	}
	return 0;
}

/*
 * Ingest the VFL context object.
 *
 * What this device keeps in its system area, enumerated completely --
 * 128 blocks, all 16 SLC pages, 4 banks, 8192 reads, no failures --
 * and dumped past each header:
 *
 *   0xc101  bank 1 blk 2085  signature: magic, versions, geometry
 *   0xc104  bank 1 blk 2084  block status bitmap, mirror on bank 2
 *   0xc105  bank 0 blk 2085  device identity, mirror on bank 3
 *
 * 0xc104 is the bitmap and nothing else: everything past the 261 bytes
 * it needs for 2088 blocks is 0xff, and all 16 pages of the block are
 * byte-identical, so the object is one bitmap replicated for
 * redundancy. 0xc105 is four-character tags stored reversed -- SCfg,
 * SrNm with the serial, FwId, HwId, HwVr, SwVr, MLB#, Regn, BMac,
 * Mod#.
 *
 * The FPart data was checked too, both halves of it, since that is the
 * obvious place to look next. The first 0x80 bytes of each object are
 * (bank, block) assignment pairs, and they are self-describing rather
 * than a directory of anything else: 0xc101 lists bank 1 blk 2085,
 * 0xc104 lists bank 2 blk 2085 and bank 1 blk 2084, 0xc105 lists bank
 * 3 and bank 0. Five entries, which is exactly the five copies of the
 * three objects above. And the signature is 0x600 bytes of which only
 * the first 0x40 carry anything: magic, versions, geometry, the string
 * "00230g)" at +0x40, one config byte at +0xb8, and zeros to the end.
 *
 * So nothing here points at the FTL checkpoint, which is what a
 * context is normally wanted for. openiBoot vsvfl reaches the FTL
 * through control_block[3] in its VFLCxt; this part has no equivalent
 * field. The checkpoint can only be found by classifying superblocks,
 * which is what recovery already does, so reading the context cannot
 * remove the scan -- the context does not know where the map is.
 *
 * The bitmap is loaded and reported but deliberately not used to skip
 * blocks. 29 blocks are marked, all inside the first 256, in a regular
 * pattern of two or three low bits per byte, which reads more like
 * reserved blocks than like wear scattered across the device. Skipping
 * a block that actually holds data would lose it, and whether a clear
 * bit means bad or reserved is not established.
 *
 * Four things here were wrong, and each of them alone was enough to
 * make the scan find nothing.
 *
 * It looked for meta[0] == 0x20. Nothing on this part carries that.
 * FPART_SCAN counts the whole system area and reports
 * meta0_top=ff/454 00/5 30/5 20/0 -- 454 erased pages, five of type
 * 0x30, and not one of 0x20. The system objects are all meta[0] 0x30,
 * the FPart special type, and what separates them is a type word at
 * meta+2. Enumerating all five across all four banks:
 *
 *   bank 1 blk 2085  0xc101  payload "xrmw"          the signature
 *   bank 0 blk 2085  0xc105  payload "SCfg".."SrNm"  config, serial
 *   bank 3 blk 2085  0xc105  mirror of bank 0
 *   bank 1 blk 2084  0xc104  fc f8 fc fc .. ff ff    block status
 *   bank 2 blk 2085  0xc104  mirror of bank 1
 *
 * 0xc105 is identity, not a map. 0xc104 is the only structural object
 * of the three, it is mirrored across CEs the way context is, and a
 * bitmap over 2088 blocks is what a block status table looks like. So
 * the context is keyed on 0xc104.
 *
 * It read from 0x100 and 0x200. An FPart object payload begins at
 * FPART_SPECIAL_HDR, 0x80, so those offsets land 0x80 and 0x180 bytes
 * into the object rather than at its start. That is why cxt_loc was
 * always 0: there was never anything at 0x100 to count.
 *
 * It read the bitmap as one byte per block with the bits meaning
 * banks, and validated it by rejecting any byte with bits set outside
 * num_cau -- mask 0x3 here. Every byte actually present is fc, f8 or
 * ff, so that test rejects all of them. It is a bit per block, not a
 * byte per block.
 *
 * And it took whichever copy was scanned last. There are two, and a
 * later one silently replaced an earlier one. The first valid copy now
 * wins and a disagreeing mirror is reported rather than applied.
 *
 * The block status table is kept separately from vfl.bank_mask. That
 * one is a byte per VBN whose bits select banks, whimory_vfl_bank()
 * resolves every read through it, and it is not this. Loading one into
 * the other is what sent reads to the wrong die when this function was
 * matching the signature block.
 */
/*
 * SysCfg tags are stored byte-reversed, so "SrNm" is 6d 4e 72 53.
 */
static bool whimory_syscfg_tag(const u8 *p, const char *tag)
{
	return p[0] == tag[3] && p[1] == tag[2] &&
	       p[2] == tag[1] && p[3] == tag[0];
}

static void whimory_syscfg_str(char *dst, size_t dstlen, const u8 *src,
			       size_t len)
{
	size_t i, n = min(len, dstlen - 1);

	for (i = 0; i < n; i++) {
		/*
		 * Stop at the first byte that is not printable text.
		 *
		 * Padding is NUL or 0xff, but a value can also be followed
		 * directly by binary belonging to the next field -- MLB# is,
		 * and substituting a placeholder for it put a stray "?" on the
		 * end of the board number. Two tags can also sit adjacent with
		 * no value between them, which CNTB and MtCl do, and that is a
		 * legitimately empty string rather than garbage.
		 */
		if (src[i] < 0x20 || src[i] > 0x7e)
			break;
		dst[i] = src[i];
	}
	dst[i] = 0;
}

/*
 * Parse the SysCfg object into something a person can read.
 *
 * The record stride is not constant -- the serial takes 16 bytes and the
 * board number more -- so rather than assume one, this locates every tag
 * it knows and treats each value as running to the next tag found. That
 * is robust to a size we guessed wrong and to tags we have not seen.
 */
static void whimory_syscfg_parse(struct whimory *w, const u8 *p,
				 unsigned int len)
{
	static const char * const tags[] = {
		"SrNm", "FwId", "HwId", "HwVr", "SwVr", "MLB#",
		"CNTB", "MtCl", "Mod#", "Regn", "BMac",
	};
	unsigned int off[ARRAY_SIZE(tags)];
	unsigned int i, o, n = 0;

	kvfree(w->syscfg.raw);
	memset(&w->syscfg, 0, sizeof(w->syscfg));
	if (len < 24 || !whimory_syscfg_tag(p, "SCfg"))
		return;
	w->syscfg.entries = get_unaligned_le32(p + 0x14);

	/*
	 * Keep the section itself, and index every 4-byte-aligned printable
	 * group in it.
	 *
	 * The decode below understands eleven tags. Anything else in the
	 * section is unreachable, and this part carries records that have not
	 * been identified. A printable group is a candidate rather than a
	 * confirmed tag -- ASCII values such as the serial match the same
	 * test -- so the candidates are reported as such, flagged when they
	 * are one of the known tags, and the verbatim bytes are kept beside
	 * them so an unrecognised record can still be read out.
	 */
	w->syscfg.raw = NULL;
	w->syscfg.raw_len = 0;
	w->syscfg.n_cand = 0;
	w->syscfg.raw = kvmalloc(len, GFP_KERNEL);
	if (w->syscfg.raw) {
		memcpy(w->syscfg.raw, p, len);
		w->syscfg.raw_len = len;
	}

	for (o = 0; o + 4 <= len && w->syscfg.n_cand < N31_SYSCFG_MAX_CAND;
	     o += 4) {
		unsigned int k, idx;
		char *t;

		for (k = 0; k < 4; k++)
			if (p[o + k] < 0x20 || p[o + k] > 0x7e)
				break;
		if (k != 4)
			continue;

		idx = w->syscfg.n_cand++;
		t = w->syscfg.cand[idx].tag;
		/* Tags sit byte-reversed on disk; record them forwards. */
		t[0] = p[o + 3];
		t[1] = p[o + 2];
		t[2] = p[o + 1];
		t[3] = p[o + 0];
		t[4] = 0;
		w->syscfg.cand[idx].off = o;
		w->syscfg.cand[idx].known = false;
		for (k = 0; k < ARRAY_SIZE(tags); k++)
			if (!strcmp(t, tags[k]))
				w->syscfg.cand[idx].known = true;
	}
	/* A record runs to the next candidate, or to the end of the section. */
	for (i = 0; i < w->syscfg.n_cand; i++)
		w->syscfg.cand[i].len =
			((i + 1 < w->syscfg.n_cand) ? w->syscfg.cand[i + 1].off
						    : len) -
			w->syscfg.cand[i].off;

	for (i = 0; i < ARRAY_SIZE(tags); i++)
		off[i] = 0;

	/* Locate every tag we know, on its 4-byte alignment. */
	for (o = 4; o + 4 <= len; o += 4) {
		for (i = 0; i < ARRAY_SIZE(tags); i++) {
			if (!off[i] && whimory_syscfg_tag(p + o, tags[i])) {
				off[i] = o;
				n++;
				break;
			}
		}
	}
	if (!n)
		return;

	for (i = 0; i < ARRAY_SIZE(tags); i++) {
		unsigned int vstart, vend, j;

		if (!off[i])
			continue;
		vstart = off[i] + 4;
		vend = len;
		/* Value runs to the next tag that was found after this one. */
		for (j = 0; j < ARRAY_SIZE(tags); j++)
			if (off[j] > off[i] && off[j] < vend)
				vend = off[j];
		if (vend <= vstart)
			continue;

		if (!strcmp(tags[i], "SrNm"))
			whimory_syscfg_str(w->syscfg.serial,
					   sizeof(w->syscfg.serial),
					   p + vstart, vend - vstart);
		else if (!strcmp(tags[i], "Mod#"))
			whimory_syscfg_str(w->syscfg.model,
					   sizeof(w->syscfg.model),
					   p + vstart, vend - vstart);
		else if (!strcmp(tags[i], "MLB#"))
			whimory_syscfg_str(w->syscfg.mlb,
					   sizeof(w->syscfg.mlb),
					   p + vstart, vend - vstart);
		else if (!strcmp(tags[i], "SwVr"))
			whimory_syscfg_str(w->syscfg.sw_ver,
					   sizeof(w->syscfg.sw_ver),
					   p + vstart, vend - vstart);
		else if (!strcmp(tags[i], "CNTB"))
			whimory_syscfg_str(w->syscfg.cnt_b,
					   sizeof(w->syscfg.cnt_b),
					   p + vstart, vend - vstart);
		else if (!strcmp(tags[i], "MtCl"))
			whimory_syscfg_str(w->syscfg.mt_cl,
					   sizeof(w->syscfg.mt_cl),
					   p + vstart, vend - vstart);
		else if (!strcmp(tags[i], "BMac") && vend - vstart >= 6) {
			memcpy(w->syscfg.mac, p + vstart, 6);
			w->syscfg.mac_ok = true;
		} else if (!strcmp(tags[i], "Regn") && vend - vstart >= 4)
			w->syscfg.region = get_unaligned_le32(p + vstart);
		else if (!strcmp(tags[i], "HwVr") && vend - vstart >= 8)
			w->syscfg.hw_ver = get_unaligned_le32(p + vstart + 4);
		else if (!strcmp(tags[i], "FwId") && vend - vstart >= 8)
			w->syscfg.fw_id = get_unaligned_le32(p + vstart + 4);
	}

	/*
	 * Touch calibration: the panel calibration block, kept for apple-grape.
	 *
	 * The SysCfg object carries a second region past the identity records,
	 * at 0x1dc0 into the payload. It is not identity and it is not map
	 * data: tags NI and MB, then per-node byte tables and 16-bit pairs.
	 * That is panel calibration, and it belongs to the touch controller
	 * rather than to storage -- but this is the only place it exists, so
	 * the FTL is where it has to be read from.
	 *
	 * Copied verbatim rather than interpreted. The layout is the touch
	 * driver problem, and guessing at it here would only add a second
	 * opinion about bytes this code has no stake in.
	 */
	/*
	 * Look for a real touch calibration container anywhere in the page.
	 *
	 * apple-grape wants the blob whose magic word is 0x53797349, and
	 * takes its calibration from decimal +350 for 0x200 bytes,
	 * byte-swapped per u32 -- so the container has to be at least 862
	 * bytes long. The region this driver has been exposing as touch_cal
	 * lives at a fixed offset and carries no such magic, so it is some
	 * other SysCfg record and cannot be what the touch controller is
	 * asking for. Scan for the magic rather than trusting the offset,
	 * and report what is actually there.
	 */
	w->syscfg.touch_cal_magic_off = -1;
	for (i = 0; i + 4 <= len; i += 4) {
		if (get_unaligned_le32(p + i) == 0x53797349u) {
			w->syscfg.touch_cal_magic_off = (int)i;
			dev_info(w->dev,
				 "SysCfg touch calibration magic at +0x%x (%u bytes to end of page)\n",
				 i, len - i);
			break;
		}
	}
	if (w->syscfg.touch_cal_magic_off < 0)
		dev_info(w->dev,
			 "SysCfg has no touch calibration magic; touch_cal region is a different record\n");

	/*
	 * Copy the container the scan actually found, whole.
	 *
	 * Stock copies 0x560 bytes from the calibration descriptor and apple-grape
	 * reads its calibration at +350 for 0x200, so the consumer needs
	 * 862 bytes measured from the magic. Two things here used to make
	 * that impossible:
	 *
	 *   - the copy came from N31_TOUCH_CAL_OFF, a fixed offset that the scan
	 *     above reports carries no touch calibration magic, so it was some other
	 *     SysCfg record entirely;
	 *   - the length was trimmed back to the last byte that was neither
	 *     0x00 nor 0xff, which silently shortens any blob whose tail is
	 *     padding -- and calibration data ending in zeros is ordinary.
	 *
	 * So: take it from the magic when the magic is present, keep the
	 * full length, and say so when the page cannot supply all of it.
	 * N31_TOUCH_CAL_OFF remains only as the fallback for a page with no
	 * magic at all, where there is nothing better to point at.
	 */
	if (w->syscfg.touch_cal_magic_off >= 0) {
		unsigned int off = (unsigned int)w->syscfg.touch_cal_magic_off;
		unsigned int n = min_t(unsigned int, len - off,
				       (unsigned int)sizeof(w->syscfg.touch_cal));

		memcpy(w->syscfg.touch_cal, p + off, n);
		w->syscfg.touch_cal_len = n;
		if (n < N31_TOUCH_CAL_LEN)
			dev_warn(w->dev,
				 "SysCfg touch calibration truncated: %u of %u bytes (page ends %u after the magic)\n",
				 n, N31_TOUCH_CAL_LEN, len - off);
		if (n < N31_TOUCH_CAL_CAL_OFF + N31_TOUCH_CAL_CAL_LEN)
			dev_warn(w->dev,
				 "SysCfg touch calibration too short for the touch calibration window (need %u, have %u)\n",
				 N31_TOUCH_CAL_CAL_OFF + N31_TOUCH_CAL_CAL_LEN, n);
	} else {
		/*
		 * No magic: this page does not carry the blob, so do not
		 * present anything as if it did.
		 *
		 * The authoritative copy comes from the A34 handoff, which
		 * U-Boot republishes in reserved DRAM and advertises as
		 * /chosen apple,n31-touch_cal-addr and apple,n31-touch_cal-size --
		 * measured on this device as 0x560 bytes, the same length
		 * stock copies and the same length apple-grape demands
		 * exactly. Handing a consumer 1376 bytes of some other
		 * SysCfg record would satisfy that length check with the
		 * wrong data, which is worse than returning nothing.
		 */
		w->syscfg.touch_cal_len = 0;
		dev_info(w->dev,
			 "SysCfg carries no touch calibration blob; the touch calibration source is the U-Boot region (/chosen apple,n31-touch_cal-*)\n");
	}
	w->syscfg.valid = true;
	dev_info(w->dev,
		 "SysCfg %u entries: serial=%s model=%s sw=%s\n",
		 w->syscfg.entries, w->syscfg.serial, w->syscfg.model,
		 w->syscfg.sw_ver);
	dev_info(w->dev, "SysCfg touch calibration: %u bytes\n",
		 w->syscfg.touch_cal_len);
}

/*
 * Hand the raw touch calibration block to whoever owns the panel. Returns its length,
 * or 0 when the SysCfg object has not been read.
 */
size_t whimory_syscfg_touch_cal(const u8 **out)
{
	struct whimory *w = whimory_dev;

	if (!w || !w->syscfg.valid || !w->syscfg.touch_cal_len)
		return 0;
	if (out)
		*out = w->syscfg.touch_cal;
	return w->syscfg.touch_cal_len;
}
EXPORT_SYMBOL_GPL(whimory_syscfg_touch_cal);

int whimory_touch_cal_show(char *buf, size_t len)
{
	const u8 *d = NULL;
	size_t n = whimory_syscfg_touch_cal(&d);
	int o = 0;
	size_t i;

	if (!n)
		return scnprintf(buf, len, "no touch_cal block\n");
	for (i = 0; i < n && o < (int)len - 80; i += 16)
		o += scnprintf(buf + o, len - o, "%04zx: %*ph\n",
				       i, (int)min_t(size_t, 16, n - i), d + i);
	return o;
}
EXPORT_SYMBOL_GPL(whimory_touch_cal_show);

int whimory_syscfg_show(char *buf, size_t len)
{
	struct whimory *w = whimory_dev;
	int n = 0;

	if (!w || !w->syscfg.valid)
		return scnprintf(buf, len, "syscfg not read\n");
	n += scnprintf(buf + n, len - n, "entries=%u\n", w->syscfg.entries);
	n += scnprintf(buf + n, len - n, "serial=%s\n", w->syscfg.serial);
	n += scnprintf(buf + n, len - n, "model=%s\n", w->syscfg.model);
	n += scnprintf(buf + n, len - n, "mlb=%s\n", w->syscfg.mlb);
	n += scnprintf(buf + n, len - n, "sw_version=%s\n", w->syscfg.sw_ver);
	n += scnprintf(buf + n, len - n, "cntb=%s\n", w->syscfg.cnt_b);
	n += scnprintf(buf + n, len - n, "mtcl=%s\n", w->syscfg.mt_cl);
	if (w->syscfg.mac_ok)
		n += scnprintf(buf + n, len - n, "bt_mac=%pM\n",
				       w->syscfg.mac);
	n += scnprintf(buf + n, len - n, "region=0x%08x\n", w->syscfg.region);
	n += scnprintf(buf + n, len - n, "hw_version=0x%08x\n", w->syscfg.hw_ver);
	n += scnprintf(buf + n, len - n, "fw_id=0x%08x\n", w->syscfg.fw_id);
	return n;
}
EXPORT_SYMBOL_GPL(whimory_syscfg_show);

int whimory_fpart_objects_show(char *buf, size_t len)
{
	struct whimory *w = whimory_dev;
	int n = 0;
	u16 i;

	if (!w)
		return scnprintf(buf, len, "no device\n");
	n += scnprintf(buf + n, len - n,
			  "# type_word bank ce cau block  what\n");
	for (i = 0; i < w->fpart_ctx.count && n < (int)len - 96; i++) {
		struct fpart_special_entry *e = &w->fpart_ctx.table[i];
		unsigned int ce, cau;
		const char *what;

		fpart_bank_to_ce_cau(w, e->bank, &ce, &cau);
		switch (e->type_word) {
		case FPART_TYPE_SIGNATURE:
			what = "signature";
			break;
		case FPART_TYPE_VFL_CXT:
			what = "block status bitmap";
			break;
		case FPART_TYPE_CONFIG:
			what = "SysCfg device identity";
			break;
		default:
			what = "unknown";
			break;
		}
		n += scnprintf(buf + n, len - n,
				       "%u 0x%04x %u %u %u %u  %s\n",
				       i, e->type_word, e->bank, ce, cau,
				       e->block, what);
	}
	return n;
}
EXPORT_SYMBOL_GPL(whimory_fpart_objects_show);

/*
 * Log only the parts of an object that carry something.
 *
 * These objects are a few hundred bytes of content in a 4 KiB page, and
 * dumping all of it buries the interesting part. This walks the page in
 * 16-byte rows and prints a row only when it is neither all zero nor all
 * 0xff, which is what fill looks like here.
 */
static void whimory_dump_sparse(struct whimory *w, const char *tag,
				const u8 *p, unsigned int len)
{
	unsigned int o, shown = 0;

	if (!fpart_dump)
		return;

	for (o = 0; o + 16 <= len && shown < 40; o += 16) {
		unsigned int i;
		bool zero = true, ones = true;

		for (i = 0; i < 16; i++) {
			if (p[o + i] != 0x00)
				zero = false;
			if (p[o + i] != 0xff)
				ones = false;
		}
		if (zero || ones)
			continue;
		shown++;
		dev_info(w->dev, "%s +%04x: %16ph\n", tag, o, p + o);
	}
	dev_info(w->dev, "%s: %u non-fill rows in %u bytes\n",
		 tag, shown, len);
}

static int n31_vfl_ingest_ctx(struct whimory *w, unsigned int ce,
			      unsigned int cau, unsigned int block,
			      const u8 *page, unsigned int page_len,
			      const u8 *meta)
{
	unsigned int nbytes, nblk, i, bad = 0;
	const u8 *payload;
	u16 type_word;

	if (!page || !meta)
		return 0;
	if (cau >= w->geom.num_cau || !w->vfl.remap[cau])
		return 0;
	if (meta[0] != FPART_META_TYPE_SPECIAL)
		return 0;

	type_word = get_unaligned_le16(meta + 2);
	if (type_word == FPART_TYPE_CONFIG && !w->syscfg.valid) {
		whimory_syscfg_parse(w, page + FPART_SPECIAL_HDR,
				     page_len - FPART_SPECIAL_HDR);
		whimory_dump_sparse(w, "C105", page, page_len);
	}
	if (type_word == FPART_TYPE_VFL_CXT && !w->vfl.bitmap_loaded)
		whimory_dump_sparse(w, "C104", page, page_len);
	if (type_word != FPART_TYPE_VFL_CXT)
		return 0;

	nblk = w->geom.blocks_per_cau;
	nbytes = DIV_ROUND_UP(nblk, 8);
	if (!nblk || page_len < FPART_SPECIAL_HDR + nbytes)
		return 0;
	payload = page + FPART_SPECIAL_HDR;

	if (!w->vfl.blk_status) {
		w->vfl.blk_status = kvmalloc(nbytes, GFP_KERNEL);
		if (!w->vfl.blk_status)
			return 0;
		memset(w->vfl.blk_status, 0xff, nbytes);
	}

	for (i = 0; i < nblk; i++)
		if (!(payload[i >> 3] & (1u << (i & 7))))
			bad++;

	/*
	 * A table claiming most of the device is unusable is not a table
	 * we found, it is bytes that happened to sit at this offset.
	 */
	if (bad > nblk / 4) {
		dev_warn(w->dev,
			 "VFL ctx blk=%u type=0x%04x rejected: %u of %u blocks marked bad\n",
			 block, type_word, bad, nblk);
		return 0;
	}

	if (!w->vfl.bitmap_loaded) {
		memcpy(w->vfl.blk_status, payload, nbytes);
		w->vfl.blk_status_bad = bad;
		w->vfl.blk_status_bank = ce * (w->geom.num_cau ? w->geom.num_cau : 1) + cau;
		w->vfl.blk_status_block = block;
		w->vfl.bitmap_loaded = 1;
		w->vfl.ctx_ce[cau] = ce;
		w->vfl.ctx_block[cau] = block;
		dev_info(w->dev,
			 "VFL ctx ce=%u cau=%u blk=%u type=0x%04x accepted: %u/%u blocks bad\n",
			 ce, cau, block, type_word, bad, nblk);
	} else if (memcmp(w->vfl.blk_status, payload, nbytes)) {
		/* Mirrors should agree. Say so if they do not; keep the first. */
		w->vfl.blk_status_mismatch++;
		dev_warn(w->dev,
			 "VFL ctx mirror ce=%u cau=%u blk=%u disagrees with blk=%u (kept)\n",
			 ce, cau, block, w->vfl.blk_status_block);
	}

	/* VBN->PBN is identity here; replacement lives in the u16 tables. */
	w->vfl.remap_count = nblk;
	return 1;
}

static int n31_vfl_open(struct whimory *w)
{
	struct s5l8740_cs_page *csp;
	u8 *page;
	u8 meta[S5L8740_NAND_META_SIZE];
	unsigned int ce, cau, b;
	int hits = 0;
	int sess;

	page = kvmalloc(S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	if (!page)
		return -ENOMEM;
	csp = kvmalloc(sizeof(*csp), GFP_KERNEL);
	if (!csp) {
		kvfree(page);
		return -ENOMEM;
	}

	/*
	 * Arm live CS for the reads, the same way FPART_SCAN does.
	 *
	 * cs_phys_read_slc() needs the DMA session; without one it returns
	 * -ENODEV before issuing anything, so the whole scan completed in
	 * about sixty milliseconds and reported ctx_hits=0 -- the same
	 * instant-failure profile the meta refusal used to produce, for a
	 * different reason.
	 *
	 * Sessions nest and begin() always succeeds, so this owns exactly
	 * its own end; an outer session keeps CS armed past it.
	 */
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		dev_warn(w->dev, "VFL_Open no DMA session (%d)\n", sess);
		kvfree(csp);
		kvfree(page);
		return sess;
	}
	s5l8740_nand_reset();

	/*
	 * Read the copies the FPart directory names, and nothing else.
	 *
	 * This used to sweep the whole system area -- every block from
	 * blocks_per_cau - vfl_tail up, eight pages each, on all four banks.
	 * That is 4096 page reads, it took seventeen seconds of a
	 * forty-second mount, and it found the context it wanted eight and a
	 * half seconds before it stopped looking, because the loop had no
	 * reason to end early.
	 *
	 * Stock does not do this. s_fpart keeps a table of special-object
	 * copies -- six bytes of {bank, block, type} per entry, indexed by
	 * type -- and sub_4F12DC walks only the copies of the one type it
	 * was asked for, taking the highest generation. fpart_locate_special()
	 * builds exactly that table and fpart_read_special_by_index() is a
	 * transcription of that walk; the scan behind them has already run
	 * for the signature by the time VFL_Open is called, so the lookup
	 * here is a cache hit and costs nothing.
	 *
	 * On this unit the table holds five entries and two of them are the
	 * 0xc104 context, so this is five page reads against 4096. The
	 * objects live at page 0 of their block -- chunk 0, payload at
	 * FPART_SPECIAL_HDR -- which is where the sweep found them too, and
	 * n31_vfl_ingest_ctx() still does the parsing and the mirror
	 * comparison exactly as before.
	 */
	{
		u16 idx;

		/*
		 * Force the directory to exist. It normally already does --
		 * n31_fpart_read_signature() ran first -- and this is then a
		 * cache lookup. Its return value is not the point: a miss
		 * here is reported by the loop below finding no context.
		 */
		fpart_locate_special(w, &idx, FPART_TYPE_VFL_CXT);
	}

	for (b = 0; b < w->fpart_ctx.count; b++) {
		const struct fpart_special_entry *e = &w->fpart_ctx.table[b];
		int got;

		cond_resched();
		fpart_bank_to_ce_cau(w, e->bank, &ce, &cau);
		/*
		 * fpart_fil_read_page(), not s5l8740_nand_page_read().
		 * page_read refuses any request carrying meta unless
		 * meta_dma_read is set, and that is deliberately off because
		 * a permanent live CS kick reboots the device -- so every
		 * read here used to return -EOPNOTSUPP before touching the
		 * NAND, and VFL_Open reported ctx_hits=0 about a region
		 * nobody had actually looked at.
		 *
		 * The shared helper also gets the plane right: the tail of
		 * each CAU is SLC, so it tries slc=1 before slc=0. Reading it
		 * as MLC returns a page of zeros that looks exactly like a
		 * successful read of an empty block.
		 */
		got = fpart_fil_read_page(w, e->bank, e->block, 0, csp, page,
					  meta);
		if (got)
			continue;
		if (n31_vfl_ingest_ctx(w, ce, cau, e->block, page,
				       S5L8740_NAND_PAGE_SIZE, meta))
			hits++;
	}
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	kvfree(csp);
	kvfree(page);
	w->vfl.ctx_hits = hits;
	w->vfl_ok = true;
	dev_info(w->dev,
		 "VFL_Open OK ctx_hits=%u remap_ents=%u bitmap=%u bad_blocks=%u src_bank=%u mirror_diff=%u\n",
		 hits, w->vfl.remap_count, w->vfl.bitmap_loaded,
		 w->vfl.blk_status_bad, w->vfl.blk_status_bank,
		 w->vfl.blk_status_mismatch);
	return 0;
}

static u32 n31_vfl_get_param(struct whimory *w, u32 selector)
{
	switch (selector) {
	case WHIMORY_VFL_PARAM_NUM_SB:
		return w->geom.num_ce * w->geom.num_cau * w->geom.user_blocks;
	default:
		return 0;
	}
}


/*
 * Read-ahead window, filled along the VBA axis.
 *
 * The first version of this prefetched consecutive physical pages of one
 * block and made sequential reads four times SLOWER (1.6 MB/s against 6.7).
 * Measured on the device, the FTL stripes consecutive LBAs across every
 * (ce, cau) before advancing the page:
 *
 *   lba+0..3   ce0/cau1 pg36 slot0..3
 *   lba+4..7   ce1/cau0 pg36 slot0..3
 *   lba+8..11  ce1/cau1 pg36 slot0..3
 *   lba+12..15 ce0/cau0 pg37 slot0..3
 *
 * so page+1 of one chip is 16 LBAs away, and a window of 8 consecutive
 * pages held nothing the reader wanted next while costing eight pages of
 * NAND to fill. Hence this version does not guess the stripe at all: it
 * walks the VBAs the caller is about to ask for, unpacks each to its
 * physical page, and fetches the distinct pages that fall out. That stays
 * correct whatever the stripe turns out to be.
 *
 * Pages are grouped by CE because a command list is single-CE, so a lookahead
 * spanning both chips costs one kick each rather than one in total.
 */
static int whimory_window_find(struct whimory_sftl *s, unsigned int ce,
			       unsigned int cau, unsigned int pblock,
			       unsigned int page)
{
	unsigned int i;

	for (i = 0; i < s->rc_count; i++) {
		if (s->rc_key[i].valid && s->rc_key[i].ce == ce &&
		    s->rc_key[i].cau == cau &&
		    s->rc_key[i].pblock == pblock &&
		    s->rc_key[i].page == page)
			return (int)i;
	}
	return -1;
}

static void whimory_window_fill(struct whimory *w, u32 vba)
{
	struct whimory_sftl *s = &w->sftl;
	struct s5l8740_ppn_ref refs[WHIMORY_RC_SLOTS];
	unsigned int ref_ce[WHIMORY_RC_SLOTS];
	unsigned int want = read_prefetch_pages;
	unsigned int i, n = 0, ce_i, base;

	if (want > WHIMORY_RC_SLOTS)
		want = WHIMORY_RC_SLOTS;

	s->rc_count = 0;
	for (i = 0; i < read_prefetch_vbas && n < want; i++) {
		u32 ce, cau, vblock, page, slot, pblock;
		unsigned int j;
		bool dup = false;

		if (whimory_unpack_vba(w, vba + i, &ce, &cau, &vblock, &page,
				       &slot))
			break;
		if (page >= s->pages_per_sb || slot >= s->vbas_per_page)
			break;
		whimory_vfl_resolve(w, vblock, &cau, &pblock);
		for (j = 0; j < n; j++) {
			if (ref_ce[j] == ce && refs[j].cau == cau &&
			    refs[j].block == pblock && refs[j].page == page) {
				dup = true;
				break;
			}
		}
		if (dup)
			continue;
		ref_ce[n] = ce;
		refs[n].cau = (u8)cau;
		refs[n].block = (u16)pblock;
		refs[n].page = (u8)page;
		n++;
	}
	if (!n)
		return;

	/*
	 * One kick per CE. Entries are emitted in the order they were
	 * discovered so slot indices stay aligned with rc_key.
	 */
	base = 0;
	for (ce_i = 0; ce_i < WHIMORY_NUM_CE_MAX; ce_i++) {
		struct s5l8740_ppn_ref grp[WHIMORY_RC_SLOTS];
		unsigned int map[WHIMORY_RC_SLOTS];
		unsigned int g = 0;

		for (i = 0; i < n; i++) {
			if (ref_ce[i] != ce_i)
				continue;
			map[g] = i;
			grp[g++] = refs[i];
		}
		if (!g)
			continue;
		rc_pages += g;
		if (s5l8740_nand_cs_read_pages_batch((u8)ce_i, grp, g,
						     s->rc_stage,
						     s->rc_stage_meta)) {
			rc_fails++;
			continue;
		}
		for (i = 0; i < g; i++) {
			unsigned int dst = base + i;

			if (dst >= WHIMORY_RC_SLOTS)
				break;
			memcpy(s->rc_data + (size_t)dst * S5L8740_NAND_PAGE_SIZE,
			       s->rc_stage + (size_t)i * S5L8740_NAND_PAGE_SIZE,
			       S5L8740_NAND_PAGE_SIZE);
			memcpy(s->rc_meta + (size_t)dst * S5L8740_NAND_META_SIZE,
			       s->rc_stage_meta + (size_t)i * S5L8740_NAND_META_SIZE,
			       S5L8740_NAND_META_SIZE);
			s->rc_key[dst].ce = ref_ce[map[i]];
			s->rc_key[dst].cau = refs[map[i]].cau;
			s->rc_key[dst].pblock = refs[map[i]].block;
			s->rc_key[dst].page = refs[map[i]].page;
			s->rc_key[dst].valid = true;
		}
		base += g;
		if (base > WHIMORY_RC_SLOTS)
			base = WHIMORY_RC_SLOTS;
		s->rc_count = base;
		rc_fills++;
	}
}

static int whimory_window_read(struct whimory *w, u32 vba, unsigned int ce,
			       unsigned int cau, unsigned int pblock,
			       unsigned int page, u8 *pagebuf, u8 *spare,
			       size_t spare_len)
{
	struct whimory_sftl *s = &w->sftl;
	int idx;

	if (!read_prefetch_pages || !s->rc_data || !s->rc_meta || !s->rc_stage)
		return -ENODEV;

	idx = whimory_window_find(s, ce, cau, pblock, page);
	if (idx < 0) {
		whimory_window_fill(w, vba);
		idx = whimory_window_find(s, ce, cau, pblock, page);
		rc_misses++;
		if (idx < 0)
			return -ENOENT;
	} else {
		rc_hits++;
	}

	memcpy(pagebuf, s->rc_data + (size_t)idx * S5L8740_NAND_PAGE_SIZE,
	       S5L8740_NAND_PAGE_SIZE);
	memcpy(spare, s->rc_meta + (size_t)idx * S5L8740_NAND_META_SIZE,
	       min_t(size_t, spare_len, S5L8740_NAND_META_SIZE));
	return 0;
}

static int n31_vfl_read_vba(struct whimory *w, u32 vba, u32 count,
			    void *data, struct whimory_meta *meta)
{
	u32 i, ce, cau, vblock, page, slot, pblock;
	u32 last_ce = ~0u, last_cau = ~0u, last_pblock = ~0u, last_page = ~0u;
	u8 *pagebuf;
	u8 spare[S5L8740_NAND_META_SIZE];
	int ret;

	if (!count || count > WHIMORY_VBAS_PER_PAGE)
		return -EINVAL;
	pagebuf = w->sftl.data_page;
	if (!pagebuf)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		u8 *dst = (u8 *)data + i * WHIMORY_LBA_SIZE;

		ret = whimory_unpack_vba(w, vba + i, &ce, &cau, &vblock,
					 &page, &slot);
		if (ret)
			return ret;
		if (page >= w->sftl.pages_per_sb ||
		    slot >= w->sftl.vbas_per_page)
			return -ERANGE;
		whimory_vfl_resolve(w, vblock, &cau, &pblock);
		if (ce != last_ce || cau != last_cau || pblock != last_pblock ||
		    page != last_page) {
			/*
			 * Cross-call page cache. The loop below already avoids
			 * re-reading a page within one call, but the block
			 * layer calls this with count == 1 for every 4 KiB, so
			 * that only ever helped the recovery scans. Four LBAs
			 * share a page, so a sequential read was fetching each
			 * page four times.
			 */
			if (!whimory_window_read(w, vba + i, ce, cau, pblock,
						 page, pagebuf, spare,
						 sizeof(spare))) {
				/* Served from the read-ahead window. */
			} else if (w->sftl.page_cache &&
				   w->sftl.page_cache_valid &&
			    ce == w->sftl.pc_ce && cau == w->sftl.pc_cau &&
			    pblock == w->sftl.pc_pblock &&
			    page == w->sftl.pc_page) {
				memcpy(pagebuf, w->sftl.page_cache,
				       S5L8740_NAND_PAGE_SIZE);
				memcpy(spare, w->sftl.page_cache_spare,
				       sizeof(spare));
			} else {
				ret = whimory_cs_read_page(w, ce, cau, pblock,
							   page, pagebuf,
							   S5L8740_NAND_PAGE_SIZE,
							   spare, sizeof(spare));
				if (ret) {
					/*
					 * Do not leave a stale key pointing at
					 * a buffer this read may have already
					 * partly overwritten.
					 */
					w->sftl.page_cache_valid = false;
					return ret;
				}
				if (w->sftl.page_cache) {
					memcpy(w->sftl.page_cache, pagebuf,
					       S5L8740_NAND_PAGE_SIZE);
					memcpy(w->sftl.page_cache_spare, spare,
					       sizeof(spare));
					w->sftl.pc_ce = ce;
					w->sftl.pc_cau = cau;
					w->sftl.pc_pblock = pblock;
					w->sftl.pc_page = page;
					w->sftl.page_cache_valid = true;
				}
			}
			last_ce = ce;
			last_cau = cau;
			last_pblock = pblock;
			last_page = page;
		}
		memcpy(dst, pagebuf + slot * WHIMORY_LBA_SIZE, WHIMORY_LBA_SIZE);
		if (meta && i == 0) {
			memset(meta, 0xff, sizeof(*meta));
			if (sizeof(spare) >= (slot + 1) * WHIMORY_META_SIZE)
				memcpy(meta, spare + slot * WHIMORY_META_SIZE,
				       sizeof(*meta));
		}
	}
	return 0;
}

static const struct whimory_vfl_ops n31_vfl_ops = {
	.major = 0,
	.minor = n31_vfl_minor,
	.init = n31_vfl_init,
	.open = n31_vfl_open,
	.get_param = n31_vfl_get_param,
	.read_vba = n31_vfl_read_vba,
};

static int whimory_vfl_open(struct whimory *w)
{
	int ret;

	ret = w->vfl_ops->init(w);
	if (ret) {
		dev_err(w->dev, "VFL_Init failed: %d\n", ret);
		return ret;
	}
	ret = w->vfl_ops->open(w);
	if (ret) {
		dev_err(w->dev, "VFL_Open failed: %d\n", ret);
		return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* SFTL recovery — classify SBs, replay BTOC/META by weave */
/* ------------------------------------------------------------------ */

/* FFFF0001 payload is {count, [lba,span]...} → unmap. */
static int whimory_sftl_apply_list(struct whimory *w, u32 vba)
{
	u8 *buf;
	u32 count, i, lba, span;
	struct whimory_meta meta;
	int ret;

	/*
	 * Must not alias data_page: n31_vfl_read_vba uses data_page as the
	 * page scratch. Prefer gc_data; else a stack-sized one-shot alloc.
	 */
	buf = w->sftl.gc_data;
	if (!buf) {
		buf = kmalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
	}
	if (!w->vfl_ops || !w->vfl_ops->read_vba) {
		if (buf != w->sftl.gc_data)
			kfree(buf);
		return -ENOMEM;
	}
	ret = w->vfl_ops->read_vba(w, vba, 1, buf, &meta);
	if (ret) {
		if (buf != w->sftl.gc_data)
			kfree(buf);
		return ret;
	}
	count = get_unaligned_le32(buf);
	if (!count || count > (WHIMORY_LBA_SIZE - 4) / 8) {
		if (buf != w->sftl.gc_data)
			kfree(buf);
		return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		lba = get_unaligned_le32(buf + 4 + 8 * i);
		span = get_unaligned_le32(buf + 8 + 8 * i);
		/* Cap runaway garbage spans from misclassified tokens. */
		if (!span || whimory_special_lba(lba) || lba >= 0x01000000u)
			break;
		if (span > WHIMORY_L2V_ROOT_SPAN)
			span = WHIMORY_L2V_ROOT_SPAN;
		ret = whimory_l2v_update(w, lba, span, w->l2v.invalid_vba);
		if (ret) {
			if (buf != w->sftl.gc_data)
				kfree(buf);
			return ret;
		}
		w->sftl.token_list_applied++;
	}
	if (buf != w->sftl.gc_data)
		kfree(buf);
	return 0;
}

static bool whimory_btoc_looks_be_lpn(const u8 *page)
{
	u32 a = get_unaligned_be32(page);
	u32 b = get_unaligned_be32(page + 4);
	u32 c = get_unaligned_be32(page + 8);
	u32 d = get_unaligned_be32(page + 12);
	unsigned int i, ok = 0;

	if (a < 0x01000000u && b == a + 1 && c == b + 1 && d == c + 1)
		return true;
	if (a == 0 && (b == 1 || b == WHIMORY_VBAS_PER_PAGE) &&
	    c == b + (b == 1 ? 1 : WHIMORY_VBAS_PER_PAGE))
		return true;
	for (i = 0; i < 16; i++) {
		u32 v = get_unaligned_be32(page + i * 4);

		if (v != 0xffffffff && v < 0x01000000u)
			ok++;
	}
	return ok >= 12;
}

/* Live N31 SFTL BTOC: 16-byte BE records, span in last byte (fmss glass). */
static bool whimory_btoc_looks_be_bte(const u8 *page)
{
	u32 weave0, lba0, lba1;
	u32 span0, span1;

	if (whimory_page_blank(page, 64))
		return false;
	weave0 = get_unaligned_be32(page);
	lba0 = get_unaligned_be32(page + 8);
	span0 = page[15];
	if (weave0 != 0 || !span0 || span0 > WHIMORY_DATA_VBAS_PER_SB ||
	    lba0 >= 0x01000000u)
		return false;
	lba1 = get_unaligned_be32(page + 16 + 8);
	span1 = page[16 + 15];
	if (!span1 || span1 > WHIMORY_DATA_VBAS_PER_SB || lba1 >= 0x01000000u)
		return false;
	if (lba1 != lba0 + span0 && lba1 + span1 != lba0 &&
	    (lba1 < lba0 || lba1 > lba0 + span0 + 8))
		return false;
	return true;
}


/*
 * BTOC is a physical-slot index only. Per-slot CS metadata is the L2V key.
 * Never L2V_Update(btoc_lpn) — zeros/holes in BTOC were poisoning L2V[0].
 */
static int whimory_l2v_update_from_slot_meta(struct whimory *w,
					     unsigned int ce, unsigned int cau,
					     unsigned int vblock,
					     unsigned int page, unsigned int slot,
					     const u8 *meta16,
					     u32 btoc_hint_lba)
{
	u32 meta_lba, vba;

	if (!meta16)
		return 0;
	if (meta16[0] != WHIMORY_META_TYPE_DATA &&
	    meta16[0] != WHIMORY_META_TYPE_DATA2)
		return 0;
	if (meta16[1] & 0x02)
		return 0;
	if (whimory_meta_erased(meta16, WHIMORY_META_SIZE))
		return 0;
	meta_lba = get_unaligned_le32(meta16 + 8);
	if (whimory_special_lba(meta_lba) || meta_lba >= 0x01000000u)
		return 0;
	if (btoc_hint_lba != 0xffffffffu && btoc_hint_lba != meta_lba) {
		w->sftl.btoc_meta_mismatch++;
		dev_dbg(w->dev,
			"btoc_meta_mismatch hint=%u meta_lba=%u ce=%u cau=%u vblock=%u pg=%u slot=%u\n",
			btoc_hint_lba, meta_lba, ce, cau, vblock, page, slot);
		/* Still trust metadata as authority. */
	}
	vba = whimory_pack_vba(w, ce, cau, vblock, page, slot);
	w->sftl.claim_weave = whimory_weave48(meta16);
	if (w->sftl.claim_source == 0)
		w->sftl.claim_source = 1;
	if ((audit_lba_winners && whimory_audit_fmss_lba(meta_lba)) ||
	    (l2v_trace_lba && meta_lba == l2v_trace_lba)) {
		struct whimory_range *prev =
			whimory_range_find(&w->ranges, meta_lba);
		u64 prev_weave = prev ? prev->weave : 0;
		u32 prev_vba = prev ? prev->vba : ~0u;
		bool win = !prev ||
			   !(prev->weave > w->sftl.claim_weave);
		const char *src = w->sftl.claim_source == 2 ? "open" :
				  w->sftl.claim_source == 3 ? "CXT" :
				  w->sftl.claim_source == 4 ? "LIST" :
				  "BTOC/meta";

		dev_info(w->dev,
			 "LBA_WINNER fmss_lba=%u candidate vba=%u "
			 "ce=%u cau=%u vblk=%u pg=%u slot=%u weave=%012llx "
			 "prev_vba=%u prev_weave=%012llx selected=%s "
			 "reason=%s source=%s\n",
			 meta_lba, vba, ce, cau, vblock, page, slot,
			 (unsigned long long)w->sftl.claim_weave,
			 prev_vba, (unsigned long long)prev_weave,
			 win ? "yes" : "no",
			 win ? (prev ? "newer_or_equal_weave" : "first")
			     : "older_weave_kept",
			 src);
	}
	if (whimory_l2v_update(w, meta_lba, 1, vba)) {
		w->sftl.claim_weave = 0;
		return -ENOMEM;
	}
	w->sftl.claim_weave = 0;
	/*
	 * Same derivation, two callers. The BTOC walk builds the map from
	 * scratch; the CXT repair path uses it to rebuild the part of the
	 * map a stale checkpoint extent had claimed. Counting both as BTOC
	 * would hide how much of a recovery was repair.
	 */
	if (w->sftl.claim_source == 3) {
		w->sftl.cxt_repair_slots++;
	} else {
		w->sftl.btoc_meta_confirmed++;
		w->sftl.btoc_l2v_updates++;
		w->sftl.btoc_recs++;
	}
	return 1;
}

static int whimory_btoc_confirm_page(struct whimory *w, unsigned int ce,
				     unsigned int cau, unsigned int vblock,
				     unsigned int page, u32 btoc_hint_base,
				     bool hint_is_page_lpn)
{
	u8 spare[S5L8740_NAND_META_SIZE];
	u8 *data = w->sftl.data_page;
	unsigned int slot;
	u32 pblock, rcau = cau;
	int ret, hits = 0;

	if (!btoc_meta_confirm)
		return 0;
	if (btoc_confirm_max &&
	    w->sftl.btoc_confirm_pages >= btoc_confirm_max) {
		w->sftl.btoc_confirm_capped++;
		return 0;
	}
	if (recover_budget_ms && w->sftl.confirm_start_jiffies &&
	    time_after(jiffies, w->sftl.confirm_start_jiffies +
		       msecs_to_jiffies(recover_budget_ms))) {
		w->sftl.btoc_confirm_budget_stop++;
		return 0;
	}
	if (!data)
		return -ENOMEM;
	/*
	 * Both steps. This did only the block translation, with the CAU the
	 * caller passed in -- so the page confirmed here could come from a
	 * different plane than the one every later read of the same VBA will
	 * fetch. Identity today; wrong the moment the bank bitmap is loaded.
	 */
	whimory_vfl_resolve(w, vblock, &rcau, &pblock);
	ret = whimory_cs_read_page(w, ce, rcau, pblock, page, data,
				   S5L8740_NAND_PAGE_SIZE, spare,
				   sizeof(spare));
	if (ret)
		return ret;
	w->sftl.btoc_confirm_pages++;
	if (recover_yield_us && (w->sftl.btoc_confirm_pages & 0x0f) == 0) {
		cond_resched();
		usleep_range(recover_yield_us, recover_yield_us + 500);
	}
	whimory_note_payload_strings(w, data, S5L8740_NAND_PAGE_SIZE);
	if (whimory_page_blank(data, 64) && whimory_meta_erased(spare, 16))
		return 0;
	for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
		u32 hint = 0xffffffffu;

		if (btoc_hint_base != 0xffffffffu) {
			if (hint_is_page_lpn)
				hint = btoc_hint_base * WHIMORY_VBAS_PER_PAGE +
				       slot;
			else if (slot == 0)
				hint = btoc_hint_base;
		}
		ret = whimory_l2v_update_from_slot_meta(w, ce, cau, vblock,
							page, slot,
							spare + slot *
							WHIMORY_META_SIZE,
							hint);
		if (ret < 0)
			return ret;
		hits += ret;
	}
	return hits;
}

/*
 * Turn a BTOC offset into the page it names.
 *
 * The offsets a BTOC carries are whole-superblock VBA offsets -- s_btoc.c
 * asserts vba == s_g_addr_to_vba(wr->sb, wr->nextVbaOfs) at line 230 -- so
 * the bank comes out of the offset, not from whichever plane's page 127
 * happened to carry the BTOC. The parsers used to divide by vbas_per_page
 * and keep the caller's (ce, cau), which is the retired per-bank reading:
 * it names a page nbanks times too early and always on one plane.
 *
 * That was never a mapping error, because none of these parsers maps
 * anything -- the BTE only chooses which pages to visit, and
 * whimory_btoc_confirm_page() then applies each slot from that slot's own
 * metadata. It was a coverage error: the sweep re-read the first quarter of
 * one plane instead of walking the superblock, and stopped at
 * WHIMORY_DATA_VBAS_PER_SB, a quarter of the way in on four banks.
 *
 * Same arithmetic as everything else now: build the VBA and unpack it.
 */
static int whimory_btoc_ofs_to_page(const struct whimory *w, u32 vblock,
				    u32 vba_ofs, unsigned int *ce,
				    unsigned int *cau, unsigned int *page,
				    unsigned int *slot)
{
	u32 vba = vblock * whimory_vbas_per_vblock(w) + vba_ofs;
	u32 c, a, vb, pg, sl;

	if (whimory_unpack_vba(w, vba, &c, &a, &vb, &pg, &sl))
		return -ERANGE;
	*ce = c;
	*cau = a;
	*page = pg;
	*slot = sl;
	return 0;
}

/* Addresses one superblock holds, and how many of them carry data. */
static u32 whimory_sb_vbas(const struct whimory *w, u32 vblock)
{
	return whimory_sb_banks(w, vblock, NULL) * w->sftl.pages_per_sb *
	       w->sftl.vbas_per_page;
}

static u32 whimory_sb_data_vbas(const struct whimory *w, u32 vblock)
{
	return whimory_sb_banks(w, vblock, NULL) *
	       WHIMORY_DATA_PAGES_PER_SB * w->sftl.vbas_per_page;
}

/* One key per (bank, page), for the visit-each-page-once dedup. */
static u32 whimory_btoc_page_key(unsigned int ce, unsigned int cau,
				 unsigned int page)
{
	return ((u32)ce << 24) | ((u32)cau << 16) | (u32)page;
}

static bool whimory_btoc_parse_be_lpn(struct whimory *w, const u8 *page,
				      unsigned int len, unsigned int ce,
				      unsigned int cau, unsigned int vblock)
{
	unsigned int i, n, hit = 0, valid = 0;
	bool page_gran;

	/*
	 * Bound by this superblock's own data-address count, not the
	 * per-bank one -- see whimory_btoc_ofs_to_page().
	 */
	n = min_t(unsigned int, len / 4, whimory_sb_data_vbas(w, vblock));
	for (i = 0; i < n; i++) {
		u32 lpn = get_unaligned_be32(page + i * 4);

		if (lpn == 0xffffffff)
			break;
		valid++;
	}
	page_gran = valid > 0 && valid <= WHIMORY_DATA_PAGES_PER_SB;
	if (w->sftl.btoc_pages_valid < 5)
		dev_info(w->dev,
			 "BTOC_BE_LPN valid=%u %s ce=%u cau=%u vblock=%u "
			 "(meta-validated)\n",
			 valid,
			 page_gran ? "page-granularity x4" : "slot-granularity",
			 ce, cau, vblock);

	if (page_gran)
		n = min(valid, (unsigned int)WHIMORY_DATA_PAGES_PER_SB);
	for (i = 0; i < n; i++) {
		u32 lpn = get_unaligned_be32(page + i * 4);
		unsigned int pg, slot;
		int got;

		w->sftl.btoc_entries_seen++;
		if (lpn == 0xffffffff || lpn == WHIMORY_LBA_BLANK) {
			w->sftl.btoc_hole_entries++;
			continue;
		}
		if (whimory_special_lba(lpn)) {
			if (lpn == WHIMORY_LBA_HOLE)
				w->sftl.btoc_hole_entries++;
			else if (lpn == WHIMORY_LBA_DELETED ||
				 lpn == WHIMORY_LBA_LIST)
				w->sftl.btoc_unmap_entries++;
			else
				w->sftl.btoc_unknown_entries++;
			w->sftl.token_hole++;
			continue;
		}
		if (lpn >= 0x01000000u)
			continue;
		/* Zero BTOC slots are holes — never poison L2V[0]. */
		if (lpn == 0) {
			w->sftl.btoc_skipped_zero++;
			continue;
		}
		if (page_gran) {
			/*
			 * One record per page, in the superblock's page
			 * order: record i is page i of bank i % nbanks.
			 */
			unsigned int bce, bcau, bpg, bsl;

			if (whimory_btoc_ofs_to_page(w, vblock,
						     i * w->sftl.vbas_per_page,
						     &bce, &bcau, &bpg, &bsl))
				break;
			got = whimory_btoc_confirm_page(w, bce, bcau, vblock,
							bpg, lpn, true);
		} else {
			unsigned int bce, bcau;

			if (whimory_btoc_ofs_to_page(w, vblock, i, &bce, &bcau,
						     &pg, &slot))
				break;
			if (slot != 0)
				continue;
			got = whimory_btoc_confirm_page(w, bce, bcau, vblock,
							pg, lpn, false);
		}
		if (got < 0)
			return hit > 0;
		if (got > 0)
			hit += got;
	}
	return hit > 0;
}

static bool whimory_btoc_parse_be_bte(struct whimory *w, const u8 *page,
				      unsigned int len, unsigned int ce,
				      unsigned int cau, unsigned int vblock)
{
	unsigned int i, recs, vba_ofs = 0, hit = 0;
	unsigned int last_pg = ~0u;
	/*
	 * Bounds from this superblock's own width. WHIMORY_VBAS_PER_SB and
	 * WHIMORY_DATA_VBAS_PER_SB are the per-bank counts and stopped the
	 * walk a quarter of the way into a four-bank superblock.
	 */
	u32 sb_vbas = whimory_sb_vbas(w, vblock);
	u32 sb_data_vbas = whimory_sb_data_vbas(w, vblock);

	recs = len / 16;
	for (i = 0; i < recs; i++) {
		const u8 *r = page + i * 16;
		u32 lba = get_unaligned_be32(r + 8);
		u32 span = r[15];
		unsigned int s;

		w->sftl.btoc_entries_seen++;
		if (!span)
			break;
		if (whimory_special_lba(lba)) {
			if (lba == WHIMORY_LBA_LIST) {
				u32 vba = whimory_pack_vba(w, ce, cau, vblock,
						vba_ofs / w->sftl.vbas_per_page,
						vba_ofs % w->sftl.vbas_per_page);

				w->sftl.btoc_holelist_ffff0001++;
				w->sftl.btoc_unmap_entries++;
				if (btoc_apply_list) {
					w->sftl.claim_source = 4;
					if (whimory_sftl_apply_list(w, vba))
						dev_warn(w->dev,
							 "BE BTE list token vba=%u failed\n",
							 vba);
					else
						w->sftl.token_list++;
					w->sftl.claim_source = 1;
				}
			} else if (lba == WHIMORY_LBA_HOLE) {
				w->sftl.btoc_token_ffff0000++;
				w->sftl.btoc_hole_entries++;
			} else if (lba == WHIMORY_LBA_DELETED) {
				w->sftl.btoc_token_ffffff00++;
				w->sftl.btoc_unmap_entries++;
			} else if (lba == WHIMORY_LBA_BLANK) {
				w->sftl.btoc_token_ffffffff++;
				w->sftl.btoc_hole_entries++;
			} else {
				w->sftl.btoc_unknown_entries++;
			}
			w->sftl.token_hole++;
			if (vba_ofs + span > sb_vbas)
				break;
			vba_ofs += span;
			continue;
		}
		if (span > sb_data_vbas || lba >= 0x01000000u)
			break;
		if (vba_ofs + span > sb_data_vbas)
			break;
		if (lba == 0) {
			w->sftl.btoc_skipped_zero++;
			vba_ofs += span;
			continue;
		}
		for (s = 0; s < span; s++) {
			unsigned int bce, bcau, pg, sl;
			u32 key;
			int got;

			if (whimory_btoc_ofs_to_page(w, vblock, vba_ofs + s,
						     &bce, &bcau, &pg, &sl))
				break;
			key = whimory_btoc_page_key(bce, bcau, pg);
			if (key == last_pg)
				continue;
			last_pg = key;
			got = whimory_btoc_confirm_page(w, bce, bcau, vblock,
							pg, 0xffffffffu, false);
			if (got < 0)
				return hit > 0;
			if (got > 0)
				hit += got;
		}
		vba_ofs += span;
	}
	return hit > 0;
}

static bool whimory_btoc_parse_bte(struct whimory *w, const u8 *page,
				   unsigned int len, unsigned int ce,
				   unsigned int cau, unsigned int vblock)
{
	unsigned int i, recs, vba_ofs = 0, hit = 0;
	unsigned int last_pg = ~0u;
	/*
	 * Bounds from this superblock's own width. WHIMORY_VBAS_PER_SB and
	 * WHIMORY_DATA_VBAS_PER_SB are the per-bank counts and stopped the
	 * walk a quarter of the way into a four-bank superblock.
	 */
	u32 sb_vbas = whimory_sb_vbas(w, vblock);
	u32 sb_data_vbas = whimory_sb_data_vbas(w, vblock);

	if (len < sizeof(struct whimory_bte) || whimory_page_blank(page, 64))
		return false;

	recs = len / sizeof(struct whimory_bte);
	if (le32_to_cpu(((const struct whimory_bte *)page)->weave_seq_add))
		dev_dbg(w->dev, "BTOC weaveSeqAdd[0] != 0\n");

	for (i = 0; i < recs; i++) {
		const struct whimory_bte *bte =
			(const struct whimory_bte *)(page + i * sizeof(*bte));
		u32 lba = le32_to_cpu(bte->lba);
		u32 span = le32_to_cpu(bte->span);
		unsigned int s;

		w->sftl.btoc_entries_seen++;
		if (!span)
			break;
		if (whimory_special_lba(lba)) {
			if (lba == WHIMORY_LBA_LIST) {
				u32 vba = whimory_pack_vba(w, ce, cau, vblock,
						vba_ofs / w->sftl.vbas_per_page,
						vba_ofs % w->sftl.vbas_per_page);
				w->sftl.btoc_unmap_entries++;
				w->sftl.btoc_holelist_ffff0001++;
				if (btoc_apply_list) {
					w->sftl.claim_source = 4;
					if (whimory_sftl_apply_list(w, vba))
						dev_warn(w->dev,
							 "list token vba=%u failed\n",
							 vba);
					else
						w->sftl.token_list++;
					w->sftl.claim_source = 1;
				}
			} else if (lba == WHIMORY_LBA_HOLE) {
				w->sftl.btoc_token_ffff0000++;
				w->sftl.btoc_hole_entries++;
			} else if (lba == WHIMORY_LBA_DELETED) {
				w->sftl.btoc_token_ffffff00++;
				w->sftl.btoc_unmap_entries++;
			} else if (lba == WHIMORY_LBA_BLANK) {
				w->sftl.btoc_token_ffffffff++;
				w->sftl.btoc_hole_entries++;
			} else {
				w->sftl.btoc_unknown_entries++;
			}
			w->sftl.token_hole++;
			if (vba_ofs + span > sb_vbas)
				break;
			vba_ofs += span;
			continue;
		}
		if (span > sb_data_vbas || lba >= 0x01000000u)
			break;
		if (vba_ofs + span > sb_data_vbas)
			break;
		if (lba == 0) {
			w->sftl.btoc_skipped_zero++;
			vba_ofs += span;
			continue;
		}
		for (s = 0; s < span; s++) {
			unsigned int bce, bcau, pg, sl;
			u32 key;
			int got;

			if (whimory_btoc_ofs_to_page(w, vblock, vba_ofs + s,
						     &bce, &bcau, &pg, &sl))
				break;
			key = whimory_btoc_page_key(bce, bcau, pg);
			if (key == last_pg)
				continue;
			last_pg = key;
			got = whimory_btoc_confirm_page(w, bce, bcau, vblock,
							pg, 0xffffffffu, false);
			if (got < 0)
				return hit > 0;
			if (got > 0)
				hit += got;
		}
		vba_ofs += span;
	}
	return hit > 0;
}

/*
 * Decode a BTOC record stream into a per-VBA LBA table. Records are 16 bytes;
 * `be` picks the big-endian form (LBA at +8 BE, span in the last byte) over
 * the little-endian struct whimory_bte. Returns how many VBAs were described.
 * Special/token LBAs and holes land as WHIMORY_LBA_BLANK.
 */
static unsigned int whimory_btoc_decode_map(const u8 *page, unsigned int len,
					    bool be, u32 *map)
{
	unsigned int i, recs = len / 16, vba_ofs = 0;

	for (i = 0; i < recs && vba_ofs < WHIMORY_DATA_VBAS_PER_SB; i++) {
		const u8 *r = page + i * 16;
		u32 lba = be ? get_unaligned_be32(r + 8) :
			       get_unaligned_le32(r + 8);
		u32 span = be ? r[15] : get_unaligned_le32(r + 12);
		unsigned int s;

		if (!span || span > WHIMORY_DATA_VBAS_PER_SB)
			break;
		if (vba_ofs + span > WHIMORY_DATA_VBAS_PER_SB)
			break;
		for (s = 0; s < span; s++)
			map[vba_ofs + s] = whimory_special_lba(lba) ?
					   WHIMORY_LBA_BLANK : lba + s;
		vba_ofs += span;
	}
	return vba_ofs;
}

/*
 * Sample a few pages of a closed superblock and report how often the BTOC
 * prediction matches the per-slot metadata. High agreement means the replay
 * can apply the BTOC directly and drop from 127 reads per SB to one.
 */
static void whimory_btoc_verify(struct whimory *w, struct whimory_sb *sb,
				unsigned int vblock, const u8 *btoc,
				unsigned int len)
{
	static const bool forms[2] = { false, true };
	u8 meta[S5L8740_NAND_META_SIZE];
	unsigned int f;

	if (!w->sftl.btoc_map || !w->sftl.data_page)
		return;

	for (f = 0; f < ARRAY_SIZE(forms); f++) {
		u32 *map = w->sftl.btoc_map;
		unsigned int vbas, pages, step, pg, n = 0;
		unsigned int agree = 0, disagree = 0, nodata = 0;
		u32 first_hint = 0, first_meta = 0;

		memset(map, 0xff, WHIMORY_DATA_VBAS_PER_SB * sizeof(*map));
		vbas = whimory_btoc_decode_map(btoc, len, forms[f], map);
		if (vbas < w->sftl.vbas_per_page)
			continue;
		pages = vbas / w->sftl.vbas_per_page;
		step = pages / (btoc_verify_pages ? btoc_verify_pages : 1);
		if (!step)
			step = 1;

		for (pg = 0; pg < pages && n < btoc_verify_pages; pg += step) {
			unsigned int slot;

			if (whimory_cs_read_page(w, sb->ce, sb->cau, sb->block,
						 pg, w->sftl.data_page,
						 S5L8740_NAND_PAGE_SIZE,
						 meta, sizeof(meta)))
				break;
			n++;
			for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
				const u8 *m = meta + slot * WHIMORY_META_SIZE;
				u32 hint = map[pg * w->sftl.vbas_per_page + slot];
				u32 mlba;

				if (!whimory_meta_is_data_raw(m) ||
				    whimory_meta_erased(m, WHIMORY_META_SIZE)) {
					nodata++;
					continue;
				}
				mlba = get_unaligned_le32(m + 8);
				if (hint == mlba) {
					agree++;
				} else {
					if (!disagree) {
						first_hint = hint;
						first_meta = mlba;
					}
					disagree++;
				}
			}
		}
		dev_info(w->dev,
			 "BTOC_VERIFY ce=%u cau=%u vblk=%u form=%s vbas=%u "
			 "pages_probed=%u agree=%u disagree=%u nodata=%u "
			 "first_hint=%u first_meta=%u\n",
			 sb->ce, sb->cau, vblock, forms[f] ? "BE" : "LE",
			 vbas, n, agree, disagree, nodata,
			 first_hint, first_meta);
	}
}

static int whimory_ingest_btoc_page(struct whimory *w, unsigned int ce,
				    unsigned int cau, unsigned int vblock,
				    const u8 *page, unsigned int len)
{
	const char *verdict = "NONE";
	int hit = 0;

	if (whimory_page_blank(page, 64)) {
		w->sftl.btoc_blank++;
		return 0;
	}
	if (whimory_btoc_looks_be_bte(page)) {
		if (whimory_btoc_parse_be_bte(w, page, len, ce, cau, vblock)) {
			verdict = "BE_BTE";
			w->sftl.btoc_be_bte++;
			hit = 1;
		}
	}
	if (!hit && whimory_btoc_looks_be_lpn(page)) {
		if (whimory_btoc_parse_be_lpn(w, page, len, ce, cau, vblock)) {
			verdict = "BE_LPN_ARRAY";
			w->sftl.btoc_be_lpn++;
			hit = 1;
		}
	}
	if (!hit && whimory_btoc_parse_bte(w, page, len, ce, cau, vblock)) {
		verdict = "LE_BTE";
		w->sftl.btoc_le_bte++;
		hit = 1;
	}

	/*
	 * Some BTOC pages carry an eight-byte header before the BTE array,
	 * and 255 of the 563 closed superblocks on this device use it. Every
	 * one of them was dropped -- 127 pages and about 2032 LBAs each --
	 * which is what left directory sectors unreadable on a volume whose
	 * FAT mounted fine.
	 *
	 * struct whimory_bte is {weave_seq_add, aux, lba, span}, so the
	 * parser was reading the header's third and fourth words as lba and
	 * span. The dumps make the misread obvious:
	 *
	 *   00000000 00000002 0000c30d 000004f2 | 000004f2 00000002 ...
	 *   ^header........^ ^read as lba/span^ | ^the real first record^
	 *
	 * span came out as 1266 against a 508-VBA superblock, so the loop
	 * broke on record zero and the page was declared unrecognised. Shift
	 * eight bytes and the same parser reads lba=0x4f2 span=2, then 0x4f3,
	 * then 0x4f4 -- ascending LBAs with uniform spans, exactly what a
	 * BTOC is.
	 *
	 * Tried only after the unshifted parse yields nothing, and only when
	 * the first word is zero as it is in every sample, so a page that
	 * already parses cannot be re-read at the wrong offset.
	 */
	if (!hit && len > 8 && get_unaligned_le32(page) == 0 &&
	    whimory_btoc_parse_bte(w, page + 8, len - 8, ce, cau, vblock)) {
		verdict = "LE_BTE_HDR8";
		w->sftl.btoc_le_bte_hdr8++;
		hit = 1;
	}

	/*
	 * A BTOC nobody claimed is a whole closed superblock missing from the
	 * map -- 127 pages, about 2032 LBAs -- and 255 of 563 were going
	 * unclaimed with no record of it beyond a count that did not
	 * distinguish "erased" from "unrecognised".
	 *
	 * The read misses that follow look like this, and they are what stops
	 * files opening on a volume whose FAT reads fine:
	 *
	 *   read miss fmss_lba=3723686 ret=-2
	 *     neighbor 3723682..85 MAPPED blk=1714 pg=43 slot=0..3
	 *     neighbor 3723686     UNMAPPED
	 *   FAT-fs: Directory bread(block 3674407) failed
	 *
	 * So the unclaimed ones are dumped. Three parsers is three guesses at
	 * a format, and the bytes say whether there is a fourth shape here or
	 * whether one of the three is rejecting pages it should accept.
	 */
	if (!hit) {
		w->sftl.btoc_unclaimed++;
		if (w->sftl.btoc_unclaimed <= 8)
			dev_info(w->dev,
				 "BTOC_UNCLAIMED n=%u ce=%u cau=%u vblock=%u first64=%32ph %32ph\n",
				 w->sftl.btoc_unclaimed, ce, cau, vblock,
				 page, page + 32);
	}
	if (ftl_diag && w->sftl.btoc_pages_read <= 8)
		dev_info(w->dev,
			 "BTOC_VERDICT ce=%u cau=%u vblock=%u %s first32=%32ph\n",
			 ce, cau, vblock, verdict, page);
	return hit;
}

static int whimory_rebuild_open_sb(struct whimory *w, struct whimory_sb *sb)
{
	unsigned int pg, slot, vblock;
	u8 *data = w->sftl.data_page;
	u8 spare[S5L8740_NAND_META_SIZE];
	int ret, hits = 0;

	vblock = whimory_vfl_virt(w, sb->cau, sb->block);
	w->sftl.claim_source = 2;
	for (pg = 0; pg < WHIMORY_DATA_PAGES_PER_SB; pg++) {
		ret = whimory_cs_read_page(w, sb->ce, sb->cau, sb->block, pg,
					   data, S5L8740_NAND_PAGE_SIZE,
					   spare, sizeof(spare));
		w->sftl.open_pages_read++;
		if (ret)
			break;
		if (whimory_page_blank(data, 64) &&
		    whimory_page_blank(spare, 16))
			break;
		for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
			const u8 *m = spare + slot * WHIMORY_META_SIZE;
			u32 lba, vba;
			struct whimory_range *prev;
			u64 weave;

			w->sftl.open_slots_seen++;
			if (m[0] != WHIMORY_META_TYPE_DATA &&
			    m[0] != WHIMORY_META_TYPE_DATA2)
				continue;
			if (m[1] & 0x02)
				continue;
			if (whimory_meta_erased(m, WHIMORY_META_SIZE))
				continue;
			lba = get_unaligned_le32(m + 8);
			w->sftl.open_slots_valid_meta++;
			if (whimory_special_lba(lba)) {
				w->sftl.open_unmap_entries++;
				continue;
			}
			if (lba >= 0x01000000u) {
				w->sftl.btoc_unknown_entries++;
				continue;
			}
			if (lba == 0) {
				w->sftl.open_skipped_zero++;
				continue;
			}
			vba = whimory_pack_vba(w, sb->ce, sb->cau, vblock, pg,
					       slot);
			weave = whimory_weave48(m);
			prev = whimory_range_find(&w->ranges, lba);
			if (prev) {
				if (weave > prev->weave)
					w->sftl.open_overrides_closed++;
				else if (weave < prev->weave)
					w->sftl.open_rejected_stale++;
				else
					w->sftl.open_unknown_order++;
			}
			w->sftl.claim_weave = weave;
			if ((l2v_trace_lba && lba == l2v_trace_lba) ||
			    (audit_lba_winners && whimory_audit_fmss_lba(lba))) {
				dev_info(w->dev,
					 "LBA_WINNER fmss_lba=%u candidate vba=%u "
					 "ce=%u cau=%u vblk=%u pg=%u slot=%u "
					 "weave=%012llx prev_vba=%u prev_weave=%012llx "
					 "source=open\n",
					 lba, vba, sb->ce, sb->cau, vblock, pg,
					 slot, (unsigned long long)weave,
					 prev ? prev->vba : ~0u,
					 prev ? (unsigned long long)prev->weave :
						0ull);
			}
			/*
			 * Propagate the real error. Flattening everything to
			 * -ENOMEM here told the caller to abort the entire
			 * recovery -- it treats any negative return that way --
			 * on the strength of a guess about what went wrong.
			 */
			ret = whimory_l2v_update(w, lba, 1, vba);
			if (ret) {
				dev_err(w->dev,
					"open rebuild: L2V update failed lba=%u vba=%u: %d\n",
					lba, vba, ret);
				w->sftl.claim_weave = 0;
				w->sftl.claim_source = 0;
				return ret;
			}
			w->sftl.claim_weave = 0;
			w->sftl.open_l2v_updates++;
			hits++;
		}
		if ((pg & 0x0f) == 0) {
			cond_resched();
			if (recover_yield_us)
				usleep_range(recover_yield_us,
					     recover_yield_us + 500);
		}
	}
	w->sftl.claim_source = 0;
	return hits;
}

static int whimory_sb_cmp(const void *a, const void *b)
{
	const struct whimory_sb *sa = a, *sb = b;

	if (sa->weave < sb->weave)
		return -1;
	if (sa->weave > sb->weave)
		return 1;
	if (sa->ce != sb->ce)
		return sa->ce < sb->ce ? -1 : 1;
	if (sa->cau != sb->cau)
		return sa->cau < sb->cau ? -1 : 1;
	if (sa->block != sb->block)
		return sa->block < sb->block ? -1 : 1;
	return 0;
}

/*
 * Compact (ce, cau, block) list of the CXT superblocks found by classify.
 * whimory_vba_is_cxt() is called once per CXT L2V record; scanning all
 * num_sb entries there was O(num_sb) per record (7840 SBs on N31).
 */
static void whimory_cxt_index_build(struct whimory *w, unsigned int nsb)
{
	struct whimory_sftl *s = &w->sftl;
	unsigned int i;

	s->n_cxt_idx = 0;
	if (!s->sbs)
		return;
	for (i = 0; i < nsb && s->n_cxt_idx < ARRAY_SIZE(s->cxt_idx); i++) {
		struct whimory_sb *sb = &s->sbs[i];

		if (sb->kind != WHIMORY_SB_CXT)
			continue;
		s->cxt_idx[s->n_cxt_idx].ce = sb->ce;
		s->cxt_idx[s->n_cxt_idx].cau = sb->cau;
		s->cxt_idx[s->n_cxt_idx].block = sb->block;
		s->n_cxt_idx++;
	}
}

/*
 * VBAs belonging to a CXT superblock hold context records, not user data,
 * so they must never enter the L2V map.
 */
static bool whimory_vba_is_cxt(struct whimory *w, u32 vba)
{
	u32 ce, cau, vblock, page, slot, phys;
	unsigned int i;

	if (whimory_unpack_vba(w, vba, &ce, &cau, &vblock, &page, &slot))
		return false;
	whimory_vfl_resolve(w, vblock, &cau, &phys);
	for (i = 0; i < w->sftl.n_cxt_idx; i++) {
		struct whimory_cxt_sb_id *c = &w->sftl.cxt_idx[i];

		if (c->ce == ce && c->cau == cau && c->block == phys)
			return true;
	}
	return false;
}

static int whimory_cxt_add_base(struct whimory *w, u32 sb, u64 weave)
{
	int i;

	if (w->n_cxt >= WHIMORY_CXT_MAX_SB)
		return -ENOSPC;
	for (i = w->n_cxt; i > 0; i--) {
		if (w->cxt[i - 1].weave >= weave)
			break;
		w->cxt[i] = w->cxt[i - 1];
	}
	w->cxt[i].sb = sb;
	w->cxt[i].weave = weave;
	w->n_cxt++;
	w->sftl.cxt_bases = w->n_cxt;
	return 0;
}

static int whimory_cxt_load_contig(struct whimory *w, const u8 *data,
				   unsigned int len)
{
	u32 lba, span, vba, i, n;

	if (len < 16)
		return 0;
	n = len / 8;
	lba = get_unaligned_le32(data);
	span = get_unaligned_le32(data + 4);
	if (span == 0xffffffff)
		return 0;
	if (span != WHIMORY_CXT_CONTIG_SPAN)
		return -EINVAL;
	if (w->cxt_lba_valid && lba != w->cxt_next_lba) {
		dev_err(w->dev,
			"cxt lba not consecutive want=%u got=%u\n",
			w->cxt_next_lba, lba);
		return -EINVAL;
	}
	w->cxt_lba_valid = true;
	for (i = 1; i < n; i++) {
		vba = get_unaligned_le32(data + 8 * i);
		span = get_unaligned_le32(data + 8 * i + 4);
		if (vba == 0xffffffff || !span)
			break;
		w->sftl.cxt_records_seen++;
		/*
		 * A record whose vba is at or above invalid_vba is a hole,
		 * and it has to be applied, not skipped.
		 *
		 * s_cxt_load.c inserts on
		 *
		 *	if (invalid_vba <= vba || !sbFilter_test(sb))
		 *
		 * so a hole is always inserted and everything else is
		 * inserted unless its superblock is in the checkpoint diff
		 * filter, which is empty at mount. whimory_l2v_update()
		 * already reads vba >= invalid_vba as an unmap, so applying
		 * the record is all that is needed.
		 *
		 * Skipping holes leaves whatever previously covered those
		 * LBAs in the tree. The read then lands on a page belonging
		 * to a different LBA and the meta check rejects it, which is
		 * the "sftl lba mismatch want=X meta=Y" failure.
		 */
		if (vba >= w->l2v.invalid_vba ||
		    !whimory_vba_is_cxt(w, vba)) {
			if (whimory_l2v_update(w, lba, span, vba))
				return -ENOMEM;
			w->sftl.cxt_l2v_updates++;
		}
		lba += span;
	}
	w->cxt_next_lba = lba;
	return 0;
}

static int whimory_cxt_handle_vba(struct whimory *w, const u8 *data,
				  const u8 *meta)
{
	u8 tag;

	if (meta[0] != WHIMORY_META_TYPE_SFTL_CXT)
		return 0;
	tag = meta[1];
	if (tag == WHIMORY_CXT_TAG_END)
		return 1;
	if (tag != WHIMORY_CXT_TAG_L2V)
		return 0;
	return whimory_cxt_load_contig(w, data, WHIMORY_LBA_SIZE);
}

static int whimory_cxt_load_sb(struct whimory *w, u32 sb_idx)
{
	struct whimory_sftl *s = &w->sftl;
	u32 ce, cau, vblock, page, slot, ofs, vba, pblock;
	u32 last_ce = ~0u, last_cau = ~0u, last_pblock = ~0u, last_page = ~0u;
	u32 zone, n, i;
	u8 *data, *gmeta;
	u8 spare[S5L8740_NAND_META_SIZE];
	int ret, done = 0;

	if (sb_idx >= s->num_sb)
		return -EINVAL;
	w->sftl.cxt_blocks_seen++;
	zone = s->gc_zone_size;
	data = s->gc_data;
	gmeta = s->gc_meta;
	if (!zone || !data || !gmeta || zone % s->vbas_per_page)
		return -ENOMEM;

	w->cxt_lba_valid = false;
	w->cxt_next_lba = 0;
	/*
	 * Read the superblock in gc_zone_size chunks, as the FTL does.
	 *
	 * Unreferenced: this and whimory_cxt_load() are kept as the literal
	 * transcription of s_cxt_load.c sub_5884D4, which is what
	 * whimory_cxt_parse_tree() was checked against. The live path is
	 * whimory_cxt_build_from_sb(). Note the bound below is the per-bank
	 * count and the walk is bank-major -- both retired -- so this is
	 * documentation, not something to call.
	 */
	for (ofs = 0; ofs < s->vbas_per_sb && !done; ofs += zone) {
		n = min(zone, s->vbas_per_sb - ofs);
		for (i = 0; i < n; i++) {
			vba = whimory_sb_ofs_to_vba(w, sb_idx, ofs + i);
			ret = whimory_unpack_vba(w, vba, &ce, &cau, &vblock,
						 &page, &slot);
			if (ret)
				return ret;
			whimory_vfl_resolve(w, vblock, &cau, &pblock);
			if (ce != last_ce || cau != last_cau ||
			    pblock != last_pblock || page != last_page) {
				ret = whimory_cs_read_page(w, ce, cau, pblock,
							   page, s->data_page,
							   S5L8740_NAND_PAGE_SIZE,
							   spare,
							   sizeof(spare));
				if (ret)
					return ret;
				last_ce = ce;
				last_cau = cau;
				last_pblock = pblock;
				last_page = page;
			}
			memcpy(data + i * WHIMORY_LBA_SIZE,
			       s->data_page + slot * WHIMORY_LBA_SIZE,
			       WHIMORY_LBA_SIZE);
			if (sizeof(spare) >= (slot + 1) * WHIMORY_META_SIZE)
				memcpy(gmeta + i * WHIMORY_META_SIZE,
				       spare + slot * WHIMORY_META_SIZE,
				       WHIMORY_META_SIZE);
			else
				memset(gmeta + i * WHIMORY_META_SIZE, 0xff,
				       WHIMORY_META_SIZE);
		}
		for (i = 0; i < n; i++) {
			ret = whimory_cxt_handle_vba(w,
						     data + i * WHIMORY_LBA_SIZE,
						     gmeta + i * WHIMORY_META_SIZE);
			if (ret < 0)
				return ret;
			if (ret > 0) {
				done = 1;
				break;
			}
		}
	}
	return 0;
}

static int __maybe_unused whimory_cxt_load(struct whimory *w)
{
	unsigned int i;
	int ret, loaded = 0;

	for (i = 0; i < w->n_cxt; i++) {
		u32 sb = w->cxt[i].sb;

		dev_info(w->dev, "s_cxt_load base sb=%u weave=%llu\n",
			 sb, w->cxt[i].weave);
		w->sftl.claim_weave = w->cxt[i].weave;
		ret = whimory_cxt_load_sb(w, sb);
		w->sftl.claim_weave = 0;
		if (ret) {
			dev_warn(w->dev, "cxt sb=%u failed %d\n", sb, ret);
			continue;
		}
		w->cxt_base_weave = w->cxt[i].weave;
		loaded = 1;
		break;
	}
	w->sftl.cxt_loaded = loaded;
	return 0;
}

/* ------------------------------------------------------------------ */
/* CXT scanner / dumper (read-only; never touches the map)             */
/* ------------------------------------------------------------------ */


/*
 * Every superblock classify tagged as CXT, newest weave first.
 *
 * whimory_cxt_add_base() only registers a superblock whose page 0 slot 0
 * carries tag BASE, which on this device is one of the four CXT blocks. The
 * BASE payload itself names the others ({count, sb, sb, ...}), and the newest
 * generation need not be the one holding the BASE marker, so the candidate
 * search has to consider all of them.
 */
static unsigned int whimory_cxt_collect_sbs(struct whimory *w,
					    struct whimory_cxt_base *out,
					    unsigned int max)
{
	struct whimory_sftl *s = &w->sftl;
	unsigned int i, n = 0;

	if (!s->sbs)
		return 0;
	for (i = 0; i < s->num_sb && n < max; i++) {
		struct whimory_sb *sb = &s->sbs[i];
		u32 vblock, idx, j;

		if (sb->kind != WHIMORY_SB_CXT)
			continue;
		vblock = whimory_vfl_virt(w, sb->cau, sb->block);
		idx = whimory_sb_index(w, sb->ce, sb->cau, vblock);
		/*
		 * One candidate per virtual block, not per plane. A CXT is a
		 * single superblock striped across all four (ce, cau) planes;
		 * counting each plane separately turned one checkpoint into
		 * four candidates with four weaves, of which the two highest
		 * appeared to contribute nothing.
		 */
		for (j = 0; j < n; j++)
			if (whimory_cxt_sb_vblock(w, out[j].sb) == vblock)
				break;
		if (j < n) {
			if (sb->weave > out[j].weave)
				out[j].weave = sb->weave;
			continue;
		}
		out[n].sb = idx;
		out[n].weave = sb->weave;
		n++;
	}
	/* Insertion sort, newest weave first; n is at most WHIMORY_CXT_MAX_SB. */
	for (i = 1; i < n; i++) {
		struct whimory_cxt_base tmp = out[i];
		int j = (int)i - 1;

		while (j >= 0 && out[j].weave < tmp.weave) {
			out[j + 1] = out[j];
			j--;
		}
		out[j + 1] = tmp;
	}
	return n;
}

static const char *whimory_cxt_tag_name(u8 tag)
{
	switch (tag) {
	case WHIMORY_CXT_TAG_BASE:
		return "BASE";
	case WHIMORY_CXT_TAG_STATS:
		return "STATS";
	case WHIMORY_CXT_TAG_SB:
		return "SB";
	case WHIMORY_CXT_TAG_L2V:
		return "TREE";
	case WHIMORY_CXT_TAG_USERSEQ:
		return "USERSEQ";
	case WHIMORY_CXT_TAG_READS:
		return "READS";
	case WHIMORY_CXT_TAG_CLEAN:
		return "CLEAN/END";
	default:
		return "?";
	}
}

/*
 * Read one VBA of a CXT superblock. Returns the 4 KiB payload in `data` and
 * the 16-byte record metadata in `meta`. Page reads are cached across the
 * four slots of a physical page by the caller.
 */
/*
 * Read one record of a checkpoint, by offset within the whole superblock.
 *
 * The offset is a native VBA offset, which means it walks all four planes
 * in the order the FTL wrote them. That matters more than it looks.
 *
 * A CXT is one superblock striped across the four (ce, cau) planes, and its
 * records run in VBA order: plane 0 slots 0..3 of page 0, then plane 1,
 * plane 2, plane 3, then page 1 of plane 0. The weave in each record's meta
 * says so directly -- 580d..5810 on plane 0, 5811..5814 on plane 1, then
 * 5815, 5819, and 581d back on plane 0.
 *
 * This used to take a bank-major superblock index and walk one plane at a
 * time, which visited the same records in the wrong order: every record of
 * plane 0, then every record of plane 1, and so on. The L2V tree cannot
 * survive that. Each record declares the LBA it continues from and the
 * parse rejects a record whose declared start does not match the running
 * cursor, so out-of-order records either abort the walk or, worse, attach a
 * run to the wrong logical position.
 *
 * It also made one checkpoint look like four candidates with four different
 * weaves, of which the two "newest" contributed nothing -- which read as a
 * stale map when it was a misread one.
 */
static int whimory_cxt_read_ofs(struct whimory *w, u32 vblock, u32 ofs,
				u8 *data, u8 *meta, u8 *spare, u32 *last_key)
{
	struct whimory_sftl *s = &w->sftl;
	u32 ce, cau, vb, page, slot, pblock, key;
	/*
	 * The offset is against the virtual block's full address stride --
	 * every virtual block owns the same window whatever its bank count.
	 * How far into that window a checkpoint runs is the caller's bound;
	 * see whimory_cxt_build_from_sb().
	 */
	u32 vba = vblock * whimory_vbas_per_vblock(w) + ofs;
	int ret;

	ret = whimory_unpack_vba(w, vba, &ce, &cau, &vb, &page, &slot);
	if (ret)
		return ret;
	whimory_vfl_resolve(w, vb, &cau, &pblock);
	key = ((ce & 0xf) << 28) | ((cau & 0xf) << 24) |
	      ((pblock & 0xffff) << 8) | (page & 0xff);
	if (key != *last_key) {
		ret = whimory_cs_read_page(w, ce, cau, pblock, page,
					   s->data_page,
					   S5L8740_NAND_PAGE_SIZE,
					   spare, S5L8740_NAND_META_SIZE);
		if (ret)
			return ret;
		*last_key = key;
	}
	memcpy(data, s->data_page + slot * WHIMORY_LBA_SIZE, WHIMORY_LBA_SIZE);
	memcpy(meta, spare + slot * WHIMORY_META_SIZE, WHIMORY_META_SIZE);
	return 0;
}

/*
 * whimory_cxt_read_vba() lived here: the same read keyed by a bank-major
 * superblock index and a per-bank offset. Its last caller was the dump, and
 * the dump was the one place still walking a checkpoint in an order nothing
 * else uses. Deleted rather than left for someone to reach for, because a
 * second walker that disagrees with the loader is how the bank-count bug
 * stayed hidden for as long as it did.
 */

/*
 * Walk one CXT superblock and report what is actually stored in it: the tag
 * of every record, the shape of the BASE header (which carries the CXT
 * superblock list), and the header of each TREE record.
 */
static void whimory_cxt_dump_sb(struct whimory *w, u32 sb_idx, u64 weave,
				unsigned int max_vbas)
{
	struct whimory_sftl *s = &w->sftl;
	u32 counts[8] = {0}, clean = 0, other = 0, end_at = ~0u;
	u32 ofs, last_key = ~0u, trees = 0, base_seen = 0;
	u8 *data = s->gc_data;
	u8 meta[WHIMORY_META_SIZE];
	u8 spare[S5L8740_NAND_META_SIZE];
	/*
	 * Walk this the way the loader does.
	 *
	 * It used to call whimory_cxt_read_vba(), the bank-major walker,
	 * while whimory_cxt_build_from_sb() uses whimory_cxt_read_ofs() --
	 * so ftl_cxt_dump showed the records of one checkpoint in an order
	 * no code path ever reads them in. It also clamped to vbas_per_sb,
	 * the per-bank count, and so covered a quarter of the block. A
	 * diagnostic that disagrees with the thing it is diagnosing is worse
	 * than none, and this one was consulted while chasing exactly the
	 * bug the disagreement was hiding.
	 */
	u32 vblock = whimory_cxt_sb_vblock(w, sb_idx);
	u32 nbanks = whimory_sb_banks(w, vblock, NULL);
	u32 sb_vbas = nbanks * s->pages_per_sb * s->vbas_per_page;

	if (!data || !s->data_page) {
		dev_err(w->dev, "CXT_DUMP sb=%u no scratch\n", sb_idx);
		return;
	}
	if (!max_vbas || max_vbas > sb_vbas)
		max_vbas = sb_vbas;

	dev_info(w->dev,
		 "CXT_DUMP_BEGIN sb=%u vblk=%u weave=%llu vbas=%u nbanks=%u\n",
		 sb_idx, vblock, (unsigned long long)weave, max_vbas, nbanks);

	for (ofs = 0; ofs < max_vbas; ofs++) {
		u8 type, tag;

		if (whimory_cxt_read_ofs(w, vblock, ofs, data, meta, spare,
					 &last_key))
			break;
		type = meta[0];
		tag = meta[1];
		if (whimory_meta_erased(meta, WHIMORY_META_SIZE)) {
			clean++;
			continue;
		}
		if (type != WHIMORY_META_TYPE_SFTL_CXT) {
			other++;
			continue;
		}
		if (tag == WHIMORY_CXT_TAG_CLEAN) {
			clean++;
			if (end_at == ~0u)
				end_at = ofs;
			break;
		}
		if (tag < ARRAY_SIZE(counts))
			counts[tag]++;
		else
			other++;

		/* One compact row per distinct tag, so the record layout of
		 * a superblock is visible without a full hex dump.
		 */
		if (tag < ARRAY_SIZE(counts) && counts[tag] == 1)
			dev_info(w->dev,
				 "CXT_REC sb=%u ofs=%u pg=%u bank=%u slot=%u type=%02x tag=%02x %s\n",
				 sb_idx, ofs,
				 (ofs / s->vbas_per_page) / nbanks,
				 (ofs / s->vbas_per_page) % nbanks,
				 ofs % s->vbas_per_page, type, tag,
				 whimory_cxt_tag_name(tag));

		if (tag == WHIMORY_CXT_TAG_BASE && base_seen++ < 2) {
			dev_info(w->dev,
				 "CXT_BASE_REC sb=%u ofs=%u meta=%16ph\n",
				 sb_idx, ofs, meta);
			dev_info(w->dev,
				 "CXT_BASE_REC sb=%u ofs=%u first64=%32ph %32ph\n",
				 sb_idx, ofs, data, data + 32);
		}
		if (tag == WHIMORY_CXT_TAG_L2V && trees < 1) {
			unsigned int b;

			for (b = 0; b < 128; b += 32)
				dev_info(w->dev,
					 "CXT_TREE_HEX sb=%u ofs=%u +%03u %32ph\n",
					 sb_idx, ofs, b, data + b);
		}
		if (tag == WHIMORY_CXT_TAG_L2V && trees++ < 3) {
			dev_info(w->dev,
				 "CXT_TREE_REC sb=%u ofs=%u hdr_lba=%u "
				 "hdr_span=0x%08x p0=(%u,%u) p1=(%u,%u) "
				 "p2=(%u,%u)\n",
				 sb_idx, ofs,
				 get_unaligned_le32(data),
				 get_unaligned_le32(data + 4),
				 get_unaligned_le32(data + 8),
				 get_unaligned_le32(data + 12),
				 get_unaligned_le32(data + 16),
				 get_unaligned_le32(data + 20),
				 get_unaligned_le32(data + 24),
				 get_unaligned_le32(data + 28));
		}
	}

	dev_info(w->dev,
		 "CXT_DUMP_END sb=%u scanned=%u clean=%u other=%u end_at=%d "
		 "base=%u stats=%u sbrec=%u tree=%u userseq=%u reads=%u\n",
		 sb_idx, ofs, clean, other, (int)end_at,
		 counts[WHIMORY_CXT_TAG_BASE], counts[WHIMORY_CXT_TAG_STATS],
		 counts[WHIMORY_CXT_TAG_SB], counts[WHIMORY_CXT_TAG_L2V],
		 counts[WHIMORY_CXT_TAG_USERSEQ],
		 counts[WHIMORY_CXT_TAG_READS]);
}

/*
 * Phase 2 entry point: report every CXT base candidate newest-weave first,
 * then dump what each one contains. Read-only — the L2V map is untouched.
 */
int whimory_cxt_dump(unsigned int max_vbas)
{
	struct whimory *w = whimory_dev;
	unsigned int i;
	struct whimory_cxt_base all[WHIMORY_CXT_MAX_SB];
	unsigned int n_all;
	int sess;

	if (!w)
		return -ENODEV;
	if (!w->sftl.sbs || !w->sftl.num_sb)
		return -ENODATA;

	dev_info(w->dev,
		 "CXT_SCAN bases=%u cxt_sbs=%u classified_sbs=%u "
		 "cxt_loaded=%d base_weave=%llu\n",
		 w->n_cxt, w->sftl.cxt_sbs, w->sftl.num_sb, w->sftl.cxt_loaded,
		 (unsigned long long)w->cxt_base_weave);

	for (i = 0; i < w->n_cxt; i++)
		dev_info(w->dev, "CXT_CAND i=%u sb=%u weave=%llu\n",
			 i, w->cxt[i].sb,
			 (unsigned long long)w->cxt[i].weave);

	n_all = whimory_cxt_collect_sbs(w, all, ARRAY_SIZE(all));
	for (i = 0; i < n_all; i++)
		dev_info(w->dev, "CXT_SB i=%u sb=%u weave=%llu\n",
			 i, all[i].sb, (unsigned long long)all[i].weave);

	if (!w->n_cxt) {
		dev_warn(w->dev,
			 "CXT_SCAN no base candidates; classify must reach the "
			 "high blocks that hold them (scan_blocks=0/1960)\n");
		return -ENOENT;
	}

	sess = s5l8740_nand_dma_session_begin();
	for (i = 0; i < n_all; i++)
		whimory_cxt_dump_sb(w, all[i].sb, all[i].weave, max_vbas);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	return 0;
}
EXPORT_SYMBOL_GPL(whimory_cxt_dump);

/* ------------------------------------------------------------------ */
/* Phase 3: CXT TREE -> candidate map, compared against the live map    */
/* ------------------------------------------------------------------ */

static unsigned int cxt_max_extents = 1048576;
module_param(cxt_max_extents, uint, 0644);
MODULE_PARM_DESC(cxt_max_extents,
		 "Candidate-map extent ceiling for the CXT TREE parser");

/*
 * CXT VBAs are in the FTL native superblock space, which is not the space
 * whimory_pack_vba() builds. Apple counts one superblock as the same virtual
 * block across every (ce, cau) plane, so a superblock holds
 * pages_per_sb * planes * vbas_per_page VBAs and the plane index sits
 * between the page and the slot:
 *
 *   vba = vblock * (pages_per_sb * planes * 4)
 *       + page * (planes * 4) + plane * 4 + slot
 *
 * whimory_pack_vba() instead gives every (ce, cau, vblock) triple its own
 * superblock index, so a raw CXT VBA lands on an unrelated page here. Two
 * consequences: translate before use, and a run of consecutive CXT VBAs is
 * only contiguous in our space within one 4-slot group, because the next
 * group belongs to a different plane.
 */
/*
 * Now that the VBA space is the native one, a CXT VBA is already a VBA.
 * All that is left is the range check the old translation did on the way
 * through -- kept because an out-of-range CXT entry is real and must not
 * be turned into a mapping onto some unrelated block.
 */
static int whimory_cxt_vba_translate(struct whimory *w, u32 cxt_vba, u32 *out)
{
	u32 planes = w->geom.num_ce * w->geom.num_cau;
	u32 per_sb;

	if (!planes || !w->sftl.vbas_per_page || !w->sftl.pages_per_sb)
		return -EINVAL;
	per_sb = w->sftl.pages_per_sb * planes * w->sftl.vbas_per_page;
	if (cxt_vba / per_sb >= whimory_vba_blocks(w))
		return -ERANGE;
	*out = cxt_vba;
	return 0;
}

static void whimory_cxt_ext_reset(struct whimory *w)
{
	w->n_cxt_ext = 0;
	w->cxt_ext_weave = 0;
	w->cxt_ext_sb = 0;
	w->cxt_ext_max_span = 0;
}

static int whimory_cxt_ext_add(struct whimory *w, u32 lba, u32 span, u32 vba,
			       u64 weave)
{
	struct whimory_cxt_extent *e;

	if (w->n_cxt_ext >= w->max_cxt_ext)
		return -ENOSPC;
	e = &w->cxt_ext[w->n_cxt_ext++];
	e->lba = lba;
	e->span = span;
	e->vba = vba;
	e->weave = weave;
	if (span > w->cxt_ext_max_span)
		w->cxt_ext_max_span = span;
	return 0;
}

/*
 * Spell a raw CXT VBA out in the terms it is built from.
 *
 * Every question about the CXT map so far -- are the holes real, is the
 * translation ceiling right -- has been answered by arguing about
 * arithmetic. A VBA printed as vblock/page/plane/slot settles them by
 * inspection: a genuine hole sentinel is far outside the geometry, while an
 * arithmetic fault lands just past a boundary.
 */
static void whimory_vba_describe(const struct whimory *w, u32 vba,
				 char *buf, size_t len)
{
	u32 per_vb = whimory_vbas_per_vblock(w);
	u32 vblock, rem, unit, nbanks, bank;
	u8 mask;

	if (!per_vb || !w->sftl.vbas_per_page || !w->sftl.pages_per_sb) {
		scnprintf(buf, len, "?");
		return;
	}
	vblock = vba / per_vb;
	rem = vba % per_vb;
	unit = rem / w->sftl.vbas_per_page;
	/*
	 * nbanks is printed too. Reading a description without it is what
	 * made this bug so hard to see: two superblocks with the same
	 * vblk/pg/slot are different physical pages when their bank counts
	 * differ, and nothing in the old line said so.
	 */
	nbanks = whimory_sb_banks(w, vblock, &mask);
	bank = whimory_bank_at(mask, unit % nbanks);
	scnprintf(buf, len, "vblk=%u pg=%u bank=%u slot=%u nbanks=%u",
		  vblock, unit / nbanks, bank,
		  rem % w->sftl.vbas_per_page, nbanks);
}

/*
 * A TREE record is {start_lba, CONTIG_SPAN} followed by (vba, span) pairs,
 * each pair advancing the logical cursor by span. Same shape as
 * whimory_cxt_load_contig(), but it appends to the candidate map instead of
 * touching L2V.
 */
static int whimory_cxt_parse_tree(struct whimory *w, const u8 *data,
				  unsigned int len, u32 *next_lba,
				  bool *lba_valid, u64 sb_weave)
{
	u32 lba, span, vba, i, n = len / 8;

	if (len < 16)
		return 0;
	lba = get_unaligned_le32(data);
	span = get_unaligned_le32(data + 4);
	if (span == 0xffffffffu) {
		/*
		 * Header says nothing follows, and until now that was
		 * indistinguishable from a record that parsed and contained
		 * nothing. They are not the same: the two newest checkpoints
		 * on this device contribute no extents while the map is built
		 * from the two older ones, leaving it a generation behind the
		 * volume -- which is what the bad reads are. If those
		 * checkpoints are being turned away here, this says so and
		 * shows the header that did it.
		 */
		w->sftl.cxt_hdr_skipped++;
		if (w->sftl.cxt_hdr_skipped <= 6)
			dev_info(w->dev,
				 "CXT_HDR_SKIP n=%u lba=%u span=0x%08x first16=%16ph\n",
				 w->sftl.cxt_hdr_skipped, lba, span, data);
		return 0;
	}
	if (span != WHIMORY_CXT_CONTIG_SPAN) {
		w->sftl.cxt_hdr_bad++;
		if (w->sftl.cxt_hdr_bad <= 6)
			dev_info(w->dev,
				 "CXT_HDR_BAD n=%u lba=%u span=0x%08x want=0x%08x first16=%16ph\n",
				 w->sftl.cxt_hdr_bad, lba, span,
				 (u32)WHIMORY_CXT_CONTIG_SPAN, data);
		return -EINVAL;
	}
	/*
	 * A discontinuity used to abort the whole checkpoint, and with it
	 * every record after this one -- on a superblock whose records run
	 * into the thousands, and whose tail is where the newest mappings
	 * live. The reasoning was that the logical cursor places every
	 * following record, so a broken cursor puts mappings at wrong LBAs.
	 *
	 * That is true of the cursor and not of the records. Each record
	 * carries its own start LBA in its header, which is what the check
	 * compares against; when the two disagree, believing the header
	 * re-anchors the cursor and the records after it land correctly.
	 * Losing the cursor is not the same as losing the records.
	 *
	 * Counted, because a checkpoint that resynchronises repeatedly is
	 * saying something about the decode that a silent recovery would
	 * hide.
	 */
	if (*lba_valid && lba != *next_lba) {
		w->sftl.cxt_resyncs++;
		if (w->sftl.cxt_resyncs <= 8)
			dev_warn(w->dev,
				 "CXT_TREE lba discontinuity want=%u got=%u -- resyncing on the record header\n",
				 *next_lba, lba);
	}
	*lba_valid = true;

	if (cxt_dump_lba && lba < cxt_dump_lba + cxt_dump_len)
		dev_info(w->dev,
			 "CXT_REC header lba=%u contig=0x%x pairs<=%u\n",
			 lba, span, n - 1);

	for (i = 1; i < n; i++) {
		vba = get_unaligned_le32(data + 8 * i);
		span = get_unaligned_le32(data + 8 * i + 4);
		if (vba == 0xffffffffu || !span)
			break;
		w->sftl.cxt_records_seen++;

		if (cxt_dump_lba && w->sftl.cxt_dumped < cxt_dump_max &&
		    lba + span > cxt_dump_lba &&
		    lba < cxt_dump_lba + cxt_dump_len) {
			bool hole = vba >= WHIMORY_CXT_VBA_HOLE ||
				    vba >= w->l2v.invalid_vba;
			char d[64];

			w->sftl.cxt_dumped++;
			whimory_vba_describe(w, vba, d, sizeof(d));
			dev_info(w->dev,
				 "CXT_PAIR[%u] lba=%u..%u vba=0x%08x span=%u %s%s\n",
				 i, lba, lba + span - 1, vba, span,
				 hole ? "HOLE" : d,
				 hole ? "" : "");
		}
		if (vba >= WHIMORY_CXT_VBA_HOLE || vba >= w->l2v.invalid_vba) {
			/*
			 * Hole: consumes logical space, maps nothing.
			 *
			 * Sampled, because "609 holes" on its own does not say
			 * whether the checkpoint is describing unmapped space
			 * or whether the ceiling is wrong. A sentinel sits far
			 * outside the geometry; a ceiling fault sits just past
			 * it. The covered LBA count tells them apart too -- a
			 * volume a quarter full should have most of its
			 * logical space in holes.
			 */
			w->sftl.cxt_hole_entries++;
			w->sftl.cxt_hole_lbas += span;
			/*
			 * Gated: CXT_MAP already reports holes= and
			 * hole_lbas=, which is what answers "are the holes
			 * real". The per-hole lines answer "is the ceiling
			 * wrong", which is a question you ask once.
			 */
			if (ftl_diag && w->sftl.cxt_hole_entries <= 12) {
				char d[64];

				whimory_vba_describe(w, vba, d, sizeof(d));
				dev_info(w->dev,
					 "CXT_HOLE n=%u vba=0x%08x (%s) lba=%u span=%u limit=0x%x\n",
					 w->sftl.cxt_hole_entries, vba, d, lba,
					 span, w->l2v.invalid_vba);
			}
			lba += span;
			continue;
		}
		while (span) {
			/*
			 * Whole runs now. This used to break every extent at
			 * the next 4-slot boundary because consecutive CXT
			 * VBAs were not contiguous in the old bank-major
			 * space. They are in the native one, so a run stays a
			 * run and the range count drops by about four times.
			 */
			u32 chunk = span;
			u32 tvba;
			int ret;

			/*
			 * A free consistency check on the bank map.
			 *
			 * A checkpoint never names an address past the end of
			 * its superblock, so an offset at or beyond
			 * nbanks * pages_per_sb * vbas_per_page says our bank
			 * count for that virtual block is too small -- the
			 * classify signal missed a member. Counted here
			 * because it costs nothing and it is the one symptom
			 * that distinguishes a bad derivation from a bad
			 * checkpoint.
			 */
			{
				u32 per_vb = whimory_vbas_per_vblock(w);
				u32 vblk = per_vb ? vba / per_vb : 0;
				u32 nb = whimory_sb_banks(w, vblk, NULL);
				u32 end = nb * w->sftl.pages_per_sb *
					  w->sftl.vbas_per_page;

				if (per_vb && vba % per_vb >= end) {
					w->sftl.sb_bank_overflow++;
					if (w->sftl.sb_bank_overflow <= 8)
						dev_warn(w->dev,
							 "CXT_BANK_SHORT vba=0x%08x vblk=%u ofs=%u >= %u (nbanks=%u) lba=%u span=%u\n",
							 vba, vblk,
							 vba % per_vb, end, nb,
							 lba, chunk);
				}
			}

			if (!whimory_cxt_vba_translate(w, vba, &tvba)) {
				ret = whimory_cxt_ext_add(w, lba, chunk, tvba,
							  sb_weave);
				if (ret) {
					/* Loud: a full table is a short map,
					 * and a short map is lost files.
					 */
					w->sftl.cxt_ext_nospc++;
					dev_warn(w->dev,
						 "CXT extent table full at %u -- map will be short\n",
						 w->n_cxt_ext);
					return ret;
				}
			} else {
				w->sftl.cxt_xlate_fail++;
				if (w->sftl.cxt_xlate_fail <= 12) {
					char d[64];

					whimory_vba_describe(w, vba, d,
							     sizeof(d));
					dev_info(w->dev,
						 "CXT_XLATE_FAIL n=%u vba=0x%08x (%s) lba=%u span=%u vba_blocks=%u\n",
						 w->sftl.cxt_xlate_fail, vba, d,
						 lba, chunk,
						 whimory_vba_blocks(w));
				}
			}
			lba += chunk;
			vba += chunk;
			span -= chunk;
		}
		continue;
	}
	*next_lba = lba;
	return 0;
}

/* Walk the records of one CXT superblock and collect every TREE extent. */
static int whimory_cxt_build_from_sb(struct whimory *w, u32 sb_idx,
				     u64 sb_weave)
{
	struct whimory_sftl *s = &w->sftl;
	u8 *data = s->gc_data;
	u8 meta[WHIMORY_META_SIZE];
	u8 spare[S5L8740_NAND_META_SIZE];
	u32 ofs, last_key = ~0u, next_lba = 0, n_l2v = 0;
	/*
	 * Eight entries, not 256.
	 *
	 * A full byte-indexed histogram is a kilobyte of stack, which put
	 * this frame at 1368 bytes -- past the warning and a sixth of an
	 * 8 KiB kernel stack, on a function that recurses into the NAND read
	 * path. Every tag this walk can meet is 1..6 or CLEAN at 0xff, and
	 * CLEAN is counted separately because it ends the walk; anything
	 * else is by definition unrecognised and only needs a count.
	 */
	u32 tag_hist[8] = { 0 };
	u32 tag_other = 0;
	u32 n_cxt_meta = 0, n_clean = 0;
	u32 vblock = whimory_cxt_sb_vblock(w, sb_idx);
	/*
	 * How many addresses this checkpoint superblock actually holds.
	 *
	 * s_cxt_load.c walks a checkpoint from 0 to the value the VFL gives
	 * for that superblock -- sub_4EFE0C, banks_in_vbn * pages_per_block *
	 * vbas_per_page -- not to a fixed per-virtual-block stride. A
	 * checkpoint on a short superblock has its records packed over the
	 * banks it has; walking the full 2048 offsets would read them in the
	 * wrong order, and the tree cannot survive that because each record
	 * declares the LBA it continues from.
	 *
	 * This one happens to be a four-bank superblock on this unit, so the
	 * two bounds agree today. They agree by luck.
	 */
	u32 nbanks = whimory_sb_banks(w, vblock, NULL);
	u32 per_sb = nbanks * s->pages_per_sb * s->vbas_per_page;
	u32 per_vb = whimory_vbas_per_vblock(w);
	bool lba_valid = false;
	int ret;

	if (!data || !s->data_page || !per_sb)
		return -ENOMEM;

	/*
	 * The whole superblock, every bank, in the order the FTL wrote it --
	 * not one bank at a time. See whimory_cxt_read_ofs().
	 */
	for (ofs = 0; ofs < per_sb; ofs++) {
		ret = whimory_cxt_read_ofs(w, vblock, ofs, data, meta, spare,
					   &last_key);
		if (!ret && cxt_trace_sb && ofs < cxt_trace_sb) {
			u32 tv = vblock * per_vb + ofs;
			unsigned int tce, tcau, tvb, tpg, tsl;
			char d[64] = "?";

			if (!whimory_unpack_vba(w, tv, &tce, &tcau, &tvb, &tpg,
						&tsl))
				scnprintf(d, sizeof(d),
					  "ce%u/cau%u/vblk%u/pg%u/slot%u",
					  tce, tcau, tvb, tpg, tsl);
			dev_info(w->dev,
				 "CXT_TRACE sb=%u ofs=%u vba=%u %s meta=%16ph data8=%8ph\n",
				 sb_idx, ofs, tv, d, meta, data);
			if (tsl == 0)
				dev_info(w->dev,
					 "CXT_TRACE   page metas s0=%16ph s1=%16ph\n",
					 spare, spare + WHIMORY_META_SIZE);
		}
		if (ret) {
			dev_warn(w->dev,
				 "CXT sb=%u read failed at ofs=%u/%u (%d) -- rest of this checkpoint dropped\n",
				 sb_idx, ofs, per_sb, ret);
			return ret;
		}
		if (meta[0] != WHIMORY_META_TYPE_SFTL_CXT)
			continue;
		n_cxt_meta++;
		if (meta[1] < ARRAY_SIZE(tag_hist))
			tag_hist[meta[1]]++;
		else
			tag_other++;
		if (meta[1] == WHIMORY_CXT_TAG_CLEAN) {
			n_clean++;
			break;
		}
		/*
		 * The BASE payload is {count, sb, sb, ...} -- s_cxt_save.c
		 * writes buf[0] = cxt->save.num_sb and then the list. That
		 * count is what the weave fast-forward is scaled by, so it is
		 * worth taking rather than guessing from how many checkpoint
		 * superblocks classify happened to find.
		 */
		if (meta[1] == WHIMORY_CXT_TAG_BASE && !w->cxt_save_num_sb) {
			u32 n = get_unaligned_le32(data);

			if (n && n <= WHIMORY_CXT_MAX_SB)
				w->cxt_save_num_sb = n;
		}
		if (meta[1] != WHIMORY_CXT_TAG_L2V)
			continue;
		n_l2v++;
		ret = whimory_cxt_parse_tree(w, data, WHIMORY_LBA_SIZE,
					     &next_lba, &lba_valid, sb_weave);
		if (ret) {
			/*
			 * Stop this checkpoint, but say what stopping cost.
			 *
			 * Carrying on past a bad record is not an option: the
			 * logical cursor is what places every record after it,
			 * so a record parsed against a broken cursor lands at
			 * the wrong LBA, and a mapping in the wrong place is
			 * worse than a mapping missing. But the records after
			 * it were silently lost before, and a checkpoint that
			 * quietly stops halfway looks exactly like one that
			 * finished.
			 */
			s->cxt_records_lost += per_sb - ofs;
			dev_warn(w->dev,
				 "CXT sb=%u parse stopped at record %u of %u (%d, %u L2V records read) -- up to %u records dropped\n",
				 sb_idx, ofs, per_sb, ret, n_l2v,
				 per_sb - ofs);
			return ret;
		}
	}
	if (!n_l2v) {
		/*
		 * A checkpoint that contributes nothing is not necessarily
		 * stale, and on this device it is not: the two newest CXT
		 * superblocks -- weave 2049391 and 2049387 -- both land here,
		 * while the map is built from the two older ones at 2049383
		 * and 2049379. That is a map one generation behind the
		 * volume, which is exactly what the bad reads look like: the
		 * checkpoint names a page whose contents were superseded, and
		 * the page still holds what it held at weave 687252.
		 *
		 * So the tags actually present are worth having. The walk
		 * only parses WHIMORY_CXT_TAG_L2V and stops at
		 * WHIMORY_CXT_TAG_CLEAN; if the newest checkpoints carry the
		 * tree under some other tag, or lead with a CLEAN record that
		 * stops the walk before the tree, this says so instead of
		 * leaving it as "superseded".
		 */
		char hb[160];
		unsigned int t, hn = 0;

		for (t = 0; t < ARRAY_SIZE(tag_hist); t++) {
			if (!tag_hist[t] || hn + 14 >= sizeof(hb))
				continue;
			hn += scnprintf(hb + hn, sizeof(hb) - hn, "%02x:%u ",
					t, tag_hist[t]);
		}
		if (tag_other && hn + 16 < sizeof(hb))
			scnprintf(hb + hn, sizeof(hb) - hn, "other:%u ",
				  tag_other);
		dev_info(w->dev,
			 "CXT sb=%u no L2V records: cxt_meta=%u clean=%u tags=%s(l2v=0x%02x clean=0x%02x)\n",
			 sb_idx, n_cxt_meta, n_clean, hb,
			 WHIMORY_CXT_TAG_L2V, WHIMORY_CXT_TAG_CLEAN);
	}
	return 0;
}

/*
 * Build the candidate map from the newest CXT base that parses cleanly.
 * Never touches w->ranges.
 */
/*
 * A total order, deliberately.
 *
 * This used to compare lba alone, which left every pair of extents with the
 * same lba -- and the candidate map is the union of four checkpoint
 * generations, so there are many -- in whatever order sort() happened to
 * leave them. sort() is heapsort and is not stable, so that order was an
 * artefact of the heap rather than of the flash: the same recovery over the
 * same NAND could seed an LBA from either generation, and the map came out
 * different on every boot.
 *
 * Ordering by weave second makes the newest extent for an lba the last one,
 * every time. whimory_cxt_seed_l2v() then replays in index order and the
 * newest legitimately wins, and whimory_cxt_lookup() takes the last match
 * for the same reason.
 */
static int whimory_cxt_ext_cmp(const void *a, const void *b)
{
	const struct whimory_cxt_extent *x = a, *y = b;

	if (x->lba != y->lba)
		return x->lba < y->lba ? -1 : 1;
	if (x->weave != y->weave)
		return x->weave < y->weave ? -1 : 1;
	/*
	 * Same lba, same generation: order on vba so the result is a function
	 * of the media and not of the heap. Two extents this alike are a
	 * malformed checkpoint either way; CXT_VBA_OVERLAP reports them.
	 */
	if (x->vba != y->vba)
		return x->vba < y->vba ? -1 : 1;
	return 0;
}

/* Total order, for the same reason whimory_cxt_ext_cmp() is one. */
static int whimory_cxt_ext_vba_cmp(const void *a, const void *b)
{
	const struct whimory_cxt_extent *x = a, *y = b;

	if (x->vba != y->vba)
		return x->vba < y->vba ? -1 : 1;
	if (x->weave != y->weave)
		return x->weave < y->weave ? -1 : 1;
	if (x->lba != y->lba)
		return x->lba < y->lba ? -1 : 1;
	return 0;
}

/*
 * Do two extents claim the same physical VBA?
 *
 * The overlap check next to this one sorts by LBA and asks whether two
 * extents claim the same logical block. That is the wrong axis for the
 * failure that is left: a checkpoint mapping lba 555968 to a VBA whose page
 * holds lba 564360, with both extents structurally sound. Two extents can
 * name disjoint LBA ranges and still point at the same place, and nothing
 * looked for it -- overlaps=0 was measuring the other axis and reading as
 * "no collisions".
 *
 * The seed arbitrates on weave now, so a VBA claimed by two generations
 * resolves the way the FTL wrote it: the newer extent wins. What ordering
 * cannot fix is two extents at the SAME weave claiming overlapping VBAs --
 * one checkpoint cannot have put two logical ranges in one physical place,
 * so that is a decode fault. The two are counted apart, because lumping
 * them together is what made a healthy map look damaged.
 */
static void whimory_cxt_check_vba_overlaps(struct whimory *w)
{
	struct whimory_cxt_extent *by_vba;
	unsigned int i, n = w->n_cxt_ext, hits = 0, shown = 0, cross_gen = 0;
	size_t bytes;

	if (n < 2)
		return;
	bytes = (size_t)n * sizeof(*by_vba);
	by_vba = kvmalloc(bytes, GFP_KERNEL);
	if (!by_vba) {
		dev_info(w->dev, "CXT_VBA_OVERLAP skipped (no memory)\n");
		return;
	}
	memcpy(by_vba, w->cxt_ext, bytes);
	sort(by_vba, n, sizeof(*by_vba), whimory_cxt_ext_vba_cmp, NULL);

	for (i = 1; i < n; i++) {
		const struct whimory_cxt_extent *p = &by_vba[i - 1];
		const struct whimory_cxt_extent *c = &by_vba[i];

		if (c->vba >= p->vba + p->span)
			continue;
		/*
		 * Different generations may legitimately describe the same
		 * physical VBA; the seed resolves that on weave. Only a
		 * same-generation collision is a fault.
		 */
		if (c->weave != p->weave) {
			cross_gen++;
			continue;
		}
		hits++;
		if (shown < 8) {
			shown++;
			dev_err(w->dev,
				"CXT_VBA_OVERLAP lba=%u span=%u vba=%u overlaps lba=%u span=%u vba=%u by %u (weave=%llu)\n",
				c->lba, c->span, c->vba,
				p->lba, p->span, p->vba,
				p->vba + p->span - c->vba,
				(unsigned long long)c->weave);
		}
	}
	if (hits)
		dev_err(w->dev,
			"CXT_VBA_OVERLAP %u extents claim a VBA another at the same weave already holds -- decode fault\n",
			hits);
	else
		dev_info(w->dev,
			 "CXT_VBA_OVERLAP none within a generation (cross_gen=%u, resolved by weave)\n",
			 cross_gen);
	kvfree(by_vba);
}

/*
 * Build the candidate map from every CXT superblock.
 *
 * The TREE is partitioned by logical range across the CXT blocks — on this
 * device sb 1702 starts at LBA 0 and sb 3662 picks up at 613939 — so taking
 * the first superblock that parses leaves most of the volume, including the
 * FAT-critical sectors, unmapped. Merge them all, then sort by LBA so the
 * lookup can binary search. Never touches w->ranges.
 */
static int whimory_cxt_build_candidate(struct whimory *w)
{
	struct whimory_cxt_base all[WHIMORY_CXT_MAX_SB];
	unsigned int i, n_all, ok = 0, overlaps = 0, regenerated = 0;
	int ret, sess;

	/*
	 * Allocated on first use: translation splits every CXT run at 4-slot
	 * plane boundaries, so the table is roughly one entry per four LBAs
	 * and is only worth its megabytes when the CXT path is exercised.
	 */
	if (!w->cxt_ext) {
		w->max_cxt_ext = cxt_max_extents;
		w->cxt_ext = kvmalloc_array(w->max_cxt_ext,
					    sizeof(*w->cxt_ext), GFP_KERNEL);
		if (!w->cxt_ext) {
			w->max_cxt_ext = 0;
			return -ENOMEM;
		}
	}
	n_all = whimory_cxt_collect_sbs(w, all, ARRAY_SIZE(all));
	if (!n_all)
		return -ENOENT;

	whimory_cxt_ext_reset(w);
	w->sftl.cxt_records_seen = 0;
	w->sftl.cxt_hole_entries = 0;
	w->sftl.cxt_xlate_fail = 0;
	w->sftl.cxt_hole_lbas = 0;
	w->sftl.cxt_ext_nospc = 0;
	w->sftl.cxt_records_lost = 0;
	w->sftl.cxt_resyncs = 0;
	w->sftl.cxt_sb_empty = 0;

	sess = s5l8740_nand_dma_session_begin();
	for (i = 0; i < n_all; i++) {
		u32 before = w->n_cxt_ext;

		ret = whimory_cxt_build_from_sb(w, all[i].sb, all[i].weave);
		if (ret) {
			dev_warn(w->dev,
				 "CXT_CAND_MAP sb=%u parse failed %d\n",
				 all[i].sb, ret);
			continue;
		}
		if (w->n_cxt_ext == before) {
			/*
			 * Counted. sbs_used=2/4 read as "two checkpoints were
			 * stale", and it may well be, but nothing here had
			 * ever checked -- a superblock that parsed fine and
			 * produced nothing left exactly the same trace as one
			 * that was never looked at.
			 */
			w->sftl.cxt_sb_empty++;
			dev_info(w->dev,
				 "CXT_CAND_MAP sb=%u weave=%llu parsed but added no extents\n",
				 all[i].sb, (unsigned long long)all[i].weave);
			continue;
		}
		ok++;
		if (all[i].weave > w->cxt_ext_weave) {
			w->cxt_ext_weave = all[i].weave;
			w->cxt_ext_sb = all[i].sb;
		}
		dev_info(w->dev,
			 "CXT_CAND_MAP sb=%u weave=%llu extents=+%u total=%u\n",
			 all[i].sb, (unsigned long long)all[i].weave,
			 w->n_cxt_ext - before, w->n_cxt_ext);
	}
	if (sess == 0)
		s5l8740_nand_dma_session_end();

	if (!w->n_cxt_ext)
		return -ENODATA;

	sort(w->cxt_ext, w->n_cxt_ext, sizeof(*w->cxt_ext),
	     whimory_cxt_ext_cmp, NULL);
	for (i = 1; i < w->n_cxt_ext; i++)
	/*
	 * Two different things used to be counted as one number.
	 *
	 * An extent that starts at the same LBA as the previous one is a
	 * second checkpoint generation describing the same range; that is
	 * expected, is now ordered by weave, and the seed resolves it. An
	 * extent that starts partway into the previous one is a genuine
	 * partial overlap and is not resolvable by ordering alone. Reporting
	 * them together made a healthy map look damaged and hid the case
	 * that matters.
	 */
	for (i = 1; i < w->n_cxt_ext; i++) {
		const struct whimory_cxt_extent *p = &w->cxt_ext[i - 1];
		const struct whimory_cxt_extent *c = &w->cxt_ext[i];

		if (c->lba >= p->lba + p->span)
			continue;
		if (c->lba == p->lba)
			regenerated++;
		else
			overlaps++;
	}

	dev_info(w->dev,
		 "CXT_MAP sbs_used=%u/%u extents=%u records=%u holes=%u "
		 "xlate_fail=%u overlaps=%u regenerated=%u base_weave=%llu\n",
		 ok, n_all, w->n_cxt_ext, w->sftl.cxt_records_seen,
		 w->sftl.cxt_hole_entries, w->sftl.cxt_xlate_fail, overlaps,
		 regenerated, (unsigned long long)w->cxt_ext_weave);
	/*
	 * The accounting the previous line was missing. hole_lbas is the one
	 * that settles whether the holes are a fault: this volume maps about
	 * 938k of 3.86M sectors, so if the holes cover roughly the other
	 * three quarters they are describing unmapped space and are correct.
	 * If they cover a little and there are hundreds of them, they are
	 * not.
	 */
	dev_info(w->dev,
		 "CXT_MAP hole_lbas=%u empty_sbs=%u records_lost=%u nospc=%u "
		 "resyncs=%u hdr_bad=%u\n",
		 w->sftl.cxt_hole_lbas, w->sftl.cxt_sb_empty,
		 w->sftl.cxt_records_lost, w->sftl.cxt_ext_nospc,
		 w->sftl.cxt_resyncs, w->sftl.cxt_hdr_bad);
	whimory_cxt_check_vba_overlaps(w);
	return 0;
}

/*
 * Compare the candidate map against the brute-force interval map, which is
 * ground truth because it comes from per-slot metadata. Any systematic
 * decode error shows up as a repeated (vba_cxt - vba_brute) delta.
 */
#define WHIMORY_CXT_DELTA_SLOTS	8

static void whimory_cxt_compare(struct whimory *w)
{
	s64 delta_val[WHIMORY_CXT_DELTA_SLOTS] = {0};
	u32 delta_cnt[WHIMORY_CXT_DELTA_SLOTS] = {0};
	u32 agree = 0, disagree = 0, absent = 0, checked = 0;
	u32 i, j, shown = 0;
	u64 covered = 0;

	for (i = 0; i < w->n_cxt_ext; i++) {
		struct whimory_cxt_extent *e = &w->cxt_ext[i];
		u32 probes[3], np = 0, k;

		covered += e->span;
		probes[np++] = e->lba;
		if (e->span > 2)
			probes[np++] = e->lba + e->span / 2;
		if (e->span > 1)
			probes[np++] = e->lba + e->span - 1;

		for (k = 0; k < np; k++) {
			u32 lba = probes[k];
			u32 want = e->vba + (lba - e->lba);
			struct whimory_range *r;
			s64 d;

			checked++;
			r = whimory_range_find(&w->ranges, lba);
			if (!r) {
				absent++;
				continue;
			}
			d = (s64)want - (s64)(r->vba + (lba - r->start));
			if (!d) {
				agree++;
				continue;
			}
			disagree++;
			if (shown++ < diag_max_lines) {
				u32 bv = r->vba + (lba - r->start);
				u32 per = w->sftl.vbas_per_sb;
				u32 dper = WHIMORY_DATA_VBAS_PER_SB;

				/*
				 * Decompose both VBAs in the 512-per-SB space
				 * we pack into and in the 508-per-SB data-only
				 * space, so a units mismatch is visible rather
				 * than inferred.
				 */
				dev_info(w->dev,
					 "CXT_CMP lba=%u cxt_vba=%u brute_vba=%u delta=%lld\n",
					 lba, want, bv, (long long)d);
				dev_info(w->dev,
					 "  cxt   sb512=%u ofs512=%u sb508=%u ofs508=%u\n",
					 want / per, want % per,
					 want / dper, want % dper);
				dev_info(w->dev,
					 "  brute sb512=%u ofs512=%u sb508=%u ofs508=%u\n",
					 bv / per, bv % per,
					 bv / dper, bv % dper);
			}
			for (j = 0; j < WHIMORY_CXT_DELTA_SLOTS; j++) {
				if (delta_cnt[j] && delta_val[j] != d)
					continue;
				delta_val[j] = d;
				delta_cnt[j]++;
				break;
			}
		}
	}

	dev_info(w->dev,
		 "CXT_COMPARE extents=%u covered_lbas=%llu checked=%u "
		 "agree=%u disagree=%u absent_in_brute=%u\n",
		 w->n_cxt_ext, covered, checked, agree, disagree, absent);
	for (j = 0; j < WHIMORY_CXT_DELTA_SLOTS; j++) {
		if (!delta_cnt[j])
			continue;
		dev_info(w->dev, "CXT_DELTA %lld x%u\n",
			 (long long)delta_val[j], delta_cnt[j]);
	}
}

/*
 * Resolve an LBA through the candidate map only.
 *
 * The table is sorted (lba, weave) ascending and holds every checkpoint
 * generation, so several extents can cover one LBA. Answering with the first
 * one a binary search happens to land on picks a generation at random, which
 * is the same fault whimory_cxt_ext_cmp() was fixed for. Find the upper
 * bound on start LBA, then walk back and keep the newest weave among the
 * extents that actually cover it.
 *
 * The walk is bounded: it stops as soon as it is further back than the
 * longest extent in the table could reach.
 */
static int whimory_cxt_lookup(struct whimory *w, u32 lba, u32 *vba_out)
{
	u32 lo = 0, hi = w->n_cxt_ext, i;
	u64 best_weave = 0;
	bool found = false;

	/* First index whose start LBA is strictly greater than lba. */
	while (lo < hi) {
		u32 mid = lo + (hi - lo) / 2;

		if (w->cxt_ext[mid].lba <= lba)
			lo = mid + 1;
		else
			hi = mid;
	}

	for (i = lo; i-- > 0; ) {
		struct whimory_cxt_extent *e = &w->cxt_ext[i];

		if (lba - e->lba >= w->cxt_ext_max_span)
			break;
		if (lba >= e->lba + e->span)
			continue;
		if (found && e->weave <= best_weave)
			continue;
		*vba_out = e->vba + (lba - e->lba);
		best_weave = e->weave;
		found = true;
	}
	return found ? 0 : -ENOENT;
}

/*
 * Read the BPB and the FAT-critical sectors through the candidate map and
 * check that each landed on a page whose metadata claims the LBA we asked
 * for. This is the gate that decides whether the CXT map is usable.
 */
static int whimory_cxt_validate(struct whimory *w, u32 fat_base)
{
	static const u32 rel[] = { 0, 1, 2, 6, 7, 8 };
	struct whimory_meta meta;
	u8 *buf;
	unsigned int i;
	u32 ok = 0, bad = 0, miss = 0;
	int ret, sess;

	if (!w->n_cxt_ext)
		return -ENODATA;
	buf = kmalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	sess = s5l8740_nand_dma_session_begin();
	for (i = 0; i < ARRAY_SIZE(rel); i++) {
		u32 lba = fat_base + rel[i];
		u32 vba = 0, mlba;

		ret = whimory_cxt_lookup(w, lba, &vba);
		if (ret) {
			miss++;
			dev_info(w->dev, "CXT_VALID lba=%u UNMAPPED\n", lba);
			continue;
		}
		ret = w->vfl_ops->read_vba(w, vba, 1, buf, &meta);
		if (ret) {
			bad++;
			dev_info(w->dev, "CXT_VALID lba=%u vba=%u read %d\n",
				 lba, vba, ret);
			continue;
		}
		mlba = le32_to_cpu(meta.lba);
		if (mlba == lba) {
			ok++;
		} else {
			bad++;
			dev_info(w->dev,
				 "CXT_VALID lba=%u vba=%u meta_lba=%u MISMATCH\n",
				 lba, vba, mlba);
		}
	}
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	kfree(buf);

	dev_info(w->dev,
		 "CXT_VALIDATE fat_base=%u ok=%u bad=%u unmapped=%u verdict=%s\n",
		 fat_base, ok, bad, miss,
		 (ok == ARRAY_SIZE(rel)) ? "USABLE" : "NOT_USABLE");
	return (ok == ARRAY_SIZE(rel)) ? 0 : -EBADMSG;
}


/*
 * Sample extents across the candidate map, read the VBA the CXT claims, and
 * print what the page metadata actually says. If the decode is merely stale
 * the page still claims the LBA we asked for; if it is wrong, meta_lba is
 * unrelated and the (expected, actual) pairs expose the transform.
 */
static void whimory_cxt_probe(struct whimory *w, unsigned int nsamples)
{
	struct whimory_meta meta;
	u8 *buf;
	u32 step, i, ok = 0, stale = 0, wrong = 0, blank = 0, zero = 0;
	int ret, sess;

	if (!w->n_cxt_ext || !nsamples)
		return;
	buf = kmalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);
	if (!buf)
		return;

	step = w->n_cxt_ext / nsamples;
	if (!step)
		step = 1;

	sess = s5l8740_nand_dma_session_begin();
	for (i = 0; i < w->n_cxt_ext; i += step) {
		struct whimory_cxt_extent *e = &w->cxt_ext[i];
		u32 mlba;

		ret = w->vfl_ops->read_vba(w, e->vba, 1, buf, &meta);
		if (ret) {
			wrong++;
			continue;
		}
		mlba = le32_to_cpu(meta.lba);
		if (mlba == e->lba) {
			ok++;
			continue;
		}
		if (mlba == 0xffffffffu)
			blank++;
		else if (!mlba)
			zero++;
		else
			wrong++;
		if (stale++ < 12)
			dev_info(w->dev,
				 "CXT_PROBE ext=%u lba=%u span=%u vba=%u "
				 "meta_type=%02x meta_lba=%u diff=%d\n",
				 i, e->lba, e->span, e->vba, meta.type, mlba,
				 (int)(mlba - e->lba));
	}
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	kfree(buf);

	dev_info(w->dev,
		 "CXT_PROBE_SUM sampled=%u ok=%u wrong=%u blank=%u zero_lba=%u\n",
		 (w->n_cxt_ext + step - 1) / step, ok, wrong, blank, zero);
}

/*
 * Seed the interval map from the CXT snapshot.
 *
 * Each extent is claimed at the weave of the checkpoint that produced it,
 * not at the newest weave in the set. That is what lets the normal winner
 * rules do their job: an older generation's extent loses to a newer one
 * covering the same LBA, and the diff replay can still override either.
 *
 * Claiming everything at w->cxt_ext_weave -- as this used to -- made every
 * seed extent look equally new, so whimory_range_update()'s stale check
 * could never fire between two of them and the last one written won
 * regardless of generation. Combined with an lba-only sort over an unstable
 * sort(), which one that was came down to the heap.
 */
/*
 * Does the page a CXT extent points at actually hold the LBA it claims?
 *
 * A checkpoint is a snapshot of the map at one commit. Its (lba, span,
 * vba) triples were true then, and stay true only while nothing moves.
 * Everything written afterwards lives in the journal, and any LBA
 * rewritten since has left its old page behind -- a page the allocator
 * has very likely recycled and refilled with data belonging to some
 * other LBA. Replay the snapshot without checking and those stale
 * triples are indistinguishable from good ones, because nothing inside
 * a CXT record can be self-inconsistent.
 *
 * Measured on this unit after RetailOS had written: seeding from CXT
 * gave mapped_lbas=950050 and a map that failed nearly every read, with
 * "sftl lba mismatch want=0xc085 meta=0xcd97a" -- thousands of scattered
 * low LBAs all resolving into one narrow band of pages that belong to
 * LBAs around 842000. Rebuilding with use_cxt=0 produced a smaller map
 * that read correctly and found the BPB. The seed was not malformed. It
 * was stale, and it was winning.
 *
 * The BTOC path never had this problem because it does not trust its own
 * index: whimory_l2v_update_from_slot_meta() reads the page and takes
 * the meta LBA as the authority, the same way the YaFTL reference does
 * in verifyUserSpares(), checking spare.lpn against the page it was
 * asked for. This holds the CXT path to that standard, so a stale
 * checkpoint costs coverage rather than correctness.
 *
 * Only the first slot of the extent is read. An extent is a contiguous
 * run laid down by one write, so its head is representative, and
 * checking every slot would multiply recovery time by the run length to
 * learn nothing new in the common case.
 */
static bool whimory_cxt_extent_confirmed(struct whimory *w,
						 const struct whimory_cxt_extent *e)
{
	u8 spare[S5L8740_NAND_META_SIZE];
	u8 *data = w->sftl.data_page;
	u32 ce, cau, vblock, page, slot;
	u32 pblock, rcau, meta_lba;
	const u8 *m;
	int ret;

	if (!cxt_meta_confirm || !data)
		return true;
	if (cxt_confirm_max && w->sftl.cxt_confirm_pages >= cxt_confirm_max)
		return true;
	if (recover_budget_ms && w->sftl.confirm_start_jiffies &&
		    time_after(jiffies, w->sftl.confirm_start_jiffies +
			       msecs_to_jiffies(recover_budget_ms)))
		return true;
	if (whimory_unpack_vba(w, e->vba, &ce, &cau, &vblock, &page, &slot))
		return true;
	if (slot >= WHIMORY_VBAS_PER_PAGE)
		return true;

	rcau = cau;
	whimory_vfl_resolve(w, vblock, &rcau, &pblock);
	ret = whimory_cs_read_page(w, ce, rcau, pblock, page, data,
				   S5L8740_NAND_PAGE_SIZE, spare, sizeof(spare));
	if (ret) {
		/*
		 * Unreadable is not evidence of staleness. Keep the extent
		 * and let the per-read meta check catch it later.
		 */
		w->sftl.cxt_confirm_unreadable++;
		return true;
	}
	w->sftl.cxt_confirm_pages++;
	if (recover_yield_us && (w->sftl.cxt_confirm_pages & 0x0f) == 0) {
		cond_resched();
		usleep_range(recover_yield_us, recover_yield_us + 500);
	}

	m = spare + slot * WHIMORY_META_SIZE;
	if (whimory_meta_erased(m, WHIMORY_META_SIZE))
		return true;
	if (m[0] != WHIMORY_META_TYPE_DATA && m[0] != WHIMORY_META_TYPE_DATA2)
		return true;

	meta_lba = get_unaligned_le32(m + 8);
	if (meta_lba == e->lba) {
		w->sftl.cxt_meta_confirmed++;
		return true;
	}
	w->sftl.cxt_meta_mismatch++;
	dev_dbg(w->dev,
		"cxt_meta_mismatch lba=%u span=%u vba=%u meta_lba=%u pg=%u slot=%u\n",
		e->lba, e->span, e->vba, meta_lba, page, slot);
	return false;
}

/*
 * Rebuild the LBAs a stale checkpoint extent had claimed, from the pages
 * themselves.
 *
 * Refusing a stale extent is necessary and on its own it is not enough.
 * Dropping it leaves the range unclaimed, and the next-best claim for
 * those LBAs is generally an even older one, so the map ends up wrong in
 * a different way: on this unit, dropping alone moved LBA 49279 from a
 * page holding someone else data to a page holding no data at all, and
 * the FAT signature check failed instead of the read.
 *
 * So the extent is re-derived rather than deleted. Every page it covers
 * is read once and each slot is applied from its own metadata, which is
 * exactly how the BTOC walk builds the map and what the YaFTL reference
 * does in restoreUserBlock(): the page says which logical block it holds,
 * and that statement is authoritative because it was written with the
 * data. A checkpoint can go stale. A page cannot lie about itself.
 *
 * Only extents that failed confirmation get here, so the cost is bounded
 * by how wrong the checkpoint is rather than by its size -- 37 extents
 * out of 3200 on this unit.
 */
static void whimory_cxt_extent_repair(struct whimory *w,
					      const struct whimory_cxt_extent *e)
{
	u8 spare[S5L8740_NAND_META_SIZE];
	u8 *data = w->sftl.data_page;
	u32 last_ce = ~0u, last_cau = ~0u, last_vblock = ~0u, last_page = ~0u;
	u32 i;

	if (!data)
		return;

	for (i = 0; i < e->span; i++) {
		u32 ce, cau, vblock, page, slot, pblock, rcau;
		int ret;

		if (whimory_unpack_vba(w, e->vba + i, &ce, &cau, &vblock,
					      &page, &slot))
			return;
		if (slot >= WHIMORY_VBAS_PER_PAGE)
			continue;

		/* One read per page, not per slot. */
		if (ce != last_ce || cau != last_cau ||
		    vblock != last_vblock || page != last_page) {
			if (recover_budget_ms && w->sftl.confirm_start_jiffies &&
			    time_after(jiffies, w->sftl.confirm_start_jiffies +
				       msecs_to_jiffies(recover_budget_ms)))
				return;
			rcau = cau;
			whimory_vfl_resolve(w, vblock, &rcau, &pblock);
			ret = whimory_cs_read_page(w, ce, rcau, pblock, page, data,
						   S5L8740_NAND_PAGE_SIZE, spare,
						   sizeof(spare));
			if (ret)
				return;
			last_ce = ce;
			last_cau = cau;
			last_vblock = vblock;
			last_page = page;
			w->sftl.cxt_repair_pages++;
			if (recover_yield_us &&
			    (w->sftl.cxt_repair_pages & 0x0f) == 0) {
				cond_resched();
				usleep_range(recover_yield_us,
					     recover_yield_us + 500);
			}
		}

		/*
		 * No BTOC hint: the checkpoint already proved itself wrong
		 * about this run, so the metadata is the only witness left.
		 */
		if (whimory_l2v_update_from_slot_meta(w, ce, cau, vblock, page,
							      slot,
							      spare + slot *
							      WHIMORY_META_SIZE,
							      0xffffffffu) < 0)
			return;
	}
}

static int whimory_cxt_seed_l2v(struct whimory *w)
{
	u32 i, seeded = 0, regenerated = 0;
	int ret;

	w->sftl.claim_source = 3;
	for (i = 0; i < w->n_cxt_ext; i++) {
		struct whimory_cxt_extent *e = &w->cxt_ext[i];

		/*
		 * Everything is applied, oldest generation first.
		 *
		 * Skipping an extent because a newer one starts at the same
		 * LBA looks tempting and is wrong: the two need not have the
		 * same span. An old extent covering lba 100 for 1000 sectors
		 * and a new one covering lba 100 for 10 describe the same
		 * start and different amounts of the volume, and dropping the
		 * old one would lose 990 sectors outright.
		 *
		 * Applying both in ascending weave order gets it right
		 * without a special case: the older lands first, the newer
		 * overwrites exactly the part it claims, and the rest of the
		 * older extent stays. whimory_range_update() protects the
		 * newer from ever being clobbered by an older claim.
		 */
		if (i && e->lba == w->cxt_ext[i - 1].lba)
			regenerated++;

		if (!whimory_cxt_extent_confirmed(w, e)) {
			whimory_cxt_extent_repair(w, e);
			cond_resched();
			continue;
		}

		w->sftl.claim_weave = e->weave;
		ret = whimory_l2v_update(w, e->lba, e->span, e->vba);
		w->sftl.claim_weave = 0;
		if (ret) {
			w->sftl.claim_source = 0;
			return ret;
		}
		seeded += e->span;
		w->sftl.cxt_l2v_updates++;
		if ((i & 0x3fff) == 0)
			cond_resched();
	}
	w->sftl.claim_source = 0;
	dev_info(w->dev,
		 "CXT_SEED extents=%u regenerated=%u lbas=%u ranges=%u base_weave=%llu\n",
		 w->n_cxt_ext, regenerated, seeded, w->sftl.range_nodes,
		 (unsigned long long)w->cxt_ext_weave);
	return 0;
}

/*
 * Fast path: seed from the CXT, then let the caller replay only the
 * superblocks newer than the checkpoint. Returns 0 when the map is seeded.
 */
static int whimory_cxt_fast_load(struct whimory *w)
{
	int ret;

	ret = whimory_cxt_build_candidate(w);
	if (ret) {
		dev_warn(w->dev,
			 "CXT fast path unavailable (%d); full replay\n",
			 ret);
		return ret;
	}
	ret = whimory_cxt_seed_l2v(w);
	if (ret)
		return ret;
	w->cxt_base_weave = w->cxt_ext_weave;
	/*
	 * The weave fast-forward, from s_cxt.c:81.
	 *
	 * Having loaded a checkpoint, stock sets the write cursor to
	 *
	 *	baseWeaveSeq + save.num_sb * s_g_vbas_per_sb + 1
	 *
	 * so that every subsequent write carries a weave at or above it. The
	 * weaves in between belong to the checkpoint's own pages. This
	 * driver has no write path and never needed the cursor, but it does
	 * need the boundary: the diff skip below has to separate "predates
	 * the checkpoint" from "written after it", and the base alone puts
	 * the checkpoint's own write span on the wrong side of that line.
	 *
	 * num_sb comes from the BASE payload when it parsed, and otherwise
	 * from the number of checkpoint superblocks classify found, which is
	 * the same number on a healthy volume.
	 */
	{
		u32 nsb = w->cxt_save_num_sb ? w->cxt_save_num_sb :
			  (w->n_cxt ? w->n_cxt : 1);

		w->cxt_top_weave = w->cxt_base_weave +
				   (u64)nsb * whimory_vbas_per_vblock(w) + 1;
		dev_info(w->dev,
			 "CXT_WEAVE base=%llu top=%llu (num_sb=%u src=%s)\n",
			 (unsigned long long)w->cxt_base_weave,
			 (unsigned long long)w->cxt_top_weave, nsb,
			 w->cxt_save_num_sb ? "BASE" : "classify");
	}
	w->sftl.cxt_loaded = true;
	/*
	 * The interval map now holds everything the extent table did, and the
	 * table is ~12 MiB on a 55 MiB device. Drop it; the Phase 3 tools
	 * reallocate it on demand.
	 */
	kvfree(w->cxt_ext);
	w->cxt_ext = NULL;
	w->max_cxt_ext = 0;
	w->n_cxt_ext = 0;
	return 0;
}

/* Phase 3 entry point: build the candidate map, compare it, validate it. */
int whimory_cxt_candidate(u32 fat_base)
{
	struct whimory *w = whimory_dev;
	int ret;

	if (!w)
		return -ENODEV;
	ret = whimory_cxt_build_candidate(w);
	if (ret) {
		dev_warn(w->dev, "CXT candidate build failed %d\n", ret);
		return ret;
	}
	whimory_cxt_compare(w);
	whimory_cxt_probe(w, 24);
	if (fat_base)
		whimory_cxt_validate(w, fat_base);
	return 0;
}
EXPORT_SYMBOL_GPL(whimory_cxt_candidate);


static void whimory_note_meta0(struct whimory *w, unsigned int ce,
			       unsigned int cau, unsigned int block,
			       unsigned int page, const u8 *data, const u8 *meta)
{
	unsigned int slot;

	if (!data || !meta)
		return;
	for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;
		const u8 *d = data + slot * WHIMORY_LBA_SIZE;
		u32 lba = get_unaligned_le32(m + 8);
		u32 vba, vblock;
		u16 bps;

		if (lba != 0)
			continue;
		if (m[0] != WHIMORY_META_TYPE_DATA &&
		    m[0] != WHIMORY_META_TYPE_DATA2)
			continue;
		w->sftl.meta0_hits++;
		if (!ftl_diag || w->sftl.meta0_hits > 24)
			continue;
		vblock = whimory_vfl_virt(w, cau, block);
		vba = whimory_pack_vba(w, ce, cau, vblock, page, slot);
		bps = get_unaligned_le16(d + 11);
		dev_info(w->dev,
			 "META0_HIT vba=%u ce=%u cau=%u blk=%u page=%u slot=%u type=%02x first64=%32ph %32ph bps=%u\n",
			 vba, ce, cau, block, page, slot, m[0], d, d + 32, bps);
	}
}

static void whimory_dump_btoc_page(struct whimory *w, const struct whimory_sb *sb,
				   u32 vblock, const u8 *page, const u8 *meta)
{
	u32 be0 = get_unaligned_be32(page);
	u32 be1 = get_unaligned_be32(page + 4);
	u32 be2 = get_unaligned_be32(page + 8);
	bool lpn = whimory_btoc_looks_be_lpn(page);

	dev_info(w->dev,
		 "BTOC_DUMP sb_ce=%u cau=%u blk=%u vblock=%u page=%u first32=%32ph meta0=%16ph be=%u %u %u%s\n",
		 sb->ce, sb->cau, sb->block, vblock, WHIMORY_BTOC_PAGE,
		 page, meta, be0, be1, be2,
		 lpn ? " (BE LPN array)" : "");
}

static void whimory_print_recovery_stats(struct whimory *w)
{
	struct whimory_sftl *s = &w->sftl;

	dev_info(w->dev,
		 "RECOVERY_STATS:\n"
		 "  scan_blocks=%u (param) user_blocks=%u\n"
		 "  fpart_sig=%u vfl_ctx_hits=%u vfl_cxt_loc=%u vfl_bitmap=%u\n"
		 "  classified_empty=%u classified_closed=%u classified_open=%u classified_cxt=%u classified_unknown=%u\n"
		 "  sb_banks known=%u partial=%u overflow=%u\n"
		 "  cxt_blocks_seen=%u cxt_records_seen=%u cxt_l2v_updates=%u\n"
		 "  cxt_meta_confirmed=%u cxt_meta_mismatch=%u cxt_confirm_pages=%u cxt_confirm_unreadable=%u\n"
		 "  cxt_repair_pages=%u cxt_repair_slots=%u\n"
		 "  btoc_pages_read=%u btoc_pages_valid=%u btoc_entries_seen=%u btoc_l2v_updates=%u\n"
		 "  btoc_meta_confirmed=%u btoc_meta_mismatch=%u btoc_skipped_zero=%u\n"
		 "  btoc_blank=%u be_bte=%u be_lpn=%u le_bte=%u hdr8=%u unclaimed=%u\n"
		 "  btoc_fallback sbs=%u pages=%u hits=%u (%u pages/sb)\n"
		 "  unknown_fallback sbs=%u pages=%u hits=%u (%u pages/sb)\n"
		 "  btoc_confirm_pages=%u btoc_confirm_capped=%u btoc_confirm_budget_stop=%u\n"
		 "  btoc_unmap_entries=%u btoc_hole_entries=%u btoc_unknown_entries=%u\n"
		 "  btoc_token_ffff0000=%u btoc_token_ffffff00=%u btoc_token_ffffffff=%u btoc_holelist_ffff0001=%u\n"
		 "  open_slots_seen=%u open_slots_valid_meta=%u open_l2v_updates=%u\n"
		 "  open_unmap_entries=%u open_skipped_zero=%u open_overrides_closed=%u "
		 "open_rejected_stale=%u open_unknown_order=%u\n"
		 "  mapped_lbas=%u mapped_ranges=%u mapped_roots=%u range_budget_stop=%u\n"
		 "  string_hits itunesdb=%u f00=%u ipod_control=%u music=%u apps=%u mp3=%u m4a=%u\n"
		 "  l2v_update_calls=%u l2v_unmap_calls=%u stale_mapping_rejected=%u "
		 "l2v_repack_roots=%u meta0_hits=%u\n",
		 scan_blocks, s->user_blocks,
		 w->sig_ok, w->vfl.ctx_hits, w->vfl.cxt_loc_count,
		 w->vfl.bitmap_loaded,
		 s->empty_sbs, s->btoc_sbs, s->open_sbs, s->cxt_sbs,
		 s->unknown_sbs,
		 s->sb_bank_known, s->sb_bank_partial, s->sb_bank_overflow,
		 s->cxt_blocks_seen, s->cxt_records_seen, s->cxt_l2v_updates,
		 s->cxt_meta_confirmed, s->cxt_meta_mismatch,
		 s->cxt_confirm_pages, s->cxt_confirm_unreadable,
		 s->cxt_repair_pages, s->cxt_repair_slots,
		 s->btoc_pages_read, s->btoc_pages_valid, s->btoc_entries_seen,
		 s->btoc_l2v_updates,
		 s->btoc_meta_confirmed, s->btoc_meta_mismatch,
		 s->btoc_skipped_zero,
		 s->btoc_blank, s->btoc_be_bte, s->btoc_be_lpn,
		 s->btoc_le_bte, s->btoc_le_bte_hdr8, s->btoc_unclaimed,
		 s->btoc_fb_sbs, s->btoc_fb_pages, s->btoc_fb_hits,
		 s->btoc_fb_sbs ? s->btoc_fb_pages / s->btoc_fb_sbs : 0,
		 s->unk_fb_sbs, s->unk_fb_pages, s->unk_fb_hits,
		 s->unk_fb_sbs ? s->unk_fb_pages / s->unk_fb_sbs : 0,
		 s->btoc_confirm_pages, s->btoc_confirm_capped,
		 s->btoc_confirm_budget_stop,
		 s->btoc_unmap_entries, s->btoc_hole_entries,
		 s->btoc_unknown_entries,
		 s->btoc_token_ffff0000, s->btoc_token_ffffff00,
		 s->btoc_token_ffffffff, s->btoc_holelist_ffff0001,
		 s->open_slots_seen, s->open_slots_valid_meta,
		 s->open_l2v_updates,
		 s->open_unmap_entries, s->open_skipped_zero,
		 s->open_overrides_closed, s->open_rejected_stale,
		 s->open_unknown_order,
		 s->mapped_lbas, s->range_nodes, s->mapped_roots,
		 s->range_budget_stop,
		 s->string_hit_itunesdb, s->string_hit_f00,
		 s->string_hit_ipod_control, s->string_hit_music,
		 s->string_hit_apps, s->string_hit_mp3, s->string_hit_m4a,
		 s->l2v_update_calls, s->l2v_unmap_calls,
		 s->stale_mapping_rejected,
		 s->l2v_repack_roots, s->meta0_hits);
}

static void whimory_scan_closed_meta0(struct whimory *w, unsigned int nsb)
{
	unsigned int i, pg, scanned = 0, cap;
	u8 spare[S5L8740_NAND_META_SIZE];
	u8 *data = w->sftl.data_page;

	cap = meta0_scan_sbs;
	if (!cap || !data)
		return;
	dev_info(w->dev,
		 "META0_SCAN deeper closed SBs=%u pages 0..%u (independent of L2V)\n",
		 cap, WHIMORY_DATA_PAGES_PER_SB - 1);
	for (i = 0; i < nsb && scanned < cap; i++) {
		struct whimory_sb *sb = &w->sftl.sbs[i];

		if (sb->kind != WHIMORY_SB_CLOSED)
			continue;
		scanned++;
		for (pg = 0; pg < WHIMORY_DATA_PAGES_PER_SB; pg++) {
			int ret;

			ret = whimory_cs_read_page(w, sb->ce, sb->cau, sb->block,
						   pg, data,
						   S5L8740_NAND_PAGE_SIZE,
						   spare, sizeof(spare));
			if (ret)
				break;
			whimory_note_meta0(w, sb->ce, sb->cau, sb->block, pg,
					   data, spare);
			if (!(pg & 0x1f))
				cond_resched();
		}
	}
}

/*
 * Sibling slots on a page need not be contiguous LBAs. Only the selected
 * map_slot is judged against requested fmss_lba. Sibling dump is VBA_DIAG.
 */
static bool whimory_audit_fmss_lba(u32 fmss_lba)
{
	/* Known BPB / critical fmss candidates + FAT-relative later. */
	switch (fmss_lba) {
	case 49216u:
	case 49279u:
	case 49280u:
	case 49285u:
	case 49286u:
	case 49311u:
	case 49317u:	/* FAT0 disk 32 @ base 49285 */
	case 51201u:	/* root @ base 49285 */
		return true;
	default:
		if (fmss_lba >= 49279u && fmss_lba < 49279u + 2048u)
			return true;
		return false;
	}
}

static bool payload_string_scan;
module_param(payload_string_scan, bool, 0644);
MODULE_PARM_DESC(payload_string_scan,
		 "Scan confirmed pages for iTunesDB/F00/mp3 strings (default N)");

/*
 * Seven strnstr() sweeps over 16 KiB, on the hottest path in the recover.
 *
 * This runs from whimory_btoc_confirm_page(), which on this volume is
 * called 26653 times -- so it is 26653 passes over a 16 KiB buffer looking
 * for "iTunesDB", "F00", "iPod_Control", "Music" and friends, inside the
 * phase that already dominates the boot. It is pure diagnostics: nothing
 * reads the counters except the RECOVERY_STATS line, and on this volume
 * every one of them prints 0.
 *
 * payload_string_scan gates it and already defaults off, so this costs
 * nothing today -- noted here only so nobody enables it during a boot-time
 * measurement and wonders where the seconds went. It stays because it was
 * useful once, for confirming that a rebuilt map really did point at a
 * filesystem.
 */
static void whimory_note_payload_strings(struct whimory *w, const u8 *data,
					 unsigned int len)
{
	if (!payload_string_scan || !data || len < 8)
		return;
	if (memchr(data, 'i', len) &&
	    strnstr((const char *)data, "iTunesDB", len))
		w->sftl.string_hit_itunesdb++;
	if (strnstr((const char *)data, "F00", len) ||
	    strnstr((const char *)data, "F01", len) ||
	    strnstr((const char *)data, "F02", len))
		w->sftl.string_hit_f00++;
	if (strnstr((const char *)data, "iPod_Control", len))
		w->sftl.string_hit_ipod_control++;
	if (strnstr((const char *)data, "Music", len))
		w->sftl.string_hit_music++;
	if (strnstr((const char *)data, "NanoApps", len) ||
	    strnstr((const char *)data, "Apps", len))
		w->sftl.string_hit_apps++;
	if (strnstr((const char *)data, ".mp3", len) ||
	    strnstr((const char *)data, ".MP3", len) ||
	    strnstr((const char *)data, "mp3", len))
		w->sftl.string_hit_mp3++;
	if (strnstr((const char *)data, ".m4a", len) ||
	    strnstr((const char *)data, ".M4A", len) ||
	    strnstr((const char *)data, "m4a", len))
		w->sftl.string_hit_m4a++;
}

static void whimory_dump_vba_page(struct whimory *w, u32 vba, u32 fmss_lba)
{
	u32 ce, cau, vblock, page, map_slot, pblock, slot;
	u8 spare[S5L8740_NAND_META_SIZE];
	u8 *data = w->sftl.data_page;
	const u8 *sel_m;
	u32 sel_meta_lba;
	u64 sel_weave;
	bool sel_type_ok, sel_lba_ok, bad;
	int ret;

	if (!data)
		return;
	if (whimory_unpack_vba(w, vba, &ce, &cau, &vblock, &page, &map_slot)) {
		dev_warn(w->dev,
			 "BAD_VBA unpack failed fmss_lba=%u vba=%u\n",
			 fmss_lba, vba);
		return;
	}
	whimory_vfl_resolve(w, vblock, &cau, &pblock);
	ret = whimory_cs_read_page(w, ce, cau, pblock, page, data,
				   S5L8740_NAND_PAGE_SIZE, spare,
				   sizeof(spare));
	if (ret) {
		dev_warn(w->dev,
			 "BAD_VBA page read fmss_lba=%u vba=%u ret=%d\n",
			 fmss_lba, vba, ret);
		return;
	}

	sel_m = spare + map_slot * WHIMORY_META_SIZE;
	sel_meta_lba = get_unaligned_le32(sel_m + 8);
	sel_weave = whimory_weave48(sel_m);
	sel_type_ok = (sel_m[0] == WHIMORY_META_TYPE_DATA ||
		       sel_m[0] == WHIMORY_META_TYPE_DATA2) &&
		      !(sel_m[1] & 0x02);
	sel_lba_ok = (sel_meta_lba == fmss_lba);
	bad = !sel_type_ok || !sel_lba_ok;

	dev_info(w->dev,
		 "VBA_DIAG fmss_lba=%u vba=%u sb=%u ofs=%u "
		 "ppn=ce%u/cau%u/vblk%u/pbn%u/pg%u selected_slot=%u "
		 "selected_meta_lba=%u selected_weave=%012llx "
		 "type=%02x flags=%02x verdict=%s\n",
		 fmss_lba, vba, s_g_vba_to_sb(w, vba), s_g_vba_to_ofs(w, vba),
		 ce, cau, vblock, pblock, page, map_slot, sel_meta_lba,
		 (unsigned long long)sel_weave, sel_m[0], sel_m[1],
		 bad ? "BAD" : "OK");

	if (bad) {
		const u8 *d = data + map_slot * WHIMORY_LBA_SIZE;

		dev_warn(w->dev,
			 "BAD_VBA fmss_lba=%u selected_slot=%u type=%02x "
			 "flags=%02x meta_lba=%u (want %u) first64=%32ph\n",
			 fmss_lba, map_slot, sel_m[0], sel_m[1], sel_meta_lba,
			 fmss_lba, d);
	}

	if (!vba_page_dump)
		return;

	for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
		const u8 *m = spare + slot * WHIMORY_META_SIZE;
		const u8 *d = data + slot * WHIMORY_LBA_SIZE;
		u32 meta_lba = get_unaligned_le32(m + 8);
		u16 bps = get_unaligned_le16(d + 11);

		dev_info(w->dev,
			 "VBA_DIAG slot=%u%s type=%02x flags=%02x meta_lba=%u "
			 "bps=%u first64=%32ph meta=%16ph\n",
			 slot, slot == map_slot ? "*" : "",
			 m[0], m[1], meta_lba, bps, d, m);
	}
}


/*
 * Batched meta pre-pass for one plane, in stock's shape.
 *
 * RetailOS scans superblocks by queueing two meta reads each -- the first
 * page and the last -- and polling once per 256 records
 * (s_cxt_diff.c:349, S_SCAN_META_SIZE). Every read in the batch lands its
 * page data in the same discarded buffer while only the 16-byte meta cursor
 * advances, so a kick covers 128 superblocks and costs 4 KiB of meta and
 * one sector of data.
 *
 * This fills meta0[] and metaN[] for blocks [0, nscan) of one (ce, cau) so
 * the classify loop that follows does no I/O at all.
 *
 * Returns 0 with both arrays filled, or negative if the scan transport is
 * unavailable, in which case the caller reads per block as before.
 */
static int whimory_scan_plane_meta(struct whimory *w, unsigned int ce,
				   unsigned int cau, unsigned int nscan,
				   u8 *meta0, u8 *metaN)
{
	struct s5l8740_ppn_ref *refs;
	u8 *mbuf;
	unsigned int b, i, per_kick;
	unsigned int last_page = w->sftl.pages_per_sb ?
				 w->sftl.pages_per_sb - 1 : WHIMORY_BTOC_PAGE;
	int ret = 0;

	if (!stock_scan)
		return -ENODEV;

	/* Two records per superblock, so half the record cap in blocks. */
	per_kick = S5L8740_NAND_BATCH_MAX / 2;

	refs = kmalloc_array(per_kick * 2, sizeof(*refs), GFP_KERNEL);
	mbuf = kmalloc_array(per_kick * 2, WHIMORY_META_SIZE, GFP_KERNEL);
	if (!refs || !mbuf) {
		kfree(refs);
		kfree(mbuf);
		return -ENOMEM;
	}

	for (b = 0; b < nscan; b += per_kick) {
		unsigned int n = min(per_kick, nscan - b);

		for (i = 0; i < n; i++) {
			refs[2 * i].cau = (u8)cau;
			refs[2 * i].block = (u16)(b + i);
			refs[2 * i].page = 0;
			refs[2 * i + 1].cau = (u8)cau;
			refs[2 * i + 1].block = (u16)(b + i);
			refs[2 * i + 1].page = (u8)last_page;
		}
		ret = s5l8740_nand_cs_scan_meta((u8)ce, refs, n * 2, mbuf);
		if (ret)
			break;
		for (i = 0; i < n; i++) {
			memcpy(meta0 + (size_t)(b + i) * WHIMORY_META_SIZE,
			       mbuf + (size_t)(2 * i) * WHIMORY_META_SIZE,
			       WHIMORY_META_SIZE);
			memcpy(metaN + (size_t)(b + i) * WHIMORY_META_SIZE,
			       mbuf + (size_t)(2 * i + 1) * WHIMORY_META_SIZE,
			       WHIMORY_META_SIZE);
		}
		w->sftl.scan_kicks++;
		cond_resched();
	}
	kfree(refs);
	kfree(mbuf);
	return ret;
}

static int whimory_sftl_recover_l2v_from_media(struct whimory *w)
{
	struct whimory_sftl *s = &w->sftl;
	unsigned int ce, cau, b, nscan, nmap, nsb = 0, i, open_done = 0;
	u8 meta0[S5L8740_NAND_META_SIZE];
	u8 meta127[S5L8740_NAND_META_SIZE];
	u8 *plane_m0 = NULL;
	u8 *plane_mN = NULL;
	u8 *p127;
	u32 *meta0_hist;
	u32 *meta127_hist;
	int ret;

	nscan = scan_blocks ? scan_blocks : s->user_blocks;
	if (nscan > s->user_blocks)
		nscan = s->user_blocks;

	/*
	 * The bank map has to cover every block a VBA may name, which is
	 * blocks_per_cau -- 2088 -- and not the 1960 the classify loop
	 * enumerates. The FTL does put user data above user_blocks: there
	 * are checkpoint extents in virtual blocks 1987..1991 and 2048 on
	 * this unit, and vblock 2048 is a three-bank superblock. Left out of
	 * the map it decoded as four banks and read one page in twelve
	 * wrong, which was the last mismatch standing after the rest of this
	 * was fixed.
	 *
	 * It costs nothing: the batched pre-pass already sweeps a whole
	 * plane in one series of kicks, so 128 more blocks per plane is
	 * within the noise of a pass that takes about 1.2 seconds. The
	 * classify loop itself still stops at nscan -- the tail is the VFL
	 * and FPart region, and enumerating it as data superblocks would put
	 * blocks into the replay that have no business there.
	 */
	nmap = whimory_vba_blocks(w);
	if (nmap < nscan)
		nmap = nscan;

	p127 = s->btoc_page;
	if (!p127)
		return -ENOMEM;

	/*
	 * Histogram of the page 0 meta type byte over every block scanned.
	 * When a class comes out at zero -- cxt=0, say -- the useful question
	 * is whether the flash has no such block or whether the test for it
	 * stopped matching, and the classify counters cannot tell those apart.
	 * This can: WHIMORY_META_TYPE_SFTL_CXT is 0x1f, so a non-zero count at
	 * 0x1f with cxt=0 means the recogniser is wrong, and a zero count means
	 * the blocks really are not there.
	 */
	meta0_hist = kcalloc(256, sizeof(*meta0_hist), GFP_KERNEL);
	meta127_hist = kcalloc(256, sizeof(*meta127_hist), GFP_KERNEL);
	s->btoc_dumps_left = ftl_diag ? 5 : 0;
	w->l2v_defer_pack = true;
	s->btoc_verified = 0;
	s->diff_replayed_sbs = 0;
	s->diff_skipped_sbs = 0;
	/*
	 * Cleared per run: a re-recover reclassifies every block, and a mask
	 * carried over from a previous scan would keep banks that this one
	 * did not see.
	 */
	if (s->sb_bank_mask)
		memset(s->sb_bank_mask, 0, s->sb_bank_blocks);
	s->sb_bank_known = 0;
	s->sb_bank_partial = 0;
	s->sb_bank_overflow = 0;
	s->confirm_start_jiffies = jiffies;
	s->btoc_confirm_budget_stop = 0;
	s->string_hit_itunesdb = 0;
	s->string_hit_f00 = 0;
	s->string_hit_apps = 0;
	s->string_hit_mp3 = 0;
	s->string_hit_m4a = 0;

	dev_info(w->dev,
		 "SFTL classify scan ce=%u cau=%u blocks=%u "
		 "btoc_confirm_max=%u recover_budget_ms=%u audit_winners=%d\n",
		 w->geom.num_ce, w->geom.num_cau, nscan,
		 btoc_confirm_max, recover_budget_ms, audit_lba_winners);

	plane_m0 = kvmalloc_array(nmap, WHIMORY_META_SIZE, GFP_KERNEL);
	plane_mN = kvmalloc_array(nmap, WHIMORY_META_SIZE, GFP_KERNEL);
	if (!plane_m0 || !plane_mN) {
		kvfree(plane_m0);
		kvfree(plane_mN);
		plane_m0 = NULL;
		plane_mN = NULL;
	}

	for (ce = 0; ce < w->geom.num_ce; ce++) {
		for (cau = 0; cau < w->geom.num_cau; cau++) {
			bool have_scan = false;

			if (plane_m0 && plane_mN)
				have_scan = !whimory_scan_plane_meta(w, ce, cau,
								     nmap,
								     plane_m0,
								     plane_mN);
			/*
			 * Bank membership for the blocks past nscan, which
			 * classify does not enumerate. Nothing else is
			 * derived from them -- they do not enter sbs[] and
			 * are never replayed -- but a VBA naming one has to
			 * decode against the right bank count.
			 */
			if (have_scan) {
				for (b = nscan; b < nmap; b++) {
					const u8 *pm0 = plane_m0 +
						(size_t)b * WHIMORY_META_SIZE;
					const u8 *pmN = plane_mN +
						(size_t)b * WHIMORY_META_SIZE;

					if (whimory_meta_is_member(pm0) ||
					    whimory_meta_is_member(pmN))
						whimory_sb_bank_note(w,
							whimory_vfl_virt(w, cau, b),
							ce, cau);
				}
			}
			for (b = 0; b < nscan; b++) {
				struct whimory_sb *sb;
				int r0, r127;

				if (nsb >= s->num_sb)
					goto classify_done;
				if ((b & 0x1f) == 0 && ftl_progress_due(w))
					ftl_progress_set(w, "classify", b, nscan),
					dev_info(w->dev,
						 "SFTL classify ce=%u cau=%u blk=%u/%u nsb=%u\n",
						 ce, cau, b, nscan, nsb);
				if (recover_yield_us && (b & 0x3) == 0) {
					cond_resched();
					usleep_range(recover_yield_us,
						     recover_yield_us + 500);
				}
				/*
				 * Page 127 is read lazily. It used to be fetched for
				 * every block alongside page 0, but meta127 is only
				 * ever consulted for the closed/BTOC test below -- so
				 * on this device 5484 empty blocks and the 4 CXT
				 * blocks paid for a full page read whose result was
				 * discarded. That is roughly 70 percent of the page
				 * 127 traffic and a third of all classify IO, against
				 * a pass that takes about 57 seconds at 7.4 ms per
				 * block.
				 *
				 * A blank page 0 with an erased meta is taken as an
				 * empty block without confirming page 127. Blocks are
				 * written from page 0 upwards, so blank-at-0 with data
				 * at 127 does not occur in normal operation. If page 0
				 * cannot be read at all the old both-failed rule still
				 * applies and page 127 is consulted.
				 */
				/*
				 * One short read answers both questions.
				 *
				 * Empty is the common answer -- about seven
				 * blocks in ten on this volume -- and slot 0
				 * settles it. But the same 4112 bytes also
				 * carry the meta that classifies a non-empty
				 * block, so there is no reason to throw them
				 * away and read the page again: the block
				 * either finishes here or escalates to the
				 * full read, and nothing reads page 0 twice.
				 */
				if (have_scan) {
					/*
					 * Both metas already in hand from the
					 * batched pre-pass, so this block
					 * costs no I/O. An erased pair marks
					 * a free superblock, which is the
					 * only test stock applies here.
					 */
					const u8 *pm0 = plane_m0 +
						(size_t)b * WHIMORY_META_SIZE;
					const u8 *pmN = plane_mN +
						(size_t)b * WHIMORY_META_SIZE;

					memset(meta0, 0xff, sizeof(meta0));
					memcpy(meta0, pm0, WHIMORY_META_SIZE);
					memset(meta127, 0xff, sizeof(meta127));
					memcpy(meta127, pmN, WHIMORY_META_SIZE);
					r0 = 0;
					r127 = 0;
					if (meta0_hist)
						meta0_hist[meta0[0]]++;
					if (whimory_meta_erased(meta0,
							WHIMORY_META_SIZE) &&
					    whimory_meta_erased(meta127,
							WHIMORY_META_SIZE)) {
						s->empty_sbs++;
						s->fast_empty_hits++;
						continue;
					}
					goto have_meta;
				}

				r0 = -EAGAIN;
				if (fast_empty_probe) {
					const u8 *d0 = NULL;
					u8 m0[WHIMORY_META_SIZE];

					if (!whimory_prefetch_slot0(w, ce, cau,
								    b, nscan,
								    &d0, m0)) {
						if (whimory_page_blank(d0, 64) &&
						    whimory_meta_erased(m0,
							WHIMORY_META_SIZE)) {
							s->empty_sbs++;
							s->fast_empty_hits++;
							continue;
						}
						if (whimory_meta0_is_conclusive(m0)) {
							/*
							 * Slots 1..3 stay 0xff
							 * so an erased-meta
							 * test on them tells
							 * the truth rather
							 * than repeating the
							 * last page's bytes.
							 */
							memset(meta0, 0xff,
							       sizeof(meta0));
							memcpy(meta0, m0,
							       WHIMORY_META_SIZE);
							memcpy(w->sftl.data_page,
							       d0,
							       S5L8740_NAND_SLOT_DATA);
							memset(w->sftl.data_page +
							       S5L8740_NAND_SLOT_DATA,
							       0xff,
							       S5L8740_NAND_PAGE_SIZE -
							       S5L8740_NAND_SLOT_DATA);
							s->fast_slot0_hits++;
							r0 = 0;
						}
					}
				}

				r127 = -EAGAIN;		/* not read yet */

				if (r0)
					r0 = whimory_cs_read_page(w, ce, cau, b, 0,
							  w->sftl.data_page,
							  S5L8740_NAND_PAGE_SIZE,
							  meta0, sizeof(meta0));
				if (!r0) {
					whimory_note_meta0(w, ce, cau, b, 0,
							   w->sftl.data_page,
							   meta0);
					if (meta0_hist)
						meta0_hist[meta0[0]]++;
				}

				if (!btoc_page_lazy)
					r127 = whimory_cs_read_page(w, ce, cau, b,
								    WHIMORY_BTOC_PAGE,
								    p127,
								    S5L8740_NAND_PAGE_SIZE,
								    meta127,
								    sizeof(meta127));

				/*
				 * Only call a block empty when page 127 agrees, or
				 * when we deliberately did not look. Checking page 0
				 * alone is what the lazy path relies on.
				 */
				if (!r0 && whimory_page_blank(w->sftl.data_page, 64) &&
				    whimory_meta_erased(meta0, 16) &&
				    (btoc_page_lazy ||
				     (!r127 && whimory_page_blank(p127, 64) &&
				      whimory_meta_erased(meta127, 16)))) {
					s->empty_sbs++;
					continue;
				}

				/* CXT is decided from meta0 alone. */
				if (r127 == -EAGAIN &&
				    (r0 || !whimory_meta_slot0_or_any_cxt(meta0))) {
					r127 = whimory_cs_read_page(w, ce, cau, b,
								    WHIMORY_BTOC_PAGE,
								    p127,
								    S5L8740_NAND_PAGE_SIZE,
								    meta127,
								    sizeof(meta127));
					if (r0 && r127)
						continue;
					if (r0 &&
					    whimory_page_blank(p127, 64) &&
					    whimory_meta_erased(meta127, 16)) {
						s->empty_sbs++;
						continue;
					}
				}
have_meta:
				sb = &s->sbs[nsb];
				sb->ce = ce;
				sb->cau = cau;
				sb->block = b;
				/*
				 * Two weaves, because they answer different
				 * questions and only one of them is any use
				 * for the CXT diff.
				 *
				 * sb->weave is page 0: the OLDEST content in
				 * the superblock. A block is filled from page
				 * 0 upwards, so page 0 was written when the
				 * block was opened -- which may have been long
				 * before the checkpoint even if the block was
				 * still being appended to long after it.
				 *
				 * sb->weave_max is the newest weave we have
				 * actually seen. For a closed superblock that
				 * is page 127, written when the block was
				 * sealed, so it is the real answer. For an
				 * open one there is no page whose weave bounds
				 * the block, which is why the replay below
				 * refuses to skip open superblocks at all.
				 */
				sb->weave = 0;
				sb->weave_max = 0;
				sb->weave_max_p127 = 0;
				if (!r0 && (whimory_meta_is_data_raw(meta0) ||
					    meta0[0] == WHIMORY_META_TYPE_SFTL_CXT))
					sb->weave = whimory_weave48(meta0);
				if (!r127 && (whimory_meta_is_data_raw(meta127) ||
					      whimory_meta_any_btoc(meta127))) {
					sb->weave_max = whimory_weave48(meta127);
					sb->weave_max_p127 = 1;
				}
				if (sb->weave_max < sb->weave) {
					sb->weave_max = sb->weave;
					sb->weave_max_p127 = 0;
				}
				if (meta127_hist && !r127)
					meta127_hist[meta127[0]]++;
				if (!r0 && whimory_meta_is_cxt_base(meta0, 0)) {
					u32 vblock = whimory_vfl_virt(w, cau, b);
					u32 sb_idx = whimory_sb_index(w, ce, cau,
								      vblock);

					sb->kind = WHIMORY_SB_CXT;
					s->cxt_sbs++;
					whimory_cxt_add_base(w, sb_idx, sb->weave);
				} else if (!r0 && whimory_meta_slot0_or_any_cxt(meta0)) {
					sb->kind = WHIMORY_SB_CXT;
					s->cxt_sbs++;
				} else if (!r127 && whimory_meta_any_btoc(meta127)) {
					sb->kind = WHIMORY_SB_CLOSED;
					s->btoc_sbs++;
				} else if ((!r0 && whimory_meta_is_data_raw(meta0)) ||
					   (!r127 && whimory_meta_is_data_raw(meta127))) {
					sb->kind = WHIMORY_SB_OPEN;
					s->open_sbs++;
				} else {
					/*
					 * Nothing recognised this block, which
					 * is not the same as it being empty --
					 * the empty test ran earlier and said
					 * no. 78 blocks land here on this
					 * volume, meta type 00 on 76 of them
					 * and 0x4b on two, and until now they
					 * were counted and then dropped before
					 * ever entering sbs[]. A block that
					 * never enters sbs[] is never
					 * replayed, so up to 2032 LBAs each
					 * were unreachable with nothing in the
					 * log to say so.
					 *
					 * They are kept now and rebuilt from
					 * per-page meta in the replay, which
					 * is the one method that needs no
					 * recognised structure at all. Cost is
					 * bounded the same way as the BTOC
					 * fallback: a blank page 0 stops it
					 * after one read.
					 */
					sb->kind = WHIMORY_SB_UNKNOWN;
					s->unknown_sbs++;
				}
				/*
				 * Bank membership, which is what every VBA in
				 * this superblock is decoded against. Only a
				 * recognised superblock counts as a member:
				 * an unreadable or unrecognised block is
				 * precisely the case the FTL left out of the
				 * superblock, and including it would put the
				 * whole block's addresses one bank wide.
				 */
				if (sb->kind == WHIMORY_SB_CLOSED ||
				    sb->kind == WHIMORY_SB_OPEN ||
				    sb->kind == WHIMORY_SB_CXT)
					whimory_sb_bank_note(w,
						whimory_vfl_virt(w, cau, b),
						ce, cau);
				nsb++;
			}
		}
	}
classify_done:
	sort(s->sbs, nsb, sizeof(s->sbs[0]), whimory_sb_cmp, NULL);
	dev_info(w->dev,
		 "SFTL classified nsb=%u closed=%u open=%u cxt=%u empty=%u unknown=%u\n",
		 nsb, s->btoc_sbs, s->open_sbs, s->cxt_sbs, s->empty_sbs,
		 s->unknown_sbs);
	/*
	 * How wide the superblocks actually are.
	 *
	 * This is the number every address on the volume is decoded against,
	 * and until now it was assumed to be four everywhere. A histogram
	 * with anything outside the maximum column is a volume where that
	 * assumption was costing reads; a histogram that is entirely in the
	 * maximum column says the assumption happened to hold.
	 */
	{
		u32 maxb = whimory_max_banks(w);
		char hb[128];
		unsigned int t, hn = 0;

		memset(s->sb_bank_hist, 0, sizeof(s->sb_bank_hist));
		s->sb_bank_known = 0;
		s->sb_bank_partial = 0;
		for (i = 0; i < s->sb_bank_blocks; i++) {
			u32 n = hweight8(s->sb_bank_mask[i]);

			if (!n)
				continue;
			s->sb_bank_known++;
			if (n < maxb)
				s->sb_bank_partial++;
			if (n < ARRAY_SIZE(s->sb_bank_hist))
				s->sb_bank_hist[n]++;
		}
		for (t = 0; t < ARRAY_SIZE(s->sb_bank_hist); t++) {
			if (!s->sb_bank_hist[t] || hn + 14 >= sizeof(hb))
				continue;
			hn += scnprintf(hb + hn, sizeof(hb) - hn, "%u:%u ",
					t, s->sb_bank_hist[t]);
		}
		dev_info(w->dev,
			 "SFTL sb banks %s(known=%u partial=%u max=%u of %u vblocks)\n",
			 hb, s->sb_bank_known, s->sb_bank_partial, maxb,
			 s->sb_bank_blocks);
	}
	/*
	 * Check the derived bank map against the VFL's own bad-block table.
	 *
	 * The bank map is built from which banks carry a record at page 0,
	 * which is evidence but not authority. The authority for "this block
	 * is unusable" is the 0xc104 object -- and it is worth being precise
	 * about what that object is, because it was the obvious candidate
	 * for a per-VBN bank table and it is not one.
	 *
	 * Dumped on this unit: object_len 0x800 at +0x24, generation 29 at
	 * +0x28, payload at +0x80, and every payload row is 0xff except the
	 * first two. Those two rows hold exactly 29 clear bits, which is the
	 * 29 of "accepted: 29/2088 blocks bad" -- so it is one bit per
	 * block over blocks_per_cau, padded, and nothing more. All 29 bad
	 * blocks sit in blocks 0..31, while the narrow superblocks measured
	 * here are vblocks 320, 1259, 1818 and 2048. The two describe
	 * different things: a superblock can be narrower than four banks
	 * without any of its blocks being factory-bad.
	 *
	 * So the FTL's per-VBN bank table (s_vfl.c sub_3D1438, one stride-
	 * sized row per VBN) is not in this object and has not been located
	 * on media.
	 *
	 * What is left to check is the object's scope, which was an
	 * assumption and not a finding: the bitmap is taken to describe the
	 * blocks of the bank it was found on, blk_status_bank. If that is
	 * right, a block it calls bad must not have that bank as a member --
	 * the superblock can still exist on the other three. So count both:
	 * how many mapped vblocks it calls bad at all, and how many of those
	 * still list that one bank.
	 *
	 * A zero in the second column supports the scope reading. A non-zero
	 * one says the bitmap is not this bank's block list, and the next
	 * person should not assume it is.
	 */
	if (w->vfl.bitmap_loaded && w->vfl.blk_status && s->sb_bank_mask) {
		u32 v, flagged = 0, conflicts = 0, checked = 0;
		u32 bank = w->vfl.blk_status_bank;

		for (v = 0; v < s->sb_bank_blocks &&
			    v < w->geom.blocks_per_cau; v++) {
			bool bad = !(w->vfl.blk_status[v >> 3] &
				     (1u << (v & 7)));

			if (!s->sb_bank_mask[v])
				continue;
			checked++;
			if (!bad)
				continue;
			flagged++;
			if (bank < 8 && (s->sb_bank_mask[v] & (1u << bank)))
				conflicts++;
		}
		s->sb_bank_conflicts = conflicts;
		dev_info(w->dev,
			 "SFTL sb banks vs VFL bitmap: checked=%u flagged_bad=%u still_list_bank%u=%u (%u bad recorded)\n",
			 checked, flagged, bank, conflicts,
			 w->vfl.blk_status_bad);
		if (conflicts)
			dev_info(w->dev,
				 "SFTL the 0xc104 bitmap is not bank %u's block list -- do not read it as one\n",
				 bank);
	}
	dev_info(w->dev, "SFTL fast-empty probe settled %u of %u blocks\n",
		 s->fast_empty_hits, s->empty_sbs);
	dev_info(w->dev, "SFTL slot0 read settled %u non-empty blocks\n",
		 s->fast_slot0_hits);
	dev_info(w->dev,
		 "SFTL batch prefetch served %u page-0 reads in %u kicks (%u per kick)\n",
		 s->pf_hits, s->pf_kicks,
		 s->pf_kicks ? s->pf_hits / s->pf_kicks : 0);

	if (meta0_hist) {
		char hb[192];
		unsigned int t, hn = 0;

		for (t = 0; t < 256; t++) {
			if (!meta0_hist[t] || hn + 16 >= sizeof(hb))
				continue;
			hn += scnprintf(hb + hn, sizeof(hb) - hn, "%02x:%u ",
					t, meta0_hist[t]);
		}
		dev_info(w->dev,
			 "SFTL meta0 types %s(cxt=0x%02x btoc=0x%02x data=0x%02x)\n",
			 hb, WHIMORY_META_TYPE_SFTL_CXT, WHIMORY_META_TYPE_BTOC,
			 WHIMORY_META_TYPE_DATA);
		kfree(meta0_hist);
		meta0_hist = NULL;
	}

	/*
	 * What page 127 actually holds, and how the superblocks sit either
	 * side of the checkpoint.
	 *
	 * Both were guesses until now. open=1619 against closed=563 is a
	 * strange shape for a mostly-static volume and says either that a
	 * great many blocks really are mid-write, or that page 127 is not
	 * where this geometry keeps its BTOC -- the type histogram tells
	 * those apart. The weave split says how much of the volume the
	 * checkpoint genuinely covers, which is the number the diff replay
	 * is supposed to act on.
	 */
	if (meta127_hist) {
		char hb[192];
		unsigned int t, hn = 0;

		for (t = 0; t < 256; t++) {
			if (!meta127_hist[t] || hn + 16 >= sizeof(hb))
				continue;
			hn += scnprintf(hb + hn, sizeof(hb) - hn, "%02x:%u ",
					t, meta127_hist[t]);
		}
		dev_info(w->dev, "SFTL meta127 types %s(page %u)\n",
			 hb, WHIMORY_BTOC_PAGE);
		kfree(meta127_hist);
		meta127_hist = NULL;
	}

	kvfree(plane_m0);
	kvfree(plane_mN);

	whimory_cxt_index_build(w, nsb);
	/*
	 * The CXT is the FTL own checkpoint: it rebuilds the bulk of the map
	 * from a handful of pages instead of every open superblock. Replay
	 * below then adopts only what is newer than its base weave.
	 */
	ret = use_cxt ? whimory_cxt_fast_load(w) : -ENOENT;
	if (ret && use_cxt) {
		/*
		 * Do not quietly become the full scan.
		 *
		 * When the checkpoint does not load, s->cxt_loaded stays
		 * false, and every skip test in the replay below is guarded
		 * on it -- so nothing is skipped and all 7623 superblocks are
		 * replayed. That is the 500-second path, and it used to be
		 * announced with a dev_info in the middle of a boot log
		 * nobody reads at the time. The visible symptom was a device
		 * that "sometimes takes eight minutes to mount", with the one
		 * line explaining it eight thousand lines up.
		 *
		 * The fast path is the supported path. If it cannot run, say
		 * so at error level and stop, so the failure is a failure
		 * rather than a stall. require_cxt=0 restores the old
		 * behaviour for a deliberate comparison against the
		 * brute-force map, and use_cxt=0 still asks for that map
		 * outright -- neither is something a boot should reach by
		 * accident.
		 */
		if (require_cxt) {
			dev_err(w->dev,
				"CXT fast path failed (%d) and require_cxt is set: refusing to full-replay %u superblocks. Set require_cxt=0 (or use_cxt=0) to rebuild from media.\n",
				ret, nsb);
			whimory_set_status(w, "cxt load failed (%d)", ret);
			w->l2v_defer_pack = false;
			return ret;
		}
		dev_warn(w->dev,
			 "CXT seed failed %d; falling back to full replay of %u superblocks -- this is the slow path\n",
			 ret, nsb);
	}
	ret = 0;
	if (s->cxt_loaded)
		dev_info(w->dev, "s_cxt_load OK bases=%u weave=%llu mapped=%u\n",
			 w->n_cxt, w->cxt_base_weave, s->range_nodes);

	for (i = 0; i < nsb; i++) {
		struct whimory_sb *sb = &s->sbs[i];
		u32 vblock = whimory_vfl_virt(w, sb->cau, sb->block);

		/*
		 * Replay is the long pole once scan_blocks is widened. Give RNDIS
		 * and the watchdog air on every superblock, not just inside
		 * whimory_rebuild_open_sb().
		 */
		cond_resched();
		if (recover_yield_us && (i & 0x3) == 0)
			usleep_range(recover_yield_us, recover_yield_us + 500);

		if (sb->kind == WHIMORY_SB_CXT)
			continue;

		/*
		 * Skip what the checkpoint already covers -- but only when we
		 * can prove it does.
		 *
		 * This used to test sb->weave, which is page 0: the oldest
		 * page in the superblock. That is the wrong end. A block being
		 * appended to right up to the moment of the crash still has an
		 * old page 0, so every superblock on the volume tested older
		 * than the checkpoint and the diff replay skipped all 2182 of
		 * them -- sbs=0 replayed. The map then reflected the volume
		 * exactly as of the last checkpoint and nothing written after
		 * it, which is precisely the missing-recent-files symptom.
		 *
		 * The fix for that was to stop skipping open superblocks at
		 * all, on the reasoning that an open block is still being
		 * written and no page in it bounds the rest. On this hardware
		 * that reasoning does not hold, and the measurement is
		 * unambiguous:
		 *
		 *   SFTL meta127 types 00:76 01:1615 1c:563 ff:6
		 *
		 * 1615 of the 1619 "open" superblocks carry user data at page
		 * 127. They are full blocks that were never sealed with a
		 * BTOC, not blocks mid-write -- the rebuild proved it by
		 * reading 126 pages per superblock before finding a blank one.
		 * A full block has a real newest page, so page 127 bounds it
		 * exactly as it bounds a closed one.
		 *
		 * Leaving them unskipped was not merely slow. It read 205194
		 * pages in 411 seconds, had 98.6 percent of the slots rejected
		 * as stale, and overrode 5726 CXT mappings with older data --
		 * on a volume where not one superblock is newer than the
		 * checkpoint (weave newer=0 older=2182). A map that had been
		 * mounting stopped mounting.
		 *
		 * So the test is the bound, not the kind: skip only when the
		 * weave came from page 127. A superblock whose page 127 gave
		 * no usable weave has nothing but page 0 to offer, which is
		 * the wrong end again, and is replayed.
		 */
		if (!sb->weave_max)
			s->weave_none++;
		else if (sb->weave_max >= w->cxt_top_weave)
			s->weave_newer++;
		else
			s->weave_older++;

		if (force_replay_vblock && vblock == force_replay_vblock) {
			dev_info(w->dev,
				 "SFTL forcing vblk=%u through replay (kind=%u weave=%llu max=%llu p127=%u base=%llu)\n",
				 vblock, sb->kind,
				 (unsigned long long)sb->weave,
				 (unsigned long long)sb->weave_max,
				 sb->weave_max_p127,
				 (unsigned long long)w->cxt_base_weave);
		} else if (use_cxt && s->cxt_loaded && cxt_fast &&
			   sb->weave_max &&
			   sb->weave_max < w->cxt_top_weave) {
			/*
			 * Skip only what the checkpoint provably covers.
			 *
			 * The line is cxt_top_weave, not the base. Weaves
			 * between the two were consumed writing the
			 * checkpoint itself, so a superblock bounded there is
			 * still one the checkpoint describes; stock reserves
			 * exactly that span before letting any user write
			 * take a weave (s_cxt.c:81). Testing against the base
			 * put those superblocks on the post-checkpoint side
			 * and replayed them for nothing.
			 *
			 * A superblock with no usable weave at all is in
			 * neither set: nothing bounds it, so there is no
			 * evidence the checkpoint describes it. Those are
			 * replayed. On this volume that is the 78 blocks
			 * classify reports as unknown, which is cheap against
			 * the 7541 it does skip.
			 */
			s->diff_skipped_sbs++;
			continue;
		} else if (use_cxt && s->cxt_loaded && sb->weave_max_p127 &&
			   sb->weave_max && sb->weave_max < w->cxt_top_weave) {
			s->diff_skipped_sbs++;
			continue;
		}
		if (use_cxt && s->cxt_loaded && !sb->weave_max_p127 &&
		    sb->weave && sb->weave < w->cxt_top_weave)
			s->diff_open_kept++;
		s->diff_replayed_sbs++;
		if (sb->kind == WHIMORY_SB_CLOSED) {
			int ingested;

			ret = whimory_cs_read_page(w, sb->ce, sb->cau, sb->block,
						   WHIMORY_BTOC_PAGE,
						   s->btoc_page,
						   S5L8740_NAND_PAGE_SIZE,
						   meta127, sizeof(meta127));
			if (ret)
				continue;
			s->btoc_pages_read++;
			if (s->btoc_verified < btoc_verify_sbs) {
				s->btoc_verified++;
				whimory_btoc_verify(w, sb, vblock, s->btoc_page,
						    S5L8740_NAND_PAGE_SIZE);
			}
			if (s->btoc_dumps_left &&
			    (s->btoc_pages_read <= 2 ||
			     whimory_btoc_looks_be_lpn(s->btoc_page))) {
				whimory_dump_btoc_page(w, sb, vblock,
						       s->btoc_page, meta127);
				s->btoc_dumps_left--;
			}
			s->claim_weave = sb->weave;
			s->claim_source = 1;
			ingested = whimory_ingest_btoc_page(w, sb->ce, sb->cau,
							    vblock, s->btoc_page,
							    S5L8740_NAND_PAGE_SIZE);
			s->claim_weave = 0;
			s->claim_source = 0;
			if (ingested) {
				s->btoc_pages_valid++;
			} else {
				/*
				 * No BTE array here -- rebuild the superblock
				 * from per-page meta instead of writing it off.
				 *
				 * 70 of these remain after the header-8 parse
				 * and they share one shape: header, aux=0x7fc,
				 * then zeros where the first record would be.
				 * The decomp of the writer (s_btoc sub_567E3C)
				 * says that cannot be a record. It writes
				 * v11[2]=lba and v11[3]=span with the caller
				 * asserting span != 0, and it keeps
				 * nextVbaOfs += span consistent with
				 * s_g_addr_to_vba(sb, nextVbaOfs) afterwards --
				 * a zero span would break that invariant. aux
				 * is not a length either; at the call site it
				 * is a per-write tag lifted from the stream
				 * context, so 0x7fc there is not the array
				 * declaring its own size. And 0x7fc is 2044,
				 * which is a superblock's 2048 VBAs less one
				 * 4-slot group.
				 *
				 * So these read as sealed-block trailers: a
				 * fill count written when the block closed,
				 * terminator after it, no BTE array at all.
				 * That is not the same as an empty superblock,
				 * and the difference is about 2032 LBAs each.
				 *
				 * Rather than decide which, measure. The
				 * rebuild costs one read against an empty
				 * block -- page 0 comes back blank and it
				 * stops -- and about 127 against a full one.
				 * The counters below say which happened.
				 */
				int fb;

				s->btoc_fb_sbs++;
				if (!btoc_meta_fallback)
					goto btoc_done;
				fb = s->open_pages_read;
				ret = whimory_rebuild_open_sb(w, sb);
				s->btoc_fb_pages += s->open_pages_read - fb;
				if (ret > 0)
					s->btoc_fb_hits += ret;
				else if (ret < 0)
					return ret;
btoc_done:
				;
			}
			if (ftl_progress_due(w))
				ftl_progress_set(w, "replay", i, nsb),
				dev_info(w->dev,
					 "SFTL replay progress i=%u/%u closed_valid=%u "
					 "open_updates=%u unmap_calls=%u stale_rej=%u "
					 "mapped=%u ranges=%u confirm=%u budget_stop=%u\n",
					 i, nsb, s->btoc_pages_valid,
					 s->open_l2v_updates, s->l2v_unmap_calls,
					 s->stale_mapping_rejected, s->mapped_lbas,
					 s->range_nodes, s->btoc_confirm_pages,
					 s->range_budget_stop);
		} else if (sb->kind == WHIMORY_SB_UNKNOWN) {
			/* Same rebuild, counted apart so the two unknowns --
			 * BTOCs with no array, and blocks with no recognised
			 * meta at all -- stay distinguishable.
			 */
			int fb = s->open_pages_read;

			s->unk_fb_sbs++;
			ret = whimory_rebuild_open_sb(w, sb);
			s->unk_fb_pages += s->open_pages_read - fb;
			if (ret > 0)
				s->unk_fb_hits += ret;
			else if (ret < 0)
				return ret;
		} else if (sb->kind == WHIMORY_SB_OPEN) {
			if (max_open_sbs && open_done >= max_open_sbs) {
				/* Counted, not silent: a cap that drops open
				 * superblocks drops recent writes, and a map
				 * that is quietly short is worse than a slow
				 * one.
				 */
				s->open_truncated++;
				continue;
			}
			ret = whimory_rebuild_open_sb(w, sb);
			if (ret > 0)
				open_done++;
			else if (ret < 0)
				return ret;
			if (ftl_progress_due(w))
				ftl_progress_set(w, "open", open_done, s->open_sbs),
				dev_info(w->dev,
					 "SFTL open progress done=%u/%u i=%u/%u "
					 "open_updates=%u ranges=%u mapped=%u\n",
					 open_done, s->open_sbs, i, nsb,
					 s->open_l2v_updates, s->range_nodes,
					 s->mapped_lbas);
		}
	}

	dev_info(w->dev,
		 "SFTL diff replay sbs=%u skipped_by_cxt=%u cxt_seeded=%u "
		 "open_kept=%u open_truncated=%u\n",
		 s->diff_replayed_sbs, s->diff_skipped_sbs, s->cxt_l2v_updates,
		 s->diff_open_kept, s->open_truncated);
	dev_info(w->dev,
		 "SFTL weave vs cxt base=%llu top=%llu: newer=%u older=%u none=%u\n",
		 w->cxt_base_weave, w->cxt_top_weave, s->weave_newer,
		 s->weave_older, s->weave_none);
	/*
	 * How deep the open rebuilds went. A genuinely open superblock stops
	 * at its first blank page, so pages/sb well under 127 says these
	 * really are partly written; pages/sb at 127 says they are full
	 * blocks that classify called open because page 127 held data rather
	 * than a BTOC, and the fix belongs in the classification instead.
	 */
	if (open_done)
		dev_info(w->dev,
			 "SFTL open rebuild sbs=%u pages=%u (%u pages/sb)\n",
			 open_done, s->open_pages_read,
			 s->open_pages_read / open_done);
	if (s->open_truncated)
		dev_warn(w->dev,
			 "SFTL %u open superblocks dropped by max_open_sbs=%u -- "
			 "recent writes in them are NOT in the map\n",
			 s->open_truncated, max_open_sbs);
	w->l2v_defer_pack = false;
	ret = whimory_l2v_build_from_ranges(w);
	if (ret) {
		/* The interval map is the lookup authority, so a failed pack
		 * is survivable as long as it holds something.
		 */
		if (!s->range_nodes)
			return ret;
		dev_warn(w->dev,
			 "L2V pack %d; using interval map (%u ranges)\n",
			 ret, s->range_nodes);
		ret = 0;
	} else {
		s->packed_ok = true;
	}
	w->l2v_ok = true;
	/*
	 * A map that hit the node ceiling is short, and short is not the same
	 * as complete. Nothing used to say so: range_budget_stop was counted
	 * and printed among forty other statistics, so a truncated map and a
	 * whole one produced the same "recover OK". The reads that follow a
	 * truncation are unmapped LBAs scattered across the volume, which
	 * reads as failing flash rather than as a driver that stopped writing
	 * down where things are.
	 */
	if (w->sftl.range_budget_stop)
		dev_err(w->dev,
			"SFTL MAP IS SHORT: %u claims dropped at max_range_nodes=%u "
			"(ranges=%u). Reads in the dropped regions will return "
			"unmapped. Raise max_range_nodes and re-run recover.\n",
			w->sftl.range_budget_stop, max_range_nodes,
			w->sftl.range_nodes);
	whimory_scan_closed_meta0(w, nsb);
	whimory_print_recovery_stats(w);
	return 0;
}

static int whimory_sftl_alloc(struct whimory *w)
{
	struct whimory_sftl *s = &w->sftl;
	u32 nsb;

	s->vbas_per_page = WHIMORY_VBAS_PER_PAGE;
	s->pages_per_sb = WHIMORY_PAGES_PER_SB;
	s->vbas_per_sb = WHIMORY_VBAS_PER_SB;
	s->user_blocks = w->geom.user_blocks;
	nsb = w->geom.num_ce * w->geom.num_cau * s->user_blocks;
	if (w->vfl_ops && w->vfl_ops->get_param) {
		u32 p = w->vfl_ops->get_param(w, WHIMORY_VFL_PARAM_NUM_SB);

		if (p)
			nsb = p;
	}
	s->num_sb = nsb;
	/*
	 * Size the L2V VBA field from the space whimory_pack_vba() actually
	 * builds, which since the move to the native layout is
	 *
	 *   blocks_per_cau x (pages_per_sb x planes x vbas_per_page)
	 *
	 * These factors were num_sb x vbas_per_sb -- 7840 x 512 -- which is
	 * the retired bank-major space. That gave bits_vba = 22 and an
	 * invalid_vba sentinel of 4194303, against a native maximum VBA of
	 * 2088 x 2048 - 1 = 4276223. Every VBA in virtual blocks 2048..2087
	 * therefore compared >= the sentinel and was read as "unmapped" in
	 * three places at once: the CXT parser scored it a hole, range_update
	 * treated it as a true unmap and erased, and read_lba returned
	 * -ENOENT.
	 *
	 * Nothing lands there today -- user data stops at user_blocks = 1960,
	 * so real VBAs top out around 4014079 -- but it is a three percent
	 * margin held by luck rather than by arithmetic. The tell is that the
	 * correct width, 23 bits, gives sentinel 0x7FFFFF, which is exactly
	 * WHIMORY_CXT_VBA_HOLE. The two were meant to agree.
	 *
	 * vbas_per_sb stays as the per-block VBA count that the BTOC parsers
	 * and the GC zone use; it is simply not the right factor for this.
	 */
	{
		u32 planes = w->geom.num_ce * w->geom.num_cau;

		s->vba_factor_a = w->geom.blocks_per_cau ?
				  w->geom.blocks_per_cau : s->user_blocks;
		s->vba_factor_b = s->pages_per_sb * (planes ? planes : 1) *
				  s->vbas_per_page;
	}
	s->nodepool_bytes = WHIMORY_MIN_NODEPOOL_BYTES;

	s->btoc_page = kvmalloc(S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	/*
	 * Cross-call NAND page cache; see the fields in struct whimory_sftl.
	 * Not fatal if it fails -- n31_vfl_read_vba() falls back to reading
	 * the page every time, which is exactly the old behaviour.
	 */
	s->page_cache = kvmalloc(S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	s->page_cache_valid = false;
	/*
	 * 64 KiB + 256 B for the classify prefetch window. Optional in the
	 * same sense as the batch buffers in the NAND driver: without it
	 * whimory_prefetch_slot0() falls through to the single-page read.
	 */
	/*
	 * Legacy per-block prefetch, used only when the batched plane scan
	 * is unavailable. Sized independently of S5L8740_NAND_BATCH_MAX,
	 * which the meta-only scan raised to 256 records -- at a full 4 KiB
	 * slot per entry that would be a megabyte for a fallback path.
	 */
	s->pf_data = kvmalloc((size_t)WHIMORY_PF_SLOTS *
			      S5L8740_NAND_SLOT_DATA, GFP_KERNEL);
	s->pf_meta = kvmalloc((size_t)WHIMORY_PF_SLOTS *
			      S5L8740_NAND_BATCH_META_SIZE, GFP_KERNEL);
	if (!s->pf_data || !s->pf_meta) {
		kvfree(s->pf_data);
		kvfree(s->pf_meta);
		s->pf_data = NULL;
		s->pf_meta = NULL;
	}
	s->pf_valid = false;
	s->pf_failed = false;
	s->pf_checked = false;
	/*
	 * 128 KiB + 512 B for the read-ahead window. Optional in the same
	 * sense as everything else on this path: without it
	 * whimory_window_read() returns -ENODEV and the single-page cache
	 * below answers, exactly as it did before.
	 */
	s->rc_data = kvmalloc((size_t)WHIMORY_RC_SLOTS *
			      S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	s->rc_meta = kvmalloc((size_t)WHIMORY_RC_SLOTS *
			      S5L8740_NAND_META_SIZE, GFP_KERNEL);
	if (!s->rc_data || !s->rc_meta) {
		kvfree(s->rc_data);
		kvfree(s->rc_meta);
		s->rc_data = NULL;
		s->rc_meta = NULL;
	}
	s->rc_stage = kvmalloc((size_t)S5L8740_NAND_PAGE_BATCH_MAX *
			       S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	s->rc_stage_meta = kvmalloc((size_t)S5L8740_NAND_PAGE_BATCH_MAX *
				    S5L8740_NAND_META_SIZE, GFP_KERNEL);
	if (!s->rc_stage || !s->rc_stage_meta) {
		kvfree(s->rc_stage);
		kvfree(s->rc_stage_meta);
		s->rc_stage = NULL;
		s->rc_stage_meta = NULL;
	}
	memset(s->rc_key, 0, sizeof(s->rc_key));
	s->rc_count = 0;
	s->rc_fills = 0;
	s->rc_hits = 0;
	s->rc_misses = 0;
	s->rc_fails = 0;
	s->pf_count = 0;
	s->pf_kicks = 0;
	s->pf_hits = 0;
	s->data_page = kvmalloc(S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	s->meta_page = kvmalloc(WHIMORY_META_SIZE * WHIMORY_VBAS_PER_PAGE *
				(WHIMORY_DATA_PAGES_PER_SB + 1), GFP_KERNEL);
	s->cs_page = kvmalloc(sizeof(*s->cs_page), GFP_KERNEL);
	s->btoc_map = kvmalloc_array(WHIMORY_DATA_VBAS_PER_SB,
				     sizeof(*s->btoc_map), GFP_KERNEL);
	s->sbs = kvcalloc(nsb, sizeof(*s->sbs), GFP_KERNEL);
	/*
	 * Sized by blocks_per_cau, not user_blocks.
	 *
	 * A VBA may name any block the VFL can address, and CXT records
	 * legitimately do -- there are extents in virtual blocks 1987..1991
	 * on this unit, above user_blocks = 1960. Classify only reaches
	 * user_blocks, so those entries stay zero and decode as all banks,
	 * which is what this driver did everywhere before the map existed.
	 */
	s->sb_bank_blocks = whimory_vba_blocks(w);
	s->sb_bank_mask = kvcalloc(s->sb_bank_blocks,
				   sizeof(*s->sb_bank_mask), GFP_KERNEL);
	if (!s->btoc_page || !s->data_page || !s->meta_page || !s->cs_page ||
	    !s->sbs || !s->btoc_map || !s->sb_bank_mask)
		return -ENOMEM;

	/*
	 * Pages needed to hold one block table of contents:
	 *
	 *   ceil(16 * vbas_per_sb / page_bytes) + 1
	 *
	 * 16 bytes per entry x 512 VBAs fits in a single 16 KiB NAND page,
	 * and the stock firmware reserves one more, giving 2.
	 */
	{
		u32 page_bytes = w->geom.page_size ?
				 w->geom.page_size : S5L8740_NAND_PAGE_SIZE;
		u32 i;

		s->max_pages_per_btoc =
			(page_bytes + 16 * s->vbas_per_sb - 1) / page_bytes + 1;
		if (!s->max_pages_per_btoc)
			return -EINVAL;
		for (i = 0; i < WHIMORY_BTOC_OPEN; i++) {
			s->btoc_lba[i] = kvmalloc_array(s->vbas_per_sb,
							sizeof(u32),
							GFP_KERNEL);
			if (!s->btoc_lba[i])
				return -ENOMEM;
			memset(s->btoc_lba[i], 0xff,
			       s->vbas_per_sb * sizeof(u32));
		}
	}

	/*
	 * The garbage-collection zone size doubles until it reaches at
	 * least 16 VBAs and must stay a multiple of vbas_per_page, so 16 is
	 * both the minimum and what N31 uses. Context load reads this many
	 * VBAs at a time into gc_data / gc_meta.
	 */
	s->gc_zone_size = WHIMORY_GC_ZONE_MIN;
	if (s->gc_zone_size % s->vbas_per_page)
		return -EINVAL;
	s->gc_data = kvmalloc((size_t)WHIMORY_LBA_SIZE * s->gc_zone_size,
			      GFP_KERNEL);
	s->gc_meta = kvmalloc((size_t)WHIMORY_META_SIZE * s->gc_zone_size,
			      GFP_KERNEL);
	if (!s->gc_data || !s->gc_meta)
		return -ENOMEM;
	/*
	 *full-size FTL: num_superblocks * user VBAs per SB.
	 * BTOC page is not host LBA space (DATA_VBAS_PER_SB).
	 */
	{
		u64 cap = (u64)nsb * WHIMORY_DATA_VBAS_PER_SB;

		w->total_4k_sectors = cap ? cap : NAND_FTL_DEFAULT_CAPACITY;
	}
	return 0;
}

static int whimory_oracle_load(struct whimory *w)
{
	const struct firmware *gfw = NULL, *rfw = NULL, *nfw = NULL, *sfw = NULL;
	int ret;

	ret = request_firmware(&sfw, WHIMORY_ORACLE_SIG, w->dev);
	if (!ret && sfw && sfw->size >= WHIMORY_SIG_SIZE) {
		ret = whimory_parse_signature(w, sfw->data);
		if (ret)
			dev_err(w->dev, "oracle signature invalid: %d\n", ret);
	}
	if (sfw)
		release_firmware(sfw);

	ret = request_firmware(&gfw, WHIMORY_ORACLE_GLOBALS, w->dev);
	if (ret) {
		dev_err(w->dev, "oracle globals missing: %d\n", ret);
		return ret;
	}
	if (gfw->size < 28) {
		release_firmware(gfw);
		return -EINVAL;
	}
	{
		u32 num_roots = get_unaligned_le32(gfw->data + 0);
		u32 nodepool = get_unaligned_le32(gfw->data + 4);
		u32 max_lba;

		if (gfw->size >= 32)
			max_lba = get_unaligned_le32(gfw->data + 28);
		else
			max_lba = (u32)w->total_4k_sectors;
		if (!num_roots || !nodepool)
			ret = -EINVAL;
		else
			ret = whimory_l2v_init(w, max_lba ? max_lba :
					       (u32)w->total_4k_sectors,
					       w->sftl.vba_factor_a,
					       w->sftl.vba_factor_b,
					       nodepool);
		if (!ret && gfw->size >= 12) {
			w->l2v.bits_vba = gfw->data[8];
			w->l2v.spanbits_vba = gfw->data[9];
			w->l2v.bits_nodeidx = gfw->data[10];
			w->l2v.spanbits_nodeidx = gfw->data[11];
			if (gfw->size >= 16)
				w->l2v.invalid_vba =
					get_unaligned_le32(gfw->data + 12);
			w->l2v.sentinel_vba = w->l2v.invalid_vba;
			if (num_roots && num_roots != w->l2v.num_roots)
				dev_warn(w->dev,
					 "oracle num_roots=%u init=%u\n",
					 num_roots, w->l2v.num_roots);
		}
	}
	release_firmware(gfw);
	if (ret)
		return ret;

	ret = request_firmware(&rfw, WHIMORY_ORACLE_ROOT, w->dev);
	if (ret)
		return ret;
	if (rfw->size < WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots) {
		release_firmware(rfw);
		return -EINVAL;
	}
	memcpy(w->l2v.root, rfw->data,
	       WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots);
	release_firmware(rfw);

	ret = request_firmware(&nfw, WHIMORY_ORACLE_NODES, w->dev);
	if (ret)
		return ret;
	if (nfw->size < w->l2v.nodepool_bytes) {
		release_firmware(nfw);
		return -EINVAL;
	}
	memcpy(w->l2v.nodes, nfw->data, w->l2v.nodepool_bytes);
	release_firmware(nfw);

	w->oracle_used = true;
	w->l2v_ok = true;
	whimory_l2v_find_frag(w);
	dev_info(w->dev,
		 "L2V oracle loaded roots=%u nodes=0x%x frag=%u/%u\n",
		 w->l2v.num_roots, w->l2v.nodepool_bytes,
		 w->l2v.frag_count, w->l2v.frag_max);
	return 0;
}

static int n31_sftl_init(struct whimory *w)
{
	if (!w->vfl_ok)
		return -ENODEV;
	w->sftl.vbas_per_page = WHIMORY_VBAS_PER_PAGE;
	w->sftl.pages_per_sb = WHIMORY_PAGES_PER_SB;
	w->sftl.vbas_per_sb = WHIMORY_VBAS_PER_SB;
	w->sftl.user_blocks = w->geom.user_blocks;
	if (!w->sftl.user_blocks || !w->sftl.vbas_per_sb)
		return -EINVAL;
	return 0;
}

static int whimory_l2v_selftest(struct whimory *w)
{
	u32 vba = ~0u, span = 0;
	int fail = 0;

	if (whimory_l2v_update(w, 0, 1, 100) ||
	    whimory_l2v_search(w, 0, &vba, &span) || vba != 100)
		fail++;
	if (whimory_l2v_update(w, 1, 10, 101) ||
	    whimory_l2v_search(w, 5, &vba, &span) || vba != 105)
		fail++;
	if (whimory_l2v_update(w, 0x7fff, 4, 200) ||
	    whimory_l2v_search(w, 0x7fff, &vba, &span) || vba != 200)
		fail++;
	if (whimory_l2v_search(w, 0x8000, &vba, &span) || vba != 201)
		fail++;
	whimory_range_free(w);
	if (w->l2v.root && w->l2v.num_roots)
		memset(w->l2v.root, 0xff,
		       WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots);
	whimory_l2v_mem_reset(&w->l2v);
	w->sftl.l2v_update_calls = 0;
	w->sftl.l2v_unmap_calls = 0;
	w->sftl.l2v_repack_roots = 0;
	dev_info(w->dev, "L2V_SELFTEST %s\n", fail ? "FAIL" : "OK");
	return fail ? -EINVAL : 0;
}

static int n31_sftl_open(struct whimory *w)
{
	int ret;
	int sess;

	/*
	 * OSOS FTL_Open:BTOC (6 slots / 2 open LBA maps),
	 *GC zone,block tables,SB
	 * state, nodepool ≥ 0x80000,L2V_Init, then s_boot.
	 */
	ret = whimory_sftl_alloc(w);
	if (ret)
		return ret;

	if (import_l2v_oracle) {
		ret = whimory_oracle_load(w);
		if (ret) {
			dev_err(w->dev, "L2V oracle load failed: %d\n", ret);
			return ret;
		}
		return 0;
	}

	ret = whimory_l2v_init(w, (u32)w->total_4k_sectors,
			       w->sftl.vba_factor_a, w->sftl.vba_factor_b,
			       w->sftl.nodepool_bytes);
	if (ret)
		return ret;
	whimory_l2v_selftest(w);

	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		dev_warn(w->dev, "SFTL recover: DMA session %d\n", sess);
		/* Continue — cs_phys_read may still one-shot arm. */
	}
	ret = whimory_sftl_recover_l2v_from_media(w);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	if (ret)
		return ret;
	return 0;
}

static const struct whimory_ftl_ops n31_sftl_ops = {
	.major = 0,
	.minor = n31_sftl_minor,
	.init = n31_sftl_init,
	.open = n31_sftl_open,
	.read_lba = n31_sftl_read_lba,
};

static int whimory_select_ops(struct whimory *w)
{
	/*
	 * OSOS dispatches VFL/FTL by signature major through a table that
	 * is not named in the static dump. N31 media is PPN VFL + SFTL;
	 * those are the only ops this module implements. Log the majors
	 * from the signature (when present) and bind the N31 ops.
	 */
	w->vfl_ops = &n31_vfl_ops;
	w->ftl = &n31_sftl_ops;
	if (w->sig_ok) {
		dev_info(w->dev,
			 "VFL_SELECT major=%u minor=%u arg=%u\n",
			 w->sig.vfl_major, w->sig.vfl_minor,
			 w->sig.flags_or_open);
		dev_info(w->dev,
			 "FTL_SELECT major=%u minor=%u\n",
			 w->sig.ftl_major, w->sig.ftl_minor);
		dev_info(w->dev,
			 "ops bound fpart=%u.%u vfl=%u.%u ftl=%u.%u\n",
			 w->sig.fpart_major, w->sig.fpart_minor,
			 w->sig.vfl_major, w->sig.vfl_minor,
			 w->sig.ftl_major, w->sig.ftl_minor);
	} else {
		dev_warn(w->dev,
			 "VFL_SELECT/FTL_SELECT skipped: sig=0, binding N31 PPN+SFTL fallback\n");
	}
	return 0;
}

static int whimory_ftl_open(struct whimory *w)
{
	int ret;

	ret = w->ftl->init(w);
	if (ret) {
		dev_err(w->dev, "FTL_Init failed: %d\n", ret);
		return ret;
	}
	ret = w->ftl->open(w);
	if (ret) {
		dev_err(w->dev, "FTL_Open failed: %d\n", ret);
		return ret;
	}
	w->ftl_ok = true;
	dev_info(w->dev, "FTL_Open OK\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Read path */
/* ------------------------------------------------------------------ */

/*
 * Explain a bad read from the live map.
 *
 * The first attempt at this read the CXT extent table, which is freed after
 * seeding -- it is about 12 MiB on a 55 MiB device -- so it printed nothing
 * at all. The interval map holds the same information and is still there.
 *
 * The question is what kind of wrong the mapping is, and the deltas already
 * hint at it. One of them is exactly -16:
 *
 *   want=0x89af4 meta=0x89ae4   delta -16
 *
 * 16 is one page of VBAs -- planes * vbas_per_page -- which is the quantum
 * you get wrong if the two VBA spaces disagree about a superblock stride.
 * Replay builds VBAs with whimory_pack_vba over pages_per_sb = 128, while
 * CXT VBAs come from the FTL. If the FTL strides over the 127 data pages
 * instead, the two differ by one page per superblock and pack/unpack cancel
 * it everywhere except where a CXT mapping meets a replayed one.
 *
 * So: look the returned LBA up as well. delta_vba tells them apart.
 *
 *   delta_vba == delta_lba  -> the run is intact and sitting at the wrong
 *                              place; a placement error
 *   delta_vba == 0          -> two LBAs claim one VBA; the map is
 *                              double-mapped and one writer overwrote the
 *                              other
 *   otherwise               -> the run itself is malformed
 */
static void whimory_explain_bad_map(struct whimory *w, u32 lba, u32 meta_lba,
				    u32 vba, u32 span, const struct whimory_meta *m)
{
	unsigned int ce, cau, vblock, page, slot;
	char dw[64];
	u32 vba2 = 0, span2 = 0;
	int r2;

	if (w->sftl.bad_map_logged >= 8)
		return;
	w->sftl.bad_map_logged++;

	whimory_vba_describe(w, vba, dw, sizeof(dw));
	r2 = whimory_l2v_search(w, meta_lba, &vba2, &span2);

	if (r2) {
		dev_err(w->dev,
			"  lba=%u -> vba=%u span=%u (%s); meta lba=%u is NOT mapped (%d), delta_lba=%d\n",
			lba, vba, span, dw, meta_lba, r2,
			(int)meta_lba - (int)lba);
		return;
	}

	{
		struct whimory_range *r = whimory_range_find(&w->ranges, lba);

		dev_err(w->dev,
			"  lba=%u -> vba=%u span=%u (%s) src=%s weave=%llu\n",
			lba, vba, span, dw,
			!r ? "?" :
			r->src == 1 ? "BTOC" :
			r->src == 2 ? "open" :
			r->src == 3 ? "CXT" :
			r->src == 4 ? "LIST" : "seed",
			r ? (unsigned long long)r->weave : 0ULL);
	}
	whimory_vba_describe(w, vba2, dw, sizeof(dw));
	dev_err(w->dev,
		"  meta lba=%u -> vba=%u span=%u (%s) | delta_lba=%d delta_vba=%d%s\n",
		meta_lba, vba2, span2, dw,
		(int)meta_lba - (int)lba, (int)vba - (int)vba2,
		((int)vba - (int)vba2) == 0 ? " DOUBLE-MAPPED" :
		(((int)meta_lba - (int)lba) == ((int)vba - (int)vba2) ?
			" run intact, misplaced" : " run malformed"));

	if (!whimory_unpack_vba(w, vba, &ce, &cau, &vblock, &page, &slot))
		dev_err(w->dev,
			"  read from ce%u/cau%u/vblk%u/pg%u/slot%u\n",
			ce, cau, vblock, page, slot);

	/*
	 * The one number that decides whether the skip rule is at fault.
	 *
	 * The mapping came from the CXT and the CXT was parsed correctly, so
	 * the checkpoint is describing a page that now holds something else:
	 * the FTL moved data there after the checkpoint was taken. That is
	 * only possible if the block was not covered by the skip, and the
	 * skip uses page 127 as the block's upper bound.
	 *
	 * If this page's weave is newer than cxt_base_weave, page 127 does
	 * not bound this block -- the block was appended to after its BTOC
	 * was written -- and the diff replay skipped a superblock it should
	 * have replayed. If it is older, the checkpoint and the NAND
	 * genuinely disagree and the fault is elsewhere.
	 */
	if (m) {
		u64 pw = whimory_weave48((const u8 *)m);

		dev_err(w->dev,
			"  page weave=%llu vs cxt base=%llu -- %s\n",
			(unsigned long long)pw,
			(unsigned long long)w->cxt_base_weave,
			pw > w->cxt_base_weave ?
				"NEWER: page 127 does not bound this block, the skip was wrong" :
				"older: written before the checkpoint");
	}

	/*
	 * And what is actually at the VBA the map gave the returned LBA.
	 *
	 * The two extents in the failing case are exactly adjacent in VBA
	 * space -- 737713 + 440 = 738153 -- and both sit in vblock 360, so
	 * the runs are placed contiguously and only their LBA labels are in
	 * question. One read settles which way round they belong: if the page
	 * at vba2 holds the LBA we originally wanted, the two runs simply
	 * have each other's labels, and the fault is in how parse_tree pairs
	 * a run with its starting LBA rather than in the VBA arithmetic.
	 *
	 * It costs a NAND read on a path that has already failed.
	 */
	{
		struct whimory_meta m2;
		/*
		 * A buffer of its own.
		 *
		 * This used to hand w->sftl.data_page to read_vba as the
		 * destination -- and read_vba uses that same buffer as its
		 * page scratch, so the copy out of it was self-overlapping
		 * and the scratch came back holding a slice of itself. Only
		 * an explainer, but an explainer that corrupts the buffer the
		 * next real read will use is worse than no explainer.
		 */
		u8 *tmp = kmalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);

		if (tmp && w->vfl_ops && w->vfl_ops->read_vba &&
		    !w->vfl_ops->read_vba(w, vba2, 1, tmp, &m2))
			dev_err(w->dev,
				"  vba=%u actually holds lba=%u type=%02x%s\n",
				vba2, le32_to_cpu(m2.lba), m2.type,
				le32_to_cpu(m2.lba) == lba ?
					"  <-- the LBA we wanted: the two runs have swapped labels" : "");

		/*
		 * The same page on the other plane convention.
		 *
		 * A plane index packs a (ce, cau) pair, and with two of each
		 * there are two ways round: ce * num_cau + cau, which is what
		 * this driver uses, or cau * num_ce + ce. They agree on planes
		 * 0 and 3 and swap 1 and 2 -- and both of the failing reads
		 * land on plane 2 and plane 1 respectively, which is exactly
		 * the half a wrong convention would break.
		 *
		 * Reading the same vblock/page/slot with ce and cau exchanged
		 * settles it in one access: if that page holds the LBA we
		 * asked for, the convention is backwards.
		 */
		if (tmp && w->vfl_ops && w->vfl_ops->read_vba &&
		    !whimory_unpack_vba(w, vba, &ce, &cau, &vblock, &page,
					&slot) &&
		    w->geom.num_ce == w->geom.num_cau) {
			u32 swapped = whimory_pack_vba(w, cau, ce, vblock,
						       page, slot);

			if (swapped != vba &&
			    !w->vfl_ops->read_vba(w, swapped, 1, tmp, &m2))
				dev_err(w->dev,
					"  plane-swapped vba=%u (ce%u/cau%u) holds lba=%u type=%02x%s\n",
					swapped, cau, ce,
					le32_to_cpu(m2.lba), m2.type,
					le32_to_cpu(m2.lba) == lba ?
						"  <-- MATCH: the ce/cau plane order is reversed" : "");
		}
		kfree(tmp);
	}
}

static int whimory_validate_meta(struct whimory *w,
				 const struct whimory_meta *m,
				 u32 expected_lba)
{
	u32 meta_lba = le32_to_cpu(m->lba);

	if (!whimory_meta_is_user_data(m)) {
		dev_err(w->dev,
			"sftl non-data meta want=0x%x type=%02x flags=%02x lba=0x%x\n",
			expected_lba, m->type, m->flags, meta_lba);
		whimory_explain_bad_map(w, expected_lba, meta_lba,
					w->bad_vba, w->bad_span, m);
		return -EIO;
	}

	if (meta_lba != expected_lba) {
		dev_err(w->dev,
			"sftl lba mismatch want=0x%x meta=0x%x type=%02x flags=%02x\n",
			expected_lba, meta_lba, m->type, m->flags);
		whimory_explain_bad_map(w, expected_lba, meta_lba,
					w->bad_vba, w->bad_span, m);
		return -EIO;
	}

	if (m->flags & 0x02) {
		dev_err(w->dev,
			"sftl uECC flag lba=0x%x type=%02x flags=%02x\n",
			expected_lba, m->type, m->flags);
		return -EIO;
	}

	return 0;
}

static int n31_sftl_read_lba(struct whimory *w, u32 lba, void *buf,
			     bool allow_blank)
{
	struct whimory_meta meta;
	u32 vba = 0, span = 0;
	u32 ce, cau, vblock, page, slot;
	int ret;

	if (!w->l2v_ok)
		return -ENODEV;
	ret = whimory_l2v_search(w, lba, &vba, &span);
	if (ret) {
		if (allow_blank && ret == -ENOENT) {
			memset(buf, 0xff, WHIMORY_LBA_SIZE);
			return 0;
		}
		return ret;
	}
	if (lba == 0)
		w->lba0_vba = vba;
	if (vba >= w->l2v.invalid_vba) {
		if (!allow_blank)
			return -ENOENT;
		memset(buf, 0xff, WHIMORY_LBA_SIZE);
		return 0;
	}
	if (ftl_diag && (whimory_audit_fmss_lba(lba) || lba < 16)) {
		if (!whimory_unpack_vba(w, vba, &ce, &cau, &vblock, &page,
					&slot))
			dev_info(w->dev,
				 "L2V lookup fmss_lba=%u -> vba=%u span=%u "
				 "ppn=ce%u/cau%u/vblk%u/pg%u/slot%u\n",
				 lba, vba, span, ce, cau, vblock, page, slot);
		else
			dev_info(w->dev,
				 "L2V lookup fmss_lba=%u -> vba=%u span=%u\n",
				 lba, vba, span);
		/* Sibling VBA_DIAG only for BPB candidates (avoid spam). */
		if (lba == 49279u || lba == 49285u || lba == 49216u ||
		    lba < 4)
			whimory_dump_vba_page(w, vba, lba);
	}
	ret = w->vfl_ops->read_vba(w, vba, 1, buf, &meta);
	if (ret) {
		/*
		 * A transport failure used to leave here without a word, so a
		 * uECC or a disarmed CS looked exactly like a map hole from
		 * the filesystem's side -- both arrive at vfat as a bare EIO.
		 * They need entirely different fixes, so say which one it is.
		 */
		dev_err_ratelimited(w->dev,
				    "sftl read failed lba=%u vba=%u: %d\n",
				    lba, vba, ret);
		return ret;
	}
	/* What the map actually answered, for the explainer below. */
	w->bad_vba = vba;
	w->bad_span = span;
	ret = whimory_validate_meta(w, &meta, lba);
	if (!ret)
		dev_dbg(w->dev,
			"meta OK fmss_lba=%u vba=%u type=%02x meta_lba=%u\n",
			lba, vba, meta.type, le32_to_cpu(meta.lba));
	return ret;
}

/*
 * Both of these used to call n31_sftl_read_lba() directly, which walks the
 * range tree and reads through w->sftl.cs_page / data_page -- shared
 * scratch, under no lock and with CS unarmed. That was survivable only
 * because the hook is installed by whimory_register_disk(), which cannot
 * run while meta_dma_read=0. It is still reachable from
 * nand_ftl_read_sector(), and apple-grape symbol_get()s that.
 *
 * Route both through whimory_read_fmss_lba(), which takes tree_lock and
 * opens a DMA session, so the entry point does not decide whether the read
 * path is safe.
 */
static int whimory_read_lba_4k(struct whimory *w, u32 lba, void *buf)
{
	int ret = whimory_read_fmss_lba(lba, buf);

	/* The block layer wants a hole to read as erased, not to fail. */
	if (ret == -ENOENT) {
		memset(buf, 0xff, WHIMORY_LBA_SIZE);
		return 0;
	}
	return ret;
}

static int whimory_ftl_read_hook(u64 lba, void *buf)
{
	struct whimory *w = whimory_dev;

	if (!w || !w->l2v_ok)
		return -ENODEV;
	if (lba >= w->total_4k_sectors)
		return -ERANGE;
	return whimory_read_fmss_lba((u32)lba, buf);
}

static int whimory_check_lba0(struct whimory *w)
{
	u8 *buf;
	int ret;
	u16 bps;
	u32 total32, rootclus, serial;

	buf = kzalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = n31_sftl_read_lba(w, 0, buf, false);
	if (ret) {
		dev_err(w->dev, "LBA0 read failed: %d\n", ret);
		goto out;
	}

	dev_info(w->dev, "LBA0 first32=%32ph\n", buf);
	dev_info(w->dev, "LBA0 next32=%32ph\n", buf + 32);

	bps = get_unaligned_le16(buf + 11);
	total32 = get_unaligned_le32(buf + 32);
	rootclus = get_unaligned_le32(buf + 44);
	serial = get_unaligned_le32(buf + 67);
	dev_info(w->dev,
		 "BPB bytes_per_sector=%u sectors_per_cluster=%u total32=%u rootclus=%u serial=%08x label=%.11s\n",
		 bps, buf[13], total32, rootclus, serial, buf + 71);

	if (bps != 4096) {
		ret = -EINVAL;
		goto out;
	}
	if (buf[0] != 0xeb && buf[0] != 0xe9) {
		ret = -EINVAL;
		goto out;
	}
	dev_info(w->dev, "meta OK lba=0\n");
	w->lba0_ok = true;
	if (total32)
		w->total_4k_sectors = total32;
out:
	kfree(buf);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Block device */
/* ------------------------------------------------------------------ */

static void whimory_submit_bio_range(struct bio *bio, u64 start_4k,
				     u64 n_4k)
{
	struct whimory *w = whimory_dev;
	struct bvec_iter iter;
	struct bio_vec bvec;
	sector_t sector = bio->bi_iter.bi_sector;
	int ret = 0;

	if (!w || !w->lba0_ok) {
		bio_io_error(bio);
		return;
	}
	if (op_is_write(bio_op(bio))) {
		bio_io_error(bio);
		return;
	}

	bio_for_each_segment(bvec, bio, iter) {
		u8 *dst = kmap_local_page(bvec.bv_page) + bvec.bv_offset;
		unsigned int done_bytes = 0;

		while (done_bytes < bvec.bv_len) {
			u32 lba4k = (u32)(start_4k + (sector >> 3));
			unsigned int off = (sector & 7) * 512;
			unsigned int n = min_t(unsigned int,
					       bvec.bv_len - done_bytes,
					       4096 - off);

			if ((u64)lba4k >= start_4k + n_4k ||
			    lba4k >= w->total_4k_sectors) {
				ret = -EIO;
				kunmap_local(dst);
				goto done;
			}
			mutex_lock(&w->bounce_lock);
			ret = whimory_read_lba_4k(w, lba4k, w->bounce);
			if (!ret)
				memcpy(dst + done_bytes, w->bounce + off, n);
			mutex_unlock(&w->bounce_lock);
			if (ret) {
				kunmap_local(dst);
				goto done;
			}
			done_bytes += n;
			sector += n >> 9;
		}
		kunmap_local(dst);
	}
done:
	if (ret)
		bio_io_error(bio);
	else
		bio_endio(bio);
}

static void whimory_submit_bio(struct bio *bio)
{
	struct whimory *w = whimory_dev;

	whimory_submit_bio_range(bio, 0, w ? w->total_4k_sectors : 0);
}

static void whimory_ipod_submit_bio(struct bio *bio)
{
	struct whimory *w = whimory_dev;

	whimory_submit_bio_range(bio, 0, w ? w->total_4k_sectors : 0);
}

static const struct block_device_operations whimory_bd_ops = {
	.owner = THIS_MODULE,
	.submit_bio = whimory_submit_bio,
};

static const struct block_device_operations whimory_ipod_ops = {
	.owner = THIS_MODULE,
	.submit_bio = whimory_ipod_submit_bio,
};

static struct gendisk *whimory_alloc_disk(struct whimory *w, const char *name,
					  const struct block_device_operations *ops)
{
	struct queue_limits lim = {
		.logical_block_size = WHIMORY_LBA_SIZE,
		.physical_block_size = WHIMORY_LBA_SIZE,
	};
	struct gendisk *gd;
	int ret;

	gd = blk_alloc_disk(&lim, NUMA_NO_NODE);
	if (IS_ERR(gd))
		return gd;
	gd->first_minor = 0;
	gd->flags = GENHD_FL_NO_PART;
	gd->fops = ops;
	gd->private_data = w;
	snprintf(gd->disk_name, DISK_NAME_LEN, "%s", name);
	set_capacity(gd, w->total_4k_sectors * (WHIMORY_LBA_SIZE / 512));
	set_disk_ro(gd, 1);
	ret = add_disk(gd);
	if (ret) {
		put_disk(gd);
		return ERR_PTR(ret);
	}
	return gd;
}

static int whimory_register_disk(struct whimory *w)
{
	struct gendisk *gd;

	if (!w->sig_ok) {
		dev_warn(w->dev,
			 "sig=0: native PASS requires FPart 0xC101 xrmw signature\n");
		if (!allow_sigless_debug)
			return -ENODEV;
	}
	if (!w->vfl_ok || !w->ftl_ok || !w->l2v_ok || !w->lba0_ok)
		return -ENODEV;

	gd = whimory_alloc_disk(w, FTL_DISK_NAME, &whimory_bd_ops);
	if (IS_ERR(gd))
		return PTR_ERR(gd);
	w->disk = gd;
	gd = whimory_alloc_disk(w, FTL_IPOD_NAME, &whimory_ipod_ops);
	if (!IS_ERR(gd))
		w->ipod_disk = gd;
	s5l8740_nand_register_ftl_read(whimory_ftl_read_hook);
	dev_info(w->dev,
		 "/dev/%s registered read-only (%llu x %uB)\n",
		 FTL_DISK_NAME, w->total_4k_sectors, WHIMORY_LBA_SIZE);
	return 0;
}

static void whimory_unregister_disk(struct whimory *w)
{
	s5l8740_nand_register_ftl_read(NULL);
	if (w->ipod_disk) {
		del_gendisk(w->ipod_disk);
		put_disk(w->ipod_disk);
		w->ipod_disk = NULL;
	}
	if (w->disk) {
		del_gendisk(w->disk);
		put_disk(w->disk);
		w->disk = NULL;
	}
}

static ssize_t whimory_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct whimory *w = whimory_dev;
	int meta_ok = s5l8740_nand_meta_transport_ok();

	if (!w)
		return sysfs_emit(buf, "no device\n");
	return sysfs_emit(buf,
			  "fil=%d sig=%d vfl=%d ftl=%d l2v=%d lba0=%d oracle=%d\n"
			  "meta_transport=%s cs_dma_safe=%d pio_meta_trusted=0 "
			  "disk_gate=%s\n"
			  "mapped_roots=%u mapped_lbas=%u btoc_sbs=%u open_sbs=%u cxt_sbs=%u empty=%u recs=%u cxt_loaded=%d packed=%d\n"
			  "lba0_vba=%u cap=%llu vbas_per_sb=%u hole=%u list=%u\n"
			  "spare_applied=%u bitmap=%u frag=%u/%u gc_zone=%u btoc_pages=%u updates=%u gen=%u free=%u list_unmapped=%u\n"
			  "search_cache hits=%u misses=%u recovery=%s\n%s\n",
			  w->fil_ok, w->sig_ok, w->vfl_ok, w->ftl_ok,
			  w->l2v_ok, w->lba0_ok, w->oracle_used,
			  meta_ok ? "enabled" : "disabled",
			  meta_ok ? 1 : 0,
			  w->disk ? "registered" :
			  (meta_ok ? "blocked_open" : "blocked_cs_phys_only"),
			  w->sftl.mapped_roots, w->sftl.mapped_lbas,
			  w->sftl.btoc_sbs, w->sftl.open_sbs, w->sftl.cxt_sbs,
			  w->sftl.empty_sbs, w->sftl.btoc_recs,
			  w->sftl.cxt_loaded, w->sftl.packed_ok,
			  w->lba0_vba, w->total_4k_sectors, w->sftl.vbas_per_sb,
			  w->sftl.token_hole, w->sftl.token_list,
			  w->vfl.spare_applied, w->vfl.bitmap_loaded,
			  w->l2v.frag_count, w->l2v.frag_max,
			  w->sftl.gc_zone_size, w->sftl.max_pages_per_btoc,
			  w->l2v.updates, w->l2v.gen, w->l2v.free_count,
			  w->sftl.token_list_applied,
			  w->sftl.search_cache_hits,
			  w->sftl.search_cache_misses,
			  whimory_recovery_state_name(),
			  w->status);
}
static DEVICE_ATTR_RO(whimory_status);

/*
 * One field per line, no prose, so a shell loop or a UI can read this
 * without parsing the human log. percent is -1 rather than 0 while the
 * total is unknown, so a bar can show indeterminate instead of snapping
 * back to the left every time a phase begins.
 */
static ssize_t recover_progress_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct whimory *w = whimory_dev;
	int pct = -1;

	if (!w)
		return sysfs_emit(buf, "state=absent\n");
	if (w->prog_total)
		/* 32-bit: cur is a block/SB index, so *100 cannot overflow, and
		 * a u64 divide would need div_u64 on arm anyway. */
		pct = min_t(unsigned int, 100,
			    w->prog_cur / (w->prog_total / 100 + 1));
	return sysfs_emit(buf,
			  "state=%s\nphase=%s\ncur=%u\ntotal=%u\n"
			  "percent=%d\nmapped_lbas=%u\ndisk=%s\n",
			  whimory_recovery_state_name(),
			  w->prog_phase ? w->prog_phase : "idle",
			  w->prog_cur, w->prog_total, pct,
			  w->sftl.mapped_lbas,
			  w->disk ? "registered" : "none");
}
static DEVICE_ATTR_RO(recover_progress);

/*
 * The decoded identity, and an index of every record the scan located.
 *
 * The decode understands eleven tags; the section holds more than that.
 * Listing the candidates with their offsets and lengths makes the rest
 * reachable, and pairing this with syscfg_raw is enough to identify a
 * record without reflashing anything. "known" marks a candidate the
 * decode already consumes, so an unknown one is a genuine find rather
 * than a duplicate of a field printed above it.
 */
static ssize_t syscfg_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct whimory *w = whimory_dev;
	unsigned int i;
	int len = 0;

	if (!w)
		return sysfs_emit(buf, "state=absent\n");
	if (!w->syscfg.valid && !w->syscfg.raw_len)
		return sysfs_emit(buf, "state=unread\n");

	len += sysfs_emit_at(buf, len,
			     "state=%s\nsection_bytes=%u\nentries=%u\n"
			     "records=%u\n",
			     w->syscfg.valid ? "valid" : "partial",
			     w->syscfg.raw_len, w->syscfg.entries,
			     w->syscfg.n_cand);
	len += sysfs_emit_at(buf, len,
			     "serial=%s\nmodel=%s\nmlb=%s\nsw_ver=%s\n"
			     "cnt_b=%s\nmt_cl=%s\n",
			     w->syscfg.serial, w->syscfg.model, w->syscfg.mlb,
			     w->syscfg.sw_ver, w->syscfg.cnt_b,
			     w->syscfg.mt_cl);
	len += sysfs_emit_at(buf, len,
			     "region=0x%08x\nhw_ver=0x%08x\nfw_id=0x%08x\n",
			     w->syscfg.region, w->syscfg.hw_ver,
			     w->syscfg.fw_id);
	if (w->syscfg.mac_ok)
		len += sysfs_emit_at(buf, len, "bt_mac=%pM\n", w->syscfg.mac);
	len += sysfs_emit_at(buf, len,
			     "touch_cal_magic_off=%d\ntouch_cal_bytes=%u\n",
			     w->syscfg.touch_cal_magic_off,
			     w->syscfg.touch_cal_len);

	/*
	 * One record per line: index, tag, offset and length into
	 * syscfg_raw, and whether the decode above claims it.
	 */
	for (i = 0; i < w->syscfg.n_cand; i++)
		len += sysfs_emit_at(buf, len,
				     "record%u=%s off=0x%04x len=%u known=%u\n",
				     i, w->syscfg.cand[i].tag,
				     w->syscfg.cand[i].off,
				     w->syscfg.cand[i].len,
				     w->syscfg.cand[i].known ? 1 : 0);
	return len;
}
static DEVICE_ATTR_RO(syscfg);

/*
 * The SysCfg section verbatim, so a record this driver does not decode
 * can still be extracted. Offsets here are the ones syscfg reports.
 */
static ssize_t syscfg_raw_read(struct file *filp, struct kobject *kobj,
			       struct bin_attribute *attr, char *buf,
			       loff_t off, size_t count)
{
	struct whimory *w = whimory_dev;

	if (!w || !w->syscfg.raw || !w->syscfg.raw_len)
		return -ENODEV;
	if (off >= w->syscfg.raw_len)
		return 0;
	if (off + count > w->syscfg.raw_len)
		count = w->syscfg.raw_len - off;
	memcpy(buf, w->syscfg.raw + off, count);
	return count;
}
static BIN_ATTR_RO(syscfg_raw, S5L8740_NAND_PAGE_SIZE);

/*
 * The touch calibration container, at its full length. Reads short when
 * the section did not carry one, which is the case whenever
 * touch_cal_bytes above is 0 -- the source is then the U-Boot region.
 */
static ssize_t touch_cal_read(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	const u8 *d = NULL;
	size_t n = whimory_syscfg_touch_cal(&d);

	if (!n)
		return -ENODEV;
	if (off >= n)
		return 0;
	if (off + count > n)
		count = n - off;
	memcpy(buf, d + off, count);
	return count;
}
static BIN_ATTR_RO(touch_cal, N31_TOUCH_CAL_LEN);

static struct attribute *ftl_attrs[] = {
	&dev_attr_whimory_status.attr,
	&dev_attr_recover_progress.attr,
	&dev_attr_syscfg.attr,
	NULL,
};

static struct bin_attribute *ftl_bin_attrs[] = {
	&bin_attr_syscfg_raw,
	&bin_attr_touch_cal,
	NULL,
};
static const struct attribute_group ftl_attr_group = {
	.attrs = ftl_attrs,
	.bin_attrs = ftl_bin_attrs,
};

static void whimory_free(struct whimory *w)
{
	unsigned int cau;

	if (!w)
		return;
	whimory_unregister_disk(w);
	whimory_range_free(w);
	whimory_l2v_free(w);
	w->syscfg.raw_len = 0;
	w->syscfg.n_cand = 0;
	kvfree(w->syscfg.raw);
	w->syscfg.raw = NULL;
	kvfree(w->sftl.btoc_page);
	w->sftl.page_cache_valid = false;
	kvfree(w->sftl.page_cache);
	w->sftl.page_cache = NULL;
	w->sftl.pf_valid = false;
	kvfree(w->sftl.pf_data);
	kvfree(w->sftl.pf_meta);
	w->sftl.pf_data = NULL;
	w->sftl.pf_meta = NULL;
	w->sftl.rc_count = 0;
	kvfree(w->sftl.rc_data);
	kvfree(w->sftl.rc_meta);
	kvfree(w->sftl.rc_stage);
	kvfree(w->sftl.rc_stage_meta);
	w->sftl.rc_data = NULL;
	w->sftl.rc_meta = NULL;
	w->sftl.rc_stage = NULL;
	w->sftl.rc_stage_meta = NULL;
	kvfree(w->sftl.data_page);
	kvfree(w->sftl.meta_page);
	kvfree(w->sftl.cs_page);
	kvfree(w->sftl.btoc_map);
	kvfree(w->cxt_ext);
	kvfree(w->sftl.sbs);
	kvfree(w->sftl.sb_bank_mask);
	w->sftl.sb_bank_mask = NULL;
	w->sftl.sb_bank_blocks = 0;
	kvfree(w->sftl.gc_data);
	kvfree(w->sftl.gc_meta);
	kvfree(w->vfl.bank_mask);
	for (cau = 0; cau < WHIMORY_BTOC_OPEN; cau++)
		kvfree(w->sftl.btoc_lba[cau]);
	for (cau = 0; cau < S5L8740_NAND_MAX_CAU; cau++) {
		kvfree(w->vfl.remap[cau]);
		kvfree(w->vfl.cxt_u16[cau]);
	}
	kfree(w->bounce);
	kfree(w);
}

static int whimory_open_stack(struct whimory *w)
{
	int ret;

	ret = whimory_fil_init(w);
	if (ret) {
		whimory_set_status(w, "FIL_Init failed %d", ret);
		return ret;
	}

	/*
	 * Without CS metadata DMA, classic Whimory open cannot validate
	 * META via page_read. Recover is still available via CS phys:
	 * echo 1 > .../ftl_sftl_recover
	 */
	if (!s5l8740_nand_meta_transport_ok()) {
		whimory_set_status(w,
				   "CS metadata DMA disabled; "
				   "use ftl_sftl_recover (CS META path) "
				   "or meta_dma_read=1 dma_dry=0");
		pr_info("s5l8740-ftl: Whimory auto-open deferred "
			"(meta transport off); run ftl_sftl_recover\n");
		return -EOPNOTSUPP;
	}

	ret = whimory_read_signature(w);
	if (ret) {
		whimory_set_status(w, "signature failed %d", ret);
		return ret;
	}
	ret = whimory_select_ops(w);
	if (ret)
		return ret;
	ret = whimory_vfl_open(w);
	if (ret) {
		whimory_set_status(w, "VFL_Open failed %d", ret);
		return ret;
	}
	ret = whimory_ftl_open(w);
	if (ret) {
		whimory_set_status(w, "FTL_Open failed %d", ret);
		return ret;
	}
	ret = whimory_check_lba0(w);
	if (ret) {
		whimory_set_status(w, "LBA0 check failed %d", ret);
		return ret;
	}
	ret = whimory_register_disk(w);
	if (ret) {
		whimory_set_status(w, "disk register failed %d", ret);
		return ret;
	}
	whimory_set_status(w, "ready");
	return 0;
}

bool whimory_l2v_ready(void)
{
	return whimory_dev && whimory_dev->l2v_ok;
}

int whimory_read_fmss_lba(u32 lba, void *buf)
{
	struct whimory *w = whimory_dev;
	int sess, ret;

	if (!w || !buf)
		return -EINVAL;
	if (!w->l2v_ok || !w->ftl || !w->ftl->read_lba)
		return -ENODEV;
	sess = s5l8740_nand_dma_session_begin();
	mutex_lock(&w->tree_lock);
	ret = w->ftl->read_lba(w, lba, buf, false);
	mutex_unlock(&w->tree_lock);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	return ret;
}

int whimory_range_walk(int (*fn)(u32 start, u32 len, u32 vba, u64 weave,
				 void *ctx),
		       void *ctx)
{
	struct whimory *w = whimory_dev;
	struct rb_node *n;
	struct whimory_range *snap;
	unsigned int i, count = 0;
	int ret = 0;

	if (!w || !fn)
		return -EINVAL;

	mutex_lock(&w->tree_lock);
	count = w->sftl.range_nodes;
	if (!count) {
		mutex_unlock(&w->tree_lock);
		return 0;
	}
	snap = kvmalloc_array(count, sizeof(*snap), GFP_KERNEL);
	if (!snap) {
		mutex_unlock(&w->tree_lock);
		return -ENOMEM;
	}
	i = 0;
	for (n = rb_first(&w->ranges); n && i < count; n = rb_next(n)) {
		struct whimory_range *r = rb_entry(n, struct whimory_range, rb);

		snap[i].start = r->start;
		snap[i].len = r->len;
		snap[i].vba = r->vba;
		snap[i].weave = r->weave;
		i++;
	}
	count = i;
	mutex_unlock(&w->tree_lock);

	for (i = 0; i < count; i++) {
		ret = fn(snap[i].start, snap[i].len, snap[i].vba,
			 snap[i].weave, ctx);
		if (ret)
			break;
	}
	kvfree(snap);
	return ret;
}

int whimory_l2v_search_phys(u32 lba, u8 *ce, u8 *cau, u16 *blk, u8 *page,
			    u8 *slot, u64 *weave)
{
	struct whimory *w = whimory_dev;
	u32 vba = ~0u, span = 0, vce, vcau, vblock, vpage, vslot, pblock;
	struct whimory_range *r;
	int ret;

	if (!w || !w->l2v_ok)
		return -ENODEV;
	mutex_lock(&w->tree_lock);
	ret = whimory_l2v_search(w, lba, &vba, &span);
	if (ret || vba >= w->l2v.invalid_vba) {
		mutex_unlock(&w->tree_lock);
		return ret ? ret : -ENOENT;
	}
	r = whimory_range_find(&w->ranges, lba);
	if (weave)
		*weave = r ? r->weave : 0;
	ret = whimory_unpack_vba(w, vba, &vce, &vcau, &vblock, &vpage, &vslot);
	if (ret) {
		mutex_unlock(&w->tree_lock);
		return ret;
	}
	whimory_vfl_resolve(w, vblock, &vcau, &pblock);
	mutex_unlock(&w->tree_lock);
	if (ce)
		*ce = (u8)vce;
	if (cau)
		*cau = (u8)vcau;
	if (blk)
		*blk = (u16)pblock;
	if (page)
		*page = (u8)vpage;
	if (slot)
		*slot = (u8)vslot;
	return 0;
}

static bool whimory_slot_has_needle(const u8 *slot, unsigned int len,
				    const char *needle)
{
	return !!strnstr((const char *)slot, needle, len);
}

/*
 * Physical string scanner — ignores L2V. Walks readable CS pages and prints
 * hits with ce/cau/block/page/slot + meta_lba. Independent of mount.
 */
int whimory_phys_string_scan(unsigned int max_blocks)
{
	struct whimory *w = whimory_dev;
	u8 *data;
	u8 spare[S5L8740_NAND_META_SIZE];
	unsigned int ce, cau, b, pg, slot, nscan, hits = 0, pages = 0;
	static const char *const needles[] = {
		"iTunesDB", "F00", "F01", "F02", "iPod_Control", "Music",
		"Apps", "NanoApps", ".mp3", ".m4a", "mp3", "m4a",
	};
	int sess;

	if (!w || !w->sftl.data_page)
		return -ENODEV;
	nscan = max_blocks ? max_blocks :
		(scan_blocks ? scan_blocks : w->sftl.user_blocks);
	if (!nscan)
		nscan = 256;
	data = w->sftl.data_page;
	sess = s5l8740_nand_dma_session_begin();
	dev_info(w->dev,
		 "PHYS_STRING_SCAN start blocks=%u (L2V ignored)\n", nscan);
	for (ce = 0; ce < w->geom.num_ce; ce++) {
		for (cau = 0; cau < w->geom.num_cau; cau++) {
			for (b = 0; b < nscan; b++) {
				for (pg = 0; pg < WHIMORY_DATA_PAGES_PER_SB;
				     pg++) {
					int ret;
					unsigned int ni;

					ret = whimory_cs_read_page(w, ce, cau,
						b, pg, data,
						S5L8740_NAND_PAGE_SIZE, spare,
						sizeof(spare));
					if (ret)
						break;
					if (whimory_page_blank(data, 64) &&
					    whimory_meta_erased(spare, 16))
						break;
					pages++;
					for (slot = 0;
					     slot < WHIMORY_VBAS_PER_PAGE;
					     slot++) {
						const u8 *d = data +
							slot * WHIMORY_LBA_SIZE;
						const u8 *m = spare +
							slot * WHIMORY_META_SIZE;
						u32 meta_lba =
							get_unaligned_le32(m + 8);

						for (ni = 0; ni < ARRAY_SIZE(needles);
						     ni++) {
							if (!whimory_slot_has_needle(
								    d,
								    WHIMORY_LBA_SIZE,
								    needles[ni]))
								continue;
							hits++;
							if (hits <= 64)
								dev_info(w->dev,
									 "PHYS_STRING hit=%s "
									 "ce=%u cau=%u blk=%u "
									 "page=%u slot=%u "
									 "meta_lba=%u type=%02x\n",
									 needles[ni],
									 ce, cau, b, pg,
									 slot, meta_lba,
									 m[0]);
							break;
						}
					}
					if ((pages & 0x7f) == 0)
						cond_resched();
				}
				if ((b & 0x7f) == 0)
					dev_info(w->dev,
						 "PHYS_STRING_SCAN progress "
						 "ce=%u cau=%u blk=%u/%u hits=%u\n",
						 ce, cau, b, nscan, hits);
			}
		}
	}
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	dev_info(w->dev,
		 "PHYS_STRING_SCAN done pages=%u hits=%u\n", pages, hits);
	return hits;
}

static int whimory_sftl_recover_cs_locked(void)
{
	struct whimory *w = whimory_dev;
	int ret, sess;

	if (!w)
		return -ENODEV;

	ret = whimory_fil_init(w);
	if (ret) {
		whimory_set_status(w, "FIL_Init failed %d", ret);
		return ret;
	}

	ret = whimory_read_signature(w);
	if (ret) {
		dev_warn(w->dev,
			 "signature %d; CS recover continues (identity VFL)\n",
			 ret);
	}

	ret = whimory_select_ops(w);
	if (ret)
		return ret;

	if (!w->vfl_ok) {
		ret = whimory_vfl_open(w);
		if (ret) {
			whimory_set_status(w, "VFL_Open failed %d", ret);
			return ret;
		}
	}

	if (!w->ftl_ok) {
		ret = whimory_ftl_open(w);
		if (ret) {
			whimory_set_status(w, "FTL_Open/recover failed %d",
					   ret);
			return ret;
		}
	} else {
		/* Re-run recover on CS META (clear prior L2V). */
		whimory_range_free(w);
		if (w->l2v.root && w->l2v.num_roots)
			memset(w->l2v.root, 0xff,
			       WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots);
		whimory_l2v_mem_reset(&w->l2v);
		w->l2v_ok = false;
		w->sftl.cxt_loaded = false;
		w->sftl.packed_ok = false;
		w->n_cxt = 0;
		w->cxt_base_weave = 0;
		w->cxt_top_weave = 0;
		w->cxt_save_num_sb = 0;
		w->sftl.btoc_sbs = 0;
		w->sftl.open_sbs = 0;
		w->sftl.empty_sbs = 0;
		w->sftl.cxt_sbs = 0;
		w->sftl.unknown_sbs = 0;
		w->sftl.btoc_pages_read = 0;
		w->sftl.btoc_pages_valid = 0;
		w->sftl.btoc_entries_seen = 0;
		w->sftl.btoc_l2v_updates = 0;
		w->sftl.btoc_meta_mismatch = 0;
		w->sftl.btoc_meta_confirmed = 0;
		w->sftl.btoc_skipped_zero = 0;
		w->sftl.btoc_confirm_pages = 0;
		w->sftl.btoc_confirm_capped = 0;
		w->sftl.btoc_confirm_budget_stop = 0;
		w->sftl.string_hit_itunesdb = 0;
		w->sftl.string_hit_f00 = 0;
		w->sftl.string_hit_apps = 0;
		w->sftl.string_hit_mp3 = 0;
		w->sftl.string_hit_m4a = 0;
		w->sftl.open_slots_seen = 0;
		w->sftl.open_slots_valid_meta = 0;
		w->sftl.open_l2v_updates = 0;
		w->sftl.range_nodes = 0;
		w->sftl.cxt_l2v_updates = 0;
		w->sftl.cxt_meta_confirmed = 0;
		w->sftl.cxt_meta_mismatch = 0;
		w->sftl.cxt_confirm_pages = 0;
		w->sftl.cxt_confirm_unreadable = 0;
		w->sftl.cxt_repair_pages = 0;
		w->sftl.cxt_repair_slots = 0;
		w->sftl.range_budget_stop = 0;
		w->sftl.stale_mapping_rejected = 0;
		w->sftl.n_cxt_idx = 0;

		sess = s5l8740_nand_dma_session_begin();
		if (sess && sess != -EBUSY)
			dev_warn(w->dev, "re-recover DMA session %d\n", sess);
		ret = whimory_sftl_recover_l2v_from_media(w);
		if (sess == 0)
			s5l8740_nand_dma_session_end();
		if (ret) {
			whimory_set_status(w, "re-recover failed %d", ret);
			return ret;
		}
	}

	if (!w->l2v_ok) {
		whimory_set_status(w, "recover OK but l2v_ok=0");
		return -EIO;
	}
	whimory_set_status(w,
			   "CS recover OK mapped_ranges=%u btoc=%u open=%u",
			   w->sftl.range_nodes, w->sftl.btoc_pages_valid,
			   w->sftl.open_l2v_updates);
	dev_info(w->dev, "%s\n", w->status);
	return 0;
}

/*
 * One boot should run one recovery. A repeat with the same knobs is a
 * no-op; a repeat with different knobs rebuilds, but not while the map is
 * already live behind a registered disk unless asked explicitly.
 */
int whimory_sftl_recover_cs(void)
{
	struct whimory *w = whimory_dev;
	int ret;

	if (!w)
		return -ENODEV;
	if (recovery_state == RECOVERY_RUNNING)
		return -EBUSY;
	if (recovery_state == RECOVERY_VALID) {
		if (recovery_params_key == whimory_recover_key()) {
			dev_info(w->dev,
				 "recover: map already valid (same params); skipping\n");
			return 0;
		}
		if (!recover_force && n31_ftl_cs_disk_registered()) {
			dev_warn(w->dev,
				 "recover: disk bound; set recover_force=1 to rebuild\n");
			return -EBUSY;
		}
	}

	recovery_state = RECOVERY_RUNNING;
	ret = whimory_sftl_recover_cs_locked();
	if (ret) {
		recovery_state = RECOVERY_FAILED;
		return ret;
	}
	recovery_state = RECOVERY_VALID;
	recovery_params_key = whimory_recover_key();
	return 0;
}

static int __init ftl_init(void)
{
	struct whimory *w;
	int ret;

	if (!s5l8740_nand_available()) {
		pr_err("s5l8740-ftl: load nand_s5l8740 first\n");
		return -ENODEV;
	}

	w = kzalloc(sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->dev = nand_ftl_device();
	mutex_init(&w->bounce_lock);
	mutex_init(&w->tree_lock);
	w->ranges = RB_ROOT;
	w->bounce = kzalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);
	if (!w->bounce) {
		kfree(w);
		return -ENOMEM;
	}
	w->total_4k_sectors = NAND_FTL_DEFAULT_CAPACITY;
	whimory_dev = w;

	ftl_pdev = platform_device_register_simple("s5l8740-ftl", -1, NULL, 0);
	if (IS_ERR(ftl_pdev)) {
		ret = PTR_ERR(ftl_pdev);
		ftl_pdev = NULL;
		whimory_free(w);
		whimory_dev = NULL;
		return ret;
	}
	w->pdev = ftl_pdev;
	w->dev = &ftl_pdev->dev;
	ret = sysfs_create_group(&ftl_pdev->dev.kobj, &ftl_attr_group);
	if (ret) {
		platform_device_unregister(ftl_pdev);
		whimory_free(w);
		whimory_dev = NULL;
		ftl_pdev = NULL;
		return ret;
	}

	ret = ftl_s5l8740_csmap_init(&ftl_pdev->dev);
	if (ret)
		dev_warn(&ftl_pdev->dev, "CS map init failed %d\n", ret);

	ret = whimory_open_stack(w);
	if (ret) {
		dev_err(w->dev,
			"Whimory open failed (%d) — NOT registering /dev/%s (fil=%d sig=%d vfl=%d ftl=%d l2v=%d lba0=%d)\n",
			ret, FTL_DISK_NAME, w->fil_ok, w->sig_ok, w->vfl_ok,
			w->ftl_ok, w->l2v_ok, w->lba0_ok);
		/*
		 * Keep the platform device so sysfs status is visible.
		 * The block disk is absent until LBA0 works.
		 */
		return 0;
	}
	return 0;
}

static void __exit ftl_exit(void)
{
	struct whimory *w = whimory_dev;

	if (ftl_pdev) {
		ftl_s5l8740_csmap_exit(&ftl_pdev->dev);
		sysfs_remove_group(&ftl_pdev->dev.kobj, &ftl_attr_group);
		platform_device_unregister(ftl_pdev);
		ftl_pdev = NULL;
	}
	whimory_dev = NULL;
	whimory_free(w);
}

module_init(ftl_init);
module_exit(ftl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5L8740 Whimory PPN SFTL read-only block driver");
MODULE_AUTHOR("n31");
MODULE_SOFTDEP("pre: nand_s5l8740");
MODULE_FIRMWARE(WHIMORY_ORACLE_SIG);
MODULE_FIRMWARE(WHIMORY_ORACLE_ROOT);
MODULE_FIRMWARE(WHIMORY_ORACLE_NODES);
MODULE_FIRMWARE(WHIMORY_ORACLE_GLOBALS);
