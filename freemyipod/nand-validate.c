#define _GNU_SOURCE
#define _LARGEFILE64_SOURCE
#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <mtd/mtd-user.h>

#define ENABLE_ACCESS_LOGGING 0
#define ENABLE_MEMORY_LOGGING 0
#define LOG_ACCESS if(ENABLE_ACCESS_LOGGING) printf
#define LOG_MEMORY if(ENABLE_MEMORY_LOGGING) printf

#define BANK_COUNT  2
#define PAGE_SIZE   2048
#define SPARE_SIZE  64       /* what the disk-mode firmware expects */
#define OOB_USED    12       /* the bytes that actually matter      */

#define DRAM_ORIG   0x08000000u
/* The disk-mode binary's DRAM slice is 0x3f5c0; 1 MB leaves plenty of
 * room for any static buffers it expects to find beyond the load image
 * (and avoids tripping kernel overcommit limits on small iPods). */
#define DRAM_SIZE   0x100000u
#define IRAM_ORIG   0x22000000u
#define IRAM_SIZE   0x40000u

#define DEFAULT_IMG_PATH "/etc/diskmode.bin"
#define IMG_LEN          0x447bcu

#define NAND_DEVICE    "/dev/mtd1"

/* Disk-mode image layout (matches the Pi version):
 *   first IRAM_IMG_LEN bytes -> IRAM, next DRAM_IMG_LEN -> DRAM. */
#define IRAM_IMG_LEN   0x51f8u
#define DRAM_IMG_LEN   0x3f5c0u

/* The disk-mode binary uses absolute addresses (e.g. branches to
 * 0x0800CFB8), so we have to put it at the *virtual* addresses it expects.
 * We don't need physical addressing — there's no DMA-capable hardware
 * involved any more — so plain MAP_ANONYMOUS|MAP_FIXED is enough, matching
 * what the original Pi version did. */
static void* map_anon_fixed(uint32_t addr, size_t len) {
    void* p = mmap((void*)(uintptr_t)addr, len,
                   PROT_READ | PROT_WRITE | PROT_EXEC,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    if (p == MAP_FAILED) {
        fprintf(stderr, "mmap anon @ 0x%08x len 0x%zx: %s\n",
                addr, len, strerror(errno));
        exit(1);
    }
    return p;
}

static void load_image(const char* path, uint8_t* dst) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) { fprintf(stderr, "open %s: %s\n", path, strerror(errno)); exit(1); }
    size_t got = 0;
    while (got < IMG_LEN) {
        ssize_t r = read(fd, dst + got, IMG_LEN - got);
        if (r < 0) { if (errno == EINTR) continue; perror("read image"); exit(1); }
        if (r == 0) {
            fprintf(stderr, "short image: got %zu, expected 0x%x\n", got, IMG_LEN);
            exit(1);
        }
        got += (size_t)r;
    }
    close(fd);
}

static int nand_fd = -1;
static uint32_t pages_per_bank;

/* Read the MTD device size from sysfs as a u64.  MEMGETINFO's size field
 * is u32 and silently wraps to 0 on a 4 GiB device — same gotcha that
 * dumpall.c documents. */
static uint64_t mtd_size_from_sysfs(const char* dev) {
    /* dev = "/dev/mtdN" -> "/sys/class/mtd/mtdN/size" */
    const char* base = strrchr(dev, '/');
    base = base ? base + 1 : dev;
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/mtd/%s/size", base);
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "open %s: %s\n", path, strerror(errno)); exit(1); }
    unsigned long long sz;
    if (fscanf(f, "%llu", &sz) != 1) {
        fprintf(stderr, "parse %s\n", path); exit(1);
    }
    fclose(f);
    return (uint64_t)sz;
}

static void nand_setup(void) {
    nand_fd = open(NAND_DEVICE, O_RDONLY);
    if (nand_fd < 0) { perror("open " NAND_DEVICE); exit(1); }

    struct mtd_info_user info;
    if (ioctl(nand_fd, MEMGETINFO, &info) < 0) {
        perror("MEMGETINFO"); exit(1);
    }
    if (info.writesize != PAGE_SIZE) {
        fprintf(stderr, "unexpected MTD writesize=%u (want %u)\n",
                info.writesize, PAGE_SIZE);
        exit(1);
    }
    if (info.oobsize < OOB_USED) {
        fprintf(stderr, "MTD oobsize=%u smaller than needed %u\n",
                info.oobsize, OOB_USED);
        exit(1);
    }

    uint64_t mtd_size = mtd_size_from_sysfs(NAND_DEVICE);
    uint64_t total_pages = mtd_size / PAGE_SIZE;
    if (total_pages == 0) {
        fprintf(stderr, "MTD size from sysfs is 0 — bailing\n");
        exit(1);
    }
    if (total_pages % BANK_COUNT) {
        fprintf(stderr, "MTD size 0x%llx not divisible by %d banks\n",
                (unsigned long long)mtd_size, BANK_COUNT);
        exit(1);
    }
    pages_per_bank = (uint32_t)(total_pages / BANK_COUNT);
    printf("[NAND] %s: size=0x%llx, %u banks of %u pages, oobsize=%u\n",
           NAND_DEVICE, (unsigned long long)mtd_size,
           BANK_COUNT, pages_per_bank, info.oobsize);
}

static uint64_t bank_page_to_byte_off(uint8_t bank, uint32_t page) {
    return ((uint64_t)bank * pages_per_bank + page) * PAGE_SIZE;
}

static void nand_read_data(uint8_t bank, uint32_t page, uint8_t* data) {
    uint64_t off = bank_page_to_byte_off(bank, page);
    size_t got = 0;
    while (got < PAGE_SIZE) {
        ssize_t r = pread(nand_fd, data + got, PAGE_SIZE - got, off + got);
        if (r < 0) { if (errno == EINTR) continue; perror("pread nand"); exit(1); }
        if (r == 0) {
            fprintf(stderr, "EOF reading bank=%u page=0x%x off=0x%llx\n",
                    bank, page, (unsigned long long)(off + got));
            exit(1);
        }
        got += (size_t)r;
    }
}

static void nand_read_spare(uint8_t bank, uint32_t page, uint8_t* spare) {
    /* Disk-mode FIL expects SPARE_SIZE bytes; only the first OOB_USED are
     * meaningful.  Pad the rest with 0xff so the upper layers see "blank". */
    memset(spare, 0xff, SPARE_SIZE);

    uint64_t off = bank_page_to_byte_off(bank, page);
    struct mtd_oob_buf oob = {
        .start  = off,           /* byte offset of the page             */
        .length = OOB_USED,      /* read just what we use               */
        .ptr    = spare,
    };
    if (ioctl(nand_fd, MEMREADOOB, &oob) < 0) {
        fprintf(stderr, "MEMREADOOB bank=%u page=0x%x: %s\n",
                bank, page, strerror(errno));
        exit(1);
    }
}

/* ---- FIL adapter functions (signatures unchanged from Pi version) ------ */

/* 4-byte READID for the 4 GB n3g (N46)'s Apple-spec'd Micron NAND part, per the
 * comment in drivers/mtd/nand/raw/s5l8702_nand.c.  The original Pi tool
 * used 0xa5d5d589 which is the 8 GB variant; the fourth byte (geometry
 * hints) differs and makes the disk-mode firmware compute a wrong block
 * size.  Populate one ID slot per bank — 2 banks on the 4 GB device. */
static uint32_t NAND_IDs[8] = {0xa5d5d52c, 0xa5d5d52c, 0, 0, 0, 0, 0, 0};

static uint32_t is_page_all_FFs(uint8_t* data_buf) {
    for (int i = 0; i < 16; i++) if (data_buf[i] != 0xff) return 0;
    return 1;
}

uint32_t FIL_readIds(void) {
    ((uint32_t*)(0x0803f464 + 0x14))[0] = (uint32_t)(uintptr_t)NAND_IDs;
    return 0;
}

/* Dump the first N reads (data preview + full 16-byte raw OOB) so we can
 * sanity-check what we're feeding the firmware.  Bumped via the
 * DUMP_FIRST_N define below. */
#define DUMP_FIRST_N 16
static int dumps_done = 0;

static void dump_read(const char* via, uint8_t bank, uint32_t page,
                      const uint8_t* data, const uint8_t* meta) {
    if (dumps_done >= DUMP_FIRST_N) return;
    dumps_done++;
    printf("[DUMP %2d] %s bank=%u page=0x%x\n", dumps_done, via, bank, page);
    printf("  data[0..15]: ");
    for (int i = 0; i < 16; i++) printf("%02x ", data[i]);
    printf("\n  meta[0..15]: ");
    for (int i = 0; i < 16; i++) printf("%02x ", meta[i]);
    printf("\n");
    fflush(stdout);
}

uint32_t FIL_readSinglePage(uint16_t bank, uint32_t page,
                            uint8_t* data, uint8_t* meta, uint8_t* corr) {
    LOG_ACCESS("[ADAPTER] FIL_readSinglePage(0x%x, 0x%x)\n", bank, page);
    nand_read_data((uint8_t)bank, page, data);
    nand_read_spare((uint8_t)bank, page, meta);
    dump_read("readSingle", (uint8_t)bank, page, data, meta);
    return is_page_all_FFs(meta);
}

uint32_t FIL_readNoECC(uint16_t bank, uint32_t page,
                       uint8_t* data, uint8_t* meta) {
    LOG_ACCESS("[ADAPTER] FIL_readNoECC(0x%x, 0x%x)\n", bank, page);
    nand_read_data((uint8_t)bank, page, data);
    nand_read_spare((uint8_t)bank, page, meta);
    return 0;
}

uint32_t FIL_readSequentialPages(uint16_t bank, uint32_t page,
                                 uint8_t* data, uint8_t* meta, uint8_t* corr) {
    LOG_ACCESS("[ADAPTER] FIL_readSequentialPages(0x%x, 0x%x)\n", bank, page);
    return is_page_all_FFs(meta);
}

uint32_t FIL_readScatteredPages(uint16_t* banks, uint32_t* pages,
                                uint8_t* data, uint8_t* meta,
                                uint16_t count, uint8_t* corr) {
    LOG_ACCESS("[ADAPTER] FIL_readScatteredPages(count=%u)\n", count);
    for (int i = 0; i < count; i++) {
        nand_read_data((uint8_t)banks[i], pages[i], data + i * PAGE_SIZE);
        nand_read_spare((uint8_t)banks[i], pages[i], meta + i * SPARE_SIZE);
    }
    return is_page_all_FFs(meta);
}

uint32_t FIL_readSinglePageNoMetadata(uint16_t bank, uint32_t page,
                                      uint8_t* data) {
    LOG_ACCESS("[ADAPTER] FIL_readSinglePageNoMetadata(0x%x, 0x%x)\n", bank, page);
    nand_read_data((uint8_t)bank, page, data);
    /* No OOB read on this path; pass a zeroed meta to the dumper so the
     * format stays uniform — useful for spotting which reads went through
     * which adapter. */
    uint8_t meta_dummy[16] = {0};
    dump_read("readNoMeta", (uint8_t)bank, page, data, meta_dummy);
    return is_page_all_FFs(data);
}

uint32_t FIL_writeScatteredPages(uint16_t* b, uint32_t* p, uint8_t* d, uint8_t* m, uint16_t c) {
    fprintf(stderr, "[ADAPTER] WARN: write ignored (read-only)\n"); return 0;
}
uint32_t FIL_writeSinglePage(uint16_t b, uint32_t p, uint8_t* d, uint8_t* m) {
    fprintf(stderr, "[ADAPTER] WARN: write ignored (read-only)\n"); return 0;
}
uint32_t FIL_writeSequentialPages(uint32_t* p, uint8_t* d, uint8_t* m, uint16_t c, uint8_t a) {
    fprintf(stderr, "[ADAPTER] WARN: write ignored (read-only)\n"); return 0;
}
uint32_t FIL_writeSinglePageNoMetadata(uint16_t b, uint32_t p, uint8_t* d) {
    fprintf(stderr, "[ADAPTER] WARN: write ignored (read-only)\n"); return 0;
}
uint32_t FIL_eraseSingleBlock(uint16_t b, uint16_t blk) {
    fprintf(stderr, "[ADAPTER] WARN: erase ignored (read-only)\n"); return 0;
}
uint32_t FIL_eraseSequentialBlocks(uint16_t* b, uint16_t* blk, uint32_t c) {
    fprintf(stderr, "[ADAPTER] WARN: erase ignored (read-only)\n"); return 0;
}
uint32_t FIL_resetAndVerifyIds(void) { return 0; }
uint32_t FIL_reset(void)              { return 0; }
uint32_t return_one(void)             { return 1; }

void* ipod_malloc(int size, void* kind)        { return calloc(1, size); }
void  ipod_free  (void* trash, void* ptr)      { free(ptr); }

void patch_function(uint32_t offset, void* function) {
    ((uint32_t*)(uintptr_t)(offset +  0))[0] = 0xe92d4200; // push {r9, lr}
    ((uint32_t*)(uintptr_t)(offset +  4))[0] = 0xe59f9008; // ldr  r9, [pc, #8]
    ((uint32_t*)(uintptr_t)(offset +  8))[0] = 0xe12fff39; // blx  r9
    ((uint32_t*)(uintptr_t)(offset + 12))[0] = 0xe8bd4200; // pop  {r9, lr}
    ((uint32_t*)(uintptr_t)(offset + 16))[0] = 0xe12fff1e; // bx   lr
    ((uint32_t*)(uintptr_t)(offset + 20))[0] = (uint32_t)(uintptr_t)function;
}
void null_function(uint32_t offset) {
    ((uint32_t*)(uintptr_t)offset)[0] = 0xe12fff1e; // bx lr
}

int main(int argc, char** argv) {
    const char* img_path = (argc > 1) ? argv[1] : DEFAULT_IMG_PATH;

    /* Map DRAM and IRAM at the virtual addresses the firmware's absolute
     * pointers expect.  Anonymous memory is fine — no DMA hardware ever
     * sees these addresses, just the CPU through the MMU. */
    void* dram = map_anon_fixed(DRAM_ORIG, DRAM_SIZE);
    void* iram = map_anon_fixed(IRAM_ORIG, IRAM_SIZE);
    printf("[INIT] DRAM mapped at %p, IRAM at %p\n", dram, iram);

    /* Read the pre-decrypted disk-mode image into DRAM, then split it
     * into the IRAM and DRAM slices the firmware expects. */
    load_image(img_path, (uint8_t*)dram);
    printf("[INIT] read 0x%x bytes from %s into DRAM\n", IMG_LEN, img_path);

    memcpy(iram, dram, IRAM_IMG_LEN);
    memmove(dram, (uint8_t*)dram + IRAM_IMG_LEN, DRAM_IMG_LEN);

    /* Open NAND through MTD. */
    nand_setup();

    /* Fake NAND register windows so the firmware's MMIO pokes have a home.
     * These don't go anywhere real — they're just scratch RAM that swallows
     * writes; the FIL patches above intercept every real NAND op. */
    map_anon_fixed(0x38a00000, 0x1000);
    map_anon_fixed(0x38e00000, 0x10000);

    /* Patch FIL entry points. */
    patch_function(0x0801A1AC, FIL_reset);
    patch_function(0x08032CF0, FIL_readIds);

    patch_function(0x08032740, FIL_readNoECC);
    patch_function(0x08032874, FIL_readScatteredPages);
    patch_function(0x080329dc, FIL_readSequentialPages);
    patch_function(0x08032b80, FIL_readSinglePage);
    patch_function(0x08032bec, FIL_readSinglePageNoMetadata);

    /* malloc / WMR_MALLOC -> libc. */
    patch_function(0x08019070, ipod_malloc);
    patch_function(0x08009d50, malloc);
    patch_function(0x08009d9c, malloc);

    /* AND_log -> printf, plus a few no-ops. */
    patch_function(0x08009e0c, printf);
    null_function(0x08009ce4);
    null_function(0x08004bc4);
    null_function(0x08007370);

    /* waitForCSInterrupt -> always-ready. */
    patch_function(0x080151f8, return_one);

    /* NAND heap base. */
    void* nand_heap_base = malloc(0x1000);
    ((uint32_t*)0x0803d384)[0] = (uint32_t)(uintptr_t)nand_heap_base;

    int (*AND_Init)(uint32_t*, uint16_t*) =
        (int (*)(uint32_t*, uint16_t*))0x0800cfb8;
    int (*FTL_Read)(uint32_t, uint32_t, uint8_t*) =
        (int (*)(uint32_t, uint32_t, uint8_t*))0x08008584;

    uint32_t nand_lba_out;
    uint16_t pages_per_block_exp_out;

    uint32_t ret = AND_Init(&nand_lba_out, &pages_per_block_exp_out);
    printf("AND_Init ret: %u\n", ret);
    printf("nand_lba_out: %u\n", nand_lba_out);
    printf("pages_per_block_exp_out: %u\n", pages_per_block_exp_out);
    printf("NAND SIZE: %2u.%2u\n",
           (nand_lba_out << 2) >> 0x14,
           0x000FFFFF & (nand_lba_out << 2));

    void* buf = malloc(0x1000);
    ret = FTL_Read(0, 2, buf);
    printf("FTL_Read returns %u\n", ret);
    return 0;
}
