#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define MIN_BLOCK 0x10   // minimal comparison block
#define PAGE_SIZE 0x1000 // mmap page alignment

bool compare_block(uint8_t *a, uint8_t *b, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

int main(int argc, char *argv[]) {
    if (argc < 3 || argc > 4) {
        printf("Usage: %s <start_address> <end_address> [verbose]\n", argv[0]);
        return 1;
    }

    uintptr_t start_phys = strtoul(argv[1], NULL, 0);
    uintptr_t end_phys   = strtoul(argv[2], NULL, 0);
    bool verbose = (argc == 4);

    if (start_phys >= end_phys) {
        fprintf(stderr, "Invalid address range\n");
        return 1;
    }

    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    // mmap memory page-aligned
    uintptr_t start_aligned = start_phys & ~(PAGE_SIZE - 1);
    uintptr_t offset = start_phys - start_aligned;
    size_t map_size = end_phys - start_aligned;

    uint8_t *mem = mmap(NULL, map_size, PROT_READ, MAP_SHARED, fd, start_aligned);
    if (mem == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    uint8_t *base = mem + offset;
    uint8_t *end  = mem + offset + (end_phys - start_phys);

    while (base + MIN_BLOCK <= end) {
        size_t block_size = MIN_BLOCK;

        // Dynamically increase block size until first candidate is found
        while (true) {
            uint8_t *candidate = base + block_size;
            if (candidate + MIN_BLOCK > end) break;

            size_t stride = candidate - base;
            if (stride == MIN_BLOCK) { // skip trivial stride silently
                goto increase_block;
            }

            if (compare_block(base, candidate, MIN_BLOCK)) {
                if (verbose) {
                    uintptr_t base_mmio = start_phys + (base - (mem + offset));
                    printf("[VERBOSE] Potential mirror at MMIO 0x%lx (stride 0x%lx, block 0x%zx)\n",
                           base_mmio, stride, block_size);
                }

                // Confirm next blocks for false positives
                bool confirmed = true;
                for (size_t off = MIN_BLOCK; off < block_size; off += MIN_BLOCK) {
                    if (!compare_block(base + off, candidate + off, MIN_BLOCK)) {
                        confirmed = false;
                        break;
                    }
                }

                if (confirmed) {
                    // Scan additional copies
                    size_t copies = 2;
                    while (true) {
                        uint8_t *next_copy = candidate + (copies - 1) * stride;
                        if (next_copy + block_size > end) break;

                        if (!compare_block(base, next_copy, block_size)) break;
                        copies++;
                    }

                    if (copies > 2) {
                        // Dynamically determine full peripheral size
                        size_t peripheral_size = block_size;
                        while (true) {
                            bool all_match = true;
                            if (base + peripheral_size + MIN_BLOCK > end) break;

                            for (size_t c = 1; c < copies; c++) {
                                uint8_t *copy_ptr = base + c * stride;
                                if (!compare_block(base + peripheral_size,
                                                   copy_ptr + peripheral_size,
                                                   MIN_BLOCK)) {
                                    all_match = false;
                                    break;
                                }
                            }

                            if (!all_match) break;
                            peripheral_size += MIN_BLOCK;
                        }

                        uintptr_t base_mmio = start_phys + (base - (mem + offset));
                        printf("Mirrored peripheral found:\n");
                        printf("  Base MMIO: 0x%lx\n", base_mmio);
                        printf("  Stride: 0x%lx\n", stride);
                        printf("  Copies: %zu\n", copies);
                        printf("  Peripheral size: 0x%zx\n\n", peripheral_size);

                        base += peripheral_size * copies; // skip past detected peripheral
                        goto next_base;
                    }
                }
            }

increase_block:
            // Dynamically increase block size if first block doesn't match
            if (candidate + block_size >= end) break;
            block_size *= 0x10;
        }

        base += MIN_BLOCK;
    next_base:
        continue;
    }

    munmap(mem, map_size);
    close(fd);
    return 0;
}
