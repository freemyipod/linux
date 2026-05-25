#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>

static size_t pagesize;

void *map_phys(int fd, uint64_t addr, size_t len, int prot)
{
    uint64_t page_base = addr & ~(pagesize - 1);
    uint64_t page_off  = addr - page_base;
    size_t map_len = page_off + len;

    void *map = mmap(NULL, map_len, prot, MAP_SHARED, fd, page_base);
    if (map == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }
    return (uint8_t *)map + page_off;
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr,
            "Usage:\n"
            "  %s dump  <phys> <len> [file]\n"
            "  %s write <phys> <len> [file]\n"
            "  %s fill  <phys> <len> <byte>\n",
            argv[0], argv[0], argv[0]);
        return 1;
    }

    pagesize = sysconf(_SC_PAGE_SIZE);

    uint64_t phys = strtoull(argv[2], NULL, 0);
    size_t len    = strtoull(argv[3], NULL, 0);

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    if (!strcmp(argv[1], "dump")) {
        void *p = map_phys(fd, phys, len, PROT_READ);
        FILE *f;
        if (argc >= 5) {
            f = fopen(argv[4], "wb");
            if (!f) { perror("fopen"); return 1; }
        } else {
            f = stdout;
        }
        fwrite(p, 1, len, f);
        if (f != stdout) fclose(f);
    }
    else if (!strcmp(argv[1], "write")) {
        void *p = map_phys(fd, phys, len, PROT_READ | PROT_WRITE);
        FILE *f;
        if (argc >= 5) {
            f = fopen(argv[4], "rb");
            if (!f) { perror("fopen"); return 1; }
        } else {
            f = stdin;
        }
        fread(p, 1, len, f);
        if (f != stdin) fclose(f);
    }
    else if (!strcmp(argv[1], "fill")) {
        if (argc < 5) {
            fprintf(stderr, "fill mode requires <byte>\n");
            return 1;
        }
        void *p = map_phys(fd, phys, len, PROT_READ | PROT_WRITE);
        int val = strtoul(argv[4], NULL, 0) & 0xff;
        memset(p, val, len);
    } else {
        fprintf(stderr, "Unknown mode '%s'\n", argv[1]);
        return 1;
    }

    close(fd);
    return 0;
}
