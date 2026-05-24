/*
 * readoob - dump the OOB area of one or more NAND pages via MTD ioctls.
 *
 * Usage: readoob <mtd-device> <page> [count]
 *   e.g. readoob /dev/mtd1 0       # OOB of page 0
 *        readoob /dev/mtd1 0 4     # OOB of pages 0..3
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <mtd/mtd-user.h>

int main(int argc, char **argv)
{
	if (argc < 3 || argc > 4) {
		fprintf(stderr, "usage: %s <mtd-device> <page> [count]\n", argv[0]);
		return 2;
	}

	const char *path = argv[1];
	unsigned long start_page = strtoul(argv[2], NULL, 0);
	unsigned long count = (argc == 4) ? strtoul(argv[3], NULL, 0) : 1;

	int fd = open(path, O_RDONLY);
	if (fd < 0) { perror("open"); return 1; }

	struct mtd_info_user info;
	if (ioctl(fd, MEMGETINFO, &info) < 0) { perror("MEMGETINFO"); return 1; }

	printf("# %s: type=%u writesize=%u oobsize=%u erasesize=%u size=%llu\n",
	       path, info.type, info.writesize, info.oobsize, info.erasesize,
	       (unsigned long long)info.size);

	uint8_t *buf = malloc(info.oobsize);
	if (!buf) { perror("malloc"); return 1; }

	for (unsigned long p = start_page; p < start_page + count; p++) {
		struct mtd_oob_buf oob = {
			.start  = (uint32_t)((unsigned long long)p * info.writesize),
			.length = info.oobsize,
			.ptr    = buf,
		};

		if (ioctl(fd, MEMREADOOB, &oob) < 0) {
			fprintf(stderr, "page %lu: MEMREADOOB: %s\n",
				p, strerror(errno));
			continue;
		}

		printf("page %lu OOB:", p);
		for (unsigned int i = 0; i < info.oobsize; i++) {
			if ((i % 16) == 0) printf("\n  %04x:", i);
			printf(" %02x", buf[i]);
		}
		printf("\n");

		/* Bad-block marker is OOB byte 0 of the first page in the block.
		 * 0xFF => good, anything else => MTD will mark this block bad. */
		if (((unsigned long long)p * info.writesize) % info.erasesize == 0) {
			printf("  bbm(byte0)=0x%02x %s\n", buf[0],
			       buf[0] == 0xff ? "(good)" : "(BAD according to marker)");
		}
	}

	free(buf);
	close(fd);
	return 0;
}
