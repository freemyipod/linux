/*
 * dumpall - dump a range of NAND pages (data + OOB) via MTD ioctls.
 *
 * Each page is written as one record: <writesize bytes data><oobsize bytes OOB>
 * (so 2048 + 64 = 2112 bytes per page on the n3g).
 *
 * Unlike busybox nanddump this doesn't trust struct mtd_info_user.size, so
 * it works on >= 4 GiB devices.
 *
 * Usage: dumpall <mtd-device> <start_page> <count> [output-file]
 *   e.g. dumpall /dev/mtd1 0 16 -          # 16 pages to stdout
 *        dumpall /dev/mtd1 0 65536 dump.bin
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
	if (argc < 4 || argc > 5) {
		fprintf(stderr,
			"usage: %s <mtd-device> <start_page> <count> [output-file]\n",
			argv[0]);
		return 2;
	}

	const char *path = argv[1];
	unsigned long long start_page = strtoull(argv[2], NULL, 0);
	unsigned long long count      = strtoull(argv[3], NULL, 0);
	const char *outpath = (argc == 5) ? argv[4] : "-";

	int fd = open(path, O_RDONLY);
	if (fd < 0) { perror("open mtd"); return 1; }

	struct mtd_info_user info;
	if (ioctl(fd, MEMGETINFO, &info) < 0) { perror("MEMGETINFO"); return 1; }

	fprintf(stderr,
		"# %s: writesize=%u oobsize=%u erasesize=%u (size field=%u — ignored)\n",
		path, info.writesize, info.oobsize, info.erasesize, info.size);
	fprintf(stderr,
		"# dumping pages [%llu..%llu), %llu bytes/record\n",
		start_page, start_page + count,
		(unsigned long long)(info.writesize + info.oobsize));

	int outfd;
	if (!strcmp(outpath, "-")) {
		outfd = STDOUT_FILENO;
	} else {
		outfd = open(outpath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
		if (outfd < 0) { perror("open out"); return 1; }
	}

	uint8_t *data = malloc(info.writesize);
	uint8_t *oob  = malloc(info.oobsize);
	if (!data || !oob) { perror("malloc"); return 1; }

	unsigned long long failed = 0;
	for (unsigned long long p = start_page; p < start_page + count; p++) {
		off_t off = (off_t)p * info.writesize;
		ssize_t n;

		if (lseek(fd, off, SEEK_SET) != off) {
			fprintf(stderr, "page %llu: lseek: %s\n", p, strerror(errno));
			failed++;
			continue;
		}
		n = read(fd, data, info.writesize);
		if (n != (ssize_t)info.writesize) {
			fprintf(stderr, "page %llu: read data: got %zd (%s)\n",
				p, n, strerror(errno));
			failed++;
			continue;
		}

		struct mtd_oob_buf ob = {
			.start  = (uint32_t)off,
			.length = info.oobsize,
			.ptr    = oob,
		};
		if (ioctl(fd, MEMREADOOB, &ob) < 0) {
			fprintf(stderr, "page %llu: MEMREADOOB: %s\n",
				p, strerror(errno));
			failed++;
			continue;
		}

		if (write(outfd, data, info.writesize) != (ssize_t)info.writesize ||
		    write(outfd, oob,  info.oobsize)   != (ssize_t)info.oobsize) {
			perror("write out");
			return 1;
		}
	}

	free(data);
	free(oob);
	if (outfd != STDOUT_FILENO) close(outfd);
	close(fd);

	fprintf(stderr, "# done. %llu pages requested, %llu failed.\n",
		count, failed);
	return failed ? 1 : 0;
}
