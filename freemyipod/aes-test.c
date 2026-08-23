// aes-test.c
//
// Simple AF_ALG skcipher encrypt/decrypt filter.
// - argv[1] = 0 => decrypt
// - argv[1] = 1 => encrypt
// - argv[2] = algorithm name
// - reads stdin in 4096-byte chunks
// - writes raw output to stdout
// - key is all zeroes
// - IV is not set
//
// Example:
//   gcc -O2 -Wall aes-test.c -o aes-test
//
// Encrypt:
//   cat plain.bin | ./aes-test 1 'ecb(aes)' > enc.bin
//
// Decrypt:
//   cat enc.bin | ./aes-test 0 'ecb(aes)' > dec.bin
//
// Notes:
// - Input size must be a multiple of AES block size (16 bytes).

#define _GNU_SOURCE

#include <errno.h>
#include <linux/if_alg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define CHUNK 4096
#define KEYLEN 16

static void die(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
    int tfmfd, opfd;
    int encrypt;
    struct sockaddr_alg sa;
    uint8_t key[KEYLEN];
    uint8_t inbuf[CHUNK];
    uint8_t outbuf[CHUNK];

    if (argc != 3) {
        fprintf(stderr, "usage: %s <0|1> <algo>\n", argv[0]);
	    fprintf(stderr, "0 = decrypt, 1 = encrypt\n");
	    fprintf(stderr, "algos: ecb(aes), cbc(aes), cbc(aes-gid), cbc(aes-uid)\n");
        return 1;
    }

    encrypt = atoi(argv[1]) ? ALG_OP_ENCRYPT : ALG_OP_DECRYPT;

    memset(key, 0, sizeof(key));
    memset(&sa, 0, sizeof(sa));

    sa.salg_family = AF_ALG;
    strcpy((char *)sa.salg_type, "skcipher");
    strcpy((char *)sa.salg_name, argv[2]);

    tfmfd = socket(AF_ALG, SOCK_SEQPACKET, 0);
    if (tfmfd < 0)
        die("socket");

    if (bind(tfmfd, (struct sockaddr *)&sa, sizeof(sa)) < 0)
        die("bind");

    if (setsockopt(tfmfd, SOL_ALG, ALG_SET_KEY, key, sizeof(key)) < 0)
        die("setsockopt(ALG_SET_KEY)");

    opfd = accept(tfmfd, NULL, 0);
    if (opfd < 0)
        die("accept");

    for (;;) {
        ssize_t n;

        n = read(STDIN_FILENO, inbuf, sizeof(inbuf));
        if (n < 0)
            die("read");

        if (n == 0)
            break;

        struct msghdr msg;
        struct cmsghdr *cmsg;
        char cbuf[CMSG_SPACE(sizeof(uint32_t))];
        struct iovec iov;

        memset(&msg, 0, sizeof(msg));
        memset(cbuf, 0, sizeof(cbuf));

        iov.iov_base = inbuf;
        iov.iov_len = n;

        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;

        msg.msg_control = cbuf;
        msg.msg_controllen = sizeof(cbuf);

        cmsg = CMSG_FIRSTHDR(&msg);
        cmsg->cmsg_level = SOL_ALG;
        cmsg->cmsg_type = ALG_SET_OP;
        cmsg->cmsg_len = CMSG_LEN(sizeof(uint32_t));

        *((uint32_t *)CMSG_DATA(cmsg)) = encrypt;

        if (sendmsg(opfd, &msg, 0) < 0)
            die("sendmsg");

        n = read(opfd, outbuf, n);
        if (n < 0)
            die("read(opfd)");

        ssize_t off = 0;

        while (off < n) {
            ssize_t w = write(STDOUT_FILENO, outbuf + off, n - off);
            if (w < 0)
                die("write");

            off += w;
        }
    }

    close(opfd);
    close(tfmfd);

    return 0;
}
