#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/if_alg.h>

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <out_size>\n", argv[0]);
        return 1;
    }

    size_t out_size = strtoul(argv[1], NULL, 10);
    if (out_size == 0) {
        fprintf(stderr, "Invalid out_size\n");
        return 1;
    }

    int sfd = socket(AF_ALG, SOCK_SEQPACKET, 0);
    if (sfd < 0) { perror("socket"); return 1; }

    struct sockaddr_alg sa = {
        .salg_type = "rng",
        .salg_name = "prng-s5l8702"
    };
    if (bind(sfd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        perror("bind"); close(sfd); return 1;
    }

    int opfd = accept(sfd, NULL, 0);
    close(sfd);
    if (opfd < 0) { perror("accept"); return 1; }

    uint8_t *out = malloc(out_size);
    if (!out) {
        perror("malloc");
        close(opfd);
        return 1;
    }

    ssize_t n = read(opfd, out, out_size);
    if (n < 0) {
        perror("read");
        free(out);
        close(opfd);
        return 1;
    }

    printf("Generated %zd bytes: ", n);
    for (ssize_t i = 0; i < n; i++)
        printf("%02X ", out[i]);
    printf("\n");

    free(out);
    close(opfd);
    return 0;
}
