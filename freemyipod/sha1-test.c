#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/if_alg.h>
#include <errno.h>

#define SHA1_DIGEST_SIZE 20

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <buffer_size>\n", argv[0]);
        return EXIT_FAILURE;
    }

    char *endptr;
    long buf_size = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0' || buf_size <= 0) {
        fprintf(stderr, "Error: Invalid buffer size '%s'\n", argv[1]);
        return EXIT_FAILURE;
    }

    unsigned char *buf = malloc(buf_size);
    if (!buf) {
        perror("malloc");
        return EXIT_FAILURE;
    }

    /* 1. Create control socket */
    int ctrl_fd = socket(AF_ALG, SOCK_SEQPACKET, 0);
    if (ctrl_fd < 0) {
        perror("socket");
        free(buf);
        return EXIT_FAILURE;
    }

    /* 2. Bind to SHA1 hash algorithm */
    struct sockaddr_alg salg;
    memset(&salg, 0, sizeof(salg));
    salg.salg_family = AF_ALG;
    strncpy(salg.salg_type, "hash", sizeof(salg.salg_type) - 1);
    strncpy(salg.salg_name, "sha1", sizeof(salg.salg_name) - 1);

    if (bind(ctrl_fd, (struct sockaddr *)&salg, sizeof(salg)) < 0) {
        perror("bind");
        close(ctrl_fd);
        free(buf);
        return EXIT_FAILURE;
    }

    /* 3. Accept operation socket */
    int op_fd = accept(ctrl_fd, NULL, 0);
    if (op_fd < 0) {
        perror("accept");
        close(ctrl_fd);
        free(buf);
        return EXIT_FAILURE;
    }

    /* Control socket is no longer needed */
    close(ctrl_fd);

    /* Optional: Explicitly set operation to digest (default for hash type) */
/*
    const char *op = "digest";
    if (setsockopt(op_fd, SOL_ALG, ALG_SET_OP, op, strlen(op)) < 0) {
        perror("setsockopt ALG_SET_OP");
        close(op_fd);
        free(buf);
        return EXIT_FAILURE;
    }
*/
    /* 4. Stream data from stdin to the kernel crypto engine */
    ssize_t n;
    while ((n = read(STDIN_FILENO, buf, buf_size)) > 0) {
        /* MSG_MORE tells the kernel that more data chunks will follow */
        if (send(op_fd, buf, n, MSG_MORE) < 0) {
            perror("send");
            close(op_fd);
            free(buf);
            return EXIT_FAILURE;
        }
    }

    if (n < 0) {
        perror("read");
        close(op_fd);
        free(buf);
        return EXIT_FAILURE;
    }

    /* 5. Finalize the hash operation by sending a zero-length message */
    if (send(op_fd, NULL, 0, 0) < 0) {
        perror("send finalize");
        close(op_fd);
        free(buf);
        return EXIT_FAILURE;
    }

    /* 6. Read the resulting digest */
    unsigned char hash[SHA1_DIGEST_SIZE];
    ssize_t hlen = recv(op_fd, hash, SHA1_DIGEST_SIZE, 0);
    if (hlen < 0) {
        perror("recv");
        close(op_fd);
        free(buf);
        return EXIT_FAILURE;
    }
    if (hlen != SHA1_DIGEST_SIZE) {
        fprintf(stderr, "Error: Expected %d digest bytes, got %zd\n", SHA1_DIGEST_SIZE, hlen);
        close(op_fd);
        free(buf);
        return EXIT_FAILURE;
    }

    /* 7. Print hex digest to stdout */
    for (int i = 0; i < SHA1_DIGEST_SIZE; i++) {
        printf("%02x", hash[i]);
    }
    printf("\n");

    close(op_fd);
    free(buf);
    return EXIT_SUCCESS;
}
