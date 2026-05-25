#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <string.h>
#include <errno.h>

int main(int argc, char *argv[]) {
    const char *device = "/dev/input/event0";

    if (argc > 1) {
        device = argv[1];
    }

    int fd = open(device, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open input device");
        return 1;
    }

    struct input_event ev;

    int x = 0, y = 0, z = 0;

    while (1) {
        ssize_t n = read(fd, &ev, sizeof(ev));

        if (n < (ssize_t)sizeof(ev)) {
            if (errno == EINTR)
                continue;
            perror("Read error");
            break;
        }

        if (ev.type == EV_ABS) {
            switch (ev.code) {
                case ABS_X:
                    x = ev.value;
                    break;
                case ABS_Y:
                    y = ev.value;
                    break;
                case ABS_Z:
                    z = ev.value;
                    break;
            }
        }

        // Emit output only when a full report is ready
        if (ev.type == EV_SYN && ev.code == SYN_REPORT) {
            printf("%d,%d,%d\n", x, y, z);
            fflush(stdout);
        }
    }

    close(fd);
    return 0;
}
