#include <fcntl.h>
#include <linux/fb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define W	320
#define H	240
#define CX	(W / 2)
#define CY	(H / 2)
#define R_OUTER	100
#define R_TICK	95
#define R_HOUR	55
#define R_MIN	85
#define R_SEC	90

#define COLOR_BG	0x0000	/* black */
#define COLOR_FACE	0xFFFF	/* white */
#define COLOR_HOUR	0xFFFF	/* white */
#define COLOR_MIN	0xFFFF	/* white */
#define COLOR_SEC	0xF800	/* red */

/* sin/cos at 60 ticks (one per second/minute), 16.16 fixed.
 *   t = 0   -> 12 o'clock (up)        (sin=0, cos=-1)
 *   t = 15  -> 3 o'clock  (right)     (sin=+1, cos=0)
 *
 * Our screen y grows downward, so we map angle -> (x, y) as:
 *   x = cx + r * sin(theta)
 *   y = cy - r * cos(theta)
 * with theta = 2*pi*t/60.
 *
 * Stored: sin_q16[60], cos_q16[60]; values in [-65536, 65536].
 */
static const int32_t sin_q16[60] = {
	      0,    6850,   13626,   20252,   26656,   32768,
	  38521,   43852,   48703,   53020,   56756,   59870,
	  62328,   64104,   65177,   65536,   65177,   64104,
	  62328,   59870,   56756,   53020,   48703,   43852,
	  38521,   32768,   26656,   20252,   13626,    6850,
	      0,   -6850,  -13626,  -20252,  -26656,  -32768,
	 -38521,  -43852,  -48703,  -53020,  -56756,  -59870,
	 -62328,  -64104,  -65177,  -65536,  -65177,  -64104,
	 -62328,  -59870,  -56756,  -53020,  -48703,  -43852,
	 -38521,  -32768,  -26656,  -20252,  -13626,   -6850,
};
static const int32_t cos_q16[60] = {
	 -65536,  -65177,  -64104,  -62328,  -59870,  -56756,
	 -53020,  -48703,  -43852,  -38521,  -32768,  -26656,
	 -20252,  -13626,   -6850,       0,    6850,   13626,
	  20252,   26656,   32768,   38521,   43852,   48703,
	  53020,   56756,   59870,   62328,   64104,   65177,
	  65536,   65177,   64104,   62328,   59870,   56756,
	  53020,   48703,   43852,   38521,   32768,   26656,
	  20252,   13626,    6850,       0,   -6850,  -13626,
	 -20252,  -26656,  -32768,  -38521,  -43852,  -48703,
	 -53020,  -56756,  -59870,  -62328,  -64104,  -65177,
};

static uint16_t *fb;
static size_t fb_bytes;
static int fb_fd = -1;
static const char *fb_path = "/dev/fb0";

static inline void put_pixel(int x, int y, uint16_t c)
{
	if ((unsigned)x < W && (unsigned)y < H)
		fb[y * W + x] = c;
}

static void fill(uint16_t c)
{
	size_t n = W * H;
	for (size_t i = 0; i < n; i++)
		fb[i] = c;
}

static void draw_line(int x0, int y0, int x1, int y1, uint16_t c)
{
	int dx = x1 - x0, dy = y1 - y0;
	int sx = dx < 0 ? -1 : 1;
	int sy = dy < 0 ? -1 : 1;
	int adx = dx < 0 ? -dx : dx;
	int ady = dy < 0 ? -dy : dy;
	int err = (adx > ady ? adx : -ady) / 2;
	int e2;

	for (;;) {
		put_pixel(x0, y0, c);
		if (x0 == x1 && y0 == y1)
			break;
		e2 = err;
		if (e2 > -adx) { err -= ady; x0 += sx; }
		if (e2 <  ady) { err += adx; y0 += sy; }
	}
}

static void draw_hand(int t60, int r_inner, int r_outer, int thick,
		      uint16_t c)
{
	int sx = (sin_q16[t60] * r_outer) >> 16;
	int sy = (cos_q16[t60] * r_outer) >> 16;
	int ix = r_inner ? (sin_q16[t60] * r_inner) >> 16 : 0;
	int iy = r_inner ? (cos_q16[t60] * r_inner) >> 16 : 0;
	int i;

	for (i = -thick; i <= thick; i++)
		draw_line(CX + ix + i, CY + iy, CX + sx + i, CY + sy, c);
	for (i = -thick; i <= thick; i++)
		draw_line(CX + ix, CY + iy + i, CX + sx, CY + sy + i, c);
}

static void draw_circle(int cx, int cy, int r, uint16_t c)
{
	int x = r, y = 0, err = 0;
	while (x >= y) {
		put_pixel(cx + x, cy + y, c);
		put_pixel(cx + y, cy + x, c);
		put_pixel(cx - y, cy + x, c);
		put_pixel(cx - x, cy + y, c);
		put_pixel(cx - x, cy - y, c);
		put_pixel(cx - y, cy - x, c);
		put_pixel(cx + y, cy - x, c);
		put_pixel(cx + x, cy - y, c);
		y++;
		err += 1 + 2 * y;
		if (2 * (err - x) + 1 > 0) {
			x--;
			err += 1 - 2 * x;
		}
	}
}

static void draw_disc(int cx, int cy, int r, uint16_t c)
{
	for (int yy = -r; yy <= r; yy++)
		for (int xx = -r; xx <= r; xx++)
			if (xx * xx + yy * yy <= r * r)
				put_pixel(cx + xx, cy + yy, c);
}

static void draw_face(void)
{
	draw_circle(CX, CY, R_OUTER, COLOR_FACE);
	/* 12 hour ticks */
	for (int i = 0; i < 12; i++) {
		int t60 = i * 5;
		int sx_o = (sin_q16[t60] * R_OUTER) >> 16;
		int sy_o = (cos_q16[t60] * R_OUTER) >> 16;
		int sx_i = (sin_q16[t60] * R_TICK)  >> 16;
		int sy_i = (cos_q16[t60] * R_TICK)  >> 16;
		draw_line(CX + sx_i, CY + sy_i,
			  CX + sx_o, CY + sy_o, COLOR_FACE);
	}
}

static volatile sig_atomic_t stop;
static void on_signal(int sig) { (void)sig; stop = 1; }

static void cleanup(void)
{
	if (fb && fb != MAP_FAILED) {
		fill(COLOR_BG);
		munmap(fb, fb_bytes);
	}
	if (fb_fd >= 0)
		close(fb_fd);
}

int main(int argc, char *argv[])
{
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	struct sigaction sa = { .sa_handler = on_signal };

	if (argc > 1)
		fb_path = argv[1];

	sigaction(SIGINT,  &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	fb_fd = open(fb_path, O_RDWR);
	if (fb_fd < 0) {
		perror(fb_path);
		return 1;
	}

	if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0 ||
	    ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
		perror("FBIOGET_*SCREENINFO");
		return 1;
	}

	if (vinfo.xres != W || vinfo.yres != H || vinfo.bits_per_pixel != 16) {
		fprintf(stderr, "expected %dx%d@16bpp, got %ux%u@%ubpp\n",
			W, H, vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
		return 1;
	}

	fb_bytes = finfo.smem_len;
	fb = mmap(NULL, fb_bytes, PROT_READ | PROT_WRITE,
		  MAP_SHARED, fb_fd, 0);
	if (fb == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	fill(COLOR_BG);

	while (!stop) {
		time_t now = time(NULL);
		struct tm tm;
		localtime_r(&now, &tm);

		fill(COLOR_BG);
		draw_face();
		draw_hand((tm.tm_hour % 12) * 5 + tm.tm_min / 12, 0, R_HOUR, 1, COLOR_HOUR);
		draw_hand(tm.tm_min, 0, R_MIN, 1, COLOR_MIN);
		draw_hand(tm.tm_sec, 0, R_SEC, 0, COLOR_SEC);
		draw_disc(CX, CY, 3, COLOR_SEC);

		sleep(1);
	}

	cleanup();
	return 0;
}
