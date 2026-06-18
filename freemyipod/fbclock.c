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

#define RGB(r, g, b) ((r) << 16 | (g) << 8 | (b))

#define COLOR_BLACK	RGB(0x00, 0x00, 0x00)
#define COLOR_WHITE	RGB(0xff, 0xff, 0xff)
#define COLOR_RED	RGB(0xff, 0x00, 0x00)

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

struct fb_api {
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	uint8_t *buf;
};

static inline int min(int a, int b) { return a < b ? a : b; }
static inline int max(int a, int b) { return a > b ? a : b; }

static uint32_t rgb_to_fb(struct fb_api *fb, uint32_t rgb)
{
	uint32_t r = (rgb >> 16) & 0xFF;
	uint32_t g = (rgb >> 8) & 0xFF;
	uint32_t b = rgb & 0xFF;

	r >>= max(8 - fb->vinfo.red.length, 0);
	g >>= max(8 - fb->vinfo.green.length, 0);
	b >>= max(8 - fb->vinfo.blue.length, 0);

	return (r << fb->vinfo.red.offset) | (g << fb->vinfo.green.offset) | (b << fb->vinfo.blue.offset);
}

static void put_pixel(struct fb_api *fb, int x, int y, uint32_t color)
{
	if ((unsigned)x >= fb->vinfo.xres || (unsigned)y >= fb->vinfo.yres)
		return;

	unsigned int bpp = fb->vinfo.bits_per_pixel;

	uint8_t *p = fb->buf + y * fb->finfo.line_length + x * ((bpp + 7) / 8);

	if (bpp == 32)             *(uint32_t *)p = color;
	else if (bpp == 15 || bpp == 16) *(uint16_t *)p = color;
	else if (bpp == 24)        { p[0] = color; p[1] = color >> 8; p[2] = color >> 16; } /* Assumes LE */
	else if (bpp == 8)         *p = color;
}

static void fill(struct fb_api *fb, uint32_t color)
{
	unsigned int bpp = fb->vinfo.bits_per_pixel;

	for (int y = 0; y < fb->vinfo.yres; y++) {
		uint8_t *row = fb->buf + y * fb->finfo.line_length;

		if (bpp == 32) {
			uint32_t *p = (uint32_t *)row;
			for (int x = 0; x < fb->vinfo.xres; x++) p[x] = color;
		} else if (bpp == 15 || bpp == 16) {
			uint16_t *p = (uint16_t *)row;
			for (int x = 0; x < fb->vinfo.xres; x++) p[x] = color;
		} else if (bpp == 24) {
			for (int x = 0; x < fb->vinfo.xres; x++) {
				uint8_t *p = row + x * 3;
				p[0] = color; p[1] = color >> 8; p[2] = color >> 16;  /* Assumes LE */
			}
		} else if (bpp == 8) {
			memset(row, color, fb->vinfo.xres);
		}
	}
}

static void draw_line(struct fb_api *fb, int x0, int y0, int x1, int y1, uint32_t color)
{
	int dx = x1 - x0, dy = y1 - y0;
	int sx = dx < 0 ? -1 : 1;
	int sy = dy < 0 ? -1 : 1;
	int adx = dx < 0 ? -dx : dx;
	int ady = dy < 0 ? -dy : dy;
	int err = (adx > ady ? adx : -ady) / 2;
	int e2;

	for (;;) {
		put_pixel(fb, x0, y0, color);
		if (x0 == x1 && y0 == y1)
			break;
		e2 = err;
		if (e2 > -adx) { err -= ady; x0 += sx; }
		if (e2 <  ady) { err += adx; y0 += sy; }
	}
}

static void draw_circle(struct fb_api *fb, int cx, int cy, int r, uint32_t color)
{
	int x = r, y = 0, err = 0;
	while (x >= y) {
		put_pixel(fb, cx + x, cy + y, color);
		put_pixel(fb, cx + y, cy + x, color);
		put_pixel(fb, cx - y, cy + x, color);
		put_pixel(fb, cx - x, cy + y, color);
		put_pixel(fb, cx - x, cy - y, color);
		put_pixel(fb, cx - y, cy - x, color);
		put_pixel(fb, cx + y, cy - x, color);
		put_pixel(fb, cx + x, cy - y, color);
		y++;
		err += 1 + 2 * y;
		if (2 * (err - x) + 1 > 0) {
			x--;
			err += 1 - 2 * x;
		}
	}
}

static void draw_disc(struct fb_api *fb, int cx, int cy, int r, uint32_t color)
{
	for (int yy = -r; yy <= r; yy++)
		for (int xx = -r; xx <= r; xx++)
			if (xx * xx + yy * yy <= r * r)
				put_pixel(fb, cx + xx, cy + yy, color);
}

static void draw_hand(struct fb_api *fb, int cx, int cy, int t60, int r_inner, int r_outer, int thick,
		      uint32_t color)
{
	int sx = (sin_q16[t60] * r_outer) >> 16;
	int sy = (cos_q16[t60] * r_outer) >> 16;
	int ix = r_inner ? (sin_q16[t60] * r_inner) >> 16 : 0;
	int iy = r_inner ? (cos_q16[t60] * r_inner) >> 16 : 0;
	int i;

	for (i = -thick; i <= thick; i++)
		draw_line(fb, cx + ix + i, cy + iy, cx + sx + i, cy + sy, color);
	for (i = -thick; i <= thick; i++)
		draw_line(fb, cx + ix, cy + iy + i, cx + sx, cy + sy + i, color);

	draw_disc(fb, cx + sx, cy + sy, thick, color);

	if (r_inner > 0)
		draw_disc(fb, cx + ix, cy + iy, thick, color);
}

static void draw_face(struct fb_api *fb, int cx, int cy, int r_outer, int r_tick, uint32_t color)
{
	draw_circle(fb, cx, cy, r_outer, color);
	/* 12 hour ticks */
	for (int i = 0; i < 12; i++) {
		int t60 = i * 5;
		int sx_o = (sin_q16[t60] * r_outer) >> 16;
		int sy_o = (cos_q16[t60] * r_outer) >> 16;
		int sx_i = (sin_q16[t60] * r_tick)  >> 16;
		int sy_i = (cos_q16[t60] * r_tick)  >> 16;
		draw_line(fb, cx + sx_i, cy + sy_i, cx + sx_o, cy + sy_o, color);
	}
}

static volatile sig_atomic_t stop;
static void on_signal(int sig) { (void)sig; stop = 1; }

int main(int argc, char *argv[])
{
	struct sigaction sa = { .sa_handler = on_signal };
	struct fb_api fb;

	sigaction(SIGINT,  &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	const char *fb_path = argc > 1 ? argv[1] : "/dev/fb0";

	int fb_fd = open(fb_path, O_RDWR);
	if (fb_fd < 0) {
		perror(fb_path);
		return 1;
	}

	if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb.vinfo) < 0 ||
	    ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb.finfo) < 0) {
		perror("FBIOGET_*SCREENINFO");
		close(fb_fd);
		return 1;
	}

	unsigned int bpp = fb.vinfo.bits_per_pixel;
	if (bpp != 8 && bpp != 15 && bpp != 16 && bpp != 24 && bpp != 32) {
		fprintf(stderr, "Unsupported color depth: %u bpp\n", bpp);
		close(fb_fd);
		return 1;
	}

	int cx = fb.vinfo.xres / 2;
	int cy = fb.vinfo.yres / 2;
	int min_dim = min(fb.vinfo.xres, fb.vinfo.yres);

	int r_outer	= min_dim * 42 / 100;
	int r_tick	= min_dim * 40 / 100;
	int r_hour	= min_dim * 23 / 100;
	int r_min	= min_dim * 35 / 100;
	int r_sec	= min_dim * 38 / 100;

	int r_dot		= max(2, min_dim / 100);
	int hand_thick	= max(1, min_dim / 200);

	uint32_t color_bg = rgb_to_fb(&fb, COLOR_BLACK);
	uint32_t color_face = rgb_to_fb(&fb, COLOR_WHITE);
	uint32_t color_hour = color_face;
	uint32_t color_min  = color_face;
	uint32_t color_sec  = rgb_to_fb(&fb, COLOR_RED);

	fb.buf = mmap(NULL, fb.finfo.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
	if (fb.buf == MAP_FAILED) {
		perror("mmap");
		close(fb_fd);
		return 1;
	}

	while (!stop) {
		time_t now = time(NULL);
		struct tm tm;
		localtime_r(&now, &tm);

		fill(&fb, color_bg);
		draw_face(&fb, cx, cy, r_outer, r_tick, color_face);
		draw_hand(&fb, cx, cy, (tm.tm_hour % 12) * 5 + tm.tm_min / 12, 0, r_hour, hand_thick, color_hour);
		draw_hand(&fb, cx, cy, tm.tm_min, 0, r_min, hand_thick, color_min);
		draw_hand(&fb, cx, cy, tm.tm_sec, 0, r_sec, hand_thick, color_sec);
		draw_disc(&fb, cx, cy, r_dot, color_sec);

		sleep(1);
	}

	fill(&fb, color_bg);
	munmap(fb.buf, fb.finfo.smem_len);
	close(fb_fd);

	return 0;
}
