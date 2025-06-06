// SPDX-License-Identifier: GPL-2.0
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define WHEEL_BASE 0x3C200000
#define GPIO_BASE 0x3CF00000
#define GPIO_OFFSET_BITS 4

#define PCON(i)	   (*((volatile uint32_t*)(GPIO_BASE + ((i) << GPIO_OFFSET_BITS))))
#define PDAT(i)	   (*((volatile uint32_t*)(GPIO_BASE + 0x04 + ((i) << GPIO_OFFSET_BITS))))
#define PUNA(i)	   (*((volatile uint32_t*)(GPIO_BASE + 0x08 + ((i) << GPIO_OFFSET_BITS))))
#define PUNB(i)	   (*((volatile uint32_t*)(GPIO_BASE + 0x0c + ((i) << GPIO_OFFSET_BITS))))

#define CLK_BASE 0x3C500000
#define PWRCONEXT			   (*(volatile uint32_t*)(CLK_BASE + 0x40))

#define WHEEL00	  (*((volatile uint32_t*)(WHEEL_BASE)))
#define WHEEL04	  (*((volatile uint32_t*)(WHEEL_BASE + 0x04)))
#define WHEEL08	  (*((volatile uint32_t*)(WHEEL_BASE + 0x08)))
#define WHEEL0C	  (*((volatile uint32_t*)(WHEEL_BASE + 0x0C)))
#define WHEEL10	  (*((volatile uint32_t*)(WHEEL_BASE + 0x10)))
#define WHEELINT	 (*((volatile uint32_t*)(WHEEL_BASE + 0x14)))
#define WHEELRX	  (*((volatile uint32_t*)(WHEEL_BASE + 0x18)))
#define WHEELTX	  (*((volatile uint32_t*)(WHEEL_BASE + 0x1C)))

#define CLICKWHEEL_DATA   WHEELRX

#define PCON10				  PCON(10)	 /* Configures the pins of port 10 */
#define PDAT10				  PDAT(10)	 /* The data register for port 10 */
#define PCON15				  PCON(15)	 /* Configures the pins of port 15 */
#define PDAT15				  PDAT(15)	 /* Configures the pins of port 15 */
#define PUNK15				  PUNB(15)	 /* Unknown thing for port 15 */

/* we use the keycodes and translation is 1 to 1 */
#define R_SC		KEY_R
#define L_SC		KEY_L

#define UP_SC		KEY_M
#define LEFT_SC		KEY_W
#define RIGHT_SC	KEY_F
#define DOWN_SC		KEY_D

#define HOLD_SC		KEY_H
#define ACTION_SC	KEY_ENTER

#define EV_SCROLL		0x1000
#define EV_TAP		   0x2000
#define EV_PRESS		 0x4000
#define EV_RELEASE	   0x8000
#define EV_TOUCH		 0x0100
#define EV_LIFT		  0x0200
#define EV_MASK		  0xff00

#define BTN_ACTION	   0x0001
#define BTN_NEXT		 0x0002
#define BTN_PREVIOUS	 0x0004
#define BTN_PLAY		 0x0008
#define BTN_MENU		 0x0010
#define BTN_HOLD		 0x0020
#define BTN_MASK		 0x00ff

#define SCROLL_LEFT	  0x0080
#define SCROLL_RIGHT	 0x0000
#define SCROLL(dist) ((dist) < 0? (-(dist) & 0x7f) | SCROLL_LEFT : (dist) & 0x7f)
#define SCROLL_MASK	  0x007f

/*
 * We really want to restrict it to less-than-0x60,
 * but that's not a power of 2 so 0x5f doesn't work as mask.
 * We'll settle for 0x7f, since the val should never get
 * above 0x5f in hardware anyway.
 */
#define TAP(loc)		 ((loc) & 0x7f)
#define TOUCH(loc)	   ((loc) & 0x7f)
#define LIFT(loc)		((loc) & 0x7f)
#define TAP_MASK		 0x007f
#define TOUCH_MASK	   0x007f
#define LIFT_MASK		0x007f

static volatile int ikb_reading;
static volatile int ikb_opened;
static volatile unsigned short ikb_events[32];
static volatile unsigned ikb_ev_head, ikb_ev_tail;

static volatile unsigned ikb_pressed_at; /* jiffy value when user touched wheel, 0 if not touching. */
static volatile unsigned ikb_first_loc; /* location where user first touched wheel */
static volatile int ikb_current_scroll; /* current scroll distance */
static volatile unsigned ikb_buttons_pressed, ikb_buttons_pressed_new; /* mask of BTN_* values */

static DECLARE_WAIT_QUEUE_HEAD (ikb_read_wait);

static void ikb_push_event (unsigned ev)
{
	if (!ikb_opened) return;

	if ((ikb_ev_head+1 == ikb_ev_tail) || (ikb_ev_head == 31 && ikb_ev_tail == 0))
		printk (KERN_ERR "dropping event %08x\n", ev);

	ikb_events[ikb_ev_head++] = ev;
	ikb_ev_head &= 31;
	wake_up_interruptible (&ikb_read_wait);
}

/* Turn the counter into a scroll event. */
static void ikb_make_scroll_event (void)
{
	if (ikb_current_scroll) {
		ikb_push_event (EV_SCROLL | SCROLL(ikb_current_scroll));
		ikb_current_scroll = 0;
	}
}

static void ikb_scroll (int dir)
{
	ikb_current_scroll += dir;
//
//	while (dir > 0) {
//		handle_scancode (R_SC, 1);
//		handle_scancode (R_SC, 0);
//		dir--;
//	}
//
//	while (dir < 0) {
//		handle_scancode (L_SC, 1);
//		handle_scancode (L_SC, 0);
//		dir++;
//	}

	if (ikb_reading)
		ikb_make_scroll_event();
}

static void handle_scroll_wheel(int new_scroll, int was_hold, int reverse)
{
	static int prev_scroll = -1;
	static int scroll_state[4][4] = {
		{0, 1, -1, 0},
		{-1, 0, 0, 1},
		{1, 0, 0, -1},
		{0, -1, 1, 0}
	};

	if ( prev_scroll == -1 ) {
		prev_scroll = new_scroll;
	}
	else if (!was_hold) {
		switch (scroll_state[prev_scroll][new_scroll]) {
		case 1:
			if (reverse) {
				/* 'r' keypress */
				ikb_scroll (1);
			}
			else {
				/* 'l' keypress */
				ikb_scroll (-1);
			}
			break;
		case -1:
			if (reverse) {
				/* 'l' keypress */
				ikb_scroll (-1);
			}
			else {
				/* 'r' keypress */
				ikb_scroll (1);
			}
			break;
		default:
			/* only happens if we get out of sync */
			break;
		}
	}

	prev_scroll = new_scroll;
}

static void ikb_handle_button (int button, int press)
{
	int sc;

	if (press)
		ikb_buttons_pressed_new |= button;
	else
		ikb_buttons_pressed_new &= ~button;
//
//	/* Send the code to the TTY driver too */
//	switch (button) {
//	case BTN_ACTION:   sc = ACTION_SC; break;
//	case BTN_PREVIOUS: sc = LEFT_SC;   break;
//	case BTN_NEXT:	 sc = RIGHT_SC;  break;
//	case BTN_MENU:	 sc = UP_SC;	 break;
//	case BTN_PLAY:	 sc = DOWN_SC;   break;
//	case BTN_HOLD:	 sc = HOLD_SC;   break;
//	default:		   sc = 0;		 break;
//	}
//
//	if (sc)
//		handle_scancode (sc, press);
}

static void ikb_start_buttons (void)
{
	ikb_buttons_pressed_new = ikb_buttons_pressed;
}

static void ikb_finish_buttons (void)
{
	/* Pressed: */
	if (ikb_buttons_pressed_new & ~ikb_buttons_pressed) {
		ikb_push_event (EV_PRESS | (ikb_buttons_pressed_new & ~ikb_buttons_pressed));
	}
	/* Released: */
	if (ikb_buttons_pressed & ~ikb_buttons_pressed_new) {
		ikb_push_event (EV_RELEASE | (ikb_buttons_pressed & ~ikb_buttons_pressed_new));
	}

	ikb_buttons_pressed = ikb_buttons_pressed_new;
}

static ssize_t ikb_read (struct file *file, char *buf, size_t nbytes, loff_t *ppos)
{
	DECLARE_WAITQUEUE (wait, current);
	ssize_t retval = 0, count = 0;

	if (nbytes == 0) return 0;

	ikb_make_scroll_event();
	ikb_reading = 1;

	add_wait_queue (&ikb_read_wait, &wait);
	while (nbytes > 0) {
		int ev;

		set_current_state (TASK_INTERRUPTIBLE);

		if (ikb_ev_head == ikb_ev_tail) {
			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			if (signal_pending (current)) {
				retval = -ERESTARTSYS;
				break;
			}
			schedule();
			continue;
		}

		for (ev = ikb_ev_tail; ev != ikb_ev_head && nbytes >= 2; ikb_ev_tail++, ikb_ev_tail &= 31, ev = ikb_ev_tail) {
			put_user (ikb_events[ev], (unsigned short *)buf);
			count += 2;
			buf += 2;
			nbytes -= 2;
		}

		break; /* only read as much as we have */
	}

	ikb_reading = 0;
	set_current_state (TASK_RUNNING);
	remove_wait_queue (&ikb_read_wait, &wait);

	return (count? count : retval);
}

static unsigned int ikb_poll (struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait (file, &ikb_read_wait, wait);

	if (ikb_ev_head != ikb_ev_tail)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

/*
 * We allow multiple opens, even though multiple readers will compete for events,
 * since usually one reader is in wait() for the other to complete.
 * (It's no worse than a bog-standard TTY device.)
 */
static int ikb_open (struct inode *inode, struct file *file)
{
	ikb_opened++;
	return 0;
}

static int ikb_release (struct inode *inode, struct file *file)
{
	ikb_opened--;
	return 0;
}
static struct file_operations ikb_fops = {
	read:	   ikb_read,
	poll:	   ikb_poll,
	open:	   ikb_open,
	release:	ikb_release,
};

static struct miscdevice ikb_misc = { MISC_DYNAMIC_MINOR, "wheel", &ikb_fops };

static irqreturn_t wheel_irq(int irq, void *pw)
{
	int clickwheel_events = WHEELINT;
	unsigned reg, status;
	static int button_mask = 0;
	static int wheel_bits16_22 = -1;
	static int wheel_events = 0;
	int wheel_delta = 0;
	int wheel_delta_abs = 0;
	int wheel_value = 0;

	/* Clear interrupts */
	if (clickwheel_events & 4) WHEELINT = 4;
	if (clickwheel_events & 2) WHEELINT = 2;
	if (clickwheel_events & 1) WHEELINT = 1;

	ikb_start_buttons();
	udelay(50);
	status = CLICKWHEEL_DATA;

	if ((status & 0x800000ff) == 0x8000001a) {
		int new_button_mask = 0;
		int new_wheel_value = 0;

		/* NB: highest wheel = 0x5F, clockwise increases */
		new_wheel_value = ((status << 9) >> 25) & 0xff;

		if ((status & 0x100) != 0) {
			new_button_mask |= 0x1;	/* Action */
			if (!(button_mask & 0x1)) {
				ikb_handle_button (BTN_ACTION, 1);
			}
		}
		else if (button_mask & 0x1) {
			ikb_handle_button (BTN_ACTION, 0);
		}

		if ((status & 0x1000) != 0) {
			new_button_mask |= 0x10;	/* Menu */
			if (!(button_mask & 0x10)) {
				ikb_handle_button (BTN_MENU, 1);
			}
		}
		else if (button_mask & 0x10) {
			ikb_handle_button (BTN_MENU, 0);
		}

		if ((status & 0x800) != 0) {
			new_button_mask |= 0x8;	/* Play/Pause */
			if (!(button_mask & 0x8)) {
				ikb_handle_button (BTN_PLAY, 1);
			}
		}
		else if (button_mask & 0x8) {
			ikb_handle_button (BTN_PLAY, 0);
		}

		if ((status & 0x200) != 0) {
			new_button_mask |= 0x2;	/* Next */
			if (!(button_mask & 0x2)) {
				ikb_handle_button (BTN_NEXT, 1);
			}
		}
		else if (button_mask & 0x2) {
			ikb_handle_button (BTN_NEXT, 0);
		}

		if ((status & 0x400) != 0) {
			new_button_mask |= 0x4;	/* Prev */
			if (!(button_mask & 0x4)) {
				ikb_handle_button (BTN_PREVIOUS, 1);
			}
		}
		else if (button_mask & 0x4) {
			ikb_handle_button (BTN_PREVIOUS, 0);
		}

		if ((status & 0x40000000) != 0) {
			/* scroll wheel down */
			new_button_mask |= 0x20;

			if (wheel_bits16_22 != -1) {
				wheel_delta = new_wheel_value - wheel_bits16_22;
				wheel_delta_abs = wheel_delta < 0 ? -wheel_delta : wheel_delta;

				if (wheel_delta_abs > 48) {
					if (wheel_bits16_22 > new_wheel_value) {
						wheel_bits16_22 -= 96;
					}
					else {
						wheel_bits16_22 += 96;
					}
				}

				wheel_delta = new_wheel_value - wheel_bits16_22;

				ikb_scroll (wheel_delta);
			} else {
				ikb_pressed_at = jiffies;
				ikb_first_loc = new_wheel_value;
				ikb_push_event (EV_TOUCH | TOUCH(new_wheel_value));
			}
			wheel_bits16_22 = new_wheel_value;
		}
		else if (button_mask & 0x20) {
			/* scroll wheel up */
			ikb_push_event (EV_LIFT | LIFT(wheel_bits16_22));
			if ((ikb_current_scroll > -4) && (ikb_current_scroll < 4) &&
				(jiffies - ikb_pressed_at < HZ/5)) {
				ikb_push_event (EV_TAP | TAP(ikb_first_loc));
			}
			ikb_make_scroll_event();

			wheel_bits16_22 = -1;
			wheel_events = 0;
			ikb_pressed_at = 0;
			ikb_first_loc = 0;
		}

		button_mask = new_button_mask;
	}
	else if ((status & 0x800000FF) == 0x8000003A) {
		wheel_value = status & 0x800000FF;
	} else {
//		int v = (unsigned)status >> 4;
//		if ((v == 0xfffffff) || (v == 0x5555555) || (v == 0xaaaaaaa)) {
//			// this happens after a Hold switch release (status is then fffffffx, aaaaaaax, 5555555x)
//			opto_i2c_init();
//		}
	}

	ikb_finish_buttons();

	return IRQ_HANDLED;
}

struct wheel_dev {
	int irq;
};

static int wheel_probe(struct platform_device *pdev)
{
	struct wheel_dev *sdev = kzalloc(sizeof(struct wheel_dev), GFP_KERNEL);
	int retval;
	if (!sdev)
		return -ENOMEM;
	platform_set_drvdata(pdev, sdev);

	sdev->irq = platform_get_irq(pdev, 0);

	int ret = devm_request_irq(&pdev->dev, sdev->irq, wheel_irq,
				   IRQF_SHARED, dev_name(&pdev->dev), sdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot claim IRQ for wheel\n");
		misc_deregister(&ikb_misc);
		kfree(sdev);
		return ret;
	}

#ifdef DEBUG
	printk(KERN_DEBUG "WHEEL: init\n");
#endif

	PWRCONEXT &= ~1;
	PCON15 = (PCON15 & ~0xFFFF0000) | 0x22220000;
	PUNK15 = 0xF0;
	WHEEL08 = 0x3A980;
	WHEEL00 = 0x280000;
	WHEEL10 = 3;
	PCON10 = (PCON10 & ~0xFF0) | 0x10;
	PDAT10 |= 2;
	WHEELTX = 0x8000023A;
	WHEEL04 |= 1;
	PDAT10 &= ~2;

	retval = misc_register(&ikb_misc);
	if(retval < 0) {
		printk(KERN_INFO "WHEEL: misc_register failed\n");
		return retval;
	}

	return 0;
}

static int
wheel_remove(struct platform_device *pdev)
{
	struct wheel_device *sdev = platform_get_drvdata(pdev);
	misc_deregister(&ikb_misc);
	kfree(sdev);

	return 0;
}

static const struct of_device_id wheel_of_match_table[] = {
	{ .compatible = "apple,wheel" },
	{ },
};

MODULE_DEVICE_TABLE(of, wheel_of_match_table);

static struct platform_driver wheel_platform_driver = {
	.driver = {
		.name = "apple,wheel",
		.of_match_table = wheel_of_match_table,
	},
	.probe = wheel_probe,
	.remove = wheel_remove,
};

module_platform_driver(wheel_platform_driver);
MODULE_LICENSE("GPL v2");
