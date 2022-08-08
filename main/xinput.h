/*
 * Copyright (C)  Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#pragma once

#include <stdint.h>


/*
 * State of the controller.
 */
struct xinput_state {
	uint8_t type;
	uint8_t size;

	union {
		uint16_t buttons;
		struct {
			uint8_t btn_up     : 1;
			uint8_t btn_down   : 1;
			uint8_t btn_left   : 1;
			uint8_t btn_right  : 1;
			uint8_t btn_start  : 1;
			uint8_t btn_select : 1;
			uint8_t btn_j1     : 1;
			uint8_t btn_j2     : 1;

			uint8_t btn_lb     : 1;
			uint8_t btn_rb     : 1;
			uint8_t btn_home   : 1;
			uint8_t btn_11     : 1;
			uint8_t btn_a      : 1;
			uint8_t btn_b      : 1;
			uint8_t btn_x      : 1;
			uint8_t btn_y      : 1;
		};
	};

	uint8_t lt, rt;
	int16_t lx, ly, rx, ry;

	uint8_t _pad1[6];
} __attribute__((__packed__));


enum xinput_feedback_type {
	XINPUT_RUMBLE = 0x00,
	XINPUT_LED = 0x01,
};


enum xinput_led {
	XINPUT_LED_OFF = 0,
	XINPUT_LED_BLINK,
	XINPUT_LED_FLASH1,
	XINPUT_LED_FLASH2,
	XINPUT_LED_FLASH3,
	XINPUT_LED_FLASH4,
	XINPUT_LED_JUST1,
	XINPUT_LED_JUST2,
	XINPUT_LED_JUST3,
	XINPUT_LED_JUST4,
	XINPUT_LED_SPIN,
	XINPUT_LED_BLINK_SLOW,
	XINPUT_LED_BLINK_FAST,
	XINPUT_LED_ALTERNATE,
};


/*
 * Return next state of the LED after the animation finishes.
 */
inline static enum xinput_led xinput_led_next(enum xinput_led led, enum xinput_led prev)
{
	switch (led) {
		case XINPUT_LED_OFF:
		case XINPUT_LED_BLINK:
		case XINPUT_LED_JUST1:
		case XINPUT_LED_JUST2:
		case XINPUT_LED_JUST3:
		case XINPUT_LED_JUST4:
		case XINPUT_LED_SPIN:
			return led;

		case XINPUT_LED_FLASH1:
			return XINPUT_LED_JUST1;

		case XINPUT_LED_FLASH2:
			return XINPUT_LED_JUST2;

		case XINPUT_LED_FLASH3:
			return XINPUT_LED_JUST3;

		case XINPUT_LED_FLASH4:
			return XINPUT_LED_JUST4;

		case XINPUT_LED_BLINK_SLOW:
		case XINPUT_LED_BLINK_FAST:
		case XINPUT_LED_ALTERNATE:
			return prev;
	}

	return XINPUT_LED_OFF;
}


/*
 * Feedback from host.
 */
struct xinput_feedback {
	enum xinput_feedback_type type : 8;
	uint8_t size;

	union {
		struct {
			uint8_t _pad1[1];
			uint8_t left;
			uint8_t right;
		} rumble;

		struct {
			enum xinput_led led : 8;
			uint8_t _pad2[2];
		};
	};

	uint8_t pad3[15];
} __attribute__((__packed__));


/*
 * XInput controller main loop.
 *
 * Takes exclusive control of the USB interface.
 * Make sure is does not clash with another device.
 */
void xinput_loop(void *arg);


/*
 * Send state of the controller to the host.
 */
void xinput_send_state(struct xinput_state *state);


/*
 * Set to be notified of incoming feedback messages.
 */
extern void (*xinput_receive_feedback_cb)(struct xinput_feedback *feedback);
