/*
 * Copyright (C) 2022 Jan Hamal Dvořák <mordae@anilinux.org>
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
#include "xinput.h"
#include "class/hid/hid.h"


struct hid_input_map {
	uint8_t btn_up;
	uint8_t btn_down;
	uint8_t btn_left;
	uint8_t btn_right;
	uint8_t btn_start;
	uint8_t btn_select;
	uint8_t btn_j1;
	uint8_t btn_j2;
	uint8_t btn_lb;
	uint8_t btn_rb;
	uint8_t btn_home;
	uint8_t btn_11;
	uint8_t btn_a;
	uint8_t btn_b;
	uint8_t btn_x;
	uint8_t btn_y;
	uint8_t lt;
	uint8_t rt;
	uint8_t lx_left, lx_right;
	uint8_t ly_up, ly_down;
	uint8_t rx_left, rx_right;
	uint8_t ry_up, ry_down;
};


void hid_loop(void *arg);
void hid_send_state(const struct xinput_state *state, const struct hid_input_map *map);
