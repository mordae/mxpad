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

#include "colors.h"

#include <math.h>


void hsv2rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
	h = fmodf(h, 360.0f);
	s = s > 1.0f ? 1.0f : (s < 0.0f ? 0.0f : s);
	v = v > 1.0f ? 1.0f : (v < 0.0f ? 0.0f : v);

	float rgb_max = v * UINT8_MAX;
	float rgb_min = rgb_max * (1.0f - s);
	float i       = h / 60.0f;
	float diff    = fmodf(h, 60.0f);
	float rgb_adj = (rgb_max - rgb_min) * diff / 60;

	switch ((int)(i)) {
		case 0:
			*r = rgb_max;
			*g = rgb_min + rgb_adj;
			*b = rgb_min;
			break;
		case 1:
			*r = rgb_max - rgb_adj;
			*g = rgb_max;
			*b = rgb_min;
			break;
		case 2:
			*r = rgb_min;
			*g = rgb_max;
			*b = rgb_min + rgb_adj;
			break;
		case 3:
			*r = rgb_min;
			*g = rgb_max - rgb_adj;
			*b = rgb_max;
			break;
		case 4:
			*r = rgb_min + rgb_adj;
			*g = rgb_min;
			*b = rgb_max;
			break;
		default:
			*r = rgb_max;
			*g = rgb_min;
			*b = rgb_max - rgb_adj;
			break;
	}
}
