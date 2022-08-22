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

#include "hid.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#include <stdint.h>


#define TUSB_DESC_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)


static const char *tag = "hid";


static const uint8_t hid_report_descriptor[] = {
	TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
};


static const uint8_t hid_configuration_descriptor[] = {
	/* Configuration number, interface count, string index, total length, attribute, power in mA */
	TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 250),

	// Interface, string index, boot, report desc len, EP In address, size & polling interval.
	TUD_HID_DESCRIPTOR(0, 0, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};


uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
	return hid_report_descriptor;
}


uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
	return 0;
}


void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}


void hid_loop(void *arg)
{
	ESP_LOGI(tag, "Starting USB Device...");

	const tinyusb_config_t tusb_cfg = {
		.configuration_descriptor = hid_configuration_descriptor,
	};

	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}


void hid_send_state(const struct xinput_state *state, const struct hid_input_map *map)
{
	/* Do nothing unless ready. */
	if (!tud_mounted())
		return;

	/* Try to wake up the host. */
	if (tud_suspended())
		tud_remote_wakeup();

	int idx = 0;
	uint8_t keycode[32] = {};

	if (state->btn_a && map->btn_a)
		keycode[idx++] = map->btn_a;

	if (state->btn_b && map->btn_b)
		keycode[idx++] = map->btn_b;

	if (state->btn_x && map->btn_x)
		keycode[idx++] = map->btn_x;

	if (state->btn_y && map->btn_y)
		keycode[idx++] = map->btn_y;

	if (state->btn_up && map->btn_up)
		keycode[idx++] = map->btn_up;

	if (state->btn_down && map->btn_down)
		keycode[idx++] = map->btn_down;

	if (state->btn_left && map->btn_left)
		keycode[idx++] = map->btn_left;

	if (state->btn_right && map->btn_right)
		keycode[idx++] = map->btn_right;

	if (state->btn_start && map->btn_start)
		keycode[idx++] = map->btn_start;

	if (state->btn_select && map->btn_select)
		keycode[idx++] = map->btn_select;

	if (state->btn_home && map->btn_home)
		keycode[idx++] = map->btn_home;

	if (state->lx >= 16383 && map->j1_right)
		keycode[idx++] = map->j1_right;
	else if (state->lx <= -16383 && map->j1_left)
		keycode[idx++] = map->j1_left;

	if (state->ly >= 16383 && map->j1_up)
		keycode[idx++] = map->j1_up;
	else if (state->ly <= -16383 && map->j1_down)
		keycode[idx++] = map->j1_down;

	tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
}
