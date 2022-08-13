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

#include "registry.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"


static const char *tag = "registry";


static volatile bool initialized = false;
static nvs_handle_t handle;


void reg_init(void)
{
	if (initialized) {
		return;
	} else {
		initialized = true;
	}

	ESP_LOGI(tag, "Initialize NVS...");

	esp_err_t err = nvs_flash_init();

	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}

	ESP_ERROR_CHECK(err);

	ESP_LOGI(tag, "Open NVS registry handle...");
	ESP_ERROR_CHECK(nvs_open("registry", NVS_READWRITE, &handle));
}


void reg_set_int(const char *name, int value)
{
	reg_init();
	ESP_ERROR_CHECK(nvs_set_i32(handle, name, value));
	ESP_ERROR_CHECK(nvs_commit(handle));
}


int reg_get_int(const char *name, int dfl)
{
	reg_init();

	int32_t value = dfl;
	esp_err_t err = nvs_get_i32(handle, name, &value);

	if (ESP_OK == err || ESP_ERR_NVS_NOT_FOUND == err)
		return value;

	ESP_ERROR_CHECK(err);
	return value;
}
