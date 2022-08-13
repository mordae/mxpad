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

#include "adc1.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"


static const char *tag = "adc1";


static adc_oneshot_unit_handle_t adc1;


void adc1_init(void)
{
	ESP_LOGI(tag, "Configure ADC1...");
	adc_oneshot_unit_init_cfg_t adc1_config = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = false,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &adc1));
}


void adc1_enable_channel(int chan)
{
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_13,
		.atten = ADC_ATTEN_DB_11,
	};

	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, chan, &config));
}


int adc1_read(int chan)
{
	int value = 0, raw;

	for (int i = 0; i < 32; i++) {
		esp_err_t err = adc_oneshot_read(adc1, chan, &raw);

		if (err == ESP_ERR_TIMEOUT) {
			ESP_LOGW(tag, "Timeout reading ADC1 channel %i", chan);
			i--;
			vTaskDelay(1);
			continue;
		} else {
			ESP_ERROR_CHECK(err);
		}

		value += raw;
	}

	return value >> 5;
}
