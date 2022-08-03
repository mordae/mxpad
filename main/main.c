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
#include "led_strip.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


static const char *tag = "main";


static void led_loop(void *arg)
{
	led_strip_handle_t led;

	ESP_LOGI(tag, "Configure LED...");
	led_strip_config_t config = {
		.strip_gpio_num = CONFIG_LED_GPIO,
		.max_leds = 1,
	};
	ESP_ERROR_CHECK(led_strip_new_rmt_device(&config, &led));

	float hue = 0;

	while (1) {
		uint8_t r, g, b;
		hsv2rgb(hue += CONFIG_LED_STEP_HUE / 10.0f, CONFIG_LED_SATURATION / 100.0f, CONFIG_LED_VALUE / 100.0f, &r, &g, &b);
		ESP_ERROR_CHECK(led_strip_set_pixel(led, 0, r, g, b));
		ESP_ERROR_CHECK(led_strip_refresh(led));
		vTaskDelay(pdMS_TO_TICKS(CONFIG_LED_STEP_MS));
	}
}


void app_main(void)
{
	xTaskCreate(led_loop, "led_loop", 4096, NULL, 0, NULL);

	while (1) {
		vTaskDelay(10000);
	}
}
