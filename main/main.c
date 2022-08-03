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

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"


static const char *tag = "main";


/* Configured ADC1. */
static adc_oneshot_unit_handle_t adc1;


#ifdef CONFIG_LED
/* Amount of ambient light. */
static float ambient_light = 1.0;
#endif


#ifdef CONFIG_LED
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
		float value = ambient_light * CONFIG_LED_VALUE / 100.0f;
		float saturation = CONFIG_LED_SATURATION / 100.0f;
		hsv2rgb(hue += CONFIG_LED_STEP_HUE / 10.0f, saturation, value, &r, &g, &b);
		ESP_ERROR_CHECK(led_strip_set_pixel(led, 0, r, g, b));
		ESP_ERROR_CHECK(led_strip_refresh(led));
		vTaskDelay(pdMS_TO_TICKS(CONFIG_LED_STEP_MS));
	}
}
#endif


#ifdef CONFIG_LIGHT
static void light_sensor_loop(void *arg)
{
	ESP_LOGI(tag, "Configure light sensor...");
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_13,
		.atten = ADC_ATTEN_DB_11,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, 0, &config));

	while (1) {
		int value;
		ESP_ERROR_CHECK(adc_oneshot_read(adc1, 0, &value));

		float amount = 0.1f + 0.9f * value / 8191.0f;

		ambient_light = 0.9f * ambient_light + 0.1f * amount;
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
#endif


static void button_loop(void *arg)
{
	gpio_config_t gpio = {
		.pin_bit_mask = BIT64(45),
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_INPUT,
		.pull_down_en = 0,
		.pull_up_en = 1,
	};
	ESP_ERROR_CHECK(gpio_config(&gpio));

	while (1) {
		int btn1 = !gpio_get_level(45);

		if (btn1) {
			ESP_LOGI(tag, "Buttons: btn1=%i", btn1);
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}


void app_main(void)
{
	ESP_LOGI(tag, "Configure ADC1...");
	adc_oneshot_unit_init_cfg_t adc1_config = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = false,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &adc1));

#ifdef CONFIG_LIGHT
	xTaskCreate(light_sensor_loop, "light_sensor_loop", 4096, NULL, 0, NULL);
#endif

#ifdef CONFIG_LED
	xTaskCreate(led_loop, "led_loop", 4096, NULL, 0, NULL);
#endif

	xTaskCreate(button_loop, "button_loop", 4096, NULL, 0, NULL);

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
