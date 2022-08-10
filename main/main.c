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


#include "colors.h"
#include "xinput.h"

#include "led_strip.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include <string.h>


static const char *tag = "main";


/* Configured ADC1. */
static adc_oneshot_unit_handle_t adc1;


#if defined(CONFIG_LED)
static led_strip_handle_t led;

/* Amount of ambient light. */
static volatile float ambient_light = 1.0;

/* LED pattern from host. And the previous one. */
static volatile enum xinput_led led_pattern = XINPUT_LED_SPIN;
static volatile enum xinput_led led_pattern_prev = XINPUT_LED_OFF;
#endif


/* State of the controller. */
static struct xinput_state state = {0};
static struct xinput_state prev_state = {0};


#if defined(CONFIG_LIGHT) || defined(CONFIG_JOY_L) || defined(CONFIG_JOY_R)
static void configure_adc1_channel(int chan)
{
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_13,
		.atten = ADC_ATTEN_DB_11,
	};

	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, chan, &config));
}


static int read_adc1_channel(int chan)
{
	int value = 0, raw;

	for (int i = 0; i < 32; i++) {
		esp_err_t res = adc_oneshot_read(adc1, chan, &raw);

		if (res == ESP_ERR_TIMEOUT) {
			ESP_LOGW(tag, "Timeout reading ADC1 channel %i", chan);
			i--;
			vTaskDelay(chan + 1);
			continue;
		}

		value += raw;
	}

	return value >> 5;
}
#endif


/*
 * Check if the new state differs sufficiently from the previous state
 * and if it does, send the new state to the host.
 */
static void maybe_send(void)
{
	if (state.buttons != prev_state.buttons)
		goto send;

	if (abs(state.lx - prev_state.lx) >= 128)
		goto send;

	if (abs(state.ly - prev_state.ly) >= 128)
		goto send;

	if (abs(state.rx - prev_state.rx) >= 128)
		goto send;

	if (abs(state.ry - prev_state.ry) >= 128)
		goto send;

	return;

send:
	ESP_LOGI(tag, "State: %-6hi %-6hi | %-6hi %-6hi | %s%s%s%s%s%s%s%s%s%s%s%s%s",
	         state.lx, state.ly, state.rx, state.ry,
		 state.btn_a ? "A" : "",
		 state.btn_b ? "B" : "",
		 state.btn_x ? "X" : "",
		 state.btn_y ? "Y" : "",
		 state.btn_up ? "^" : "",
		 state.btn_down ? "v" : "",
		 state.btn_left ? "<" : "",
		 state.btn_right ? ">" : "",
		 state.btn_start ? "T" : "",
		 state.btn_select ? "E" : "",
		 state.btn_home ? "H" : "",
		 state.btn_j1 ? "1" : "",
		 state.btn_j2 ? "2" : "");

	memcpy(&prev_state, &state, sizeof(state));
	xinput_send_state(&state);
}


#if defined(CONFIG_LED)
static void led_xinput_feedback_cb(struct xinput_feedback *feedback)
{
	/* We only handle the LED. No rumble motors. */
	if (0x01 != feedback->type)
		return;

	led_pattern_prev = xinput_led_next(led_pattern, led_pattern_prev);
	led_pattern = feedback->led;
	ESP_LOGI(tag, "LED pattern=%u, prev=%u", led_pattern, led_pattern_prev);
}


static void led_set_hue(float hue)
{
	uint8_t r, g, b;
	float value = ambient_light * CONFIG_LED_VALUE / 100.0f;
	float saturation = CONFIG_LED_SATURATION / 100.0f;
	hsv2rgb(hue, saturation, value, &r, &g, &b);
	ESP_ERROR_CHECK(led_strip_set_pixel(led, 0, r, g, b));
	ESP_ERROR_CHECK(led_strip_refresh(led));
}


static void led_off(void)
{
	ESP_ERROR_CHECK(led_strip_set_pixel(led, 0, 0, 0, 0));
	ESP_ERROR_CHECK(led_strip_refresh(led));
}


static void led_loop(void *arg)
{
	ESP_LOGI(tag, "Configure LED...");
	led_strip_config_t config = {
		.strip_gpio_num = CONFIG_LED_GPIO,
		.max_leds = 1,
	};
	ESP_ERROR_CHECK(led_strip_new_rmt_device(&config, &led));

	float red = 0.0;
	float orange = 45.0;
	float yellow = 60.0;
	float green = 120.0;
	float turqoise = 180;
	float blue = 240.0;

	int i;

	void *state[] = {
		&&off, &&blink, &&flash1, &&flash2, &&flash3, &&flash4,
		&&just1, &&just2, &&just3, &&just4, &&spin,
		&&blink_slow, &&blink_fast, &&alternate,
	};

	goto *state[led_pattern];

next:
	led_pattern = xinput_led_next(led_pattern, led_pattern_prev);
	goto *state[led_pattern];

off:
	led_off();
	vTaskDelay(pdMS_TO_TICKS(100));
	goto next;

blink:
	led_set_hue(green);
	vTaskDelay(pdMS_TO_TICKS(500));
	led_set_hue(red);
	vTaskDelay(pdMS_TO_TICKS(500));
	goto next;

flash1:
	for (i = 0; i < 3; i++) {
		led_set_hue(green);
		vTaskDelay(pdMS_TO_TICKS(400));
		led_off();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	goto next;

just1:
	led_set_hue(green);
	vTaskDelay(pdMS_TO_TICKS(500));
	goto next;

flash2:
	for (i = 0; i < 3; i++) {
		led_set_hue(red);
		vTaskDelay(pdMS_TO_TICKS(400));
		led_off();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	goto next;

just2:
	led_set_hue(red);
	vTaskDelay(pdMS_TO_TICKS(500));
	goto next;

flash3:
	for (i = 0; i < 3; i++) {
		led_set_hue(blue);
		vTaskDelay(pdMS_TO_TICKS(400));
		led_off();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	goto next;

just3:
	led_set_hue(blue);
	vTaskDelay(pdMS_TO_TICKS(500));
	goto next;

flash4:
	for (i = 0; i < 3; i++) {
		led_set_hue(yellow);
		vTaskDelay(pdMS_TO_TICKS(400));
		led_off();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	goto next;

just4:
	led_set_hue(yellow);
	vTaskDelay(pdMS_TO_TICKS(500));
	goto next;

spin:
	for (i = 0; i < 360; i += 10) {
		led_set_hue(i);
		vTaskDelay(pdMS_TO_TICKS(30));
	}
	goto next;

blink_slow:
	for (i = 0; i < 5; i++) {
		led_set_hue(orange);
		vTaskDelay(pdMS_TO_TICKS(400));
		led_off();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	goto next;

blink_fast:
	for (i = 0; i < 5; i++) {
		led_set_hue(turqoise);
		vTaskDelay(pdMS_TO_TICKS(400));
		led_off();
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	goto next;

alternate:
	for (i = 0; i < 5; i++) {
		led_set_hue(orange);
		vTaskDelay(pdMS_TO_TICKS(500));
		led_set_hue(turqoise);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	goto next;
}
#endif


#if defined(CONFIG_LIGHT)
static void light_sensor_loop(void *arg)
{
	ESP_LOGI(tag, "Configure light sensor...");
	configure_adc1_channel(CONFIG_LIGHT_CHAN);

	while (1) {
		int value = read_adc1_channel(CONFIG_LIGHT_CHAN);

		float amount = 0.1f + 0.9f * value / 8191.0f;

		ambient_light = 0.9f * ambient_light + 0.1f * amount;
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
#endif


static void button_loop(void *arg)
{
	gpio_config_t gpio = {
		.pin_bit_mask = BIT64(CONFIG_BTN_A)
		              | BIT64(CONFIG_BTN_B)
		              | BIT64(CONFIG_BTN_X)
		              | BIT64(CONFIG_BTN_Y)
		              | BIT64(CONFIG_BTN_DOWN)
		              | BIT64(CONFIG_BTN_RIGHT)
		              | BIT64(CONFIG_BTN_LEFT)
		              | BIT64(CONFIG_BTN_UP)
		              | BIT64(CONFIG_BTN_START)
		              | BIT64(CONFIG_BTN_SELECT)
		              | BIT64(CONFIG_BTN_HOME)
#if defined(CONFIG_JOY_L_BTN)
			      | BIT64(CONFIG_JOY_L_BTN)
#endif
#if defined(CONFIG_JOY_R_BTN)
			      | BIT64(CONFIG_JOY_R_BTN)
#endif
			      ,
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_INPUT,
		.pull_down_en = 0,
		.pull_up_en = 1,
	};
	ESP_ERROR_CHECK(gpio_config(&gpio));

	while (1) {
		state.btn_a = !gpio_get_level(CONFIG_BTN_A);
		state.btn_b = !gpio_get_level(CONFIG_BTN_B);
		state.btn_x = !gpio_get_level(CONFIG_BTN_X);
		state.btn_y = !gpio_get_level(CONFIG_BTN_Y);
		state.btn_down = !gpio_get_level(CONFIG_BTN_DOWN);
		state.btn_right = !gpio_get_level(CONFIG_BTN_RIGHT);
		state.btn_left = !gpio_get_level(CONFIG_BTN_LEFT);
		state.btn_up = !gpio_get_level(CONFIG_BTN_UP);
		state.btn_start = !gpio_get_level(CONFIG_BTN_START);
		state.btn_select = !gpio_get_level(CONFIG_BTN_SELECT);
		state.btn_home = !gpio_get_level(CONFIG_BTN_HOME);

#if defined(CONFIG_JOY_L_BTN)
		state.btn_j1 = !gpio_get_level(CONFIG_JOY_L_BTN);
#else
		state.btn_j1 = 0;
#endif
#if defined(CONFIG_JOY_R_BTN)
		state.btn_j2 = !gpio_get_level(CONFIG_JOY_R_BTN);
#else
		state.btn_j2 = 0;
#endif

		maybe_send();
		vTaskDelay(pdMS_TO_TICKS(16));
	}
}


#if defined(CONFIG_JOY_L) || defined(CONFIG_JOY_R)
inline static int joy_apply_calib(int value, int calib)
{
	float min = INT16_MIN;
	float max = INT16_MAX;

	float fvalue = value;
	float fcalib = calib;

	fvalue -= fcalib;

	if (fvalue >= 0.0) {
		fvalue *= max / (max - fcalib);
	} else {
		fvalue *= min / (min - fcalib);
	}

	return fvalue;
}


static void joy_loop(void *arg)
{
	ESP_LOGI(tag, "Configure joysticks...");

#if defined(CONFIG_JOY_L)
	configure_adc1_channel(CONFIG_JOY_L_CHAN_X);
	configure_adc1_channel(CONFIG_JOY_L_CHAN_Y);
	int cal_lx = 0, cal_ly = 0;
#endif

#if defined(CONFIG_JOY_R)
	configure_adc1_channel(CONFIG_JOY_R_CHAN_X);
	configure_adc1_channel(CONFIG_JOY_R_CHAN_Y);
	int cal_rx = 0, cal_ry = 0;
#endif

	ESP_LOGI(tag, "Calibrate joysticks...");
	int calibration = 30;

	while (1) {
#if defined(CONFIG_JOY_L)
		int lx = -(read_adc1_channel(CONFIG_JOY_L_CHAN_X) - 4096) * 8;
		int ly = (read_adc1_channel(CONFIG_JOY_L_CHAN_Y) - 4096) * 8;

		if (calibration > 0) {
			cal_lx = (7 * cal_lx + lx) / 8;
			cal_ly = (7 * cal_ly + ly) / 8;
		} else {
			lx = joy_apply_calib(lx, cal_lx);
			ly = joy_apply_calib(ly, cal_ly);

			state.lx = (3 * state.lx + lx) / 4;
			state.ly = (3 * state.ly + ly) / 4;
		}
#endif

#if defined(CONFIG_JOY_R)
		int rx = -(read_adc1_channel(CONFIG_JOY_R_CHAN_X) - 4096) * 8;
		int ry = (read_adc1_channel(CONFIG_JOY_R_CHAN_Y) - 4096) * 8;

		if (calibration > 0) {
			cal_rx = (7 * cal_rx + rx) / 8;
			cal_ry = (7 * cal_ry + ry) / 8;
		} else {
			rx = joy_apply_calib(rx, cal_rx);
			ry = joy_apply_calib(ry, cal_ry);

			state.rx = (3 * state.rx + rx) / 4;
			state.ry = (3 * state.ry + ry) / 4;
		}
#endif

		if (calibration > 0) {
			calibration -= 1;
		} else if (calibration == 0) {
			calibration -= 1;
			ESP_LOGI(tag, "Joysticks calibrated.");
		} else {
			maybe_send();
		}

		vTaskDelay(pdMS_TO_TICKS(16));
	}
}
#endif


void app_main(void)
{
	ESP_LOGI(tag, "Configure ADC1...");
	adc_oneshot_unit_init_cfg_t adc1_config = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = false,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &adc1));

	ESP_LOGI(tag, "GPIO 21/26/33 hack...");
	gpio_config_t hack = {
		.pin_bit_mask = BIT64(21) | BIT64(26) | BIT64(33),
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_INPUT,
		.pull_down_en = 0,
		.pull_up_en = 0,
	};
	ESP_ERROR_CHECK(gpio_config(&hack));

#if defined(CONFIG_LIGHT)
	xTaskCreate(light_sensor_loop, "light_sensor_loop", 4096, NULL, 0, NULL);
#endif

#if defined(CONFIG_LED)
	xinput_receive_feedback_cb = led_xinput_feedback_cb;
	xTaskCreate(led_loop, "led_loop", 4096, NULL, 0, NULL);
#endif

#if defined(CONFIG_JOY_L) || defined(CONFIG_JOY_R)
	xTaskCreate(joy_loop, "joy_loop", 4096, NULL, 1, NULL);
#endif

	xTaskCreate(button_loop, "button_loop", 4096, NULL, 0, NULL);
	xTaskCreate(xinput_loop, "xinput_loop", 4096, NULL, 0, NULL);

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
