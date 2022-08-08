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

#include "xinput.h"

#if !defined(CFG_TUSB_DEBUG)
#define CFG_TUSB_DEBUG CONFIG_USB_DEBUG_LEVEL
#endif
#include "tinyusb.h"
#include "device/usbd_pvt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <stdint.h>


static const char tag[] = "xinput";


static uint8_t endpoint_in = 0;
static uint8_t endpoint_out = 0;


void (*xinput_receive_feedback_cb)(struct xinput_feedback *feedback) = NULL;


static const tusb_desc_device_t device_desc = {
	.bLength            = sizeof(device_desc),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0200,  // bcdUSB 2.00
	.bDeviceClass       = 0xff,
	.bDeviceSubClass    = 0xff,
	.bDeviceProtocol    = 0xff,
	.bMaxPacketSize0    = 0x40,
	.idVendor           = 0x045e,  // Microsoft Corp.
	.idProduct          = 0x028e,  // Xbox360 Controller
	.bcdDevice          = 0x0114,
	.iManufacturer      = 0x01,    // string_desc[1]
	.iProduct           = 0x02,    // string_desc[2]
	.iSerialNumber      = 0x03,    // string_desc[3]
	.bNumConfigurations = 0x01,
};


static const char *string_desc[] = {
	(const char[]){0x09, 0x04}, // Supported Language: English
	"Generic",                  // Manufacturer
	"XInput Controller",        // Product
	"MXPAD-r4",                 // Serial Number
};


static const uint8_t xinput_desc[] = {
	0x09,        // bLength
	0x02,        // bDescriptorType (Configuration)
	0x30, 0x00,  // wTotalLength 48
	0x01,        // bNumInterfaces 1
	0x01,        // bConfigurationValue
	0x00,        // iConfiguration (String Index)
	0x80,        // bmAttributes
	0xFA,        // bMaxPower 500mA

	0x09,        // bLength
	0x04,        // bDescriptorType (Interface)
	0x00,        // bInterfaceNumber 0
	0x00,        // bAlternateSetting
	0x02,        // bNumEndpoints 2
	0xFF,        // bInterfaceClass
	0x5D,        // bInterfaceSubClass
	0x01,        // bInterfaceProtocol
	0x00,        // iInterface (String Index)

	0x10,        // bLength
	0x21,        // bDescriptorType (HID)
	0x10, 0x01,  // bcdHID 1.10
	0x01,        // bCountryCode
	0x24,        // bNumDescriptors
	0x81,        // bDescriptorType[0] (Unknown 0x81)
	0x14, 0x03,  // wDescriptorLength[0] 788
	0x00,        // bDescriptorType[1] (Unknown 0x00)
	0x03, 0x13,  // wDescriptorLength[1] 4867
	0x01,        // bDescriptorType[2] (Unknown 0x02)
	0x00, 0x03,  // wDescriptorLength[2] 768
	0x00,        // bDescriptorType[3] (Unknown 0x00)

	0x07,        // bLength
	0x05,        // bDescriptorType (Endpoint)
	0x81,        // bEndpointAddress (IN/D2H)
	0x03,        // bmAttributes (Interrupt)
	0x20, 0x00,  // wMaxPacketSize 32
	0x01,        // bInterval 1 (unit depends on device speed)

	0x07,        // bLength
	0x05,        // bDescriptorType (Endpoint)
	0x01,        // bEndpointAddress (OUT/H2D)
	0x03,        // bmAttributes (Interrupt)
	0x20, 0x00,  // wMaxPacketSize 32
	0x08,        // bInterval 8 (unit depends on device speed)
};


void xinput_loop(void *arg)
{
	ESP_LOGI(tag, "Starting USB Device...");
	const tinyusb_config_t tusb_cfg = {
		.device_descriptor = &device_desc,
		.string_descriptor = string_desc,
		.configuration_descriptor = xinput_desc,
	};

	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	while (1) {
		bool more = true;

		while (more && tud_ready() && usbd_edpt_ready(0, endpoint_out)) {
			struct xinput_feedback buf = {0};
			usbd_edpt_claim(0, endpoint_out);
			more = usbd_edpt_xfer(0, endpoint_out, (void *)(&buf), 20);
			usbd_edpt_release(0, endpoint_out);

			if (buf.size == 0) {
				ESP_LOGI(tag, "Received zero-length packet");
				continue;
			}

			ESP_LOGI(tag, "Received feedback (type=%hhu, size=%hhu)", buf.type, buf.size);

			if (xinput_receive_feedback_cb)
				xinput_receive_feedback_cb(&buf);
		}

		vTaskDelay(pdMS_TO_TICKS(16));
	}
}


void xinput_send_state(struct xinput_state *state)
{
	/* Do nothing unless ready. */
	if (!tud_ready() || !endpoint_in || !usbd_edpt_ready(0, endpoint_in))
		return;

	/* Make sure the header is sane. */
	state->type = 0x00;
	state->size = sizeof(*state);

	/* Try to wake up the host. */
	if (tud_suspended())
		tud_remote_wakeup();

	/* Transmit. */
	usbd_edpt_claim(0, endpoint_in);
	usbd_edpt_xfer(0, endpoint_in, (void *)(state), state->size);
	usbd_edpt_release(0, endpoint_in);
}


static void xinput_init(void)
{
	ESP_LOGI(tag, "Initializing...");
}


static void xinput_reset(uint8_t rhport)
{
	ESP_LOGW(tag, "Reset");
}


static uint16_t xinput_open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len)
{
	ESP_LOGI(tag, "Opening...");

	uint16_t drv_len = sizeof(*itf) + itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t) + 16;
	TU_ASSERT(drv_len <= max_len, 0);

	uint8_t found = 0;
	tusb_desc_endpoint_t const *desc = (void const *)tu_desc_next(itf);

	while (found < itf->bNumEndpoints) {
		tusb_desc_endpoint_t const *desc_ep = (void const *)desc;

		if (TUSB_DESC_ENDPOINT == tu_desc_type(desc_ep)) {
			TU_ASSERT(usbd_edpt_open(rhport, desc_ep));

			if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
				endpoint_in = desc_ep->bEndpointAddress;
				ESP_LOGI(tag, "endpoint_in = %hhu", endpoint_in);
			} else {
				endpoint_out = desc_ep->bEndpointAddress;
				ESP_LOGI(tag, "endpoint_out = %hhu", endpoint_out);
			}

			found += 1;
		}

		desc = (void const *)tu_desc_next(desc);
	}

	return drv_len;
}


static bool xinput_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
	ESP_LOGI(tag, "Control XFER (stage=%hhu)", stage);
	return true;
}


static bool xinput_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t len)
{
	if (ep_addr != endpoint_out)
		return true;

	ESP_LOGI(tag, "Incoming XFER (len=%lu, result=%u)", len, result);
	return true;
}


static usbd_class_driver_t const xinput_driver = {
#if CFG_TUSB_DEBUG >= 2
	.name             = "XInput",
#endif
	.init             = xinput_init,
	.reset            = xinput_reset,
	.open             = xinput_open,
	.control_xfer_cb  = xinput_control_xfer_cb,
	.xfer_cb          = xinput_xfer_cb,
};


/*
 * Called by TinyUSB to allow us to add drivers.
 */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
	ESP_LOGI(tag, "Registered Device Class Driver");
	*driver_count = 1;
	return &xinput_driver;
}
