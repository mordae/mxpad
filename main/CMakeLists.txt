idf_component_register(
	SRCS
		"main.c"
		"colors.c"
		"xinput.c"

	INCLUDE_DIRS
		"."

	PRIV_REQUIRES
		driver
		led_strip
		esp_adc
		tinyusb
)
