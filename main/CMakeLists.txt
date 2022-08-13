idf_component_register(
	SRCS
		"main.c"
		"adc1.c"
		"colors.c"
		"xinput.c"

	INCLUDE_DIRS
		"."

	PRIV_REQUIRES
		soc
		driver
		led_strip
		esp_adc
		tinyusb
)
