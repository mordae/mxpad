idf_component_register(
	SRCS
		"adc1.c"
		"colors.c"
		"main.c"
		"registry.c"
		"xinput.c"

	INCLUDE_DIRS
		"."

	PRIV_REQUIRES
		soc
		driver
		led_strip
		esp_adc
		tinyusb
		nvs_flash
)
