idf_component_register(
	SRCS
		"main.c"
		"colors.c"

	INCLUDE_DIRS
		"."

	PRIV_REQUIRES
		"led_strip"
)
