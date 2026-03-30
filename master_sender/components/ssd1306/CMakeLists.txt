set(component_srcs "ssd1306.c" "ssd1306_i2c_legacy.c" "ssd1306_spi.c")
set(component_requires "driver esp_driver_spi esp_driver_i2c esp_driver_gpio")


idf_component_register(SRCS "${component_srcs}" REQUIRES "${component_requires}" INCLUDE_DIRS ".")
