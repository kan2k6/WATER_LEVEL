idf_component_register(
    SRCS "water_level.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_uart
        esp_driver_spi
        fatfs
        sdmmc
        nvs_flash
        esp_timer
        json
        mqtt
        esp_netif
        esp_event
)