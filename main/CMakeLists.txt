idf_component_register(
    SRCS 
        "main.c"
        "motor_control.c"
        "led_handler.c"
        "command_handler.c"
        "ble_handler.c"
        "icm42688p_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_timer
        nvs_flash
        bt
        led_strip
)
