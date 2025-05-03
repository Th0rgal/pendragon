idf_component_register(SRCS "main.c"
                            "sensor_monitor.c"
                            "command_handler.c"
                            "motor_control.c"
                            "led_handler.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver led_strip nvs_flash)
