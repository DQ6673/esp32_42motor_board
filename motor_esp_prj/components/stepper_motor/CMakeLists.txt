set(srcs "stepper_motor_encoder.c" "stepper_app.c")

set(includes ".")

set(requires    "driver"
                "speed_switch"
                "user_console"
                "user_nvs"
                "console"
                "fatfs"
                "nvs_flash"
                )


idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES ${requires}
                       )