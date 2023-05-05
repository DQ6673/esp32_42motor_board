set(srcs "main.c")

set(includes ".")

set(requires    "driver"
                "ec11_encoder"           
                "stepper_motor"
                "freq_test"
                "speed_switch"
                "user_console"
                "user_nvs"
                )


idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES ${requires}
                       )