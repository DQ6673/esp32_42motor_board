set(srcs "user_console.c")

set(includes ".")

set(requires    "driver"
                "console"
                "stepper_motor"
                "fatfs"
                )


idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES ${requires}
                       )