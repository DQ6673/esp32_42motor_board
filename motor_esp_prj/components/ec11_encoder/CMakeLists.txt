set(srcs "ec11_encoder.c")

set(includes ".")

set(requires    "driver"
                "stepper_motor"
                )


idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES ${requires}
                       )