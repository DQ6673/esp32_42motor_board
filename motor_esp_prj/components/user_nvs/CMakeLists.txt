set(srcs "user_nvs.c")

set(includes ".")

set(requires    "driver"
                "nvs_flash"
                )


idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES ${requires}
                       )