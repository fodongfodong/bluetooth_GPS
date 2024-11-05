idf_component_register(SRCS "wificonnect.c" "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver bt nvs_flash
                    )
