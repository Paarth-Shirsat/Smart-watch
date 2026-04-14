idf_component_register(SRCS "main.c"
                            "src/i2c_manager.c"
                            "src/bmp388.c"
                            "src/lsm6dso.c"
                            "src/max30102.c"
                            "src/max_m10s.c"
                    INCLUDE_DIRS "." "include")
