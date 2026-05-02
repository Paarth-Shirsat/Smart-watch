idf_component_register(SRCS "gc9a01.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_system)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)