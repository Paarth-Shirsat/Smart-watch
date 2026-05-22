cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
target_include_directories(app PRIVATE src)
project(smart_watch)

target_sources(app PRIVATE
    src/main.c
    src/events/periodic_events.c
    src/events/imu_events.c
    src/events/baro_events.c
    src/sensors/imu_sensor.c
    src/sensors/baro_sensor.c
)