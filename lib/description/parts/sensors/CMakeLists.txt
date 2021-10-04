# ###############################################################################
# # Library of sensors
# ###############################################################################

add_library(sensor_lib STATIC
imu/imu.cpp
)

target_include_directories(sensor_lib PUBLIC 
imu
)

target_link_libraries(sensor_lib INTERFACE
primitives_lib
)


