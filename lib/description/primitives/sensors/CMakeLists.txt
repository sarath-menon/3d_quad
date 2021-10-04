###############################################################################
# Library of primitives
###############################################################################

add_library(sensor_primitives_lib STATIC
distance_sensor/distance_sensor.cpp
)

target_include_directories(sensor_primitives_lib PUBLIC distance_sensor)

# target_link_libraries(sensor_primitives_lib PRIVATE
# helper_lib
# Eigen3::Eigen
# yaml-cpp
# )

