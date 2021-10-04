# ###############################################################################
# # Library of primitives
# ###############################################################################

add_library(primitives_lib INTERFACE)

target_link_libraries(primitives_lib INTERFACE
structural_primitives_lib
actuator_primitives_lib
sensor_primitives_lib
)

add_subdirectory(structural)
add_subdirectory(sensors)
add_subdirectory(actuators)
