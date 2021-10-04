# ###############################################################################
# # Library of parts
# ###############################################################################

add_library(parts_lib INTERFACE)

target_link_libraries(parts_lib INTERFACE
structural_lib
actuator_lib
sensor_lib
)

add_subdirectory(structural)
add_subdirectory(actuators)
add_subdirectory(sensors)