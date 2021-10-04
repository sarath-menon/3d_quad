###############################################################################
# Library of primitives
###############################################################################

add_library(actuator_primitives_lib STATIC
motor/motor.cpp
)

target_include_directories(actuator_primitives_lib PUBLIC motor)

target_link_libraries(actuator_primitives_lib PUBLIC
helper_lib
)

