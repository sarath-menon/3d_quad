# ###############################################################################
# # Library of actuators
# ###############################################################################


add_library(actuator_lib STATIC
motor_propeller_pair/motor_propeller_pair.cpp
)

target_include_directories(actuator_lib PUBLIC 
motor_propeller_pair
)

target_link_libraries(actuator_lib PRIVATE
primitives_lib
# Eigen3::Eigen
# helper_lib
# yaml-cpp
)



