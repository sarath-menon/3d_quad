# ###############################################################################
# # Library of structural elements
# ###########################################################

add_library(structural_lib STATIC
arm/arm.cpp
quadcopter_frame/quadcopter_frame.cpp
)

target_include_directories(structural_lib PUBLIC 
arm
quadcopter_frame
)

target_link_libraries(structural_lib PRIVATE
primitives_lib
# Eigen3::Eigen
# helper_lib
# yaml-cpp
)

