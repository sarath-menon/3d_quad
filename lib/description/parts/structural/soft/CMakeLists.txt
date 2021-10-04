# ###############################################################################
# # Soft Gripper
# ###############################################################################
add_library(soft_lib STATIC
gripper/gripper.cpp
)

target_include_directories(soft_lib PUBLIC 
gripper
)

target_link_libraries(soft_lib PRIVATE
primitives_lib
# Eigen3::Eigen
# helper_lib
# yaml-cpp
)


