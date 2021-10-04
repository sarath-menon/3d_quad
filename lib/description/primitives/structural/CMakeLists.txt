###############################################################################
# Library of primitives
###############################################################################

add_library(structural_primitives_lib STATIC
rigidbody/rigidbody.cpp
softbody/softbody.cpp
)

target_include_directories(structural_primitives_lib PUBLIC rigidbody)
target_include_directories(structural_primitives_lib PUBLIC softbody)

target_link_libraries(structural_primitives_lib PUBLIC
helper_lib
)

