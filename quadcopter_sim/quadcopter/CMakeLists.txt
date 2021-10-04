

add_library(quadcopter_lib
src/description.cpp
src/initial_conditions.cpp
src/common.cpp
src/dynamics.cpp
src/euler_integration.cpp
src/sensing.cpp
)


target_include_directories(quadcopter_lib PUBLIC include)

target_link_libraries(quadcopter_lib  PRIVATE 
primitives_lib
parts_lib
helper_lib
)


target_link_libraries(quadcopter_lib  PRIVATE 
yaml-cpp
)