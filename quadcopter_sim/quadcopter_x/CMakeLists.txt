

add_library(quadcopter_x_lib
src/description.cpp
src/initial_conditions.cpp
src/motor_speed_to_thrust.cpp
src/dynamics.cpp
src/euler_integration.cpp
src/sensing.cpp
)


target_include_directories(quadcopter_x_lib PUBLIC include)

target_link_libraries(quadcopter_x_lib  PRIVATE 
primitives_lib
parts_lib
comm_lib
)


target_link_libraries(quadcopter_x_lib  PRIVATE 
yaml-cpp
)