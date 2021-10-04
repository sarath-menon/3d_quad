
add_library(controller_lib STATIC
src/pid_controllers/reduced_attitude_pid.cpp
src/pid_controllers/vertical_pid.cpp
src/pid_controllers/horizontal_pid.cpp
src/sub_controllers/horizontal_controller.cpp
src/sub_controllers/vertical_controller.cpp
src/sub_controllers/angular_rate_controller.cpp
)

target_include_directories(controller_lib PUBLIC include)

target_link_libraries(controller_lib PRIVATE
quadcopter_lib
primitives_lib
parts_lib
yaml-cpp
)

