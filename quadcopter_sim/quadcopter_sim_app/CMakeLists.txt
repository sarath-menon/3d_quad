

add_executable(quad_3d
src/main.cpp
src/plot.cpp
)

target_include_directories(quad_3d PUBLIC include)

target_link_libraries(quad_3d  PRIVATE 
comm_lib
quadcopter_lib
primitives_lib
parts_lib
# helper_lib
simulator_lib
)


target_link_libraries(quad_3d  PRIVATE 
yaml-cpp
mahi::gui
fastcdr 
fastrtps
)