# ###############################################################################
# # Library for simulator
# ###########################################################

add_library(simulator_lib STATIC
src/simulator.cpp
)

target_include_directories(simulator_lib PUBLIC include)

# target_link_libraries(numerical_integrators PRIVATE
# primitives_lib
# )

