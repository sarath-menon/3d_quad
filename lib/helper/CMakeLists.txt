###############################################################################
# Library of helper functions
###############################################################################

add_library(helper_lib STATIC
src/utils.cpp
)

target_include_directories(helper_lib PUBLIC include)

