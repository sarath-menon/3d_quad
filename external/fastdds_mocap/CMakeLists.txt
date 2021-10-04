cmake_minimum_required(VERSION 3.16)
project("mocap_subscriber" VERSION 1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

###############################################################################
# CMake build rules
###############################################################################
set(IS_TOP_LEVEL TRUE)
if(PROJECT_SOURCE_DIR)
    set(IS_TOP_LEVEL FALSE)
endif()


###############################################################################
# Load external packages
###############################################################################

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

###############################################################################
# Library for message types
###############################################################################

add_library(msg_lib
include/mocap_quadcopter.cxx
)

target_link_libraries(msg_lib
fastcdr 
fastrtps
)
###############################################################################
# Library for publishers, subscirbers and such
###############################################################################

add_library(mocap_fastdds_lib
include/mocap_quadcopterPubSubTypes.cxx
include/mocap_quadcopterPublisher.cxx
include/mocap_quadcopterSubscriber.cxx
)

target_link_libraries(mocap_fastdds_lib
fastcdr 
fastrtps
)

target_include_directories(mocap_fastdds_lib PUBLIC include)



