#pragma once
#include "QuadMotorCommandPubSubTypes.h"
#include "quadcopter_msgs/msgs/QuadMotorCommand.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::QuadMotorCommand msg;
} // namespace sub
