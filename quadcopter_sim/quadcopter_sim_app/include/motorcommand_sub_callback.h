#pragma once
#include "QuadMotorCommandPubSubTypes.h"
#include "default_subscriber.h"
#include "quadcopter_msgs/msgs/QuadMotorCommand.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::QuadMotorCommand msg;
bool new_data{false};
} // namespace sub

// Subscriber callback - gets executed when a sample is received
inline void DDSSubscriber::SubListener::on_data_available(
    eprosima::fastdds::dds::DataReader *reader) {
  eprosima::fastdds::dds::SampleInfo info;

  if (reader->take_next_sample(&sub::msg, &info) == ReturnCode_t::RETCODE_OK) {
    if (info.valid_data) {
      // std::cout << "Sample received, count=" << samples << std::endl;

      { // Protection against race condition using mutex
        std::lock_guard lock(m);
        sub::new_data = true;
      }
      cv.notify_one();
    }
  }
}