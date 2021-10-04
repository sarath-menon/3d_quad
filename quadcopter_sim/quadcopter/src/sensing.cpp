#include "quadcopter.h"

/// Represents the quadcopter
void Quadcopter::sensor_read() {

  // Initialize random number generator
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d{0, 0.000};
}
