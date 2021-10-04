#include "quadcopter.h"

/// Represents the quadcopter
void Quadcopter::euler_step(const float dt) {
  frame.euler_step(dt);

  // Set quadcopter state = quadcopter frame state
  set_position(frame.position());
  set_velocity(frame.velocity());
  set_orientation(frame.orientation());
  set_angular_velocity(frame.angular_velocity());
}

/// Represents the quadcopter
void Quadcopter::attitude_tune_euler_step(const float dt) {
  frame.attitude_tune_euler_step(dt);

  // Set quadcopter state = quadcopter frame state
  set_position(frame.position());
  set_velocity(frame.velocity());
  set_orientation(frame.orientation());
  set_angular_velocity(frame.angular_velocity());
}
