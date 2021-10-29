#include "quadcopter_x.h"

/// Represents the quadcopter
void QuadcopterX::euler_step(const float dt) {

  frame.set_state(position(), velocity(), orientation(), angular_velocity());
  frame.euler_step(dt);

  this->set_state(frame.position(), frame.velocity(), frame.orientation(),
                  frame.angular_velocity());
}
