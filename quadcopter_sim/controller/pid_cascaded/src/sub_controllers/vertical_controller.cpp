#include "pid_cascaded.h"
#include <iostream>

float PidCascadedController::altitude_controller(const Quadcopter &quad,
                                                 const float altitude_target,
                                                 const float dt) {
  // Compute error
  const float altitude_error = altitude_target - quad.position()(2);

  // Compute control input
  float thrust_command = z_pos_pid(altitude_error, k_p__z, k_i__z, k_d__z, dt);

  // Quadcopter Motors have a maximum and minimum speed limit
  thrust_command =
      limit(ff_thrust + thrust_command, quad.thrust_max(), quad.thrust_min());

  std::cout << "Altitude error:" << altitude_error << '\n';

  return thrust_command;
};
