#include "pid_cascaded.h"
#include <iostream>

float PidCascadedController::y_pos_controller(const Quadcopter &quad,
                                              const float y_pos_target,
                                              const float dt) {
  // Compute error
  const float y_pos_error = y_pos_target - quad.position()(1);

  std::cout << "Horizontal error:" << y_pos_error << '\n';

  // Compute required attitude
  float attitude_command =
      y_pos_pid(y_pos_error, k_p__y, k_i__y, k_d__y, dt) / 9.81;

  attitude_command =
      -limit(attitude_command, quad.roll_max(), -quad.roll_max());

  std::cout << "Attitude command:" << attitude_command << '\n';

  return attitude_command;
};

float PidCascadedController::x_pos_controller(const Quadcopter &quad,
                                              const float x_pos_target,
                                              const float dt) {
  // Compute error
  const float x_pos_error = x_pos_target - quad.position()(0);

  std::cout << "Horizontal error:" << x_pos_error << '\n';

  // Compute required attitude
  float attitude_command =
      x_pos_pid(x_pos_error, k_p__x, k_i__x, k_d__x, dt) / 9.81;

  attitude_command =
      limit(attitude_command, quad.pitch_max(), -quad.pitch_max());

  std::cout << "Attitude command:" << attitude_command << '\n';

  return attitude_command;
};