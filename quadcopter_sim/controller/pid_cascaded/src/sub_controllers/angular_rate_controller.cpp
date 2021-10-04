#include "pid_cascaded.h"

float PidCascadedController::roll_angle_controller(const Quadcopter &quad,
                                                   const float attitude_target,
                                                   const float dt) {
  const float angle_error = attitude_target - quad.frame.euler_orientation()(0);
  //   std::cout << "Roll angle error:" << angle_error << '\n';

  float torque_command =
      roll_angle_pid(angle_error, k_p__r, k_i__r, k_d__r, dt);

  torque_command =
      limit(torque_command, quad.roll_torque_max(), -quad.roll_torque_max());

  return torque_command;
};

float PidCascadedController::pitch_angle_controller(const Quadcopter &quad,
                                                    const float attitude_target,
                                                    const float dt) {
  const float angle_error = attitude_target - quad.frame.euler_orientation()(1);
  //   std::cout << "Roll angle error:" << angle_error << '\n';

  float torque_command =
      pitch_angle_pid(angle_error, k_p__p, k_i__p, k_d__p, dt);

  torque_command =
      limit(torque_command, quad.roll_torque_max(), -quad.roll_torque_max());

  return torque_command;
};