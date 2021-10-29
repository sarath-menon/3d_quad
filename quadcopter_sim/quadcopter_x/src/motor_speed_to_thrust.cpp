#include "quadcopter_x.h"
#include <iostream>

cpp_msg::ThrustTorqueCommand &
QuadcopterX::motor_speed_to_thrust_torque(const float motor_commands[4]) {

  // Get thrust corresponding to motor speed
  for (int i = 0; i < 4; i++) {
    propeller_thrusts[i] =
        motor[i].motor_speed_to_thrust_map(motor_commands[i]);
  }

  // Compute moment arm length
  const float moment_arm = frame.arm_length() / std::sqrtf(2);

  // Compute net thrust acting on quadcopter
  thrust_torque_cmd_.thrust = propeller_thrusts[0] + propeller_thrusts[1] +
                              propeller_thrusts[2] + propeller_thrusts[3];

  // Compute net torque acting on the quadcopter
  thrust_torque_cmd_.roll_torque =
      (propeller_thrusts[0] - propeller_thrusts[1] - propeller_thrusts[2] +
       propeller_thrusts[3]) *
      moment_arm;

  thrust_torque_cmd_.pitch_torque =
      (-propeller_thrusts[0] - propeller_thrusts[1] + propeller_thrusts[2] +
       propeller_thrusts[3]) *
      moment_arm;

  thrust_torque_cmd_.yaw_torque =
      (propeller_thrusts[0] - propeller_thrusts[1] + propeller_thrusts[2] -
       propeller_thrusts[3]) *
      motor[0].k_t();

  // Print values for debugging

  // std::cout << "Propeller thrusts in simulator:" << propeller_thrusts[0] <<
  // '\t'
  //           << propeller_thrusts[1] << '\t' << propeller_thrusts[2] << '\t'
  //           << propeller_thrusts[3] << '\n';

  // std::cout << "Simulator thrust command:" << body_thrust(2) << '\n';

  // std::cout << "Torques in simulator:" << body_torques(0) << '\t'
  //           << body_torques(1) << '\t' << body_torques(2) << '\n';

  return thrust_torque_cmd_;
}
