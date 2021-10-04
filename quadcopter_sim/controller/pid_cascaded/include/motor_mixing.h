#pragma once
#include "quadcopter.h"
#include <iostream>
#include <math.h>
#include <math_helper.h>
#include <matrix/math.hpp>

void motor_mixing(const Quadcopter &quad, float motor_commands[4],
                  const matrix::Vector3f body_thrust_command,
                  const matrix::Vector3f body_torque_command, const float k_f,
                  const float arm_length) {

  matrix::Vector<float, 4> body_thrust_torques;
  matrix::Vector<float, 4> motor_thrusts;

  body_thrust_torques(0) = body_thrust_command(2);
  body_thrust_torques(1) = body_torque_command(0);
  body_thrust_torques(2) = body_torque_command(1);
  body_thrust_torques(3) = body_torque_command(2);

  motor_thrusts = quad.mixer_matrix() * body_thrust_torques;

  std::cout << "Unclipped Propeller thrusts in Controller: f1:"
            << motor_thrusts(0) << "\tf2:" << motor_thrusts(1)
            << "\tf3:" << motor_thrusts(2) << "\tf4:" << motor_thrusts(3)
            << std::endl;

  for (int i = 0; i < 4; i++) {
    motor_thrusts(i) = limit(motor_thrusts(i), quad.motor[i].thrust_max(),
                             quad.motor[i].thrust_min());
    motor_commands[i] = sqrt(motor_thrusts(i) / k_f);
  }

  std::cout << "Clipped Propeller thrusts in Controller: f1:"
            << motor_thrusts(0) << "\tf2:" << motor_thrusts(1)
            << "\tf3:" << motor_thrusts(2) << "\tf4:" << motor_thrusts(3)
            << std::endl;

  std::cout << "Thrust command before  motor mixing:" << body_thrust_command(2)
            << '\n';
  std::cout << "Thrust command after motor mixing:"
            << motor_thrusts(0) + motor_thrusts(1) + motor_thrusts(2) +
                   motor_thrusts(3)
            << '\n';

  std::cout << "Torque commands before  motor mixing:" << body_torque_command(0)
            << '\t' << body_torque_command(1) << '\t' << body_torque_command(2)
            << '\n';
  std::cout << "Torque commands after motor mixing:"
            << (motor_thrusts(1) - motor_thrusts(3)) * arm_length << '\t'
            << (motor_thrusts(0) - motor_thrusts(2)) * arm_length << '\t'
            << "yaw torque formula" << '\n';

  // std::cout << "Motor commands:" << motor_commands[0] << '\t'
  //           << motor_commands[1] << '\t' << motor_commands[2] << '\t'
  //           << motor_commands[3] << '\n';
}
