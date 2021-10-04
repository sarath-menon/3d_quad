#include "quadcopter.h"
#include <iostream>

/// Represents the quadcopter
void Quadcopter::dynamics(const float motor_commands[4], const float dt) {

  // Convert motor speed to body thrust, torques produced
  motor_speed_to_thrust_torque(motor_commands);

  // Dynamics of the quadcopter frame
  frame.dynamics(body_thrust_command_, body_torque_command_, dt);

  std::cout << "Motor commands in sim:" << motor_commands[0] << '\t'
            << motor_commands[1] << '\t' << motor_commands[2] << '\t'
            << motor_commands[3] << '\n';
  std::cout << "Body thrust command in sim:" << body_thrust_command_(2) << '\t'
            << "Body torque command in sim:" << body_torque_command_(0) << '\t'
            << body_torque_command_(1) << '\t' << body_torque_command_(2)
            << '\n';
}

/// Represents the quadcopter
void Quadcopter::dynamics_direct_thrust_torque(
    const matrix::Vector3f &body_thrust_command,
    const matrix::Vector3f &body_torque_command, const float dt) {

  // Set internal variables
  body_thrust_command_ = body_thrust_command;
  body_torque_command_ = body_torque_command;

  // Dynamics of the quadcopter frame
  frame.dynamics(body_thrust_command_, body_torque_command_, dt);

  std::cout << "Body thrust command in sim:" << body_thrust_command(2) << '\t'
            << "Body torque command in sim:" << body_torque_command(0) << '\t'
            << body_torque_command(1) << '\t' << body_torque_command(2) << '\n';
}
