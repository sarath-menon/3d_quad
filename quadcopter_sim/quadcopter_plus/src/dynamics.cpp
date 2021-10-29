#include "quadcopter_plus.h"
#include <iostream>

/// Represents the quadcopter
void QuadcopterPlus::dynamics(const float motor_commands[4]) {

  // Convert motor speed to body thrust, torques produced
  motor_speed_to_thrust_torque(motor_commands);

  frame.set_state(position_, velocity_, orientation_, angular_velocity_);

  // Dynamics of the quadcopter frame
  frame.dynamics(body_thrust_command_, body_torque_command_);

  // acceleration, angular acceleration from frame dynamics
  acceleration_ = frame.acceleration();
  angular_acceleration_ = frame.angular_acceleration();

  // std::cout << "Motor commands in sim:" << motor_commands[0] << '\t'
  //           << motor_commands[1] << '\t' << motor_commands[2] << '\t'
  //           << motor_commands[3] << '\n';
  // std::cout << "Body thrust command in sim:" << body_thrust_command_(2) <<
  // '\t'
  //           << "Body torque command in sim:" << body_torque_command_(0) <<
  //           '\t'
  //           << body_torque_command_(1) << '\t' << body_torque_command_(2)
  //           << '\n';
}

/// Represents the quadcopter
void QuadcopterPlus::dynamics_direct_thrust_torque(
    const matrix::Vector3f &body_thrust_command,
    const matrix::Vector3f &body_torque_command) {

  // Set internal variables
  body_thrust_command_ = body_thrust_command;
  body_torque_command_ = body_torque_command;

  frame.set_state(position_, velocity_, orientation_, angular_velocity_);

  // Dynamics of the quadcopter frame
  frame.dynamics(body_thrust_command_, body_torque_command_);

  // acceleration, angular acceleration from frame dynamics
  acceleration_ = frame.acceleration();
  angular_acceleration_ = frame.angular_acceleration();

  // std::cout << "Body thrust command in sim:" << body_thrust_command(2) <<
  // '\t'
  //           << "Body torque command in sim:" << body_torque_command(0) <<
  //           '\t'
  //           << body_torque_command(1) << '\t' << body_torque_command(2) <<
  //           '\n';
}
