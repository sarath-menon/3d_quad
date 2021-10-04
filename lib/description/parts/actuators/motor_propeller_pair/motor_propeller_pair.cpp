#include "motor_propeller_pair.h"

// BldcMotor::BldcMotor(std::string parameter_path) : Motor(parameter_path) {}

void MotorPropellerPair::dynamics(const float commanded_motor_speed) {
  // get motor thrust from motor speed here. For now, declsre variable
  float commanded_thrust;

  actual_thrust_dot_ =
      (1 / time_constant_) * (commanded_thrust - actual_thrust_);
}

float MotorPropellerPair::motor_speed_to_thrust_map(const float motor_command) {

  float commanded_thrust = motor_command * motor_command * k_f_;

  // For now, let actual thrust = commanded thryst
  return commanded_thrust;
}
