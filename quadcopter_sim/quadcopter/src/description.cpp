#include "quadcopter.h"

/// Represents the quadcopter
void Quadcopter::set_parameters() {

  // Load YAML file containing quadcopter properties
  YAML::Node yaml_file = YAML::LoadFile(
      "quadcopter_sim/quadcopter_sim_app/parameters/quad_properties.yaml");
  ///////////////////////////////////////////////////////////////////////////////////////////
  // Set frame parameters
  ///////////////////////////////////////////////////////////////////////////////////////////

  // Set frame mass
  frame.set_mass(yaml_file["mass"].as<float>());

  // Set moment arm
  frame.set_arm_length(yaml_file["arm_length"].as<float>());

  // Set inertia matrix
  float frame_inertia[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  frame_inertia[0][0] = yaml_file["inertia_xx"].as<float>();
  frame_inertia[1][1] = yaml_file["inertia_yy"].as<float>();
  frame_inertia[2][2] = yaml_file["inertia_zz"].as<float>();

  frame.set_inertia_matrix(frame_inertia);
  frame.set_inertia_matrix_inverse();

  // Set linear and angular drag coeffeicnts
  float linear_drag_coeff[3] = {0, 0, 0};
  float angular_drag_coeff[3] = {0, 0, 0};

  linear_drag_coeff[0] = yaml_file["linear_drag_coeff_x"].as<float>();
  linear_drag_coeff[1] = yaml_file["linear_drag_coeff_y"].as<float>();
  linear_drag_coeff[2] = yaml_file["linear_drag_coeff_z"].as<float>();

  angular_drag_coeff[0] = yaml_file["angular_drag_coeff_x"].as<float>();
  angular_drag_coeff[1] = yaml_file["angular_drag_coeff_y"].as<float>();
  angular_drag_coeff[2] = yaml_file["angular_drag_coeff_z"].as<float>();

  frame.set_linear_drag_coeff(linear_drag_coeff);
  frame.set_angular_drag_coeff(angular_drag_coeff);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Set motor parameters
  ///////////////////////////////////////////////////////////////////////////////////////////

  // Set motor mass
  for (int i = 0; i < 4; i++) {
    // motor[i].set_max_speed(yaml_file["arm_length"].as<float>());
    // motor[i].set_min_speed(yaml_file["arm_length"].as<float>());
    motor[i].set_thrust_max(yaml_file["thrust_max"].as<float>());
    motor[i].set_thrust_min(yaml_file["thrust_min"].as<float>());

    motor[i].set_time_constant(yaml_file["arm_length"].as<float>());

    motor[i].set_k_f(yaml_file["k_f"].as<float>());
    motor[i].set_k_t(yaml_file["k_t"].as<float>());
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Set general quadcopter parameters
  ///////////////////////////////////////////////////////////////////////////////////////////

  // Set maximum roll and pitch angle
  set_roll_max(yaml_file["roll_max"].as<float>());
  // Set maximum roll and pitch angle
  set_pitch_max(yaml_file["pitch_max"].as<float>());

  // Maximum thrust can be produced by the quadcopter
  thrust_max_ = motor[0].thrust_max() * 4;

  // Maximum thrust can be produced by the quadcopter
  thrust_min_ = motor[0].thrust_min() * 4;

  // Maximum torque that can be produced by the motors
  roll_torque_max_ =
      (motor[1].thrust_max() - motor[1].thrust_min()) * frame.arm_length();

  pitch_torque_max_ =
      (motor[0].thrust_max() - motor[0].thrust_min()) * frame.arm_length();

  // Thrust allocation matrix

  mixer_matrix_(0, 0) = 0.25;
  mixer_matrix_(0, 1) = 0.0;
  mixer_matrix_(0, 2) = 0.5 / frame.arm_length();
  mixer_matrix_(0, 3) = 0.25 / motor[0].k_t();

  mixer_matrix_(1, 0) = 0.25;
  mixer_matrix_(1, 1) = 0.5 / frame.arm_length();
  mixer_matrix_(1, 2) = 0.0;
  mixer_matrix_(1, 3) = -0.25 / motor[0].k_t();

  mixer_matrix_(2, 0) = 0.25;
  mixer_matrix_(2, 1) = 0.0;
  mixer_matrix_(2, 2) = -0.5 / frame.arm_length();
  mixer_matrix_(2, 3) = 0.25 / motor[0].k_t();

  mixer_matrix_(3, 0) = 0.25;
  mixer_matrix_(3, 1) = -0.5 / frame.arm_length();
  mixer_matrix_(3, 2) = 0.0;
  mixer_matrix_(3, 3) = -0.25 / motor[0].k_t();
}
