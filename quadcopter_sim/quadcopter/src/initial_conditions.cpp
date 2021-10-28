#include "quadcopter.h"

/// Represents the quadcopter
void Quadcopter::set_initial_conditions(const std::string &path) {
  // Safety check, see if file exists
  safety_checks::yaml_file_check(path);

  // Load YAML file containing quadcopter properties
  YAML::Node yaml_file = YAML::LoadFile(path);

  // Set initial position
  matrix::Vector3f intial_position;

  intial_position(0) = yaml_file["x"].as<float>();
  intial_position(1) = yaml_file["y"].as<float>();
  intial_position(2) = yaml_file["z"].as<float>();

  set_position(intial_position);

  // Set initial orientation
  matrix::Quatf intial_orientation;

  intial_orientation(0) = yaml_file["q_w"].as<float>();
  intial_orientation(1) = yaml_file["q_x"].as<float>();
  intial_orientation(2) = yaml_file["q_y"].as<float>();
  intial_orientation(3) = yaml_file["q_z"].as<float>();

  set_orientation(intial_orientation);

  // Set initial velocity
  matrix::Vector3f intial_velocity;

  intial_velocity(0) = yaml_file["x_dot"].as<float>();
  intial_velocity(1) = yaml_file["y_dot"].as<float>();
  intial_velocity(2) = yaml_file["z_dot"].as<float>();

  set_velocity(intial_velocity);

  // Set initial angular velocity
  matrix::Vector3f intial_angular_velocity;

  intial_angular_velocity(0) = yaml_file["omega_x"].as<float>();
  intial_angular_velocity(1) = yaml_file["omega_y"].as<float>();
  intial_angular_velocity(2) = yaml_file["omega_z"].as<float>();

  set_angular_velocity(intial_angular_velocity);
}
