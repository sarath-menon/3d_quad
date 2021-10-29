#pragma once
#include "motor_propeller_pair.h"
#include "quadcopter_frame.h"
#include "rigidbody.h"
#include "safety_checks.h"
#include <math.h>
#include <matrix/math.hpp>
#include <random>
#include <string>
#include <yaml-cpp/yaml.h>
/// Represents the quadcopter
class QuadcopterPlus : public RigidBody {

public:
  // Load parts from the library
  QuadcopterFrame frame;
  MotorPropellerPair motor[4];

  // Private variables
private:
  // Thrust produced by each propeller
  float propeller_thrusts[4] = {0, 0, 0, 0};

  // // Net thrust acting on the quadcopter
  // matrix::Vector3f body_thrust;
  // // Net torques acting on the quadcopter
  // matrix::Vector3f body_torques;

  // Net thrust acting on the quadcopter
  matrix::Vector3f body_thrust_command_;
  // Net torques acting on the quadcopter
  matrix::Vector3f body_torque_command_;

  // Maximum thrust can be produced by the quadcopter
  float thrust_max_{};

  // Maximum thrust can be produced by the quadcopter
  float thrust_min_{};

  // Mixer matrix
  matrix::SquareMatrix<float, 4> mixer_matrix_;

  // Public function
public:
  /// Loads the quadcopter properties from the yaml file
  void set_parameters(const std::string &path);
  // Set initial conditions of the quadcopter
  void set_initial_conditions(const std::string &path);
  // Read sensor values
  void sensor_read();
  // // COnvert motor speed to thrust and torque exerted in quadcopter frame
  void motor_speed_to_thrust_torque(const float motor_commands[4]);
  // Quadcopter dynamics
  void dynamics(const float motor_commands[4]);
  // Quadcopter dynamics with direct body thryst, torque input
  void
  dynamics_direct_thrust_torque(const matrix::Vector3f &body_thrust_command,
                                const matrix::Vector3f &body_torque_command);
  // Quadcopter dynamics
  void euler_step(const float dt);
  // Rotation only simulation for tuning attitude controller
  void attitude_tune_euler_step(const float dt);

public:
  /// Getter function
  const float thrust_max() const { return thrust_max_; }
  /// Getter function
  const float thrust_min() const { return thrust_min_; }

  /// Getter function
  const matrix::SquareMatrix<float, 4> mixer_matrix() const {
    return mixer_matrix_;
  }
};
