#pragma once
#include "motor_propeller_pair.h"
#include "quadcopter_frame.h"
#include "rigidbody.h"
#include <math.h>
#include <matrix/math.hpp>
#include <random>
#include <yaml-cpp/yaml.h>
/// Represents the quadcopter
class Quadcopter : public RigidBody {

public:
  // Load parts from the library
  QuadcopterFrame frame;
  MotorPropellerPair motor[4];

  // Private variables
private:
  // Maximum roll angle for quadcopter
  float roll_max_ = 0;

  // Maximum pitch angle for quadcopter
  float pitch_max_ = 0;

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
  float thrust_max_ = 0;

  // Maximum thrust can be produced by the quadcopter
  float thrust_min_ = 0;

  // Maximum torque that can be produced by the motors
  float roll_torque_max_ = 0;
  float pitch_torque_max_ = 0;

  // Mixer matrix
  matrix::SquareMatrix<float, 4> mixer_matrix_;

  // Public function
public:
  /// Loads the quadcopter properties from the yaml file
  void set_parameters();
  // Set initial conditions of the quadcopter
  void set_initial_conditions();
  // Read sensor values
  void sensor_read();
  // // COnvert motor speed to thrust and torque exerted in quadcopter frame
  void motor_speed_to_thrust_torque(const float motor_commands[4]);
  // Quadcopter dynamics
  void dynamics(const float motor_commands[4], const float dt);
  // Quadcopter dynamics with direct body thryst, torque input
  void
  dynamics_direct_thrust_torque(const matrix::Vector3f &body_thrust_command,
                                const matrix::Vector3f &body_torque_command,
                                const float dt);
  // Quadcopter dynamics
  void euler_step(const float dt);
  // Rotation only simulation for tuning attitude controller
  void attitude_tune_euler_step(const float dt);

public:
  /// Getter function
  const float roll_max() const { return roll_max_; }
  /// Getter function
  const float pitch_max() const { return pitch_max_; }
  /// Getter function
  const float thrust_max() const { return thrust_max_; }
  /// Getter function
  const float thrust_min() const { return thrust_min_; }
  /// Getter function
  const float roll_torque_max() const { return roll_torque_max_; }
  /// Getter function
  const float pitch_torque_max() const { return pitch_torque_max_; }
  /// Getter function
  const matrix::SquareMatrix<float, 4> mixer_matrix() const {
    return mixer_matrix_;
  }

public:
  /// Setter function
  void set_roll_max(float roll_max) { roll_max_ = roll_max; }
  /// Setter function
  void set_pitch_max(float pitch_max) { pitch_max_ = pitch_max; }
};
