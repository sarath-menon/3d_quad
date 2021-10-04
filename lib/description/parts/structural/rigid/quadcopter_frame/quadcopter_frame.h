#pragma once
#include "rigidbody.h"

/// Represents the quadcopter
class QuadcopterFrame : public RigidBody {

public:
  /// Loads the quadcopter properties from the yaml file
  QuadcopterFrame();

  // Variables
protected:
  // Geometrical Properties ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Linear drag coefficient
  matrix::Matrix3f linear_drag_coeff_;

  // /// Angular drag coefficient
  matrix::Matrix3f angular_drag_coeff_;

  /// Distance from the quadcopter's center of mass to the propellor
  float arm_length_ = 0;

  /// Layout matrix
  matrix::SquareMatrix<float, 4> layout_;

  // Variables for dynamics function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// velocity of rigid body
  matrix::Vector3<float> velocity_;

  // Absolute velocity
  matrix::Vector3<float> abs_velocity;

  /// velocity of rigid body
  matrix::Vector3<float> acceleration_;

  /// Orientation as Z-Y-X Euler angle
  matrix::Eulerf euler_orientation_;

  // /// 2nd Time derivative of rigid body orientation
  // matrix::Vector3<float> orientation_ddot_;

  /// Angular acceleration of the rigid body
  matrix::Vector3<float> angular_acceleration_;

  /// Rotation matrix - body to inertial frame
  matrix::Dcmf _R_OB;

  /// No need to computer inverse of inertia matrix in each function call
  matrix::SquareMatrix<float, 3> inertia_matrix_inverse_;

  // Constants
private:
  // Acceleration due to gravity
  const matrix::Vector3f gravity_acc = matrix::Vector3f(0, 0, 9.81);

public:
  /// Quadcopter Dynamics
  void dynamics(const matrix::Vector3<float> body_thrust,
                const matrix::Vector3<float> body_torque, const float dt);

  void euler_step(const float dt);

  // Rotation only simulation for tuning attitude controller
  void attitude_tune_euler_step(const float dt);

public:
  /// Getter function
  float linear_drag_coeff_x() const { return linear_drag_coeff_(0, 0); }
  float linear_drag_coeff_y() const { return linear_drag_coeff_(1, 1); }
  float linear_drag_coeff_z() const { return linear_drag_coeff_(2, 2); }

  /// Getter function
  float angular_drag_coeff_x() const { return angular_drag_coeff_(0, 0); }
  float angular_drag_coeff_y() const { return angular_drag_coeff_(0, 0); }
  float angular_drag_coeff_z() const { return angular_drag_coeff_(0, 0); }

  /// Getter function
  float arm_length() const { return arm_length_; }

  /// Getter function
  matrix::Eulerf euler_orientation() const { return euler_orientation_; }

  // /// Getter function
  // matrix::Vector3<float> position_ddot() const { return position_ddot_; }

  /// Getter function
  matrix::Vector3<float> acceleration() const { return acceleration_; }

  /// Getter function
  matrix::Vector3<float> angular_acceleration() const {
    return angular_acceleration_;
  }

public:
  void set_linear_drag_coeff(float data[3]) {
    matrix::Vector3f linear_drag_coeff(data);
    linear_drag_coeff_ = diag(linear_drag_coeff);
  }

  void set_angular_drag_coeff(float data[3]) {
    matrix::Vector3f angular_drag_coeff(data);
    angular_drag_coeff_ = diag(angular_drag_coeff);
  }

  void set_arm_length(float arm_length) { arm_length_ = arm_length; }

  void set_inertia_matrix_inverse() {
    inertia_matrix_inverse_(0, 0) = 1 / inertia_matrix_(0, 0);
    inertia_matrix_inverse_(1, 1) = 1 / inertia_matrix_(1, 1);
    inertia_matrix_inverse_(1, 1) = 1 / inertia_matrix_(1, 1);
  }

  void set_euler_orientation() {
    matrix::Eulerf euler_orientation(orientation_);
    euler_orientation_ = euler_orientation * (180 / M_PI);
  }
};