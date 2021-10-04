#pragma once

#include "utils.h"
#include <matrix/math.hpp>

/// Represents a Rigid body
class RigidBody {
private:
public:
  /// Loads the rigid body properties from the yaml file
  // RigidBody(std::string parameter_path);
  // RigidBody(){};

  // Variables
protected:
  // Mass and Inertia Matrix ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Mass of the rigid body
  float mass_ = 0;

  // Inertia matrix of the rigid body

  matrix::SquareMatrix<float, 3> inertia_matrix_;
  // Position and Orietnation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Position of the rigid body
  matrix::Vector3<float> position_;

  /// Derivative of position represenation of the rigid body
  matrix::Vector3<float> position_dot_;

  /// Velocity of the rigid body
  matrix::Vector3<float> velocity_;

  /// Orientation of the rigid body as Quaternion (q_0, q_x, q_y, q_z)
  matrix::Quatf orientation_;

  // Time derivatives of Position and Orientnation
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Time derivative of quaternion representing rigid body orientation
  matrix::Quatf orientation_dot_;

  /// Angular position_dot of the rigid body
  matrix::Vector3<float> angular_velocity_;

public:
  /// Have to be defined in the child class
  virtual void dynamics(){};

  // Getter Functions
public:
  /// Getter function
  const float mass() const { return mass_; }
  /// Getter function
  const matrix::SquareMatrix<float, 3> &inertia_matrix() const {
    return inertia_matrix_;
  }
  /// Getter function
  const matrix::Vector3<float> &position() const { return position_; }
  /// Getter function
  const matrix::Quatf &orientation() const { return orientation_; }
  /// Getter function
  const matrix::Vector3<float> &position_dot() const { return position_dot_; }
  /// Getter function
  const matrix::Vector3<float> &velocity() const { return velocity_; }
  /// Getter function
  const matrix::Quatf &orientation_dot() const { return orientation_dot_; }
  /// Getter function
  const matrix::Vector3<float> &angular_velocity() const {
    return angular_velocity_;
  }

  /// Setter function
  void set_mass(float mass) { mass_ = mass; }

  void set_inertia_matrix(float data[3][3]) {
    matrix::SquareMatrix<float, 3> inertia_matrix(data);
    inertia_matrix_ = inertia_matrix;
  }
  /// Setter function
  void set_position(matrix::Vector3<float> position) { position_ = position; }

  /// Setter function
  void set_orientation(matrix::Quatf orientation) {
    orientation_ = orientation;
  }

  /// Setter function
  void set_position_dot(matrix::Vector3<float> position_dot) {
    position_dot_ = position_dot;
  }

  void set_velocity(matrix::Vector3<float> velocity) { velocity_ = velocity; }

  /// Setter function
  void set_orientation_dot(matrix::Quatf orientation_dot) {
    orientation_dot_ = orientation_dot;
  }
  /// Setter function
  void set_angular_velocity(matrix::Vector3<float> angular_velocity) {
    angular_velocity_ = angular_velocity;
  }
};
