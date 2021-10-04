#pragma once

#include <matrix/math.hpp>

/// Represents a Soft body
class SoftBody {

private:
public:
  /// Loads the soft body properties from the yaml file
  // SoftBody(std::string parameter_path);

  // Variables
protected:
  // Mass and Inertia Matrix ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Mass of the soft body
  float mass_ = 0;

  /// Inertia matrix of the soft body
  matrix::SquareMatrix<float, 3> inertia_matrix_;

  // Position and Orietnation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Position of the soft body
  matrix::Vector3<float> position_;

  /// Orientation of the soft body as Quaternion (q_0, q_x, q_y, q_z)
  matrix::Quatf orientation_;

  // Time derivatives of Position and Orietnation
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Time derivative of soft body position
  matrix::Vector3<float> position_dot_;

  /// Time derivative of quaternion representing soft body  orientation
  matrix::Vector<float, 4> orientation_dot_;

public:
  /// Have to be defined in the child class
  virtual void Dynamics() = 0;

  // Getter Functions
public:
  /// Getter function
  // const float &mass() const { return mass_; }
  // /// Getter function
  // const Eigen::Matrix3d &inertia_matrix() const { return inertia_matrix_; }
  // /// Getter function
  // const Eigen::Vector3d &position() const { return position_; }
  // /// Getter function
  // const Eigen::Vector4d &orientation() const { return orientation_; }
  // /// Getter function
  // const Eigen::Vector3d &position_dot() const { return position_dot_; }
  // /// Getter function
  // const Eigen::Vector4d &orientation_dot() const { return orientation_dot_; }
};
