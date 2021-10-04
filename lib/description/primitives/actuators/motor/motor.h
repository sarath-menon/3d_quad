#pragma once

#include "utils.h"
#include <matrix/math.hpp>
using namespace matrix;

/// Represents a Motor
class Motor
{

public:
  /// Loads the Motor properties from the yaml file
  // Motor();

  // Variables
protected:
  // Mass and Inertia Matrix ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Mass of the Motor
  float mass_;

  /// Inertia matrix of the Motor
  matrix::SquareMatrix<float, 3> inertia_matrix_;

  // Relative Position and Orietnation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // /// Position of the Motor
  // matrix::Vector3<float> relative_position_;

  // /// Orientation of the Motor as Quaternion (q_0, q_x, q_y, q_z)
  // matrix::Quatf relative_orientation_;

  // Motor in RPM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  int speed_ = 0;

  // Other properties ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Maximum speed of the motor
  int max_speed_ = 0;

  // Minimum speed of the motor
  int min_speed_ = 0;

public:
  /// Have to be defined in the child class
  virtual void Dynamics(){};

  // Getter Functions
public:
  /// Getter function
  const float &mass() const { return mass_; }
  /// Getter function
  const matrix::SquareMatrix<float, 3> &inertia_matrix() const
  {
    return inertia_matrix_;
  }

  // /// Getter function
  // const matrix::Vector3<float> &relative_position() const
  // {
  //   return relative_position_;
  // }
  // /// Getter function
  // const matrix::Quatf &relative_orientation() const
  // {
  //   return relative_orientation_;
  // }

  /// Getter function
  const int &speed() const { return speed_; }
  /// Getter function
  const int &min_speed() const { return min_speed_; }
  /// Getter function
  const int &max_speed() const { return max_speed_; }

  /// Setter function
  void set_mass(float mass) { mass_ = mass; }

  /// Setter function
  void set_inertia_matrix(float data[3][3])
  {
    matrix::SquareMatrix<float, 3> inertia_matrix(data);
    inertia_matrix_ = inertia_matrix;
  }

  // /// Setter function
  // void set_relative_position(matrix::Vector3<float> relative_position) {
  //   relative_position_ = relative_position;
  // }
  // /// Setter function
  // void set_relative_orientation(matrix::Quatf relative_orientation) {
  //   relative_orientation_ = relative_orientation;
  // }
  /// Setter function
  void set_speed(int speed) { speed_ = speed; }
  /// Setter function
  void set_min_speed(int min_speed) { min_speed_ = min_speed; }
  /// Setter function
  void set_max_speed(int max_speed) { max_speed_ = max_speed; }
};
