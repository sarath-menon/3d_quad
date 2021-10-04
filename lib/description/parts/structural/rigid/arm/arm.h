#pragma once
#include "rigidbody.h"

/// Represents the swinging arm
class Arm : public RigidBody {

private:
public:
  /// Loads the arm properties from the yaml file
  // Arm(std::string parameter_path);

  // By default, everything is initialized to zero
protected:
  // Position and Orientation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Angle of the swinging arm with respect to the inertial frame
  double arm_angle_ = 0;

  // Time derivatives of Position and Orietnation
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Time derivative of the arm angle
  double arm_angle_dot_ = 0;

  // Geometrical Properties ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Length of the swinging arm
  float arm_length_ = 0;

public:
  /// Arm Dynamics
  virtual void Dynamics(){};

public:
  /// Getter function
  float arm_angle() const { return arm_angle_; }
  /// Getter function
  float arm_angle_dot() const { return arm_angle_dot_; }
  /// Getter function
  float arm_length() const { return arm_length_; }
};