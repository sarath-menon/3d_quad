#pragma once
#include "softbody.h"

/**
 * @brief Represents the swinging arm
 *
 * @param path_to_parameters Relative path to the parameters file
 * ### Example
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.cpp
 * RigidBody Arm("arm_parameters.yaml");
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

/// Represents the gripper
class Gripper : public SoftBody {

public:
  /// Loads the gripper properties from the yaml file
  // Gripper(std::string parameter_path);

protected:
  // By default, everything is initialized to zero
  // public:

  // Position and Orientation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Indicates the configuration of the gripper with a variable between 0
  /// and 1. 0
  // means fully closed and 1 means fully open
  double config_ = 0;

  // Time derivatives of Position and Orietnation
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Time derivative of gripper configuration
  double config_dot_ = 0;

public:
  /// Gripper Dynamics
  virtual void Dynamics(){};

public:
  /// Getter function
  float config() const { return config_; }
  /// Getter function
  float config_dot() const { return config_dot_; }
};