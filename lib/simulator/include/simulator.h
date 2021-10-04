#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

/// Represents the quadcopter
class Simulator {

private:
  // Euler integration timestep
  float dt_ = 0;
  // Number of numerical integration steps
  float euler_steps_ = 0;
  // Activate fastdds pose publisher
  bool pose_pub_flag_ = false;
  // SImulation timestep in milliseconds
  int sim_time_;

public:
  // Loads the quadcopter properties from the yaml file
  void set_parameters(std::string path);

public:
  /// Getter function
  const float dt() const { return dt_; }
  /// Getter function
  const float euler_steps() const { return euler_steps_; }
  /// Getter function
  const int sim_time() const { return sim_time_; }
  /// Getter function
  const bool pose_pub_flag() const { return pose_pub_flag_; }
};
