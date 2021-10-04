#pragma once
#include <matrix/math.hpp>

/// Represents the quadcopter
class Simulator {

private:
public:
  /// Loads the quadcopter properties from the yaml file
  // Simulator();

  // Variables
protected:
  // Geometrical Properties ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /// Linear drag coefficient
  // std::string which_numerical_integrator;

  /// Numerical Integrators
public:
  /// Euler forward Integrator
  float euler_forward_step(float x_k, float f_dot_x, float dt);
  matrix::Vector3f euler_forward_step(matrix::Vector3f x_k,
                                      matrix::Vector3f f_dot_x, float dt);
  matrix::Quatf euler_forward_step(matrix::Quatf x_k, matrix::Quatf f_dot_x,
                                   float dt);

  // float rungeKutta_step(float x_k, float f_dot_x, float dt);

public:
  // /// Getter function
  // float linear_drag_coeff() const { return linear_drag_coeff_; }

public:
  // /// Setter function
  // void set_linear_drag_coeff(float linear_drag_coeff)
  // {
  //     linear_drag_coeff_ = linear_drag_coeff;
  // }
};
