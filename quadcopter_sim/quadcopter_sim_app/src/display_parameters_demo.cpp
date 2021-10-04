#include "quadcopter.h"
#include <iostream>

int main() {
  // Assume we are getting motor thrust commands from the low level controller
  // as an array of floats: [motor_1_speed,....., motor_4_speed]

  Quadcopter quad;
  quad.set_parameters();
  quad.set_initial_conditions();

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Display parameters
  ///////////////////////////////////////////////////////////////////////////////////////////

  std::cout << "Quadcopter Mass:" << quad.frame.mass() << '\n';

  std::cout << "Linear drag coefficients:" << quad.frame.linear_drag_coeff_x()
            << '\t' << quad.frame.linear_drag_coeff_y() << '\t'
            << quad.frame.linear_drag_coeff_z() << '\n';

  std::cout << "Length of Moment arm:" << quad.frame.moment_arm() << '\n';

  std::cout << "Maximum motor thrust:" << quad.motor[0].thrust_max() << '\n';

  std::cout << "Minimum motor thrust:" << quad.motor[0].thrust_min() << '\n';

  std::cout << "Max roll and pitch angles:" << quad.roll_pitch_max() << '\n';

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Display Initial Conditions
  ///////////////////////////////////////////////////////////////////////////////////////////

  std::cout << "Initial Position:" << quad.frame.position()(0) << '\t'
            << quad.frame.position()(1) << '\t' << quad.frame.position()(2)
            << '\n';

  std::cout << "Initial Orientation:" << quad.frame.orientation()(0) << '\t'
            << quad.frame.orientation()(1) << '\t'
            << quad.frame.orientation()(2) << '\t'
            << quad.frame.orientation()(3) << '\n';

  std::cout << "Initial Velocity:" << quad.frame.velocity()(0) << '\t'
            << quad.frame.velocity()(1) << '\t' << quad.frame.velocity()(2)
            << '\n';

  std::cout << "Initial Angular Velocity:" << quad.frame.angular_velocity()(0)
            << '\t' << quad.frame.angular_velocity()(1) << '\t'
            << quad.frame.angular_velocity()(2) << '\n';

  std::cout << "Motor velocity -> Thrust Constant:" << MotorPropellerPair::k_f()
            << '\n';

  return 0;
}
