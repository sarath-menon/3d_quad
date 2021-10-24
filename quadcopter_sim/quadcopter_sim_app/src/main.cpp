#include "include_helper.h"

int main() {

  Quadcopter quad;
  Simulator sim;

  sim.set_parameters("quadcopter_sim/quadcopter_sim_app/parameters/"
                     "simulation_parameters.yaml");

  quad.set_parameters();
  quad.set_initial_conditions();

  // Create participant. Argument-> Domain id, QOS name
  DefaultParticipant dp(0, "quad_simulator_2d_qos");

  // Create mocap data publisher
  DDSPublisher mocap_pub(idl_msg::MocapPubSubType(), "mocap_pose",
                         dp.participant());
  mocap_pub.init();

  // Create motor command subscriber
  DDSSubscriber motor_sub(idl_msg::QuadMotorCommandPubSubType(),
                          "motor_commands", dp.participant());
  motor_sub.init();

  // Give time to match pub,sub. Important! Do not delete
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Start Simulation
  ///////////////////////////////////////////////////////////////////////////////////////////
  for (int i = 0; i < sim.euler_steps(); i++) {
    // Print simulation timestep
    std::cout << "Timestep:" << i + 1 << '\n';

    // Get system state
    quad.sensor_read();

    // Construct mocap message
    cpp_msg::Mocap mocap_msg;

    mocap_msg.header.id = "srl_quad_sim";
    mocap_msg.header.timestamp = i + 1;

    mocap_msg.pose.position.x = quad.position()(0);
    mocap_msg.pose.position.y = quad.position()(1);
    mocap_msg.pose.position.z = quad.position()(2);

    mocap_msg.pose.orientation_quat.w = quad.orientation()(0);
    mocap_msg.pose.orientation_quat.x = quad.orientation()(1);
    mocap_msg.pose.orientation_quat.y = quad.orientation()(2);
    mocap_msg.pose.orientation_quat.z = quad.orientation()(3);

    mocap_msg.pose.orientation_euler.roll = quad.euler_orientation()(0);
    mocap_msg.pose.orientation_euler.pitch = quad.euler_orientation()(1);
    mocap_msg.pose.orientation_euler.yaw = quad.euler_orientation()(2);

    // Send mocap message
    mocap_pub.publish(mocap_msg);

    { // wait for control command from subscriber
      std::unique_lock<std::mutex> lock(motor_sub.listener.m);
      motor_sub.listener.cv.wait(lock, [] { return sub::new_data; });

      // Reset flag when data received
      sub::new_data = false;
    }

    // // Insert delay for real time visualization
    // std::this_thread::sleep_for(std::chrono::milliseconds(sim.sim_time()));

    // Dynamics function that accepts bidy thrust, torque commands
    quad.dynamics(sub::msg.motorspeed);

    std::cout << "Position:" << quad.position()(0) << '\t' << quad.position()(1)
              << '\t' << quad.position()(2) << '\n';

    // Simulate using explicit Euler integration
    quad.euler_step(sim.dt());

    // Not working !!!!!   (Using simulator module)
    //////////////////////////////////////////////////////////////////////////////////
    // // Simulate one timestep
    // sim.simulate_step(quad.position(), quad.velocity(), quad.acceleration(),
    //                   quad.orientation(), quad.angular_velocity(),
    //                   quad.angular_acceleration());

    // quad.set_state(sim.position(), sim.velocity(), sim.orientation(),
    //                sim.angular_velocity());

    // Plot variables for debugging
    //////////////////////////////////////////////////////////////////////////////////

    std::cout << "Euler angle:" << quad.frame.euler_orientation()(0) << '\t'
              << quad.frame.euler_orientation()(1) << '\t'
              << quad.frame.euler_orientation()(2) << '\n';

    std::cout << '\n';
    //////////////////////////////////////////////////////////////////////////////////

    if (plot_flags::plot_enable) {
      // Set variables for plotting
      plot_var::x[i] = quad.position()(0);
      plot_var::y[i] = quad.position()(1);
      plot_var::z[i] = quad.position()(2);

      plot_var::roll_angle[i] = quad.frame.euler_orientation()(0);
      plot_var::pitch_angle[i] = quad.frame.euler_orientation()(1);
      plot_var::yaw_angle[i] = quad.frame.euler_orientation()(2);

      plot_var::t[i] = i * sim.dt();
    }
  }

  if (plot_flags::plot_enable) {
    // Initialize visualizer
    MyApp app;
    app.run();
  }

  return 0;
}
