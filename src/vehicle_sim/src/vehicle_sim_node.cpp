#include "vehicle_sim_node.hpp"

Vehicle2DSim::Vehicle2DSim()
    : Node("vehicle_2d_sim")
{
  // inputs
  this->declare_parameter("vehicle_name", "tt02");
  rclcpp::Parameter vehicle_name = this->get_parameter("vehicle_name");

  // publisher for vehicle odometry and URDF transforms
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // joy subscription for control inputs
  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&Vehicle2DSim::joy_callback, this, std::placeholders::_1));

  // timer for simulation steps
  timer_sim_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&Vehicle2DSim::timer_simulation, this));

  // timer for publishing odometry
  timer_pub_ = this->create_wall_timer(
      std::chrono::duration<double>(0.01),
      std::bind(&Vehicle2DSim::publishToROS, this));

  // init time
  last_time_ = this->now();

  // init ROS data
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  joint_state.name = {"base_frl_joint", "frl_steering_joint", "frl_wheel_joint",
                      "base_frr_joint", "frr_steering_joint", "frr_wheel_joint",
                      "base_rrl_joint", "rrl_steering_joint", "rrl_wheel_joint",
                      "base_rrr_joint", "rrr_steering_joint", "rrr_wheel_joint"};
  

  // init vehicle
  const std::string package_share_directory = ament_index_cpp::get_package_share_directory("vehicle_sim");
  const std::string config_path = package_share_directory +
                                  "/../../../../src/vehicle_sim/src/configs/" +
                                  vehicle_name.value_to_string() +
                                  ".json";
  const Vehicle2DConfig config = Vehicle2DConfig::loadFromFile(config_path);
  vehicle_ = Vehicle2D(config);

  // log info
  RCLCPP_INFO(this->get_logger(), "Vehicle simulator started - listening to joystick inputs");
  RCLCPP_INFO(this->get_logger(), "Running simulation at %.1f Hz. Input frequency may be lower", 1.0 / dt);
}

void Vehicle2DSim::publishToROS()
{
  // get data
  double X, Y, Z, yaw;
  double vx, vy, wz;
  double steering_angle;
  double wheel_rotations[4];
  vehicle_data = vehicle_.getVehicle2DData();
  vehicle_data.getPosition(X, Y);
  vehicle_data.getOrientation(yaw);
  vehicle_data.getLinearVelocities(vx, vy);
  vehicle_data.getAngularVelocities(wz);
  vehicle_data.getWheelRotations(wheel_rotations);
  steering_angle = vehicle_data.getSteeringAngle();
  Z = vehicle_.getVehicle2DConfig().getZcg();

  // convert to ROS data
  time_now_ = this->now();
  q.setRPY(0, 0, yaw);
  getOdometry(X, Y, vx, vy, wz);
  getTransforms(X, Y, Z);
  getJointStates(steering_angle, wheel_rotations);

  // publish to ROS
  this->odom_publisher_->publish(odom);
  this->tf_broadcaster_->sendTransform(transform);
  this->joint_publisher_->publish(joint_state);
}

void Vehicle2DSim::getJointStates(const double steering_angle, const double wheel_rotations[4])
{
  joint_state.header.stamp = time_now_;
  joint_state.position = {0.0, steering_angle, wheel_rotations[0],
                          0.0, steering_angle, wheel_rotations[1],
                          0.0, 0.0, wheel_rotations[2],
                          0.0, 0.0, wheel_rotations[3]};
}

void Vehicle2DSim::getTransforms(const double X, const double Y, const double Z)
{
  // base link only
  transform.header.stamp = time_now_;
  transform.transform.translation.x = X;
  transform.transform.translation.y = Y;
  transform.transform.translation.z = Z;
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
}

void Vehicle2DSim::getOdometry(const double X, const double Y, const double vx, const double vy, const double wz)
{
  // publish odometry
  odom.header.stamp = time_now_;

  // pose data
  odom.pose.pose.position.x = X;
  odom.pose.pose.position.y = Y;
  odom.pose.pose.position.z = vehicle_.getVehicle2DConfig().getZcg();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  // twist data
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = wz;
}

void Vehicle2DSim::joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg)
{
  // get inputs
  steering_input_ = msg->axes.size() > 0 ? msg->axes[0] : 0.0;
  throttle_input_ = msg->axes.size() > 4 ? (-msg->axes[4] + 1) / 2 : 0.0;
  brake_input_ = msg->axes.size() > 5 ? (-msg->axes[5] + 1) / 2 : 0.0;
  handbrake_input_ = msg->buttons.size() > 0 ? msg->buttons[0] : 0.0;

  // TODO combine throttle and brake inputs, because not using brakes yet
  throttle_input_ = throttle_input_ - brake_input_;
  brake_input_ = 0.0;
}

void Vehicle2DSim::timer_simulation()
{
  rclcpp::Time current_time = this->now();

  uint16_t n_steps = (uint16_t)((current_time - last_time_).seconds() / dt);
  last_time_ = current_time;

  // call the stepSimulation function to catch up with ROS time
  for (uint16_t i = 0; i < n_steps; ++i)
  {
    vehicle_.stepSimulation(dt, steering_input_, throttle_input_, brake_input_);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle2DSim>());
  rclcpp::shutdown();
  return 0;
}
