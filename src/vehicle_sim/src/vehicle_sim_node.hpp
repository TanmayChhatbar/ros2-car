#include <memory>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "Vehicle2D.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class Vehicle2DSim : public rclcpp::Node
{
public:
  Vehicle2DSim();

private:
  nav_msgs::msg::Odometry odom;
  sensor_msgs::msg::JointState joint_state;
  geometry_msgs::msg::TransformStamped transform;

  void getOdometry(const double X, const double Y, const double vx, const double vy, const double wz);
  void getTransforms(const double X, const double Y, const double Z);
  void getJointStates(const double steering_angle, const double wheel_rotations[4]);

  void publishToROS();
  void joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg);
  void timer_simulation();

  // ROS-related
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_sim_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::Time last_time_;
  rclcpp::Time time_now_;
  tf2::Quaternion q;

  // vehicle simulation
  double steering_input_ = 0.0;
  double throttle_input_ = 0.0;
  double brake_input_ = 0.0;
  double handbrake_input_ = 0.0;
  double dt = 0.001; // simulation time step
  Vehicle2D vehicle_;
  Vehicle2DData vehicle_data;
};

int main(int argc, char *argv[]);
