#ifndef WM_ROBOTEQ_HARDWARE_INTERFACE_H
#define WM_ROBOTEQ_HARDWARE_INTERFACE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include <nav_msgs/Odometry.h>
#include "sensor_msgs/JointState.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"

#define PI  3.141592653

class WMRoboteqHardwareInterface : public hardware_interface::RobotHW
{
public:
  WMRoboteqHardwareInterface();
  ~WMRoboteqHardwareInterface();
  bool WMRoboteqHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  void encoder_callback();
  void read();
  void write();

private:

    hardware_interface::VelocityJointInterface joint_velocity_interface_;
    hardware_interface::JointStateInterface    joint_state_interface_;

    //double linearToAngular(const double &travel) const;

    //double angularToLinear(const double &angle) const;

    double joint_position_[NUM_JOINTS];
    double joint_effort_[NUM_JOINTS];
    double joint_velocity_command_[NUM_JOINTS];
    double joint_velocity_[NUM_JOINTS];
    std::string joint_name_[NUM_JOINTS];

    roboteq::Controller controller_;

    //double wheel_radius;
    double max_speed_;
    int pi_num_;

    ros::NodeHandle nh_;
};

#endif
