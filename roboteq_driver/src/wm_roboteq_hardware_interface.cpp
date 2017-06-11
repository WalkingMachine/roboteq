//
// Created by gortium on 27/04/17.
//
#include <stdio.h>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <wm_roboteq_hardware_interface/wm_roboteq_hardware_interface.h>


WMRoboteqHardwareInterface::WMRoboteqHardwareInterface()
    : joint_name_({"roboteq_join"}),
      joint_position_{0.0},
      joint_velocity_{0.0},
      joint_effort_{0.0},
      joint_velocity_command_{0.0} {}

bool WMRoboteqHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
    robot_hw_nh.getParam("joint_name", joint_name_[0]);

    // Connect and register the joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_name_[0], &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]));

    registerInterface(&joint_state_interface_);

    // Connect and register the joint velocity interface
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_name_[0]), &joint_velocity_command_[0]));

    registerInterface(&joint_velocity_interface_);

    // std::string port = "/dev/ttyUSB0";
    std::string port;
    int32_t baud = 115200;
    robot_hw_nh.getParam("port", port);
    robot_hw_nh.getParam("baud", baud);

    // Interface to motor controller.
    roboteq::Controller controller_(port.c_str(), baud);
    //
    while (!controller_.connected()) {
        controller_.connect();
        ROS_INFO("POST USED : %s", port.c_str());
        sleep(1);
    }
    controller_.getId();
    ROS_INFO("ID: %i", controller_.id);

    std::string ns = "/drive" + boost::lexical_cast<std::string>(controller_.id);

    std::string use_channels = "false";
    robot_hw_nh.getParam("use_channels", use_channels);
    if (use_channels == "true")
    {
      controller_.use_channels = true;
      // Setup channels.
      if (robot_hw_nh.hasParam("channels")) {
        XmlRpc::XmlRpcValue channel_namespaces;
        robot_hw_nh.getParam("channels", channel_namespaces);
        ROS_ASSERT(channel_namespaces.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < channel_namespaces.size(); ++i)
        {
          ROS_INFO("THERE IS A CHANNEL");
          ROS_ASSERT(channel_namespaces[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          controller_.addChannel(new roboteq::Channel(1 + i, ns, &controller_, controller_.id));
        }
      } else {
        ROS_INFO("NO CHANNEL, will use default /drive<ID> ");

        // Default configuration is a single channel in the node's namespace.
        controller_.addChannel(new roboteq::Channel(1, ns, &controller_, controller_.id));
      }
    }
    else {controller_.use_channels = false;}

    // Test connection.
    if (controller_.connected())
    {
        ROS_DEBUG("Connection to %s at %i baud is successful.", port.c_str(), baud);
    }
    else
    {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
      sleep(1);
    }
}

void WMRoboteqHardwareInterface::read()
{
    if (this->controller_.connected())
    {
        this->controller_.spinOnce();
        this->controller_.getFeedback(joint_velocity_[0]);
    }
    else
    {
        ROS_DEBUG("Problem connecting to serial device.");
        ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
        sleep(1);
    }
}

# TODO
void WMRoboteqHardwareInterface::write()
{
    // Reset command timeout.
    timeout_timer_.stop();
    timeout_timer_.start();

    // Update mode of motor driver. We send this on each command for redundancy against a
    // lost message, and the MBS script keeps track of changes and updates the control
    // constants accordingly.
    controller_->command << "VAR" << channel_num_ << static_cast<int>(command.mode) << controller_->send;

    if (command.mode == roboteq_msgs::Command::MODE_VELOCITY)
    {
        // Get a -1000 .. 1000 command as a proportion of the maximum RPM.
        int roboteq_velocity = to_rpm(command.setpoint) / max_rpm_ * 1000.0;
        ROS_DEBUG_STREAM("Commanding " << roboteq_velocity << " velocity to motor driver.");

        // Write mode and command to the motor driver.
        controller_->command << "G" << channel_num_ << roboteq_velocity << controller_->send;
    }
    else if (command.mode == roboteq_msgs::Command::MODE_POSITION)
    {
        // Convert the commanded position in rads to encoder ticks.
        int roboteq_position = to_encoder_ticks(command.setpoint);
        ROS_DEBUG_STREAM("Commanding " << roboteq_position << " position to motor driver.");

        // Write command to the motor driver.
        controller_->command << "P" << channel_num_ << roboteq_position << controller_->send;
    }
    else
    {
        ROS_WARN_STREAM("Command received with unknown mode number, dropping.");
    }

    controller_->flush();
    last_mode_ = command.mode;
}