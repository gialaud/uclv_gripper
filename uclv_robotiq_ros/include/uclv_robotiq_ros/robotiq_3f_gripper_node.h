#ifndef UCLV_ROBOTIQ_ROS__ROBOTIQ_3F_GRIPPER_NODE_H
#define UCLV_ROBOTIQ_ROS__ROBOTIQ_3F_GRIPPER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uclv_robotiq_interfaces/msg/gripper_status.hpp"
#include "uclv_robotiq_interfaces/msg/finger_command_array.hpp"
#include "uclv_robotiq/robotiq_3f_gripper.h"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_robotiq_interfaces/action/change_mode.hpp"

// namespace uclv
// {
//     class Robotiq3fGripperROS
//     {
//     public:
//         Robotiq3fGripperROS(const std::shared_ptr<Robotiq3fGripper> &gripper);

//         ~Robotiq3fGripperROS();

//         void init();
//         void start();
//         void spin();

//     private:
//         void _changeModeExecuteCallback(const uclv_robotiq_interfaces::action::ChangeMode::Goal::ConstSharedPtr &goal);
//         void _fingersCommandCallback(const uclv_robotiq_interfaces::msg::FingerCommandArray::ConstSharedPtr& msg);
//         bool _activateCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
//         bool _resetCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
//         bool _stopCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
//         bool _openCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
//         bool _closeCallback(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);

//     private:
//         std::shared_ptr<Robotiq3fGripper> _gripper;
//     };
// } // namespace uclv

#endif // UCLV_ROBOTIQ_ROS__ROBOTIQ_3F_GRIPPER_NODE_H