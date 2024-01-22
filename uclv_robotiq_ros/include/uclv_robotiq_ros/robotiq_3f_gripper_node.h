#ifndef UCLV_ROBOTIQ_ROS__ROBOTIQ_3F_GRIPPER_NODE_H
#define UCLV_ROBOTIQ_ROS__ROBOTIQ_3F_GRIPPER_NODE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <uclv_robotiq_msgs/GripperStatus.h>
#include <uclv_robotiq_msgs/FingerCommandArray.h>
#include <uclv_robotiq_msgs/ChangeModeAction.h>
#include "uclv_robotiq/robotiq_3f_gripper.h"

namespace uclv
{
    class Robotiq3fGripperROS
    {
    public:
        Robotiq3fGripperROS(const boost::shared_ptr<Robotiq3fGripper> &gripper,
                            const ros::NodeHandle &nh_private = ros::NodeHandle("~"),
                            const ros::NodeHandle &nh = ros::NodeHandle());

        ~Robotiq3fGripperROS();

        void init();
        void start();
        void spin();

    private:
        void _spinOnce(const ros::WallDuration &timeout = ros::WallDuration(0.0));
        void _changeModeExecuteCallback(const uclv_robotiq_msgs::ChangeModeGoalConstPtr &goal);
        void _fingersCommandCallback(const uclv_robotiq_msgs::FingerCommandArrayConstPtr &msg);
        bool _activateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool _resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool _stopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool _openCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool _closeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;
        boost::shared_ptr<Robotiq3fGripper> _gripper;

        ros::Publisher _gripper_status_pub;
        ros::Subscriber _fingers_command_sub;

        ros::CallbackQueue _callback_queue;

        std::unique_ptr<actionlib::SimpleActionServer<uclv_robotiq_msgs::ChangeModeAction>> _as_change_mode;

        ros::ServiceServer _srv_activate;
        ros::ServiceServer _srv_reset;
        ros::ServiceServer _srv_stop;
        ros::ServiceServer _srv_open;
        ros::ServiceServer _srv_close;
    };
} // namespace uclv

#endif // UCLV_ROBOTIQ_ROS__ROBOTIQ_3F_GRIPPER_NODE_H