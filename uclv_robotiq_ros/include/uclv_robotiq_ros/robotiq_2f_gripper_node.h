#ifndef UCLV_ROBOTIQ_ROS__ROBOTIQ_2F_GRIPPER_NODE_H
#define UCLV_ROBOTIQ_ROS__ROBOTIQ_2F_GRIPPER_NODE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <uclv_robotiq_msgs/GripperStatus.h>
#include <uclv_robotiq_msgs/FingerCommandArray.h>
#include <uclv_robotiq/robotiq_2f_gripper.h>

namespace uclv
{
    class Robotiq2fGripperROS
    {
    public:
        Robotiq2fGripperROS(const boost::shared_ptr<Robotiq2fGripper> &gripper,
                            const ros::NodeHandle &nh_private = ros::NodeHandle("~"),
                            const ros::NodeHandle &nh = ros::NodeHandle());

        ~Robotiq2fGripperROS();

        void init();
        void start();
        void spin();

    private:
        void spinOnce(const ros::WallDuration &timeout = ros::WallDuration(0.0));
        void fingersCommandCallback(const uclv_robotiq_msgs::FingerCommandArrayConstPtr &msg);
        bool activateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool stopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool openCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool closeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;
        boost::shared_ptr<Robotiq2fGripper> _gripper;

        ros::Publisher _gripper_status_pub;
        ros::Subscriber _fingers_command_sub;

        ros::CallbackQueue _callback_queue;

        ros::ServiceServer _srv_activate;
        ros::ServiceServer _srv_reset;
        ros::ServiceServer _srv_stop;
        ros::ServiceServer _srv_open;
        ros::ServiceServer _srv_close;
    };
} // namespace uclv

#endif // UCLV_ROBOTIQ_ROS__ROBOTIQ_2F_GRIPPER_NODE_H