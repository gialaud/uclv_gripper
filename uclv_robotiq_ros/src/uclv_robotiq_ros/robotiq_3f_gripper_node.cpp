#include "uclv_robotiq_ros/robotiq_3f_gripper_node.h"

namespace uclv
{

    Robotiq3fGripperROS::Robotiq3fGripperROS(const boost::shared_ptr<Robotiq3fGripper> &gripper,
                                             const ros::NodeHandle &nh_private,
                                             const ros::NodeHandle &nh)
        : _nh(nh), _nh_private(nh_private), _gripper(gripper)
    {
    }

    Robotiq3fGripperROS::~Robotiq3fGripperROS()
    {
    }

    void Robotiq3fGripperROS::init()
    {
        _nh.setCallbackQueue(&_callback_queue);

        _gripper_status_pub = _nh.advertise<uclv_robotiq_msgs::GripperStatus>("gripper_status", 1);
        _fingers_command_sub = _nh.subscribe("fingers_command", 1, &Robotiq3fGripperROS::_fingersCommandCallback, this);

        _as_change_mode = std::unique_ptr<
            actionlib::SimpleActionServer<uclv_robotiq_msgs::ChangeModeAction>>(
            new actionlib::SimpleActionServer<
                uclv_robotiq_msgs::ChangeModeAction>(
                _nh,
                "change_mode",
                boost::bind(&Robotiq3fGripperROS::_changeModeExecuteCallback, this, _1),
                false));

        _srv_activate = _nh.advertiseService("activate", &Robotiq3fGripperROS::_activateCallback, this);
        _srv_reset = _nh.advertiseService("reset", &Robotiq3fGripperROS::_resetCallback, this);
        _srv_stop = _nh.advertiseService("stop", &Robotiq3fGripperROS::_stopCallback, this);
        _srv_open = _nh.advertiseService("open", &Robotiq3fGripperROS::_openCallback, this);
        _srv_close = _nh.advertiseService("close", &Robotiq3fGripperROS::_closeCallback, this);

        // _gripper->readStatus();
    }

    void Robotiq3fGripperROS::start()
    {
        _as_change_mode->start();
    }

    void Robotiq3fGripperROS::spin()
    {
        ros::Rate loop_rate(100.0);

        while (ros::ok())
        {
            _spinOnce();

            uclv_robotiq_msgs::GripperStatusPtr gripper_status_msg(new uclv_robotiq_msgs::GripperStatus);

            GripperStatus gripper_status = _gripper->getGripperStatus();
            std::vector<Finger> fingers_status = _gripper->getFingersStatus();

            gripper_status_msg->gACT = gripper_status.gACT;
            gripper_status_msg->gMOD = gripper_status.gMOD;
            gripper_status_msg->gGTO = gripper_status.gGTO;
            gripper_status_msg->gOBJ = gripper_status.gOBJ;
            gripper_status_msg->gSTA = gripper_status.gSTA;
            gripper_status_msg->gFLT = gripper_status.gFLT;

            gripper_status_msg->fingers.resize(fingers_status.size());
            for (size_t i = 0; i < fingers_status.size(); ++i)
            {
                gripper_status_msg->fingers[i].finger_id = i;
                gripper_status_msg->fingers[i].gPR = fingers_status[i].status.gPR;
                gripper_status_msg->fingers[i].gPO = fingers_status[i].status.gPO;
                gripper_status_msg->fingers[i].gCU = fingers_status[i].status.gCU;
                gripper_status_msg->fingers[i].gDT = fingers_status[i].status.gDT;
            }

            gripper_status_msg->header.stamp = ros::Time::now();
            _gripper_status_pub.publish(gripper_status_msg);

            loop_rate.sleep();
        }
    }

    void Robotiq3fGripperROS::_spinOnce(const ros::WallDuration &timeout)
    {
        _callback_queue.callAvailable(timeout);
    }

    void Robotiq3fGripperROS::_changeModeExecuteCallback(const uclv_robotiq_msgs::ChangeModeGoalConstPtr &goal)
    {
        uclv::OperationMode mode = static_cast<uclv::OperationMode>(goal->target_mode);
        bool individual_finger_control = goal->individual_finger_control;
        bool individual_scissor_control = goal->individual_scissor_control;

        bool mode_changed = false;
        if (individual_finger_control)
        {
            _gripper->individualFingerControl(individual_finger_control);
        }
        if (individual_scissor_control)
        {
            _gripper->individualScissorControl(individual_scissor_control);
        }
        else
        {
            mode_changed = _gripper->changeMode(mode); // this will wait for mode change to complete
        }

        uclv_robotiq_msgs::ChangeModeResult result;
        result.success = mode_changed;
        _as_change_mode->setSucceeded(result);
    }

    bool Robotiq3fGripperROS::_activateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        _gripper->activate();
        res.success = true;
        res.message = "Gripper activated";
        return true;
    }

    bool Robotiq3fGripperROS::_resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        _gripper->reset();
        res.success = true;
        res.message = "Gripper reset";
        return true;
    }

    bool Robotiq3fGripperROS::_stopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        _gripper->stop();
        res.success = true;
        res.message = "Gripper stopped";
        return true;
    }

    bool Robotiq3fGripperROS::_openCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        _gripper->open(true);
        res.success = true;
        res.message = "Gripper opened";
        return true;
    }

    bool Robotiq3fGripperROS::_closeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        _gripper->close(true);
        res.success = true;
        res.message = "Gripper closed";
        return true;
    }

    void Robotiq3fGripperROS::_fingersCommandCallback(const uclv_robotiq_msgs::FingerCommandArrayConstPtr &msg)
    {
        for (const auto &finger_command : msg->fingers_command)
        {
            uint8_t id = finger_command.finger_id;
            _gripper->setSpeed(finger_command.speed, id);
            _gripper->setForce(finger_command.force, id);
            _gripper->move(finger_command.position, id);
        }
    }
} // namespace uclv

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotiq_3f_gripper_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // get parameters
    // modbus type
    std::string modbus_type = "tcp";
    nh_private.getParam("modbus_type", modbus_type);

    // modbus tcp parameters
    std::string ip_address;
    int port;
    // check if ip_address and port are provided
    if (modbus_type == "tcp" || modbus_type == "rtu_over_tcp")
    {
        if (!nh_private.hasParam("ip_address") || !nh_private.hasParam("port"))
        {
            ROS_ERROR("ip_address and port parameters must be provided for modbus_type %s", modbus_type.c_str());
            return -1;
        }
        nh_private.getParam("ip_address", ip_address);
        nh_private.getParam("port", port);
    }

    // modbus rtu parameters
    std::string serial_port;
    int baudrate;
    // check if port_name and baudrate are provided
    if (modbus_type == "rtu")
    {
        if (!nh_private.hasParam("serial_port") || !nh_private.hasParam("baudrate"))
        {
            ROS_ERROR("serial_port and baudrate parameters must be provided for modbus_type %s", modbus_type.c_str());
            return -1;
        }
        nh_private.getParam("serial_port", serial_port);
        nh_private.getParam("baudrate", baudrate);
    }

    // modbus slave id
    int slave_id = 2;
    nh_private.getParam("slave_id", slave_id);

    boost::shared_ptr<uclv::Robotiq3fGripper> gripper;

    if (modbus_type == "tcp")
    {
        gripper.reset(new uclv::Robotiq3fGripper(ip_address, (uint16_t)port, (uint8_t)slave_id, false));
    }
    else if (modbus_type == "rtu")
    {
        gripper.reset(new uclv::Robotiq3fGripper(serial_port, (uint8_t)slave_id));
    }
    else if (modbus_type == "rtu_over_tcp")
    {
        gripper.reset(new uclv::Robotiq3fGripper(ip_address, (uint16_t)port, (uint8_t)slave_id, true));
    }
    else
    {
        ROS_ERROR("Invalid modbus_type parameter: %s", modbus_type.c_str());
        return -1;
    }

    uclv::Robotiq3fGripperROS gripper_ros(gripper, nh_private, nh);

    gripper_ros.init();
    gripper_ros.start();
    gripper_ros.spin();

    return 0;
}
