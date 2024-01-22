#include "uclv_robotiq_ros/robotiq_2f_gripper_node.h"

using namespace uclv;

Robotiq2fGripperROS::Robotiq2fGripperROS(const boost::shared_ptr<Robotiq2fGripper> &gripper,
                                         const ros::NodeHandle &nh_private,
                                         const ros::NodeHandle &nh)
    : _nh(nh), _nh_private(nh_private), _gripper(gripper)
{
}

Robotiq2fGripperROS::~Robotiq2fGripperROS()
{
}

void Robotiq2fGripperROS::init()
{
    _nh.setCallbackQueue(&_callback_queue);

    _gripper_status_pub = _nh.advertise<uclv_robotiq_msgs::GripperStatus>("gripper_status", 1);
    _fingers_command_sub = _nh.subscribe("fingers_command", 1, &Robotiq2fGripperROS::fingersCommandCallback, this);

    _srv_activate = _nh.advertiseService("activate", &Robotiq2fGripperROS::activateCallback, this);
    _srv_reset = _nh.advertiseService("reset", &Robotiq2fGripperROS::resetCallback, this);
    _srv_stop = _nh.advertiseService("stop", &Robotiq2fGripperROS::stopCallback, this);
    _srv_open = _nh.advertiseService("open", &Robotiq2fGripperROS::openCallback, this);
    _srv_close = _nh.advertiseService("close", &Robotiq2fGripperROS::closeCallback, this);
}

void Robotiq2fGripperROS::start()
{
    // activate the gripper and start eventual servers
}

void Robotiq2fGripperROS::spin()
{
    ros::Rate loop_rate(200.0);

    while (ros::ok())
    {
        spinOnce();

        uclv_robotiq_msgs::GripperStatusPtr gripper_status_msg(new uclv_robotiq_msgs::GripperStatus);

        GripperStatus gripper_status = _gripper->getGripperStatus();
        Finger fingers_status = _gripper->getFingersStatus();

        gripper_status_msg->gACT = gripper_status.gACT;
        gripper_status_msg->gMOD = gripper_status.gMOD;
        gripper_status_msg->gGTO = gripper_status.gGTO;
        gripper_status_msg->gOBJ = gripper_status.gOBJ;
        gripper_status_msg->gSTA = gripper_status.gSTA;
        gripper_status_msg->gFLT = gripper_status.gFLT;

        gripper_status_msg->fingers.resize(1);
        gripper_status_msg->fingers[0].finger_id = 0;
        gripper_status_msg->fingers[0].gPR = fingers_status.status.gPR;
        gripper_status_msg->fingers[0].gPO = fingers_status.status.gPO;
        gripper_status_msg->fingers[0].gCU = fingers_status.status.gCU;

        gripper_status_msg->header.stamp = ros::Time::now();
        _gripper_status_pub.publish(gripper_status_msg);

        loop_rate.sleep();
    }
}

void Robotiq2fGripperROS::spinOnce(const ros::WallDuration &timeout)
{
    _callback_queue.callAvailable(timeout);
}

bool Robotiq2fGripperROS::activateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    _gripper->activate();
    res.success = true;
    res.message = "Gripper activated";
    return true;
}

bool Robotiq2fGripperROS::resetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    _gripper->reset();
    res.success = true;
    res.message = "Gripper reset";
    return true;
}

bool Robotiq2fGripperROS::stopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    _gripper->stop();
    res.success = true;
    res.message = "Gripper stopped";
    return true;
}

bool Robotiq2fGripperROS::openCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    _gripper->open(true);
    res.success = true;
    res.message = "Gripper opened";
    return true;
}

bool Robotiq2fGripperROS::closeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    _gripper->close(true);
    res.success = true;
    res.message = "Gripper closed";
    return true;
}

void Robotiq2fGripperROS::fingersCommandCallback(const uclv_robotiq_msgs::FingerCommandArrayConstPtr &msg)
{
    _gripper->setSpeed(msg->fingers_command[0].speed);
    _gripper->setForce(msg->fingers_command[0].force);
    _gripper->move(msg->fingers_command[0].position);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotiq_2f_gripper_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // get parameters
    // modbus type
    std::string modbus_type = "rtu_over_tcp";
    nh_private.getParam("modbus_type", modbus_type);

    // modbus tcp parameters
    std::string ip_address;
    int port;
    // check if ip_address and port are provided
    if (modbus_type == "rtu_over_tcp")
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
    int slave_id = 9;
    nh_private.getParam("slave_id", slave_id);

    boost::shared_ptr<uclv::Robotiq2fGripper> gripper;

    if (modbus_type == "rtu")
    {
        gripper.reset(new uclv::Robotiq2fGripper(serial_port, (uint8_t)slave_id));
    }
    else if (modbus_type == "rtu_over_tcp")
    {
        gripper.reset(new uclv::Robotiq2fGripper(ip_address, (uint16_t)port, (uint8_t)slave_id));
    }
    else
    {
        ROS_ERROR("Invalid modbus_type parameter: %s", modbus_type.c_str());
        return -1;
    }

    Robotiq2fGripperROS gripper_ros(gripper, nh_private, nh);

    gripper_ros.init();
    // gripper_ros.start();
    gripper_ros.spin();

    return 0;
}
