#include <rclcpp/rclcpp.hpp>
#include "uclv_robotiq_interfaces/msg/gripper_status.hpp"
#include "uclv_robotiq_interfaces/msg/finger_command_array.hpp"
#include "uclv_robotiq/robotiq_2f_gripper.h"
#include "std_srvs/srv/trigger.hpp"

namespace uclv
{
    class Robotiq2fGripperROS : public rclcpp::Node
    {
        private:
        rclcpp::Publisher<uclv_robotiq_interfaces::msg::GripperStatus>::SharedPtr _gripper_status_pub;
        std::string gripper_ip;
        double frequency;
        int port;
        rclcpp::TimerBase::SharedPtr timer_status;
        uclv_robotiq_interfaces::msg::GripperStatus gripper_status_msg;
        std::shared_ptr<Robotiq2fGripper> _gripper;
        rclcpp::Subscription<uclv_robotiq_interfaces::msg::FingerCommandArray>::SharedPtr _fingers_command_sub;
        int slave_id;
        bool reset_and_activate;
        std::string modbus_type;
        std::string serial_port;
        int baudrate;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_activate;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_reset;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_stop;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_open;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_close;

        public:
        Robotiq2fGripperROS(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("robotiq_2f_gripper", options)
        {
            using namespace std::placeholders;
            
            gripper_ip = this->declare_parameter<std::string>("gripper_ip", "192.168.1.110");
            frequency = this->declare_parameter<double>("frequency", 200.0);
            serial_port = this->declare_parameter<std::string>("serial_port","/dev/ttyUSB0");
            port = this->declare_parameter<int>("port", 54321);
            slave_id = this->declare_parameter<int>("slave_id", 9);
            baudrate = this->declare_parameter<int>("baudrate", 115200);
            reset_and_activate = this->declare_parameter<bool>("reset_and_activate", true);
            modbus_type = this->declare_parameter<std::string>("modbus_type", "rtu_over_tcp");

            if (modbus_type == "rtu")
            {
                _gripper.reset(new uclv::Robotiq2fGripper(serial_port, (uint8_t)slave_id));
            }
            else if (modbus_type == "rtu_over_tcp")
            {
                _gripper.reset(new uclv::Robotiq2fGripper(gripper_ip, (uint16_t)port, (uint8_t)slave_id));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid modbus_type parameter: %s", modbus_type.c_str());
                return;
            }

            if(reset_and_activate)
            {
                //sleep for 0.5 second to allow the gripper to reset
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                reset_gripper(); 
                //sleep for 0.5 second to allow the gripper to reset
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                activate_gripper();
            }

            _fingers_command_sub = this->create_subscription<uclv_robotiq_interfaces::msg::FingerCommandArray>(
            "fingers_command", 1, std::bind(&Robotiq2fGripperROS::fingers_command_callbk, this, _1));

            timer_status = this->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(1000.0 / frequency)), std::bind(&Robotiq2fGripperROS::status_callback, this));

            _gripper_status_pub = this->create_publisher<uclv_robotiq_interfaces::msg::GripperStatus>("gripper_status", 10);

            service_activate =
                this->create_service<std_srvs::srv::Trigger>("gripper_activate", std::bind(&Robotiq2fGripperROS::activateCallback, this, _1, _2));
            service_reset =
                this->create_service<std_srvs::srv::Trigger>("gripper_reset", std::bind(&Robotiq2fGripperROS::resetCallback, this, _1, _2));
            service_stop =
                this->create_service<std_srvs::srv::Trigger>("gripper_stop", std::bind(&Robotiq2fGripperROS::stopCallback, this, _1, _2));
            service_open =
                this->create_service<std_srvs::srv::Trigger>("gripper_open", std::bind(&Robotiq2fGripperROS::openCallback, this, _1, _2));
            service_close =
                this->create_service<std_srvs::srv::Trigger>("gripper_close", std::bind(&Robotiq2fGripperROS::closeCallback, this, _1, _2));
        }

        void status_callback()
        {
            GripperStatus gripper_status = _gripper->getGripperStatus();
            Finger fingers_status = _gripper->getFingersStatus();

            gripper_status_msg.g_act = gripper_status.gACT;
            gripper_status_msg.g_mod = gripper_status.gMOD;
            gripper_status_msg.g_gto = gripper_status.gGTO;
            gripper_status_msg.g_obj = gripper_status.gOBJ;
            gripper_status_msg.g_sta = gripper_status.gSTA;
            gripper_status_msg.g_flt = gripper_status.gFLT;

            gripper_status_msg.fingers.resize(1);
            gripper_status_msg.fingers[0].finger_id = 0;
            gripper_status_msg.fingers[0].g_pr = fingers_status.status.gPR;
            gripper_status_msg.fingers[0].g_po = fingers_status.status.gPO;
            gripper_status_msg.fingers[0].g_cu = fingers_status.status.gCU;

            gripper_status_msg.header.stamp = this->get_clock()->now();
            _gripper_status_pub->publish(gripper_status_msg);
        }

        void fingers_command_callbk(const uclv_robotiq_interfaces::msg::FingerCommandArray::ConstSharedPtr& msg)
        {
            _gripper->setSpeed(msg->fingers_command[0].speed);
            _gripper->setForce(msg->fingers_command[0].force);
            _gripper->move(msg->fingers_command[0].position);
        }

        void activate_gripper()
        {
            _gripper->activate();
        }

        void reset_gripper()
        {
            _gripper->reset();
        }

        bool activateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
            (void)req;
            activate_gripper();
            res->success = true;
            res->message = "Gripper activated";
            return true;
        }

        bool resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
            (void)req;
            reset_gripper();
            res->success = true;
            res->message = "Gripper reset";
            return true;
        }

        bool stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
            (void)req;
            _gripper->stop();
            res->success = true;
            res->message = "Gripper stopped";
            return true;
        }

        bool openCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
            (void)req;
            _gripper->open(true);
            res->success = true;
            res->message = "Gripper opened";
            return true;
        }

        bool closeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
            (void)req;
            _gripper->close(true);
            res->success = true;
            res->message = "Gripper closed";
            return true;
        }
    };
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto robotiq_2f_gripper_node = std::make_shared<uclv::Robotiq2fGripperROS>();
  rclcpp::spin(robotiq_2f_gripper_node);
  rclcpp::shutdown();

  return 0;
}