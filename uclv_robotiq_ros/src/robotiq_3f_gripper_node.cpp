#include "uclv_robotiq_ros/robotiq_3f_gripper_node.h"

namespace uclv
{
    class Robotiq3fGripperROS : public rclcpp::Node
    {
        using ChangeMode = uclv_robotiq_interfaces::action::ChangeMode;
        using GoalHandleChangeMode = rclcpp_action::ServerGoalHandle<ChangeMode>;
        
        private:
        rclcpp::Publisher<uclv_robotiq_interfaces::msg::GripperStatus>::SharedPtr _gripper_status_pub;
        std::string gripper_ip;
        double frequency;
        int port;
        rclcpp::TimerBase::SharedPtr timer_status;
        uclv_robotiq_interfaces::msg::GripperStatus gripper_status_msg;
        std::shared_ptr<Robotiq3fGripper> _gripper;
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
        rclcpp_action::Server<ChangeMode>::SharedPtr action_server_change_mode;

        public:
        Robotiq3fGripperROS(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("robotiq_3f_gripper", options)
        {
            using namespace std::placeholders;

            gripper_ip = this->declare_parameter<std::string>("gripper_ip", "192.168.1.11");
            frequency = this->declare_parameter<double>("frequency", 200.0);
            serial_port = this->declare_parameter<std::string>("serial_port","/dev/ttyUSB0");
            port = this->declare_parameter<int>("port", 54321);
            slave_id = this->declare_parameter<int>("slave_id", 9);
            baudrate = this->declare_parameter<int>("baudrate", 115200);
            reset_and_activate = this->declare_parameter<bool>("reset_and_activate", true);
            modbus_type = this->declare_parameter<std::string>("modbus_type", "rtu_over_tcp");
            if (modbus_type == "tcp")
            {
                _gripper.reset(new uclv::Robotiq3fGripper(gripper_ip, (uint16_t)port, (uint8_t)slave_id, false));
            }
            else if (modbus_type == "rtu")
            {
                _gripper.reset(new uclv::Robotiq3fGripper(serial_port, (uint8_t)slave_id));
            }
            else if (modbus_type == "rtu_over_tcp")
            {
                _gripper.reset(new uclv::Robotiq3fGripper(gripper_ip, (uint16_t)port, (uint8_t)slave_id, true));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid modbus_type parameter: %s", modbus_type.c_str());
                return;
            }
            if(reset_and_activate)
            {
                reset_gripper();
                usleep(1000000); // wait for 1 second
                activate_gripper();
            }
            _gripper_status_pub = this->create_publisher<uclv_robotiq_interfaces::msg::GripperStatus>("threef_gripper_status", 10);
            _fingers_command_sub = this->create_subscription<uclv_robotiq_interfaces::msg::FingerCommandArray>(
                "threef_fingers_command", 10, std::bind(&Robotiq3fGripperROS::fingers_command_callbk, this, _1));
            
            action_server_change_mode = rclcpp_action::create_server<ChangeMode>(
                this,
                "threef_change_mode",
                std::bind(&Robotiq3fGripperROS::handle_change_mode_goal, this, _1, _2),
                std::bind(&Robotiq3fGripperROS::handle_change_mode_cancel, this, _1),
                std::bind(&Robotiq3fGripperROS::handle_change_mode_accepted, this, _1));

            timer_status = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / frequency)), std::bind(&Robotiq3fGripperROS::status_callback, this));
            
                service_activate =
                this->create_service<std_srvs::srv::Trigger>("threef_gripper_activate", std::bind(&Robotiq3fGripperROS::activateCallback, this, _1, _2));
            service_reset =
                this->create_service<std_srvs::srv::Trigger>("threef_gripper_reset", std::bind(&Robotiq3fGripperROS::resetCallback, this, _1, _2));
            service_stop =
                this->create_service<std_srvs::srv::Trigger>("threef_gripper_stop", std::bind(&Robotiq3fGripperROS::stopCallback, this, _1, _2));
            service_open =
                this->create_service<std_srvs::srv::Trigger>("threef_gripper_open", std::bind(&Robotiq3fGripperROS::openCallback, this, _1, _2));
            service_close =
                this->create_service<std_srvs::srv::Trigger>("threef_gripper_close", std::bind(&Robotiq3fGripperROS::closeCallback, this, _1, _2));
        }

        ~Robotiq3fGripperROS(){
        }
        
        void status_callback()
        {
            GripperStatus gripper_status = _gripper->getGripperStatus();
            std::vector<Finger> fingers_status = _gripper->getFingersStatus();

            gripper_status_msg.g_act = gripper_status.gACT;
            gripper_status_msg.g_mod = gripper_status.gMOD;
            gripper_status_msg.g_gto = gripper_status.gGTO;
            gripper_status_msg.g_obj = gripper_status.gOBJ;
            gripper_status_msg.g_sta = gripper_status.gSTA;
            gripper_status_msg.g_flt = gripper_status.gFLT;

            gripper_status_msg.fingers.resize(fingers_status.size());
            for (size_t i = 0; i < fingers_status.size(); ++i)
            {
                gripper_status_msg.fingers[i].finger_id = i;
                gripper_status_msg.fingers[i].g_pr = fingers_status[i].status.gPR;
                gripper_status_msg.fingers[i].g_po = fingers_status[i].status.gPO;
                gripper_status_msg.fingers[i].g_cu = fingers_status[i].status.gCU;
                gripper_status_msg.fingers[i].g_dt = fingers_status[i].status.gDT;
            }

            gripper_status_msg.header.stamp = this->get_clock()->now();
            _gripper_status_pub->publish(gripper_status_msg);
        }

        rclcpp_action::GoalResponse handle_change_mode_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ChangeMode::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to change mode");
            (void)uuid;
            if (goal->target_mode == 0 || goal->target_mode == 1 || goal->target_mode == 2 || goal->target_mode == 3)
            {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid mode: %d", goal->target_mode);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        rclcpp_action::CancelResponse handle_change_mode_cancel(const std::shared_ptr<GoalHandleChangeMode> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel ChangeMode goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_change_mode_accepted(const std::shared_ptr<GoalHandleChangeMode> goal_handle)
        {
            using namespace std::placeholders;
            std::thread{ std::bind(&Robotiq3fGripperROS::execute_change_mode, this, _1), goal_handle }.detach();
        }

        void execute_change_mode(const std::shared_ptr<GoalHandleChangeMode> goal_handle)
        {
            auto goal = goal_handle->get_goal();
            auto result = std::make_shared<ChangeMode::Result>();

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

            if (mode_changed)
            {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Mode changed successfully");
            }
            else
            {
                result->success = false;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Failed to change mode");
            }
        }

        void fingers_command_callbk(const uclv_robotiq_interfaces::msg::FingerCommandArray::ConstSharedPtr& msg)
        {
            for(const auto& finger_command : msg->fingers_command)
            {
                uint8_t finger_id = finger_command.finger_id;
                _gripper->setSpeed(finger_command.speed, finger_id);
                _gripper->setForce(finger_command.force, finger_id);
                _gripper->move(finger_command.position, finger_id);
            }
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

  auto robotiq_3f_gripper_node = std::make_shared<uclv::Robotiq3fGripperROS>();
  rclcpp::spin(robotiq_3f_gripper_node);
  rclcpp::shutdown();

  return 0;
}