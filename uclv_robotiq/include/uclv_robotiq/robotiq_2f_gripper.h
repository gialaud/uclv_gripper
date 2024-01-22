// Header file for Robotiq 2 fingeres grippers class

#ifndef UCLV_ROBOTIQ__ROBOTIQ_2F_GRIPPER_H
#define UCLV_ROBOTIQ__ROBOTIQ_2F_GRIPPER_H

#include <cstdint>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include "uclv_modbus/modbus_tcp.h"
#include "uclv_modbus/modbus_rtu.h"
#include "uclv_robotiq/robotiq_utils.h"

#define MESSAGE_INTERVAL 5 // milliseconds

namespace uclv
{

    class Robotiq2fGripper
    {
    public:
        // Constructor for using Modbus RTU over TCP
        Robotiq2fGripper(const std::string &ip_address, uint16_t port, uint8_t slave_id);
        // Constructor for using Modbus RTU
        Robotiq2fGripper(const std::string &port, uint8_t slave_id);

        ~Robotiq2fGripper();

        void activate();
        void reset();
        void stop();
        void open(bool wait = false);
        void close(bool wait = false);

        void move(uint8_t position, bool wait = false);
        void setSpeed(uint8_t speed);
        void setForce(uint8_t force);

        // getters
        GripperStatus getGripperStatus();
        Finger getFingersStatus();

    // private methods
    private:
        bool readStatus();
        bool sendCommand();

        // default constructor
        Robotiq2fGripper() = default;
        // method for waiting for motion complete
        void waitForMotionComplete();

        // methods for updating the status in a separate thread
        void updateStatus();
        void startUpdateStatusThread();
        void stopUpdateStatusThread();        

        // printing method for debugging
        void printStatus();

        // attributes
    private:
        // modbus client using unique ptr of std
        std::unique_ptr<uclv::Modbus> _client;

        // first status/command register
        uint16_t _first_status_register;
        uint16_t _first_command_register;

        // gripper request
        uint8_t _rACT{0};
        uint8_t _rGTO{0};
        uint8_t _rATR{0};
        uint8_t _rADR{0};

        GripperStatus _status;

        // fingers and
        Finger _finger;

        // last message time
        std::chrono::time_point<std::chrono::system_clock> _last_message_time;
    
        // thread for updating the status
        std::thread _update_status_thread;
        bool _update_status_thread_running{false};
        std::mutex _command_mutex;
        bool _new_command{false};

        // first start flag
        bool _first_start{true};
    };

} // namespace uclv

#endif // UCLV_ROBOTIQ__ROBOTIQ_2F_GRIPPER_H
