// Header file for the Robotiq 3F gripper class

#ifndef UCLV_ROBOTIQ__ROBOTIQ_3F_GRIPPER_H
#define UCLV_ROBOTIQ__ROBOTIQ_3F_GRIPPER_H

#include <cstdint>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include "uclv_modbus/modbus_tcp.h"
#include "uclv_modbus/modbus_rtu.h"
#include "uclv_robotiq/robotiq_utils.h"

#define MESSAGE_INTERVAL 10 // milliseconds

namespace uclv
{
    enum OperationMode
    {
        BASIC,
        PINCH,
        WIDE,
        SCISSOR
    };
    enum FingerID
    {
        FINGER_A,
        FINGER_B,
        FINGER_C,
        FINGER_S
    };

    class Robotiq3fGripper
    {
    public:
        // Constructor for using Modbus TCP or RTU over TCP
        Robotiq3fGripper(const std::string &ip_address, uint16_t port, uint8_t slave_id, bool rtu_over_tcp = false);
        // Constructor for using Modbus RTU
        Robotiq3fGripper(const std::string &port, uint8_t slave_id);

        ~Robotiq3fGripper();

        void activate();
        void reset();
        void stop();
        void open(bool wait = false);
        void close(bool wait = false);

        void individualScissorControl(bool activate);
        void individualFingerControl(bool activate);
        bool changeMode(OperationMode mode);

        void move(uint8_t position, uint8_t finger_id, bool wait = false);
        void setSpeed(uint8_t speed, uint8_t finger_id);
        void setForce(uint8_t force, uint8_t finger_id);

        void move(uint8_t position, bool wait = false);
        void setSpeed(uint8_t speed);
        void setForce(uint8_t force);

        // getters
        GripperStatus getGripperStatus();
        std::vector<Finger> getFingersStatus();

    // private methods
    private:
        
        bool sendCommand();
        bool readStatus();

        // default constructor
        Robotiq3fGripper() = default;
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
        std::unique_ptr<Modbus> _client;
        Modbus::FunctionCode _read_function_code;

        // first status/command register
        uint16_t _first_status_register;
        uint16_t _first_command_register;

        // gripper request
        uint8_t _rACT{0};
        uint8_t _rMOD{0};
        uint8_t _rGTO{0};
        uint8_t _rATR{0};
        // mode request
        uint8_t _rICF{0};
        uint8_t _rICS{0};

        GripperStatus _status;

        // fingers and ids
        Finger _finger_A;
        Finger _finger_B;
        Finger _finger_C;
        Finger _finger_S; // scissor

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

#endif // UCLV_ROBOTIQ__ROBOTIQ_3F_GRIPPER_H
