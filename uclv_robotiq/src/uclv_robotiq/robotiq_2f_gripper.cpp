// Implementation of the Robotiq Hand-e gripper class

#include "uclv_robotiq/robotiq_2f_gripper.h"
#include <bitset>

namespace uclv
{
// variable for debugging
#define ROBOTIQ_2F_GRIPPER_DEBUG 0

    Robotiq2fGripper::Robotiq2fGripper(const std::string &ip_address, uint16_t port, uint8_t slave_id)
    {
        _client = std::unique_ptr<uclv::Modbus>(new uclv::ModbusRTU(ip_address.c_str(), port, slave_id));
        _first_status_register = 0x07D0;
        _first_command_register = 0x03E8;
        _client->setLittleEndian();
        setSpeed(100);
        setForce(100);
        startUpdateStatusThread();
    }

    Robotiq2fGripper::Robotiq2fGripper(const std::string &port, uint8_t slave_id)
    {
        _client = std::unique_ptr<uclv::Modbus>(new uclv::ModbusRTU(port.c_str(), slave_id));
        _first_status_register = 0x07D0;
        _first_command_register = 0x03E8;
        _client->setLittleEndian();
        setSpeed(100);
        setForce(100);
        startUpdateStatusThread();
    }

    Robotiq2fGripper::~Robotiq2fGripper()
    {
        std::cout << "Called Robotiq2fGripper destructor" << std::endl;
        stopUpdateStatusThread();
        _client->disconnect();
    }

    void Robotiq2fGripper::updateStatus()
    {
        while (_update_status_thread_running)
        {
            // in case there is a command to be sent, send it
            if (_new_command)
            {
                // lock the mutex for avoiding the request is modified while sending it
                std::lock_guard<std::mutex> lock(_command_mutex);
                sendCommand();
                _new_command = false;
            }
            readStatus();
            // usleep(10000); // the sleep should be addressed in the sending of messages

            if (_first_start)
            {
                if (_status.gACT == 1 && _status.gSTA == 0) // powered on for the first time, reset needed
                {
                    reset();
                }
                else
                {
                    _rACT = _status.gACT;
                    _rGTO = _status.gGTO;
                }
                _first_start = false;               
            }
        }
    }

    void Robotiq2fGripper::startUpdateStatusThread()
    {
        _update_status_thread_running = true;
        _update_status_thread = std::thread(&Robotiq2fGripper::updateStatus, this);
    }

    void Robotiq2fGripper::stopUpdateStatusThread()
    {
        _update_status_thread_running = false;
        if (_update_status_thread.joinable())
        {
            _update_status_thread.join();
        }
    }

    void Robotiq2fGripper::activate()
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _rACT = 1;
            _new_command = true;
        }
        // wait for activation to complete
        // print for debugging
#if DEBUG
        std::cout << "Robotiq2fGripper::activate: waiting for activation to complete" << std::endl;
#endif
    }

    void Robotiq2fGripper::reset()
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _rACT = 0;
        _new_command = true;
    }

    void Robotiq2fGripper::stop()
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _rGTO = 0;
        _new_command = true;
    }

    void Robotiq2fGripper::open(bool wait)
    {
        move(0, wait);
    }

    void Robotiq2fGripper::close(bool wait)
    {
        move(255, wait);
    }

    void Robotiq2fGripper::move(uint8_t position, bool wait)
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _finger.rPR = position;
        _rGTO = 1; // set to one for activating the movement
        _new_command = true;
    }

    void Robotiq2fGripper::setSpeed(uint8_t speed)
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _finger.rSP = speed;
        // the register will be set with the next command
    }

    void Robotiq2fGripper::setForce(uint8_t force)
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _finger.rFR = force;
        // the register will be set with the next command
    }

    void Robotiq2fGripper::waitForMotionComplete()
    {
        while (_new_command || _status.gOBJ == 0)
        {
            usleep(1000); // wait 1 ms
        }
    }

    bool Robotiq2fGripper::readStatus()
    {
        // Wait for the minimum interval between messages
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _last_message_time).count() < MESSAGE_INTERVAL)
        {
            usleep(1000); // wait 1 ms
        }

        // Read the status registers
        int num_status_registers = 3;
        uint8_t values[num_status_registers * 2]; // each register is 2 bytes
        _client->readRegisters(Modbus::READ_HOLDING_REGISTERS, _first_status_register, num_status_registers, values);

        // Update the last message time from system clock time (using chrono)
        _last_message_time = std::chrono::system_clock::now();

        // Parse the status registers
        // Parse values[0], i.e., GRIPPER STATUS
        _status.gACT = values[0] & 0x01;        // gACT is bit 0 of the first register
        _status.gGTO = (values[0] & 0x08) >> 3; // gGTO is bit 3 of the first register
        _status.gSTA = (values[0] & 0x30) >> 4; // gSTA is bits 4 and 5 of the first register
        _status.gOBJ = (values[0] & 0xC0) >> 6; // gOBJ is bits 6 and 7 of the first register
        // Parse register[2], i.e., FAULT STATUS
        _status.gFLT = values[2] & 0x0F; // gFLT is bits 0 to 3 of the third register
        // Parse other values
        _finger.status.gPR = values[3];
        _finger.status.gPO = values[4];
        _finger.status.gCU = values[5];

#if ROBOTIQ_2F_GRIPPER_DEBUG
        printStatus();
#endif

        return true;
    }

    bool Robotiq2fGripper::sendCommand()
    {
        // Build the request message, filling it initially with zeros
        int num_command_registers = 3; // each register is 2 bytes
        uint8_t request[num_command_registers * 2] = {0};
        // byte 0 <- bit 0: rACT, bbit 3: rGTO, bit 4: rATR, bit 5: rADR, bits 6-7: zeros
        request[0] = _rACT | (_rGTO << 3) | (_rATR << 4) | (_rADR << 5);
        // byte 1 <- all zeros
        // byte 2 <- all zeros
        // byte 3-5 <- finger: rPR, rSP, rFR
        request[3] = _finger.rPR;
        request[4] = _finger.rSP;
        request[5] = _finger.rFR;

        // Wait for the minimum interval between messages
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _last_message_time).count() < MESSAGE_INTERVAL)
        {
            usleep(1000); // wait 1 ms
        }

        // Send the request message
        _client->writeRegisters(Modbus::WRITE_MULTIPLE_REGISTERS, _first_command_register, num_command_registers, request);

        // Update the last message time from system clock time (using chrono)
        _last_message_time = std::chrono::system_clock::now();
        return true;
    }

    GripperStatus Robotiq2fGripper::getGripperStatus()
    {
        return _status;
    }

    Finger Robotiq2fGripper::getFingersStatus()
    {
        return _finger;
    }

    void Robotiq2fGripper::printStatus()
    {
        std::cout << "gACT: " << (int)_status.gACT << std::endl;
        std::cout << "gGTO: " << (int)_status.gGTO << std::endl;
        std::cout << "gSTA: " << (int)_status.gSTA << std::endl;
        std::cout << "gOBJ: " << (int)_status.gOBJ << std::endl;
        std::cout << "gFLT: " << (int)_status.gFLT << std::endl;
        std::cout << "gPR: " << (int)_finger.status.gPR << std::endl;
        std::cout << "gPO: " << (int)_finger.status.gPO << std::endl;
        std::cout << "gCU: " << (int)_finger.status.gCU << std::endl;
    }
} // namespace uclv