// Implementation of the Robotiq 3F gripper class

#include "uclv_robotiq/robotiq_3f_gripper.h"
#include <bitset>

namespace uclv
{

// variable for debugging
#define ROBOTIQ_3F_GRIPPER_DEBUG 0

    Robotiq3fGripper::Robotiq3fGripper(const std::string &ip_address, uint16_t port, uint8_t slave_id, bool rtu_over_tcp)
    {
        if (rtu_over_tcp)
        {
            _client = std::unique_ptr<uclv::Modbus>(new uclv::ModbusRTU(ip_address.c_str(), port, slave_id));
            _first_status_register = 0x07D0;
            _first_command_register = 0x03E8;
            _read_function_code = Modbus::READ_HOLDING_REGISTERS;
        }
        else
        {
            _client = std::unique_ptr<uclv::Modbus>(new uclv::ModbusTCP(ip_address.c_str(), port, slave_id));
            _first_status_register = 0x0000;
            _first_command_register = 0x0000;
            _read_function_code = Modbus::READ_INPUT_REGISTERS;
        }

#if ROBOTIQ_3F_GRIPPER_DEBUG
        std::cout << "Robotiq3fGripper::Robotiq3fGripper: " << ip_address << " " << port << " " << (int)slave_id << " " << rtu_over_tcp << std::endl;
#endif
        _client->setLittleEndian();
        setSpeed(100);
        setForce(100);
        startUpdateStatusThread();
    }

    Robotiq3fGripper::Robotiq3fGripper(const std::string &port, uint8_t slave_id)
    {
        _client = std::unique_ptr<uclv::Modbus>(new uclv::ModbusRTU(port.c_str(), slave_id));
        _first_status_register = 0x07D0;
        _first_command_register = 0x03E8;
        _read_function_code = Modbus::FunctionCode::READ_HOLDING_REGISTERS;
        _client->setLittleEndian();
        setSpeed(100);
        setForce(100);
        startUpdateStatusThread();

#if ROBOTIQ_3F_GRIPPER_DEBUG
        std::cout << "Robotiq3fGripper::Robotiq3fGripper: " << port << " " << (int)slave_id << std::endl;
#endif
    }

    Robotiq3fGripper::~Robotiq3fGripper()
    {
        stopUpdateStatusThread();
        _client->disconnect();
    }

    void Robotiq3fGripper::updateStatus()
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
                _rACT = _status.gACT;
                _rMOD = _status.gMOD;
                _rGTO = _status.gGTO;
                _first_start = false;
            }
        }
    }

    void Robotiq3fGripper::startUpdateStatusThread()
    {
        _update_status_thread_running = true;
        _update_status_thread = std::thread(&Robotiq3fGripper::updateStatus, this);
    }

    void Robotiq3fGripper::stopUpdateStatusThread()
    {
        _update_status_thread_running = false;
        if (_update_status_thread.joinable())
        {
            _update_status_thread.join();
        }
    }

    void Robotiq3fGripper::activate()
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _rACT = 1;
            _new_command = true;
        }
        // wait for activation to complete
        std::cout << "Robotiq3fGripper::activate: waiting for activation to complete" << std::endl;
        /* ********************* CHECK WHAT REGISTER CONTROL FOR DETECTING ACTIVATION ********************** */
        while (_new_command || _status.gSTA != 3) // wait that the command is sent and the gripper is activated
        {
            usleep(5000); // wait 5 ms
        }
    }

    void Robotiq3fGripper::reset()
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _rACT = 0;
            _new_command = true;
        }
        while (_new_command || _status.gACT != 0)
        {
            usleep(5000); // wait 5 ms
        }
    }

    void Robotiq3fGripper::stop()
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _rGTO = 0;
            _new_command = true;
        }
        while (_new_command || _status.gGTO != 0)
        {
            usleep(5000); // wait 5 ms
        }
    }

    void Robotiq3fGripper::open(bool wait)
    {
        move(0, wait);
    }

    void Robotiq3fGripper::close(bool wait)
    {
        move(255, wait);
    }

    void Robotiq3fGripper::individualScissorControl(bool activate)
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _rGTO = 0; // set to zero for avoiding unwanted movements
            _rICS = activate;
            _new_command = true;
        }
        while (_new_command || _status.gSTA != 3)
        {
            usleep(5000); // wait 5 ms
        }
    }

    void Robotiq3fGripper::individualFingerControl(bool activate)
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _rGTO = 0; // set to zero for avoiding unwanted movements
            _rICF = activate;
            _new_command = true;
        }
        while (_new_command || _status.gSTA != 3)
        {
            usleep(5000); // wait 5 ms
        }
    }

    bool Robotiq3fGripper::changeMode(OperationMode mode)
    {
        if (_rICS == 1)
        {
            std::cout << "!!!!! Robotiq3fGripper::changeMode: cannot change mode while in individual scissor control !!!!!" << std::endl;
            return false; // cannot change mode while in individual scissor control
        }

        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _rGTO = 0; // set to zero for avoiding unwanted movements

            _rMOD = static_cast<uint8_t>(mode);
            _new_command = true;
        }

        while (_new_command || _status.gSTA != 3)
        {
            usleep(5000); // wait 5 ms
        }

        return true;
    }

    void Robotiq3fGripper::move(uint8_t position, uint8_t finger_id, bool wait)
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            if (finger_id == FINGER_S)
            {
                if (_rICS == 1)
                {
                    _finger_S.rPR = position;
                }
                else
                {
                    std::cout << "!!!!! Robotiq3fGripper::move: cannot move scissor while not in individual scissor control !!!!!" << std::endl;
                    return;
                }
            }
            else
            {
                if (_rICF == 1)
                {
                    switch (finger_id)
                    {
                    case FINGER_A:
                        _finger_A.rPR = position;
                        break;
                    case FINGER_B:
                        _finger_B.rPR = position;
                        break;
                    case FINGER_C:
                        _finger_C.rPR = position;
                        break;
                    default:
                        // invalid finger id, print error message and return
                        std::cout << "!!!!! Robotiq3fGripper::move: invalid finger id !!!!!" << std::endl;
                        return;
                        break;
                    }
                }
                else
                {
                    _finger_A.rPR = position;
                    _finger_B.rPR = position;
                    _finger_C.rPR = position;
                    std::cout << "!!!!! Robotiq3fGripper::move: cannot move single finger while not in individual finger control, moving all fingers !!!!!" << std::endl;
                }
            }

            _rGTO = 1; // set to one for activating the movement
            _new_command = true;
        }

        if (wait)
        {
            waitForMotionComplete();
        }
    }

    void Robotiq3fGripper::setSpeed(uint8_t speed, uint8_t finger_id)
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        switch (finger_id)
        {
        case FINGER_A:
            _finger_A.rSP = speed;
            break;
        case FINGER_B:
            _finger_B.rSP = speed;
            break;
        case FINGER_C:
            _finger_C.rSP = speed;
            break;
        case FINGER_S:
            _finger_S.rSP = speed;
            break;
        default:
            std::cout << "!!!!! Robotiq3fGripper::setSpeed: invalid finger id !!!!!" << std::endl;
            break;
        }
    }

    void Robotiq3fGripper::setForce(uint8_t force, uint8_t finger_id)
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        switch (finger_id)
        {
        case FINGER_A:
            _finger_A.rFR = force;
            break;
        case FINGER_B:
            _finger_B.rFR = force;
            break;
        case FINGER_C:
            _finger_C.rFR = force;
            break;
        case FINGER_S:
            _finger_S.rFR = force;
            break;
        default:
            std::cout << "!!!!! Robotiq3fGripper::setForce: invalid finger id !!!!!" << std::endl;
            break;
        }
    }

    void Robotiq3fGripper::move(uint8_t position, bool wait)
    {
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _finger_A.rPR = position;
            _finger_B.rPR = position;
            _finger_C.rPR = position;
            _finger_S.rPR = position;
            _rGTO = 1; // set to one for activating the movement
            _new_command = true;
        }
        if (wait)
        {
            waitForMotionComplete();
        }
    }

    void Robotiq3fGripper::setSpeed(uint8_t speed)
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _finger_A.rSP = speed;
        _finger_B.rSP = speed;
        _finger_C.rSP = speed;
        _finger_S.rSP = speed;
    }

    void Robotiq3fGripper::setForce(uint8_t force)
    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _finger_A.rFR = force;
        _finger_B.rFR = force;
        _finger_C.rFR = force;
        _finger_S.rFR = force;
    }

    void Robotiq3fGripper::waitForMotionComplete()
    {
        while (_new_command || _status.gOBJ == 0) // wait until the gripper is in motion
        {
            usleep(1000); // wait 1 ms
        }
    }

    bool Robotiq3fGripper::readStatus()
    {
        // Wait for the minimum interval between messages
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _last_message_time).count() < MESSAGE_INTERVAL)
        {
            usleep(1000); // wait 1 ms
        }

        // Read the status registers
        int num_status_registers = 8;
        uint8_t values[num_status_registers * 2]; // each register is 2 bytes
        _client->readRegisters(_read_function_code, _first_status_register, num_status_registers, values);

        // Update the last message time from system clock time (using chrono)
        _last_message_time = std::chrono::system_clock::now();

        // Parse values[0], i.e., GRIPPER STATUS
        _status.gACT = values[0] & 0x01;        // gACT is bit 0 of the first register
        _status.gMOD = (values[0] & 0x06) >> 1; // gMOD is bits 1 and 2 of the first register
        _status.gGTO = (values[0] & 0x08) >> 3; // gGTO is bit 3 of the first register
        _status.gSTA = (values[0] & 0x30) >> 4; // gIMC is bits 4 and 5 of the first register
        _status.gOBJ = (values[0] & 0xC0) >> 6; // gOBJ is bits 6 and 7 of the first register
        // Parse values[1], i.e., OBJECT STATUS
        _finger_A.status.gDT = values[1] & 0x03;        // gDTA is bits 0 and 1 of the second register
        _finger_B.status.gDT = (values[1] & 0x0C) >> 2; // gDTB is bits 2 and 3 of the second register
        _finger_C.status.gDT = (values[1] & 0x30) >> 4; // gDTC is bits 4 and 5 of the second register
        _finger_S.status.gDT = (values[1] & 0xC0) >> 6; // gDTS is bits 6 and 7 of the second register
        // Parse values[2], i.e., FAULT STATUS
        _status.gFLT = values[2] & 0x0F; // gFLT is bits 0 to 3 of the third register
        // Parse other values
        _finger_A.status.gPR = values[3];
        _finger_A.status.gPO = values[4];
        _finger_A.status.gCU = values[5];
        _finger_B.status.gPR = values[6];
        _finger_B.status.gPO = values[7];
        _finger_B.status.gCU = values[8];
        _finger_C.status.gPR = values[9];
        _finger_C.status.gPO = values[10];
        _finger_C.status.gCU = values[11];
        _finger_S.status.gPR = values[12];
        _finger_S.status.gPO = values[13];
        _finger_S.status.gCU = values[14];

#if ROBOTIQ_3F_GRIPPER_DEBUG
        _printStatus();
#endif

        return true;
    }

    bool Robotiq3fGripper::sendCommand()
    {
        // Build the request message, filling it initially with zeros
        int num_command_registers = 8; // each register is 2 bytes
        uint8_t request[num_command_registers * 2] = {0};
        // byte 0 <- bit 0: rACT, bits 1-2: rMOD, bit 3: rGTO, bit 4: rATR, bits 5-7: zeros
        request[0] = _rACT | (_rMOD << 1) | (_rGTO << 3) | (_rATR << 4);
        // byte 1 <- bits 0-1: zeros, bit 2: rICF, bit 3: rICS, bits 4-7: zeros
        request[1] = (_rICF << 2) | (_rICS << 3);
        // byte 2 <- bits 0-7: zeros
        // byte 3-5 <- fingerA: rPR, rSP, rFR
        request[3] = _finger_A.rPR;
        request[4] = _finger_A.rSP;
        request[5] = _finger_A.rFR;
        // byte 6-8 <- fingerB: rPR, rSP, rFR
        request[6] = _finger_B.rPR;
        request[7] = _finger_B.rSP;
        request[8] = _finger_B.rFR;
        // byte 9-11 <- fingerC: rPR, rSP, rFR
        request[9] = _finger_C.rPR;
        request[10] = _finger_C.rSP;
        request[11] = _finger_C.rFR;
        // byte 12-14 <- fingerS: rPR, rSP, rFR
        request[12] = _finger_S.rPR;
        request[13] = _finger_S.rSP;
        request[14] = _finger_S.rFR;
        // byte 15 <- bits 0-7: zeros

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

    GripperStatus Robotiq3fGripper::getGripperStatus()
    {
        return _status;
    }

    std::vector<Finger> Robotiq3fGripper::getFingersStatus()
    {
        std::vector<Finger> fingers;
        fingers.push_back(_finger_A);
        fingers.push_back(_finger_B);
        fingers.push_back(_finger_C);
        fingers.push_back(_finger_S);
        return fingers;
    }

    void Robotiq3fGripper::printStatus()
    {
        std::cout << "gACT: " << (int)_status.gACT << std::endl;
        std::cout << "gMOD: " << (int)_status.gMOD << std::endl;
        std::cout << "gGTO: " << (int)_status.gGTO << std::endl;
        std::cout << "gIMC: " << (int)_status.gSTA << std::endl;
        std::cout << "gOBJ: " << (int)_status.gOBJ << std::endl;
        std::cout << "gFLT: " << (int)_status.gFLT << std::endl;
        std::cout << "gDTA: " << (int)_finger_A.status.gDT << std::endl;
        std::cout << "gDTB: " << (int)_finger_B.status.gDT << std::endl;
        std::cout << "gDTC: " << (int)_finger_C.status.gDT << std::endl;
        std::cout << "gDTS: " << (int)_finger_S.status.gDT << std::endl;
        std::cout << "gPR: " << (int)_finger_A.status.gPR << " " << (int)_finger_B.status.gPR << " " << (int)_finger_C.status.gPR << " " << (int)_finger_S.status.gPR << std::endl;
        std::cout << "gPO: " << (int)_finger_A.status.gPO << " " << (int)_finger_B.status.gPO << " " << (int)_finger_C.status.gPO << " " << (int)_finger_S.status.gPO << std::endl;
        std::cout << "gCU: " << (int)_finger_A.status.gCU << " " << (int)_finger_B.status.gCU << " " << (int)_finger_C.status.gCU << " " << (int)_finger_S.status.gCU << std::endl;
    }
} // namespace uclv
