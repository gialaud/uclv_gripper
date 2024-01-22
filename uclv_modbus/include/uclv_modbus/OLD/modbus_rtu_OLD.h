#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include "uclv_modbus/modbus.h"

namespace uclv {

/**
 * @brief Class for handling Modbus RTU communication protocol.
 */
class ModbusRTU : public Modbus{
public:
    /**
     * @brief Constructor for ModbusRTU class.
     * @param port The serial port to use for communication.
     * @param baudrate The baudrate to use for communication.
     * @param slave_id The slave ID to use for communication.
     */
    ModbusRTU(const std::string& port, uint32_t baudrate, uint8_t slave_id);

    /**
     * @brief Destructor for ModbusRTU class.
     */
    ~ModbusRTU();

    /**
     * @brief Connects to the Modbus RTU device.
     * @return True if the connection was successful, false otherwise.
     */
    bool connect() override;

    /**
     * @brief Disconnects from the Modbus RTU device.
     */
    void disconnect() override;

    /**
     * @brief Checks if the Modbus RTU device is connected.
     * @return True if the device is connected, false otherwise.
     */
    bool isConnected() const override;

    /**
     * @brief Reads a number of registers from the Modbus RTU device.
     * @param start_address The starting address of the registers to read.
     * @param num_registers The number of registers to read.
     * @param values The vector to store the read values in.
     * @return True if the read was successful, false otherwise.
     */
    bool readRegisters(uint8_t function_code, uint16_t start_address, uint16_t num_registers, std::vector<uint16_t>& values) override;

    /**
     * @brief Writes a number of registers to the Modbus RTU device.
     * @param start_address The starting address of the registers to write.
     * @param values The vector of values to write.
     * @return True if the write was successful, false otherwise.
     */
    bool writeRegisters(uint16_t start_address, const std::vector<uint16_t>& values) override;

private:
    std::string _port; /**< The serial port to use for communication. */
    uint32_t _baudrate; /**< The baudrate to use for communication. */
    uint8_t _slave_id; /**< The slave ID to use for communication. */
    int _fd; /**< The file descriptor for the serial port. */
    bool _connected; /**< Flag indicating if the device is connected. */

    /**
     * @brief Sends a request to the Modbus RTU device.
     * @param request The request buffer to send.
     * @param request_length The length of the request buffer.
     * @return True if the request was sent successfully, false otherwise.
     */
    bool _sendRequest(const uint8_t* request, uint16_t request_length);

    /**
     * @brief Receives a response from the Modbus RTU device.
     * @param response The response buffer to receive.
     * @param response_length The length of the response buffer.
     * @return True if the response was received successfully, false otherwise.
     */
    uint16_t _receiveResponse(uint8_t* response, uint16_t response_length);
};

} // namespace uclv

#endif // MODBUS_RTU_H
