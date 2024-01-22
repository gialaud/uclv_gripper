#ifndef MODBUS_RTU_TCP_H
#define MODBUS_RTU_TCP_H

#include <cstdint>
#include <vector>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include "uclv_modbus/modbus.h"

namespace uclv {

/**
 * @brief Class for communicating with a Modbus RTU device over TCP.
 * 
 * This class provides methods for connecting to and communicating with a Modbus RTU device over TCP.
 * It supports reading and writing a number of registers from/to the device.
 */
class ModbusRTUOverTCP : public Modbus {
public:
    /**
     * @brief Constructor for ModbusRTUOverTCP class.
     * @param ip_address The IP address of the Modbus RTU device.
     * @param port The port number to use for communication.
     * @param slave_id The slave ID to use for communication.
     */
    ModbusRTUOverTCP(const std::string& ip_address, uint16_t port, uint8_t slave_id);

    /**
     * @brief Destructor for ModbusRTUOverTCP class.
     */
    ~ModbusRTUOverTCP();

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
    std::string _ip_address; /**< The IP address of the Modbus RTU device. */
    uint16_t _port; /**< The port number to use for communication. */
    uint8_t _slave_id; /**< The slave ID to use for communication. */
    int _sockfd; /**< The file descriptor for the socket. */
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

#endif // MODBUS_RTU_TCP_H
