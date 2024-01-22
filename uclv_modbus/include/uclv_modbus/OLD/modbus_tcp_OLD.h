#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "uclv_modbus/modbus.h"

namespace uclv {

/**
 * @brief Class for handling Modbus TCP communication protocol.
 */
class ModbusTCP : public Modbus {
public:
    /**
     * @brief Constructor for ModbusTCP class.
     * @param ip_address The IP address of the Modbus TCP device.
     * @param port The port number of the Modbus TCP device.
     * @param slave_id The slave ID to use for communication.
     */
    ModbusTCP(const std::string& ip_address, uint16_t port, uint8_t slave_id);

    /**
     * @brief Destructor for ModbusTCP class.
     */
    ~ModbusTCP();

    /**
     * @brief Connects to the Modbus TCP device.
     * @return True if the connection was successful, false otherwise.
     */
    bool connect() override;

    /**
     * @brief Disconnects from the Modbus TCP device.
     */
    void disconnect() override;

    /**
     * @brief Checks if the Modbus TCP device is connected.
     * @return True if the device is connected, false otherwise.
     */
    bool isConnected() const override;

    /**
     * @brief Reads a number of registers from the Modbus TCP device.
     * @param start_address The starting address of the registers to read.
     * @param num_registers The number of registers to read.
     * @param values The vector to store the read values in.
     * @return True if the read was successful, false otherwise.
     */
    bool readRegisters(uint8_t function_code, uint16_t start_address, uint16_t num_registers, std::vector<uint16_t>& values) override;

    /**
     * @brief Writes a number of registers to the Modbus TCP device.
     * @param start_address The starting address of the registers to write.
     * @param values The vector of values to write.
     * @return True if the write was successful, false otherwise.
     */
    bool writeRegisters(uint16_t start_address, const std::vector<uint16_t>& values) override;

private:
    std::string _ip_address; /**< The IP address of the Modbus TCP device. */
    uint16_t _port; /**< The port number of the Modbus TCP device. */
    int _sockfd; /**< The socket file descriptor. */
    bool _connected; /**< Flag indicating if the device is connected. */
    uint8_t _slave_id; /**< The slave ID to use for communication. */
    uint16_t _transaction_id; /**< The transaction ID. */

    /**
     * @brief Sends a request to the Modbus TCP device.
     * @param request The request buffer to send.
     * @param request_length The length of the request buffer.
     * @return True if the request was sent successfully, false otherwise.
     */
    bool _sendRequest(const uint8_t* request, uint16_t request_length);

    /**
     * @brief Receives a response from the Modbus TCP device.
     * @param response The response buffer to receive.
     * @param response_length The length of the response buffer.
     * @return True if the response was received successfully, false otherwise.
     */
    uint16_t _receiveResponse(uint8_t* response, uint16_t response_length);
};

} // namespace uclv

#endif // MODBUS_TCP_H