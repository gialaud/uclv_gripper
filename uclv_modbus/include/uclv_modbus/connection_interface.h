#ifndef UCLV_MODBUS__CONNECTION_INTERFACE_H
#define UCLV_MODBUS__CONNECTION_INTERFACE_H

#include <cstdint>
#include <cstddef>
#include <unistd.h>
#include <iostream>

namespace uclv {

/**
 * @brief Abstract class for handling the connection interface to a Modbus device.
 */

class ConnectionInterface {
public:
    /**
     * @brief Constructor for ConnectionInterface class.
     */
    ConnectionInterface() {}

    /**
     * @brief Destructor for ConnectionInterface class.
     */
    ~ConnectionInterface();

    /**
     * @brief Connects to the Modbus device.
     * @return True if the connection was successful, false otherwise.
     */
    virtual bool connect() = 0;

    /**
     * @brief Disconnects from the Modbus device.
     */
    virtual void disconnect();

    /**
     * @brief Sends data to the Modbus device and receives data from the Modbus device.
     * @param send_data The data to send.
     * @param send_length The length of the data to send.
     * @param receive_data The buffer to store the received data in.
     * @param receive_length The length of the data to receive.
     */
    virtual void sendAndReceive(const uint8_t* send_data, size_t send_length, uint8_t* receive_data, size_t receive_length);

protected:

    /**
     * @brief Sends data to the Modbus device.
     * @param data The data to send.
     * @param length The length of the data to send.
     */
    virtual void send(const uint8_t* data, size_t length);

    /**
     * @brief Receives data from the Modbus device.
     * @param data The buffer to store the received data in.
     * @param length The length of the data to receive.
     */
    virtual void receive(uint8_t* data, size_t length);

protected:
    /**
     * @brief File descriptor for the connection.
     */
    int fd_{-1};
};

} // namespace uclv

#endif // UCLV_MODBUS__CONNECTION_INTERFACE_H