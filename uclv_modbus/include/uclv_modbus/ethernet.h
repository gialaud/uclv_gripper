#ifndef UCLV_MODBUS__ETHERNET_H
#define UCLV_MODBUS__ETHERNET_H

#include <sys/socket.h>
#include <arpa/inet.h>

#include "uclv_modbus/connection_interface.h"

namespace uclv {

/**
 * @brief Class for handling the connection interface to a Modbus device via Ethernet.
 */

class Ethernet : public ConnectionInterface {
public:
    /**
     * @brief Default constructor for Ethernet class is removed
     */
    Ethernet() = delete;

    /**
     * @brief Constructor for Ethernet class.
     * @param ip_address IP address of the Modbus device.
     * @param port Port of the Modbus device.
     */
    Ethernet(const char* ip_address, int port);

    /**
     * @brief Destructor for Ethernet class.
     */
    ~Ethernet();

    /**
     * @brief Connects to the Modbus device via Ethernet.
     * @return True if the connection was successful, false otherwise.
     */
    bool connect();

private:
    /**
     * @brief IP address of the Modbus device.
     */
    const char* ip_address_;

    /**
     * @brief Port of the Modbus device.
     */
    int port_;
};

} // namespace uclv

#endif // UCLV_MODBUS__ETHERNET_H