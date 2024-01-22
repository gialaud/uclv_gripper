#ifndef UCLV_MODBUS__USB_H
#define UCLV_MODBUS__USB_H

#include <termios.h>
#include <fcntl.h>

#include "uclv_modbus/connection_interface.h"

namespace uclv {

/**
 * @brief Class for handling the connection interface to a Modbus device via USB.
 */
class USB : public ConnectionInterface {
public:
    /**
     * @brief Default constructor for USB class is removed
     */
    USB() = delete;

    /**
     * @brief Constructor for USB class.
     * @param device_path Path to the USB device.
     */
    USB(const char* device_path);

    /**
     * @brief Destructor for USB class.
     */
    ~USB();

    /**
     * @brief Connects to the Modbus device via USB.
     * @return True if the connection was successful, false otherwise.
     */
    bool connect();

private:
    /**
     * @brief Path to the USB device.
     */
    const char* device_path_;
};

} // namespace uclv

#endif // UCLV_MODBUS__USB_H
