#ifndef MODBUS_H
#define MODBUS_H

#include <cstdint>
#include <vector>
#include <string>
#include <unistd.h>
#include <iostream>
#include "uclv_modbus/crc_calc.h"

namespace uclv {
/**
 * @brief Abstract class for handling Modbus communication protocol.
 */
class Modbus {
public:
    /**
     * @brief Destructor for Modbus class.
     */
    virtual ~Modbus() {}

    /**
     * @brief Connects to the Modbus device.
     * @return True if the connection was successful, false otherwise.
     */
    virtual bool connect() = 0;

    /**
     * @brief Disconnects from the Modbus device.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Checks if the Modbus device is connected.
     * @return True if the device is connected, false otherwise.
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief Reads a number of registers from the Modbus device.
     * @param start_address The starting address of the registers to read.
     * @param num_registers The number of registers to read.
     * @param values The vector to store the read values in.
     * @return True if the read was successful, false otherwise.
     */
    virtual bool readRegisters(uint8_t function_code, uint16_t start_address, uint16_t num_registers, std::vector<uint16_t>& values) = 0;

    /**
     * @brief Writes a number of registers to the Modbus device.
     * @param start_address The starting address of the registers to write.
     * @param values The vector of values to write.
     * @return True if the write was successful, false otherwise.
     */
    virtual bool writeRegisters(uint16_t start_address, const std::vector<uint16_t>& values) = 0;
};

} // namespace uclv

#endif // MODBUS_H
