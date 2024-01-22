#ifndef UCLV_MODBUS__MODBUS_H
#define UCLV_MODBUS__MODBUS_H

#include <cstdint>
#include <vector>
#include <string>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <iomanip>
#include "uclv_modbus/connection_interface.h"

namespace uclv
{
    /**
     * @brief Abstract class for handling Modbus communication protocol.
     */
    class Modbus
    {
    public:
        enum FunctionCode
        {
            READ_COILS = 0x01,
            READ_DISCRETE_INPUTS = 0x02,
            READ_HOLDING_REGISTERS = 0x03,
            READ_INPUT_REGISTERS = 0x04,
            WRITE_SINGLE_COIL = 0x05,
            WRITE_SINGLE_REGISTER = 0x06,
            WRITE_MULTIPLE_COILS = 0x0F,
            WRITE_MULTIPLE_REGISTERS = 0x10,
            READ_AND_WRITE_MULTIPLE_REGISTERS = 0x17
        };

        /**
         * @brief Constructor for Modbus class.
         * @param slave_id The slave ID of the Modbus device.
         */
        Modbus(uint8_t slave_id);

        /**
         * @brief Destructor for Modbus class.
         */
        virtual ~Modbus() {}

        /**
         * @brief Connects to the Modbus device.
         * @return True if the connection was successful, false otherwise.
         */
        bool connect();

        /**
         * @brief Disconnects from the Modbus device.
         */
        void disconnect();

        /**
         * @brief Set the endianness of the Modbus communication as little endian.
         */
        void setLittleEndian();

        /**
         * @brief Read registers from the Modbus device.
         * @param function_code The function code of the request.
         * @param address The address of the first register to read.
         * @param num_registers The number of registers to read.
         * @param buffer The buffer to store the read values in (16 bits values).
         */
        void readRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, uint16_t *buffer);

        /**
         * @brief Read registers from the Modbus device.
         * @param function_code The function code of the request.
         * @param address The address of the first register to read.
         * @param num_registers The number of registers to read.
         * @param buffer The buffer to store the read values in (8 bits values).
         */
        void readRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, uint8_t *buffer);

        /**
         * @brief Write registers to the Modbus device.
         * @param function_code The function code of the request.
         * @param address The address of the register to write.
         * @param value The value to write.
         */
        void writeRegister(FunctionCode function_code, uint16_t address, uint16_t value);

        /**
         * @brief Write registers to the Modbus device.
         * @param function_code The function code of the request.
         * @param address The address of the first register to write.
         * @param num_registers The number of registers to write.
         * @param values The values to write. (16 bits)  
         */
        void writeRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, const uint16_t *values);
        /**
         * @brief Write registers to the Modbus device.
         * @param function_code The function code of the request.
         * @param address The address of the first register to write.
         * @param num_registers The number of registers to write.
         * @param values The values to write. (8 bits)  
         */
        void writeRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, const uint8_t *values);
        /**
         * @brief Read and write registers to the Modbus device.
         * @param read_address The address of the first register to read.
         * @param num_read_registers The number of registers to read.
         * @param response_buffer The buffer to store the response in (only registers values - 16 bits).
         * @param write_address The address of the first register to write.
         * @param num_write_registers The number of registers to write.
         * @param values The buffer to read the write data from (16 bits).
         */
        void readWriteRegisters(uint16_t read_address, uint16_t num_read_registers, uint16_t *response_buffer, uint16_t write_address, uint16_t num_write_registers, const uint16_t *values);

        /**
         * @brief Read and write registers to the Modbus device.
         * @param read_address The address of the first register to read.
         * @param num_read_registers The number of registers to read.
         * @param response_buffer The buffer to store the response in (only registers values - 8 bits).
         * @param write_address The address of the first register to write.
         * @param num_write_registers The number of registers to write.
         * @param values The buffer to read the write data from (8 bits).
         */
        void readWriteRegisters(uint16_t read_address, uint16_t num_read_registers, uint8_t *response_buffer, uint16_t write_address, uint16_t num_write_registers, const uint8_t *values);

    protected:
    // Methods

        /**
         * @brief Send the Modbus reading request.
         * @param request The request to send.
         * @param request_length The length of the request.
         * @param response_buffer The buffer to store the response in (only registers values).
         * @param response_length The length of the response (only registers values).
         */ 
        virtual void sendRequest(uint8_t *request, uint16_t request_length, uint8_t *response_buffer, uint16_t response_length) = 0;

    // Attributes
        /**
         * @brief The slave ID to use for communication.
         */
        uint8_t slave_id_;

        /**
         * @brief The connection interface to use for communication.
         */
        std::unique_ptr<ConnectionInterface> connection_interface_;

        /**
         * @brief Parameter to store the endianness of the Modbus communication.
         */
        bool is_little_endian_{false};
    };

} // namespace uclv

#endif // UCLV_MODBUS__MODBUS_H
