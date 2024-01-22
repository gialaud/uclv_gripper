#include "uclv_modbus/modbus.h"

namespace uclv
{
    #define MODBUS_DEBUG 0
    Modbus::Modbus(uint8_t slave_id) : slave_id_(slave_id)
    {
    }

    bool Modbus::connect()
    {
        return connection_interface_->connect();
    }

    void Modbus::disconnect()
    {
        connection_interface_->disconnect();
    }

    void Modbus::setLittleEndian()
    {
        is_little_endian_ = true;
    }

    void Modbus::readRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, uint16_t *buffer)
    {
        uint8_t request[6];
        uint8_t temp_response_buffer[3 + num_registers * 2]; // used for the raw Modbus response (slave id + function code + num. bytes + 2 bytes per register)

        request[0] = static_cast<uint8_t>(slave_id_);
        request[1] = static_cast<uint8_t>(function_code);
        request[2] = static_cast<uint8_t>(address >> 8);
        request[3] = static_cast<uint8_t>(address & 0xFF);
        request[4] = static_cast<uint8_t>(num_registers >> 8);
        request[5] = static_cast<uint8_t>(num_registers & 0xFF);

#if MODBUS_DEBUG
        std::cout << "Modbus::readRegisters: sending request:" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)request[i] << " ";
        }
        std::cout << std::endl;
#endif

        sendRequest(request, 6, temp_response_buffer, 3 + num_registers * 2);

#if MODBUS_DEBUG
        std::cout << "Modbus::readRegisters: received response:" << std::endl;
        for (int i = 0; i < 3 + num_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)temp_response_buffer[i] << " ";
        }
        std::cout << std::endl;
#endif

        // check if the response is valid
        if (temp_response_buffer[0] != slave_id_ ||
            temp_response_buffer[1] != static_cast<uint8_t>(function_code) ||
            temp_response_buffer[2] != num_registers * 2)
        {
            throw std::runtime_error("Received invalid response from Modbus device");
        }

        // copy the registers values to the buffer (8 bits to 16 bits)
        for (int i = 0; i < num_registers; i++)
        {
            if (is_little_endian_)
            {
                buffer[i] = temp_response_buffer[3 + i * 2] | (temp_response_buffer[3 + i * 2 + 1] << 8);
            }
            else
            {
                buffer[i] = (temp_response_buffer[3 + i * 2] << 8) | temp_response_buffer[3 + i * 2 + 1];
            }
        }

#if MODBUS_DEBUG
        std::cout << "Modbus::readRegisters: copied response to buffer:" << std::endl;
        for (int i = 0; i < num_registers; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(4) << buffer[i] << " ";
        }
        std::cout << std::endl;
#endif
    }

    void Modbus::readRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, uint8_t *buffer)
    {
        uint8_t request[6];
        uint8_t temp_response_buffer[3 + num_registers * 2]; // used for the raw Modbus response

        request[0] = static_cast<uint8_t>(slave_id_);
        request[1] = static_cast<uint8_t>(function_code);
        request[2] = static_cast<uint8_t>(address >> 8);
        request[3] = static_cast<uint8_t>(address & 0xFF);
        request[4] = static_cast<uint8_t>(num_registers >> 8);
        request[5] = static_cast<uint8_t>(num_registers & 0xFF);

#if MODBUS_DEBUG
        std::cout << "Modbus::readRegisters: sending request:" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)request[i] << " ";
        }
        std::cout << std::endl;
#endif

        sendRequest(request, 6, temp_response_buffer, 3 + num_registers * 2);

#if MODBUS_DEBUG
        std::cout << "Modbus::readRegisters: received response:" << std::endl;
        for (int i = 0; i < 3 + num_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)temp_response_buffer[i] << " ";
        }
        std::cout << std::endl;
#endif

        // check if the response is valid
        if (temp_response_buffer[0] != slave_id_ ||
            temp_response_buffer[1] != static_cast<uint8_t>(function_code) ||
            temp_response_buffer[2] != num_registers * 2)
        {
            throw std::runtime_error("Received invalid response from Modbus device");
        }

        // copy the registers values to the buffer
        std::copy(temp_response_buffer + 3, temp_response_buffer + 3 + num_registers * 2, buffer); // copy only the registers values
    }

    void Modbus::writeRegister(FunctionCode function_code, uint16_t address, uint16_t value)
    {
        uint8_t request[6];
        uint8_t temp_response_buffer[6]; // used for the raw Modbus response (echo of the request)

        request[0] = static_cast<uint8_t>(slave_id_);
        request[1] = static_cast<uint8_t>(function_code);
        request[2] = static_cast<uint8_t>(address >> 8);
        request[3] = static_cast<uint8_t>(address & 0xFF);
        request[4] = static_cast<uint8_t>(value >> 8);
        request[5] = static_cast<uint8_t>(value & 0xFF);

#if MODBUS_DEBUG
        std::cout << "Modbus::writeRegister: sending request:" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)request[i] << " ";
        }
        std::cout << std::endl;
#endif

        sendRequest(request, 6, temp_response_buffer, 6);

#if MODBUS_DEBUG
        std::cout << "Modbus::writeRegister: received response:" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)temp_response_buffer[i] << " ";
        }
        std::cout << std::endl;
#endif

        // check if the response is valid
        if (temp_response_buffer[0] != slave_id_ ||
            temp_response_buffer[1] != static_cast<uint8_t>(function_code) ||
            temp_response_buffer[2] != request[2] ||
            temp_response_buffer[3] != request[3] ||
            temp_response_buffer[4] != request[4] ||
            temp_response_buffer[5] != request[5])
        {
            throw std::runtime_error("Received invalid response from Modbus device");
        }
    }

    void Modbus::writeRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, const uint16_t *values)
    {
        uint8_t request[7 + num_registers * 2];
        uint8_t temp_response_buffer[6]; // used for the raw Modbus response

        request[0] = static_cast<uint8_t>(slave_id_);
        request[1] = static_cast<uint8_t>(function_code);
        request[2] = static_cast<uint8_t>(address >> 8);
        request[3] = static_cast<uint8_t>(address & 0xFF);
        request[4] = static_cast<uint8_t>(num_registers >> 8);
        request[5] = static_cast<uint8_t>(num_registers & 0xFF);
        request[6] = static_cast<uint8_t>(num_registers * 2);

        for (int i = 0; i < num_registers; i++)
        {
            if (is_little_endian_)
            {
                request[7 + i * 2] = static_cast<uint8_t>(values[i] & 0xFF);
                request[7 + i * 2 + 1] = static_cast<uint8_t>(values[i] >> 8);
            }
            else
            {
                request[7 + i * 2] = static_cast<uint8_t>(values[i] >> 8);
                request[7 + i * 2 + 1] = static_cast<uint8_t>(values[i] & 0xFF);
            }
        }

#if MODBUS_DEBUG
        std::cout << "Modbus::writeRegisters: sending request:" << std::endl;
        for (int i = 0; i < 7 + num_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)request[i] << " ";
        }
        std::cout << std::endl;
#endif

        sendRequest(request, 7 + num_registers * 2, temp_response_buffer, 6);

#if MODBUS_DEBUG
        std::cout << "Modbus::writeRegisters: received response:" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)temp_response_buffer[i] << " ";
        }
        std::cout << std::endl;
#endif

        // check if the response is valid
        if (temp_response_buffer[0] != slave_id_ ||
            temp_response_buffer[1] != static_cast<uint8_t>(function_code) ||
            temp_response_buffer[2] != request[2] ||
            temp_response_buffer[3] != request[3] ||
            temp_response_buffer[4] != request[4] ||
            temp_response_buffer[5] != request[5])
        {
            throw std::runtime_error("Received invalid response from Modbus device");
        }
    }

    void Modbus::writeRegisters(FunctionCode function_code, uint16_t address, uint16_t num_registers, const uint8_t *values)
    {
        uint8_t request[7 + num_registers * 2];
        uint8_t temp_response_buffer[6]; // used for the raw Modbus response

        request[0] = static_cast<uint8_t>(slave_id_);
        request[1] = static_cast<uint8_t>(function_code);
        request[2] = static_cast<uint8_t>(address >> 8);
        request[3] = static_cast<uint8_t>(address & 0xFF);
        request[4] = static_cast<uint8_t>(num_registers >> 8);
        request[5] = static_cast<uint8_t>(num_registers & 0xFF);
        request[6] = static_cast<uint8_t>(num_registers * 2);

        // copy the values to the request
        std::copy(values, values + num_registers * 2, request + 7);

#if MODBUS_DEBUG
        std::cout << "Modbus::writeRegisters: sending request:" << std::endl;
        for (int i = 0; i < 7 + num_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)request[i] << " ";
        }
        std::cout << std::endl;
#endif

        sendRequest(request, 7 + num_registers * 2, temp_response_buffer, 6);

#if MODBUS_DEBUG
        std::cout << "Modbus::writeRegisters: received response:" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)temp_response_buffer[i] << " ";
        }
        std::cout << std::endl;
#endif
        
        // check if the response is valid
        if (temp_response_buffer[0] != slave_id_ ||
            temp_response_buffer[1] != static_cast<uint8_t>(function_code) ||
            temp_response_buffer[2] != request[2] ||
            temp_response_buffer[3] != request[3] ||
            temp_response_buffer[4] != request[4] ||
            temp_response_buffer[5] != request[5])
        {
            throw std::runtime_error("Received invalid response from Modbus device");
        }
    }

    void Modbus::readWriteRegisters(uint16_t read_address, uint16_t num_read_registers, uint16_t *response_buffer, uint16_t write_address, uint16_t num_write_registers, const uint16_t *write_values)
    {
        uint8_t request[11 + num_write_registers * 2];
        uint8_t temp_response_buffer[3 + num_read_registers * 2]; // used for the raw Modbus response

        request[0] = static_cast<uint8_t>(slave_id_);
        request[1] = static_cast<uint8_t>(FunctionCode::READ_AND_WRITE_MULTIPLE_REGISTERS);
        request[2] = static_cast<uint8_t>(read_address >> 8);
        request[3] = static_cast<uint8_t>(read_address & 0xFF);
        request[4] = static_cast<uint8_t>(num_read_registers >> 8);
        request[5] = static_cast<uint8_t>(num_read_registers & 0xFF);
        request[6] = static_cast<uint8_t>(write_address >> 8);
        request[7] = static_cast<uint8_t>(write_address & 0xFF);
        request[8] = static_cast<uint8_t>(num_write_registers >> 8);
        request[9] = static_cast<uint8_t>(num_write_registers & 0xFF);
        request[10] = static_cast<uint8_t>(num_write_registers * 2);

        for (int i = 0; i < num_write_registers; i++)
        {
            if (is_little_endian_)
            {
                request[11 + i * 2] = static_cast<uint8_t>(write_values[i] & 0xFF);
                request[11 + i * 2 + 1] = static_cast<uint8_t>(write_values[i] >> 8);
            }
            else
            {
                request[11 + i * 2] = static_cast<uint8_t>(write_values[i] >> 8);
                request[11 + i * 2 + 1] = static_cast<uint8_t>(write_values[i] & 0xFF);
            }
        }

#if MODBUS_DEBUG
        std::cout << "Modbus::readWriteRegisters: sending request:" << std::endl;
        for (int i = 0; i < 11 + num_write_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)request[i] << " ";
        }
        std::cout << std::endl;
#endif

        sendRequest(request, 11 + num_write_registers * 2, temp_response_buffer, 3 + num_read_registers * 2);

#if MODBUS_DEBUG
        std::cout << "Modbus::readWriteRegisters: received response:" << std::endl;
        for (int i = 0; i < 3 + num_read_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)temp_response_buffer[i] << " ";
        }
        std::cout << std::endl;
#endif

        // check if the response is valid
        if (temp_response_buffer[0] != slave_id_ ||
            temp_response_buffer[1] != static_cast<uint8_t>(FunctionCode::READ_AND_WRITE_MULTIPLE_REGISTERS) ||
            temp_response_buffer[2] != num_read_registers * 2)
        {
            throw std::runtime_error("Received invalid response from Modbus device");
        }

        // copy the registers values to the buffer (8 bits to 16 bits)
        for (int i = 0; i < num_read_registers; i++)
        {
            if (is_little_endian_)
            {
                response_buffer[i] = temp_response_buffer[3 + i * 2] | (temp_response_buffer[3 + i * 2 + 1] << 8);
            }
            else
            {
                response_buffer[i] = (temp_response_buffer[3 + i * 2] << 8) | temp_response_buffer[3 + i * 2 + 1];
            }
        }
    }

    void Modbus::readWriteRegisters(uint16_t read_address, uint16_t num_read_registers, uint8_t *response_buffer, uint16_t write_address, uint16_t num_write_registers, const uint8_t *write_values)
    {
        uint8_t request[11 + num_write_registers * 2];
        uint8_t temp_response_buffer[3 + num_read_registers * 2]; // used for the raw Modbus response

        request[0] = static_cast<uint8_t>(slave_id_);
        request[1] = static_cast<uint8_t>(FunctionCode::READ_AND_WRITE_MULTIPLE_REGISTERS);
        request[2] = static_cast<uint8_t>(read_address >> 8);
        request[3] = static_cast<uint8_t>(read_address & 0xFF);
        request[4] = static_cast<uint8_t>(num_read_registers >> 8);
        request[5] = static_cast<uint8_t>(num_read_registers & 0xFF);
        request[6] = static_cast<uint8_t>(write_address >> 8);
        request[7] = static_cast<uint8_t>(write_address & 0xFF);
        request[8] = static_cast<uint8_t>(num_write_registers >> 8);
        request[9] = static_cast<uint8_t>(num_write_registers & 0xFF);
        request[10] = static_cast<uint8_t>(num_write_registers * 2);

        std::copy(write_values, write_values + num_write_registers * 2, request + 11);

#if MODBUS_DEBUG
        std::cout << "Modbus::readWriteRegisters: sending request:" << std::endl;
        for (int i = 0; i < 11 + num_write_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)request[i] << " ";
        }
        std::cout << std::endl;
#endif

        sendRequest(request, 11 + num_write_registers * 2, temp_response_buffer, 3 + num_read_registers * 2);

#if MODBUS_DEBUG
        std::cout << "Modbus::readWriteRegisters: received response:" << std::endl;
        for (int i = 0; i < 3 + num_read_registers * 2; i++)
        {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)temp_response_buffer[i] << " ";
        }
        std::cout << std::endl;
#endif

        // check if the response is valid
        if (temp_response_buffer[0] != slave_id_ ||
            temp_response_buffer[1] != static_cast<uint8_t>(FunctionCode::READ_AND_WRITE_MULTIPLE_REGISTERS) ||
            temp_response_buffer[2] != num_read_registers * 2)
        {
            throw std::runtime_error("Received invalid response from Modbus device");
        }

        // copy the registers values to the buffer
        std::copy(temp_response_buffer + 3, temp_response_buffer + 3 + num_read_registers * 2, response_buffer);
    }
}