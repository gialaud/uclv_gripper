#include "uclv_modbus/modbus_rtu.h"

namespace uclv
{
    ModbusRTU::ModbusRTU(const char* device_path, uint8_t slave_id) : Modbus(slave_id)
    {
        connection_interface_ = std::unique_ptr<USB>(new USB(device_path));
    }

    ModbusRTU::ModbusRTU(const char* ip_address, int port, uint8_t slave_id) : Modbus(slave_id)
    {
        connection_interface_ = std::unique_ptr<Ethernet>(new Ethernet(ip_address, port));
    }

    ModbusRTU::~ModbusRTU()
    {
        std::cout << "Called ModbusRTU destructor" << std::endl;
        
    }

    void ModbusRTU::sendRequest(uint8_t *request, uint16_t request_length, uint8_t *response_buffer, uint16_t response_length)
    {
        // add crc to request
        uint16_t crc = calculateCRC(request, request_length);
        uint8_t request_with_crc[request_length + 2];
        std::copy(request, request + request_length, request_with_crc);
        request_with_crc[request_length] = crc & 0xFF;
        request_with_crc[request_length + 1] = crc >> 8;

        uint8_t temp_response_buffer[response_length + 2];
        // send request checking for errors in the reply
        connection_interface_->sendAndReceive(request_with_crc, request_length + 2, temp_response_buffer, response_length + 2);

        // TODO: check for errors in the reply (CRC)

        // remove crc from response
        std::copy(temp_response_buffer, temp_response_buffer + response_length, response_buffer);
    }
} // namespace uclv