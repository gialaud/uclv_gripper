#include "uclv_modbus/modbus_tcp.h"

namespace uclv
{
    ModbusTCP::ModbusTCP(const char* ip_address, int port, uint8_t slave_id) : Modbus(slave_id)
    {
        connection_interface_ = std::unique_ptr<Ethernet>(new Ethernet(ip_address, port));
    }

    ModbusTCP::~ModbusTCP()
    {
        std::cout << "Called ModbusTCP destructor" << std::endl;
    }

    void ModbusTCP::sendRequest(uint8_t *request, uint16_t request_length, uint8_t *response_buffer, uint16_t response_length)
    {
        // increment transaction id
        transaction_id_++;

        // add tcp header to request
        uint8_t request_with_header[request_length + 6];

        request_with_header[0] = 0x00;
        request_with_header[1] = 0x00;
        request_with_header[2] = 0x00;
        request_with_header[3] = 0x00;
        request_with_header[4] = (request_length) >> 8;
        request_with_header[5] = (request_length) & 0xFF;
        std::copy(request, request + request_length, request_with_header + 6);

        // // debugging print
        // std::cout << "Request: ";
        // for (int i = 0; i < request_length + 6; i++)
        // {
        //     std::cout << std::hex << (int)request_with_header[i] << " ";
        // }
        // std::cout << std::endl;

        uint8_t temp_response_buffer[response_length + 6];
        // send request checking for errors in the header
        connection_interface_->sendAndReceive(request_with_header, request_length + 6, temp_response_buffer, response_length + 6);

        // std::cout << "Temp response: ";
        // for (int i = 0; i < response_length + 6; i++)
        // {
        //     std::cout << std::hex << (int)temp_response_buffer[i] << " ";
        // }
        // std::cout << std::endl;

        if (temp_response_buffer[0] != 0x00 || temp_response_buffer[1] != 0x00 ||
            temp_response_buffer[2] != 0x00 || temp_response_buffer[3] != 0x00 ||
            temp_response_buffer[4] != ((response_length) >> 8) || temp_response_buffer[5] != ((response_length) & 0xFF))
        {
            throw std::runtime_error("Error in Modbus TCP response header");
        }

        // remove tcp header from response
        std::copy(temp_response_buffer + 6, temp_response_buffer + response_length + 6, response_buffer);
        // std::cout << "Response: ";
        // for (int i = 0; i < response_length; i++)
        // {
        //     std::cout << std::hex << (int)response_buffer[i] << " ";
        // }
        // std::cout << std::endl;
    }
} // namespace uclv
