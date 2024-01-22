#include "uclv_modbus/modbus_rtu_tcp.h"

uclv::ModbusRTUOverTCP::ModbusRTUOverTCP(const std::string &ip_address, uint16_t port, uint8_t slave_id)
    : _sockfd(-1), _ip_address(ip_address), _port(port), _connected(false), _slave_id(slave_id)
{
}

uclv::ModbusRTUOverTCP::~ModbusRTUOverTCP()
{
    disconnect();
}

bool uclv::ModbusRTUOverTCP::connect()
{
    _sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (_sockfd == -1)
    {
        throw std::runtime_error("Failed to create socket");
    }
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(_port);
    int rc = inet_pton(server_address.sin_family, _ip_address.c_str(), &server_address.sin_addr);;
    if (rc <= 0)
    {
        close(_sockfd);
        throw std::runtime_error("Invalid address");
    }
    rc = ::connect(_sockfd, (struct sockaddr *)&server_address, sizeof(server_address));
    if (rc < 0)
    {
        close(_sockfd);
        throw std::runtime_error("Failed to connect to server");
    }

    _connected = true;
    return true;
}

void uclv::ModbusRTUOverTCP::disconnect()
{
    if (_sockfd != -1)
    {
        close(_sockfd);
        _sockfd = -1;
    }
}

bool uclv::ModbusRTUOverTCP::isConnected() const
{
    return _connected;
}

bool uclv::ModbusRTUOverTCP::readRegisters(uint8_t function_code, uint16_t start_address, uint16_t num_registers, std::vector<uint16_t>& values)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // Build the request message
    uint8_t request[8];
    request[0] = _slave_id;
    request[1] = function_code; // Function code
    request[2] = start_address >> 8;
    request[3] = start_address & 0xFF;
    request[4] = num_registers >> 8;
    request[5] = num_registers & 0xFF;
    uint16_t crc = calculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = crc >> 8;

    std::cerr << "readRegisters function" << std::endl;

    // Send the request message
    if (!_sendRequest(request, 8))
    {
        throw std::runtime_error("Failed to send request");
    }

    // Receive the response message
    uint16_t response_length = 3 + num_registers * 2 + 2; // 3 bytes for the header, 2 bytes per register, 2 bytes for the CRC
    uint8_t response[response_length];
    uint16_t bytes_read = _receiveResponse(response, response_length);

    // print response message
    // std::cerr << "Response: ";
    // for (int i = 0; i < response_length; i++)
    // {
    //     std::cerr << std::hex << (int)response[i] << " ";
    // }
    std::cerr << std::dec << std::endl;

    // Check if the response message is valid
    if (response[0] != _slave_id ||       // Check the slave ID
        response[1] != function_code ||            //  Check the function code
        response[2] != num_registers * 2) // Check the number of bytes
    {
        throw std::runtime_error("Invalid response message");
    }

    // Extract the register values from the response message
    values.clear();
    const uint8_t *ptr = response + 3; // Skip the header
    for (int i = 0; i < num_registers; i++)
    {
        uint16_t value = *ptr++ | (*ptr++ << 8); // Combine the two bytes in little endian order (*ptr++ is the same as *ptr; ptr++;)
        values.push_back(value);
    }
    return true;
}

bool uclv::ModbusRTUOverTCP::writeRegisters(uint16_t start_address, const std::vector<uint16_t>& values)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // Build the request message
    uint16_t num_registers = values.size();
    uint8_t request_length = 7 + num_registers * 2 + 2; // 7 bytes for the header, 2 bytes per register, 2 bytes for the CRC
    uint8_t request[request_length];
    request[0] = _slave_id;
    request[1] = 0x10; // Function code
    request[2] = start_address >> 8;
    request[3] = start_address & 0xFF;
    request[4] = num_registers >> 8;
    request[5] = num_registers & 0xFF;
    request[6] = num_registers * 2;
    for (int i = 0; i < num_registers; i++)
    {
        // Split the 16-bit value into two bytes in little endian order
        request[7 + i * 2] = values[i] & 0xFF;
        request[8 + i * 2] = values[i] >> 8;
    }
    uint16_t crc = calculateCRC(request, request_length - 2); // Exclude the last 2 bytes (CRC)
    request[7 + num_registers * 2] = crc & 0xFF;
    request[8 + num_registers * 2] = crc >> 8;

    std::cerr << "writeRegisters function" << std::endl;
    // Send the request message
    if (!_sendRequest(request, request_length))
    {
        throw std::runtime_error("Failed to send request");
    }

    // Receive the response message
    uint16_t response_length = 8;
    uint8_t response[response_length];
    uint16_t bytes_read = _receiveResponse(response, response_length);

    // print response message
    std::cerr << "Response: ";
    for (int i = 0; i < response_length; i++)
    {
        std::cerr << std::hex << (int)response[i] << " ";
    }
    std::cerr << std::dec << std::endl;

    // Check if the response message is valid
    if (response[0] != _slave_id ||                                                 // Check the slave ID
        response[1] != 0x10 ||                                                      // Check the function code
        response[2] != start_address >> 8 || response[3] != start_address & 0xFF || // Check the starting address
        response[4] != num_registers >> 8 || response[5] != num_registers & 0xFF)   // Check the number of registers
    {
        throw std::runtime_error("Invalid response message");
    }

    return true;
}

bool uclv::ModbusRTUOverTCP::_sendRequest(const uint8_t* request, uint16_t request_length)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // print request message
    std::cerr << "Request: ";
    for (int i = 0; i < request_length; i++)
    {
        std::cerr << std::hex << (int)request[i] << " ";
    }
    std::cerr << std::dec << std::endl;

    // Send the request message
    ssize_t bytes_written = write(_sockfd, request, request_length);
    if (bytes_written == -1)
    {
        throw std::runtime_error("Failed to write to serial");
    }

    return true;
}

uint16_t uclv::ModbusRTUOverTCP::_receiveResponse(uint8_t* response, uint16_t response_length)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // Wait for the response message
    ssize_t bytes_read = read(_sockfd, &response, response_length);
    if (bytes_read == -1)
    {
        throw std::runtime_error("Failed to read from serial");
    }

    return bytes_read;
}