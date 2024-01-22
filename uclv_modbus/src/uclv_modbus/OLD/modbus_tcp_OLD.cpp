#include "uclv_modbus/modbus_tcp.h"

uclv::ModbusTCP::ModbusTCP(const std::string &ip_address, uint16_t port, uint8_t slave_id)
    : _sockfd(-1), _ip_address(ip_address), _port(port), _connected(false), _slave_id(slave_id), _transaction_id(1)
{
}

uclv::ModbusTCP::~ModbusTCP()
{
    disconnect();
}

bool uclv::ModbusTCP::connect()
{
    _sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (_sockfd == -1)
    {
        throw std::runtime_error("Failed to create socket");
    }
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(_port);
    int rc = inet_pton(server_address.sin_family, _ip_address.c_str(), &server_address.sin_addr);
    ;
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

void uclv::ModbusTCP::disconnect()
{
    if (_sockfd != -1)
    {
        close(_sockfd);
        _sockfd = -1;
    }
}

bool uclv::ModbusTCP::isConnected() const
{
    return _connected;
}

bool uclv::ModbusTCP::readRegisters(uint8_t function_code, uint16_t start_address, uint16_t num_registers, std::vector<uint16_t> &values)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // Build the request message
    uint8_t request[12];
    request[0] = _transaction_id >> 8;   // Transaction identifier (high byte)
    request[1] = _transaction_id & 0xFF; // Transaction identifier (low byte)
    request[2] = 0x00;                   // Protocol identifier (high byte)
    request[3] = 0x00;                   // Protocol identifier (low byte)
    request[4] = 0x00;                   // Length (high byte)
    request[5] = 0x06;                   // Length (low byte)
    request[6] = _slave_id;              // Unit identifier
    request[7] = function_code;                   // Function code
    request[8] = start_address >> 8;
    request[9] = start_address & 0xFF;
    request[10] = num_registers >> 8;
    request[11] = num_registers & 0xFF;

    std::cerr << "readRegisters function" << std::endl;

    // Send the request message
    if (!_sendRequest(request, 12))
    {
        throw std::runtime_error("Failed to send request");
    }

    // Receive the response message
    uint16_t response_length = 9 + num_registers * 2; // 9 bytes for the header, 2 bytes per register
    uint8_t response[response_length];
    uint16_t bytes_read = _receiveResponse(response, response_length);

    // print response message
    std::cerr << "Response: ";
    for (int i = 0; i < response_length; i++)
    {
        std::cerr << std::hex << (int)response[i] << " ";
    }
    std::cerr << std::dec << std::endl;

    if (response[0] != request[0] || response[1] != request[1] || // Check the transaction ID
        response[7] != function_code ||                                    // Check the function code
        response[8] != num_registers * 2)                         // Check the number of bytes
    {
        throw std::runtime_error("Invalid response message");
    }
    values.clear();
    const uint8_t *ptr = response + 9; // Skip the header
    for (int i = 0; i < num_registers; i += 2)
    {
        uint16_t value = *ptr++ | (*ptr++ << 8);
        values.push_back(value);
    }
    // update transaction id using ternary operator
    _transaction_id = (_transaction_id == 0) ? 1 : _transaction_id + 1;

    return true;
}

bool uclv::ModbusTCP::writeRegisters(uint16_t start_address, const std::vector<uint16_t> &values)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // Build the request message
    uint16_t num_registers = values.size();
    uint8_t request_length = 6 + 7 + num_registers * 2; // 6 bytes for tcp header, 7 bytes for modbus header, 2 bytes per register
    uint8_t request[request_length];
    request[0] = _transaction_id >> 8;        // Transaction identifier (high byte)
    request[1] = _transaction_id & 0xFF;      // Transaction identifier (low byte)
    request[2] = 0x00;                        // Protocol identifier (high byte)
    request[3] = 0x00;                        // Protocol identifier (low byte)
    request[4] = (request_length - 6) >> 8;   // Length (high byte)
    request[5] = (request_length - 6) & 0xFF; // Length (low byte)
    request[6] = _slave_id;                   // Unit identifier
    request[7] = 0x10;                        // Function code
    request[8] = start_address >> 8;
    request[9] = start_address & 0xFF;
    request[10] = num_registers >> 8;
    request[11] = num_registers & 0xFF;
    request[12] = num_registers * 2;
    for (int i = 0; i < num_registers; i++)
    {
        request[13 + i * 2] = values[i] & 0xFF;
        request[14 + i * 2] = values[i] >> 8;
    }

    std::cerr << "writeRegisters function" << std::endl;
    // Send the request message
    if (!_sendRequest(request, request_length))
    {
        throw std::runtime_error("Failed to send request");
    }

    // Receive the response message
    uint16_t response_length = 12;
    uint8_t response[response_length];
    uint16_t bytes_read = _receiveResponse(response, response_length);

    // print response message
    std::cerr << "Response: ";
    for (int i = 0; i < response_length; i++)
    {
        std::cerr << std::hex << (int)response[i] << " ";
    }
    std::cerr << std::dec << std::endl;

    // Check the response message
    if (response[0] != request[0] || response[1] != request[1] ||                   // Check the transaction ID
        response[7] != 0x10 ||                                                      // Check the function code
        response[8] != start_address >> 8 || response[9] != start_address & 0xFF || // Check the start address
        response[10] != num_registers >> 8 || response[11] != num_registers & 0xFF) // Check the number of registers
    {
        throw std::runtime_error("Invalid response message");
    }

    // update transaction id using ternary operator
    _transaction_id = (_transaction_id == 0) ? 1 : _transaction_id + 1;

    return true;
}

bool uclv::ModbusTCP::_sendRequest(const uint8_t *request, uint16_t request_length)
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
    // print bytes written and request length
    std::cerr << "Bytes written: " << bytes_written << std::endl;
    std::cerr << "Request length: " << request_length << std::endl;
    if (bytes_written != request_length)
    {
        throw std::runtime_error("Failed to write to socket");
    }

    return true;
}

uint16_t uclv::ModbusTCP::_receiveResponse(uint8_t *response, uint16_t response_length)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // Wait for the response message
    ssize_t bytes_read = read(_sockfd, &response, response_length);
    // print bytes read and response length
    std::cerr << "Bytes read: " << bytes_read << std::endl;
    std::cerr << "Response length: " << response_length << std::endl;
    if (bytes_read != response_length)
    {
        throw std::runtime_error("Failed to read from socket");
    }

    return bytes_read;
}