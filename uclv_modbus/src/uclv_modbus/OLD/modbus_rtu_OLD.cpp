#include "uclv_modbus/modbus_rtu.h"

uclv::ModbusRTU::ModbusRTU(const std::string &port, uint32_t baudrate, uint8_t slave_id)
    : _fd(-1), _port(port), _baudrate(baudrate), _connected(false), _slave_id(slave_id)
{
}

uclv::ModbusRTU::~ModbusRTU()
{
    disconnect();
    _connected = false;
}

bool uclv::ModbusRTU::connect()
{
    // open the serial port
    _fd = open(_port.c_str(), O_RDWR);
    if (_fd == -1)
    {
        throw std::runtime_error("Failed to open serial port " + _port);
    }

    struct termios options;
    // get current settings checking for errors in getting them
    if (tcgetattr(_fd, &options) != 0)
    {
        throw std::runtime_error("Failed to get serial port attributes");
    }
    // set baudrate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // settings for control modes (c_cflag)
    options.c_cflag |= (CREAD | CLOCAL); // set local mode and enable receiver
    options.c_cflag &= ~CSIZE;           // clear the size bits
    options.c_cflag |= CS8;              // and set 8 bits per character
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~PARENB;          // no parity

    // settings for local modes (c_lflag)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input

    // settings for input modes (c_iflag)
    options.c_iflag &= ~(INPCK);                // disable parity check
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control

    // settings for output modes (c_oflag)
    options.c_oflag &= ~OPOST; // raw output

    // settings for special characters (c_cc)
    options.c_cc[VMIN] = 0;  // minimum number of characters to read
    options.c_cc[VTIME] = 20; // time to wait for data (tenths of seconds)

    // apply the settings
    if (tcsetattr(_fd, TCSANOW, &options) != 0)
    {
        throw std::runtime_error("Failed to set serial port attributes");
    }

    _connected = true;
    return true;
}

void uclv::ModbusRTU::disconnect()
{
    if (_fd != -1)
    {
        close(_fd);
        _fd = -1;
    }
}

bool uclv::ModbusRTU::isConnected() const
{
    return _connected;
}

bool uclv::ModbusRTU::readRegisters(uint8_t function_code, uint16_t start_address, uint16_t num_registers, std::vector<uint16_t> &values)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // Build the request message
    uint8_t request[8];
    request[0] = _slave_id;
    request[1] = function_code;
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
    // std::cerr << std::dec << std::endl;

    // Check if the response message is valid
    if (response[0] != _slave_id || // Check the slave ID
        response[1] != function_code || //  Check the function code
        response[2] != num_registers * 2) // Check the number of bytes
    {
        std::cerr << "Invalid response message" << std::endl;
        if (response[0] != _slave_id)
        {
            std::cerr << "uncorrect slave_id\n";
            std::cerr << "response[0]: " << std::hex << (int)response[0] << ", slave_id: " << (int)_slave_id << std::endl;
        }
        if (response[1] != 0x04)
        {
            std::cerr << "uncorrect function code\n";
            std::cerr << "response[1]: " << std::hex << (int)response[1] << ", function code: " << (int)function_code << std::endl;
        }
        if (response[2] != num_registers * 2)
        {
            std::cerr << "uncorrect number of bytes\n";
            std::cerr << "response[2]: " << std::hex << (int)response[2] << ", num_registers * 2: " << (int)(num_registers * 2) << std::endl;
        }
        //throw std::runtime_error("Invalid response message");
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

bool uclv::ModbusRTU::writeRegisters(uint16_t start_address, const std::vector<uint16_t> &values)
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
        uint16_t(response[2] << 8 | response[3]) != start_address ||                 // Check the starting address
        uint16_t(response[4] << 8 | response[5]) != num_registers)   // Check the number of registers
    {
        throw std::runtime_error("Invalid response message");
    }

    return true;
}

bool uclv::ModbusRTU::_sendRequest(const uint8_t *request, uint16_t request_length)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // print request message
    // std::cerr << "Request: ";
    // for (int i = 0; i < request_length; i++)
    // {
    //     std::cerr << std::hex << (int)request[i] << " ";
    // }
    // std::cerr << std::dec << std::endl;

    // Send the request message
    ssize_t bytes_written = write(_fd, request, request_length);
    std::cerr << "bytes_written: " << bytes_written << std::endl;
    if (bytes_written == -1)
    {
        throw std::runtime_error("Failed to write to serial");
    }
    // flush the input and output buffers
    if (tcflush(_fd, TCIOFLUSH) != 0)
    {
        throw std::runtime_error("Failed to flush serial port buffers");
    }

    return true;
}

uint16_t uclv::ModbusRTU::_receiveResponse(uint8_t *response, uint16_t response_length)
{
    // Check if the connection is established
    if (!_connected)
    {
        throw std::runtime_error("Connection is not established");
    }

    // send and receive data from serial


    // Wait for the response message
    ssize_t bytes_read = read(_fd, response, response_length);
    if (bytes_read == -1)
    {
        throw std::runtime_error("Failed to read from serial");
    }
    else if (bytes_read < response_length) // read remaining bytes
    {
        while (bytes_read < response_length)
        {
            ssize_t bytes_read_now = read(_fd, response + bytes_read, response_length - bytes_read);
            if (bytes_read_now == -1)
            {
                throw std::runtime_error("Failed to read from serial");
            }
            bytes_read += bytes_read_now;
        }
    }

    return bytes_read;
}

