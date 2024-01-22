#include "uclv_modbus/connection_interface.h"

#include <iomanip>

namespace uclv
{
#define CONNECTION_INTERFACE_DEBUG 0

    ConnectionInterface::~ConnectionInterface()
    {
        std::cout << "Called ConnectionInterface destructor" << std::endl;
        disconnect();
    }

    void ConnectionInterface::disconnect()
    {
        if (fd_ != -1)
        {
            close(fd_);
            fd_ = -1;
        }
    }

    void ConnectionInterface::send(const uint8_t *data, size_t length)
    {
        ssize_t bytes_written = write(fd_, data, length);

#if CONNECTION_INTERFACE_DEBUG
        std::cout << "Wrote " << bytes_written << " of " << length << " bytes" << std::endl;
        std::cout << "Wrote: ";
        for (int i = 0; i < bytes_written; i++)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
        }
        std::cout << std::endl;
#endif

        if (bytes_written == -1)
        {
            throw std::runtime_error("Failed to write to device");
        }
        else if (bytes_written < length) // print error message if not all bytes were written
        {
            throw std::runtime_error("Failed to write all bytes to device (wrote " + std::to_string(bytes_written) +
                                     " of " + std::to_string(length) + " bytes)");
        }
    }

    void ConnectionInterface::receive(uint8_t *data, size_t length)
    {
        ssize_t bytes_read = read(fd_, data, length);

#if CONNECTION_INTERFACE_DEBUG
        std::cout << "Read " << bytes_read << " of " << length << " bytes" << std::endl;
        for (int i = 0; i < bytes_read; i++)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
        }
        std::cout << std::endl;
#endif

        if (bytes_read == -1)
        {
            throw std::runtime_error("Failed to read from device");
        }
        else if (bytes_read < length) // read remaining bytes
        {
            while (bytes_read < length)
            {
                if(data[0] == 0xff) // bug during activation of Hand-e gripper (to investigate!!!!!!!!!!!)
                {
                    bytes_read = 0;
                }
#if CONNECTION_INTERFACE_DEBUG
                std::cout << "Read remaining " << length - bytes_read << " bytes" << std::endl;
#endif
                ssize_t bytes_read_now = read(fd_, data + bytes_read, length - bytes_read);
                if (bytes_read_now == -1)
                {
                    throw std::runtime_error("Failed to read all bytes from device (read " + std::to_string(bytes_read) +
                                             " of " + std::to_string(length) + " bytes)");
                }
                bytes_read += bytes_read_now;
            }
        }
    }

    void ConnectionInterface::sendAndReceive(const uint8_t *send_data, size_t send_length, uint8_t *receive_data, size_t receive_length)
    {
        send(send_data, send_length);
        receive(receive_data, receive_length);
    }

} // namespace uclv