#include "uclv_modbus/ethernet.h"

namespace uclv {

    Ethernet::Ethernet(const char* ip_address, int port) : ip_address_(ip_address), port_(port)
    {
        connect();
    }

    Ethernet::~Ethernet()
    {
        std::cout << "Called Ethernet destructor" << std::endl;
        disconnect();
    }

    bool Ethernet::connect()
    {
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ == -1)
        {
            throw std::runtime_error("Failed to create socket with ip address " + std::string(ip_address_) + " and port " + std::to_string(port_));
        }
        struct sockaddr_in server_address;
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(port_);
        int rc = inet_pton(server_address.sin_family, ip_address_, &server_address.sin_addr);
        if (rc <= 0)
        {
            close(fd_);
            throw std::runtime_error("Invalid address");
        }
        rc = ::connect(fd_, (struct sockaddr *)&server_address, sizeof(server_address));
        if (rc < 0)
        {
            close(fd_);
            throw std::runtime_error("Failed to connect to server");
        }

        return true;
    }

} // namespace uclv

