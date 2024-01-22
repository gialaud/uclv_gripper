#include "uclv_modbus/usb.h"

namespace uclv {

    USB::USB(const char* device_path) : device_path_(device_path)
    {
        connect();
    }

    USB::~USB()
    {
        std::cout << "Called USB destructor" << std::endl;
        disconnect();
    }

    bool USB::connect()
    {
        // open the serial port
    fd_ = open(device_path_, O_RDWR);
    if (fd_ == -1)
    {
        throw std::runtime_error("Failed to open serial port " + std::string(device_path_));
    }

    struct termios options;
    // get current settings checking for errors in getting them
    if (tcgetattr(fd_, &options) != 0)
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
    if (tcsetattr(fd_, TCSANOW, &options) != 0)
    {
        throw std::runtime_error("Failed to set serial port attributes");
    }

    return true;
}

} // namespace uclv