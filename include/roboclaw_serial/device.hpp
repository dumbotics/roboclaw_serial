#pragma once

#include <string>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <memory>

namespace roboclaw_serial
{

class SerialDevice
{
public:
    typedef std::shared_ptr<SerialDevice> SharedPtr;

    SerialDevice() = default;

    SerialDevice(const std::string device)
    {
        connect(device);
    };
    virtual ~SerialDevice() { disconnect(); }

    virtual bool connect(const std::string& device)
    {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
        connected_ = fd_ != -1;

        if (connected_)
        {
            setSerialDeviceOptions();
        }
        else
        {
            std::cerr << "Failed to open serial device: " << device << std::endl;
            perror("Error");
        }

        return connected_;
    }

    virtual void disconnect()
    {
        if (connected_)
        {
            close(fd_);
            connected_ = false;
        }
    }

    bool connected() const { return connected_; }

    virtual std::size_t write(const std::byte * buffer, std::size_t count)
    {
        ssize_t result = ::write(fd_, buffer, count);
        if (result < 0)
        {
            // Error writing to device
            throw std::range_error("Error writing to the device!");
        }
        return static_cast<std::size_t>(result) == count;
    }

    virtual std::size_t read(std::byte* buffer, std::size_t count)
    {
        fd_set set;
        struct timeval timeout;

        /* Initialize the file descriptor set. */
        FD_ZERO (&set);
        FD_SET (fd_, &set);

        /* Initialize the timeout data structure. */
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;  // 10ms

        /* select returns 0 if timeout, 1 if input available, -1 if error. */
        int res = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
        if(res < 0)
        {
            throw std::range_error("Error reading from the serial device!");
        }
        else if(res == 0)
        {
            throw std::runtime_error("Read timeout!");
        }
        ssize_t result = ::read(fd_, buffer, count);
        if (result < 0)
        {
            // Error reading from the device
            throw std::range_error("Error reading from the serial device!");
        }

        return static_cast<std::size_t>(result);
    }

protected:
    bool connected_ = false;

private:
    void setSerialDeviceOptions()
    {
        struct termios options;
        tcgetattr(fd_, &options);
        options.c_cflag = CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &options);

        // Set the file descriptor to non-blocking mode
        fcntl(fd_, F_SETFL, O_NONBLOCK);
    }

    int fd_ = -1;
};

} // namespace roboclaw_serial
