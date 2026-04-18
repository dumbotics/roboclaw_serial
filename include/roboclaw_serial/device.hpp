// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace roboclaw_serial
{

class SerialDevice
{
public:
  typedef std::shared_ptr<SerialDevice> SharedPtr;

  SerialDevice() = default;

  explicit SerialDevice(const std::string & device) {connect(device);}
  virtual ~SerialDevice() {disconnect();}

  void setReadTimeoutUs(const std::size_t timeout_us) {read_timeout_us_ = timeout_us;}

  std::size_t readTimeoutUs() const {return read_timeout_us_;}

  void setBaudRate(const speed_t baud_rate) {baud_rate_ = baud_rate;}

  virtual bool connect(const std::string & device)
  {
    if (connected_) {
      disconnect();
    }

    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    connected_ = fd_ != -1;

    if (connected_) {
      try {
        setSerialDeviceOptions();
      } catch (const std::exception & e) {
        std::cerr << "Failed to configure serial device: " << device << " (" << e.what() << ")"
                  << std::endl;
        disconnect();
      }
    } else {
      std::cerr << "Failed to open serial device: " << device << std::endl;
      perror("Error");
    }

    return connected_;
  }

  virtual void disconnect()
  {
    if (connected_) {
      close(fd_);
      connected_ = false;
      fd_ = -1;
    }
  }

  bool connected() const {return connected_;}

  virtual std::size_t write(const std::byte * buffer, std::size_t count)
  {
    if (!connected_ || fd_ < 0) {
      throw std::runtime_error("Serial device is not connected!");
    }

    while (true) {
      const ssize_t result = ::write(fd_, buffer, count);
      if (result >= 0) {
        return static_cast<std::size_t>(result);
      }

      if (errno == EINTR) {
        continue;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        waitForWritable();
        continue;
      }

      throw std::range_error("Error writing to the device!");
    }
  }

  virtual std::size_t read(std::byte * buffer, std::size_t count)
  {
    if (!connected_ || fd_ < 0) {
      throw std::runtime_error("Serial device is not connected!");
    }

    waitForReadable();

    while (true) {
      const ssize_t result = ::read(fd_, buffer, count);
      if (result >= 0) {
        return static_cast<std::size_t>(result);
      }

      if (errno == EINTR) {
        continue;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        waitForReadable();
        continue;
      }

      throw std::range_error("Error reading from the serial device!");
    }
  }

protected:
  bool connected_ = false;

private:
  void waitForReadable() const
  {
    while (true) {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(fd_, &set);

      struct timeval timeout = timeoutStruct();
      const int ready = select(fd_ + 1, &set, nullptr, nullptr, &timeout);
      if (ready > 0) {
        return;
      }

      if (ready == 0) {
        throw std::runtime_error("Read timeout!");
      }

      if (errno == EINTR) {
        continue;
      }

      throw std::range_error("Error reading from the serial device!");
    }
  }

  void waitForWritable() const
  {
    while (true) {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(fd_, &set);

      struct timeval timeout = timeoutStruct();
      const int ready = select(fd_ + 1, nullptr, &set, nullptr, &timeout);
      if (ready > 0) {
        return;
      }

      if (ready == 0) {
        throw std::runtime_error("Write timeout!");
      }

      if (errno == EINTR) {
        continue;
      }

      throw std::range_error("Error writing to the device!");
    }
  }

  struct timeval timeoutStruct() const
  {
    struct timeval timeout;
    timeout.tv_sec = static_cast<time_t>(read_timeout_us_ / 1000000U);
    timeout.tv_usec = static_cast<suseconds_t>(read_timeout_us_ % 1000000U);
    return timeout;
  }

  void setSerialDeviceOptions()
  {
    struct termios options {};
    if (tcgetattr(fd_, &options) < 0) {
      throw std::runtime_error("Unable to read serial options");
    }

    cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= CS8;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (cfsetispeed(&options, baud_rate_) < 0 || cfsetospeed(&options, baud_rate_) < 0) {
      throw std::runtime_error("Unable to set baud rate");
    }

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &options) < 0) {
      throw std::runtime_error("Unable to apply serial options");
    }

    // Set the file descriptor to non-blocking mode
    const int flags = fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw std::runtime_error("Unable to set non-blocking mode");
    }
  }

  speed_t baud_rate_ = B38400;
  std::size_t read_timeout_us_ = 10000;
  int fd_ = -1;
};

}  // namespace roboclaw_serial
