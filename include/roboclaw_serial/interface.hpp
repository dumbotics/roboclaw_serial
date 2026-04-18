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

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "roboclaw_serial/command.hpp"
#include "roboclaw_serial/crc.hpp"
#include "roboclaw_serial/device.hpp"
#include "roboclaw_serial/serialized_buffer.hpp"

namespace roboclaw_serial
{
class Interface
{
public:
  typedef std::shared_ptr<Interface> SharedPtr;
  Interface() = default;
  explicit Interface(const SerialDevice::SharedPtr & device) {setDevice(device);}

  void setDevice(const SerialDevice::SharedPtr & device) {device_ = device;}

  template<typename Request>
  void read(Request & request, const unsigned char address = 128)
  {
    if constexpr (Request::read_command == uint8_t(Command::NONE)) {
      throw std::invalid_argument("Request does not support reads");
    }
    request.fields = read<Request>(address);
  }

  template<typename Request>
  typename Request::ArgsTuple read(const unsigned char address = 128)
  {
    if constexpr (Request::read_command == uint8_t(Command::NONE)) {
      throw std::invalid_argument("Request does not support reads");
    }
    ensureConnected();

    // Prevent parallel reads/writes
    std::lock_guard<std::mutex> lock(mutex_);

    this->bufferSetupRead<Request>(address);

    crc_ = 0;
    for (const auto & byte : buffer_) {
      crc16::update(crc_, byte);
    }

    // Write the read command, retrying partial writes.
    writeAll(buffer_.data(), buffer_.size());

    // Read exactly-sized fixed payloads (numeric requests) and read-until-idle for
    // dynamic payloads (e.g. firmware version strings).
    std::size_t bytes_read = 0;
    if constexpr (requestHasDynamicSize<Request>()) {
      buffer_.resize(buffer_.max_size());
      bytes_read = readUntilIdle(buffer_.data(), buffer_.size());
    } else {
      constexpr std::size_t response_size = requestResponseSize<Request>();
      buffer_.resize(response_size);
      readExactly(buffer_.data(), buffer_.size());
      bytes_read = response_size;
    }

    buffer_.resize(bytes_read);
    if (buffer_.size() < sizeof(uint16_t)) {
      throw std::logic_error("response was too short");
    }

    // Extract the CRC from the the back of the buffer
    auto recv_crc = buffer_.pop_back<uint16_t>();

    // Update the CRC with the new data and ensure it matches
    for (auto byte : buffer_) {
      crc16::update(crc_, byte);
    }

    if (crc_ != recv_crc) {
      throw std::logic_error("CRCs do not match");
    }

    // Extract the data
    typename Request::ArgsTuple fields;
    std::apply([&](auto &&... args) {buffer_.unpack(args ...);}, fields);

    return fields;
  }

  // template <typename Request, typename...Args>
  // int write(Args&&...args)
  // {
  //     // Convert to tuple and use tuple-based write call
  //     return write<Request>(std::forward_as_tuple(args...));
  // }

  template<typename Request>
  void write(const Request & request, const unsigned char address = 128)
  {
    if constexpr (Request::write_command == uint8_t(Command::NONE)) {
      throw std::invalid_argument("Request does not support writes");
    }

    // Write the fields to the roboclaw
    write<Request>(request.fields, address);
  }

  template<typename Request>
  void write(const typename Request::ArgsTuple & fields, const unsigned char address = 128)
  {
    if constexpr (Request::write_command == uint8_t(Command::NONE)) {
      throw std::invalid_argument("Request does not support writes");
    }
    ensureConnected();

    // Prevent parallel read/writes
    std::lock_guard<std::mutex> lock(mutex_);

    // Initialize buffer with Write request, fields, and CRC
    this->bufferSetupWrite<Request>(address, fields);

    // Write the request, retrying partial writes.
    writeAll(buffer_.data(), buffer_.size());

    if (!this->readAck()) {
      throw std::logic_error("did not get an ack!");
    }
  }

private:
  template<typename Request>
  void bufferPackFields(const typename Request::ArgsTuple & fields)
  {
    auto pushToBuffer = [this](const auto &... items) {(this->buffer_.push_back(items), ...);};

    std::apply(pushToBuffer, fields);
  }

  template<typename Request>
  void bufferUnpackFields(typename Request::ArgsTuple & fields)
  {
    std::apply(
      [this](auto &&... args) {
        ((args = this->buffer_.pop_front<std::decay_t<decltype(args)>>()), ...);
      },
      fields);
  }

  // Compute the CRC
  void bufferAddCRC16()
  {
    // Calculate the CRC16 value
    crc_ = 0;
    for (const auto & byte : buffer_) {
      crc16::update(crc_, byte);
    }

    // Add the CRC6 to the end of the buffer
    buffer_.push_back(crc_);
  }

  // Setup the buffer for a write request
  template<typename Request>
  void bufferSetupWrite(const unsigned char address, const typename Request::ArgsTuple & fields)
  {
    // Set size of the buffer to 0
    buffer_.clear();

    // Serialize request into buffer
    buffer_.push_back(address);
    buffer_.push_back(Request::write_command);

    // Pack the tuple into the buffer
    this->bufferPackFields<Request>(fields);

    // Add the CRC to the buffer
    this->bufferAddCRC16();
  }

  // Setup the buffer for a read request
  template<typename Request>
  void bufferSetupRead(const unsigned char address)
  {
    // Set size of the buffer to 0
    buffer_.clear();

    // Serialize request into buffer
    buffer_.push_back(address);
    buffer_.push_back(Request::read_command);
  }

  bool readAck()
  {
    // We only expect an ACK from the roboclaw
    buffer_.resize(1);
    readExactly(buffer_.data(), buffer_.size());

    return buffer_.pop_back() == ACK;
  }

  void ensureConnected() const
  {
    if (!device_ || !device_->connected()) {
      throw std::runtime_error("serial device is not connected");
    }
  }

  void writeAll(const std::byte * data, const std::size_t size)
  {
    std::size_t bytes_written = 0;
    while (bytes_written < size) {
      const auto written = device_->write(data + bytes_written, size - bytes_written);
      if (written == 0) {
        throw std::runtime_error("serial write made no progress");
      }
      bytes_written += written;
    }
  }

  void readExactly(std::byte * data, const std::size_t size)
  {
    std::size_t bytes_read = 0;
    while (bytes_read < size) {
      const auto received = device_->read(data + bytes_read, size - bytes_read);
      if (received == 0) {
        throw std::runtime_error("serial read made no progress");
      }
      bytes_read += received;
    }
  }

  std::size_t readUntilIdle(std::byte * data, const std::size_t max_size)
  {
    std::size_t bytes_read = 0;
    while (bytes_read < max_size) {
      try {
        const auto received = device_->read(data + bytes_read, max_size - bytes_read);
        if (received == 0) {
          break;
        }
        bytes_read += received;
      } catch (const std::runtime_error &) {
        if (bytes_read == 0) {
          throw;
        }
        break;
      }
    }

    return bytes_read;
  }

  template<typename Tuple, std::size_t ... Indices>
  static constexpr std::size_t tupleStaticByteSize(std::index_sequence<Indices...>)
  {
    return (sizeof(std::tuple_element_t<Indices, Tuple>) + ... + 0U);
  }

  template<typename Tuple, std::size_t ... Indices>
  static constexpr bool tupleHasDynamicType(std::index_sequence<Indices...>)
  {
    return (std::is_same_v<std::tuple_element_t<Indices, Tuple>, std::string> || ... || false);
  }

  template<typename Request>
  static constexpr bool requestHasDynamicSize()
  {
    using ArgsTuple = typename Request::ArgsTuple;
    return tupleHasDynamicType<ArgsTuple>(std::make_index_sequence<std::tuple_size_v<ArgsTuple>>{});
  }

  template<typename Request>
  static constexpr std::size_t requestResponseSize()
  {
    using ArgsTuple = typename Request::ArgsTuple;
    return tupleStaticByteSize<ArgsTuple>(std::make_index_sequence<std::tuple_size_v<ArgsTuple>>{}) +
           sizeof(uint16_t);
  }

  const std::byte ACK = std::byte(255U);
  uint16_t crc_;
  SerialDevice::SharedPtr device_;
  SerializedBuffer<64> buffer_;
  std::mutex mutex_;
};

}  // namespace roboclaw_serial
