#pragma once

#include "roboclaw_serial/crc.hpp"
#include "roboclaw_serial/device.hpp"
#include "roboclaw_serial/serialization.hpp"
#include "roboclaw_serial/command.hpp"
#include "roboclaw_serial/serialized_buffer.hpp"

#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <vector>
#include <memory>
#include <array>
#include <mutex>

namespace roboclaw_serial
{
    class Interface
    {
    public:
        typedef std::shared_ptr<Interface> SharedPtr;
        Interface() = default;
        Interface(const SerialDevice::SharedPtr& device)
        {
            setDevice(device);
        }

        void setDevice(const SerialDevice::SharedPtr& device)
        {
            device_ = device;
        }

        template <typename Request>
        void read(Request& request, const unsigned char address=128)
        {
            request.fields = read<Request>(address);
        }

        template <typename Request>
        typename Request::ArgsTuple read(const unsigned char address=128)
        {
            this->bufferSetupRead<Request>(address);

            crc_ = 0;
            for (const auto& byte : buffer_)
            {
                updateCRC16b(crc_, byte);
            }

            // Write the buffer to the serial device
            device_->write(buffer_.data(), buffer_.size());

            // Set the buffer to the size of the fields, size of CRC
            buffer_.resize(buffer_.max_size());

            // Read the response from the device
            std::size_t bytes_read = device_->read(buffer_.data(), buffer_.size());

            buffer_.resize(bytes_read);

            // Extract the CRC from the the back of the buffer
            auto recv_crc = buffer_.pop_back<uint16_t>();

            // Update the CRC with the new data and ensure it matches
            for (auto byte : buffer_)
            {
                updateCRC16b(crc_, byte);
            }

            if (crc_ != recv_crc)
            {
                throw std::logic_error("CRCs do not match");
            }

            // Extract the data
            typename Request::ArgsTuple fields;
            std::apply([&](auto&&... args) { buffer_.unpack(args...); }, fields);

            return fields;
        }

        // template <typename Request, typename...Args>
        // int write(Args&&...args)
        // {
        //     // Convert to tuple and use tuple-based write call
        //     return write<Request>(std::forward_as_tuple(args...));
        // }

        template <typename Request>
        void write(const Request& request, const unsigned char address=128)
        {
            // Write the fields to the roboclaw
            write<Request>(request.fields, address);
        }

        template <typename Request>
        void write(const typename Request::ArgsTuple& fields, const unsigned char address=128)
        {
            // Initialize buffer with Write request, fields, and CRC
            this->bufferSetupWrite<Request>(address, fields);

            // Write the request
            device_->write(buffer_.data(), buffer_.size());

            if (!this->readAck())
            {
                throw std::logic_error("did not get an ack!");
            }
        }

    private:

        template <typename Request>
        void bufferPackFields(const typename Request::ArgsTuple& fields)
        {
            auto pushToBuffer = [this](const auto&... items) {
                (this->buffer_.push_back(items), ...);
            };

            std::apply(pushToBuffer, fields);
        }

        template <typename Request>
        void bufferUnpackFields(typename Request::ArgsTuple& fields)
        {
            std::apply([this](auto&&... args) { 
                ((args = this->buffer_.pop_front<std::decay_t<decltype(args)>>()), ...);
            }, fields);
        }

        // Compute the CRC
        void bufferAddCRC16()
        {
            // Calculate the CRC16 value
            crc_ = 0;
            for (const auto& byte : buffer_)
            {
                updateCRC16b(crc_, byte);
            }

            // Add the CRC6 to the end of the buffer
            buffer_.push_back(crc_);
        }

        // Setup the buffer for a write request
        template <typename Request>
        void bufferSetupWrite(const unsigned char address, const typename Request::ArgsTuple& fields)
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
        template <typename Request>
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
            device_->read(buffer_.data(), buffer_.size());

            return buffer_.pop_back() == ACK;
        }

        const std::byte ACK = std::byte(255U);
        unsigned short crc_;
        SerialDevice::SharedPtr device_;
        SerializedBuffer<64> buffer_;
        std::mutex mutex_;

    };

} // namespace roboclaw_serial
