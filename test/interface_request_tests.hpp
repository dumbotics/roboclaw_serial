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

// Including necessary libraries and files.
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dummy_serial_device.hpp"
#include "roboclaw_serial/command.hpp"
#include "roboclaw_serial/interface.hpp"

// Helper function to create a byte vector from char vector
std::vector<std::byte> create_byte_vector(std::vector<unsigned char> list)
{
  std::vector<std::byte> result;
  for (auto val : list) {
    result.push_back(static_cast<std::byte>(val));
  }
  return result;
}

struct Data
{
  std::string requestType;  // "read" or "write"
  const std::vector<unsigned char> writeBytes;
  const std::vector<unsigned char> readBytes;
};

template<typename RequestType>
struct TestData
{
  typename RequestType::ArgsTuple args;
  Data data;
};

class TestExecutor : public ::testing::Test
{
protected:
  // Method to execute test
  template<typename RequestType>
  void executeTest(const TestData<RequestType> & p)
  {
    // Create a dummy device
    auto device = std::make_shared<DummyTestDevice>(
      create_byte_vector(p.data.readBytes),
      create_byte_vector(p.data.writeBytes));

    // Update the interface to use the dummy device for this test
    interface_.setDevice(device);

    // Initialize a request for communication
    RequestType request;

    // If this is a write request, set the request arguments to the test values
    if (p.data.requestType == "write") {
      request.fields = p.args;
      ASSERT_NO_THROW(interface_.write(request));
    }

    if (p.data.requestType == "read") {
      // Test that values can be "read" from the device
      interface_.read(request);
      ASSERT_TRUE(p.args == request.fields);
    }

    // Disconnect the device
    device->disconnect();
  }

private:
  roboclaw_serial::Interface interface_;
};

class ChunkedDummyDevice : public roboclaw_serial::SerialDevice
{
public:
  ChunkedDummyDevice(
    std::vector<std::byte> expected_write_buffer, std::vector<std::byte> read_buffer,
    std::size_t max_write_chunk, std::size_t max_read_chunk)
  : expected_write_buffer_(std::move(expected_write_buffer)),
    read_buffer_(std::move(read_buffer)),
    max_write_chunk_(max_write_chunk),
    max_read_chunk_(max_read_chunk)
  {
    connect("dummy chunked device");
  }

  bool connect(const std::string &) override
  {
    connected_ = true;
    return true;
  }

  void disconnect() override {connected_ = false;}

  std::size_t write(const std::byte * buffer, std::size_t count) override
  {
    const std::size_t chunk = std::min(max_write_chunk_, count);
    for (std::size_t i = 0; i < chunk; ++i) {
      observed_write_buffer_.push_back(buffer[i]);
    }

    return chunk;
  }

  std::size_t read(std::byte * buffer, std::size_t count) override
  {
    const std::size_t remaining = read_buffer_.size() - read_cursor_;
    if (remaining == 0) {
      return 0;
    }

    const std::size_t chunk = std::min({remaining, count, max_read_chunk_});
    for (std::size_t i = 0; i < chunk; ++i) {
      buffer[i] = read_buffer_[read_cursor_ + i];
    }

    read_cursor_ += chunk;
    return chunk;
  }

  bool writeBufferMatches() const {return observed_write_buffer_ == expected_write_buffer_;}

private:
  std::vector<std::byte> expected_write_buffer_;
  std::vector<std::byte> observed_write_buffer_;
  std::vector<std::byte> read_buffer_;
  std::size_t read_cursor_ = 0;
  std::size_t max_write_chunk_;
  std::size_t max_read_chunk_;
};

TEST_F(TestExecutor, WriteVelocityPIDConstantsM1SerializationTest)
{
  this->executeTest(
    TestData<roboclaw_serial::VelocityPIDConstantsM1>{
    {
      0,
      8631,
      878,
      201562
    },  // request arguments
    {
      "write",  // requestType
      {
        0x80,  // Address byte
        0x1c,  // Command byte
        0x00, 0x00, 0x00, 0x00,  // P parameter
        0x00, 0x00, 0x21, 0xb7,  // I parameter
        0x00, 0x00, 0x03, 0x6e,  // D parameter
        0x00, 0x03, 0x13, 0x5a,  // QPPS value
        0x7d, 0x8e
      },  // writeBytes
      {0xff}  // readBytes
    }});
}

TEST_F(TestExecutor, ReadEncoderCountersSerializationTest)
{
  this->executeTest(
    TestData<roboclaw_serial::EncoderCounters>{
    {12528, 53212},  // request arguments
    {
      "read",  // requestType
      {0x80, 0x4e},  // writeBytes
      {0x00, 0x00, 0x30, 0xf0, 0x00, 0x00, 0xcf, 0xdc, 0xd4, 0xdb}  // readBytes
    }});
}

TEST_F(TestExecutor, WriteRetriesUntilCompleteWhenDeviceWritesPartialChunks)
{
  auto device = std::make_shared<ChunkedDummyDevice>(
    create_byte_vector(
      {0x80, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0xb7, 0x00, 0x00, 0x03, 0x6e, 0x00,
       0x03, 0x13, 0x5a, 0x7d, 0x8e}),
    create_byte_vector({0xff}), 3, 1);

  roboclaw_serial::Interface interface(device);
  roboclaw_serial::VelocityPIDConstantsM1 request;
  request.fields = std::make_tuple(0, 8631, 878, 201562);

  ASSERT_NO_THROW(interface.write(request));
  ASSERT_TRUE(device->writeBufferMatches());
}

TEST_F(TestExecutor, ReadRetriesUntilCompleteWhenDeviceReadsPartialChunks)
{
  auto device = std::make_shared<ChunkedDummyDevice>(
    create_byte_vector({0x80, 0x4e}),
    create_byte_vector({0x00, 0x00, 0x30, 0xf0, 0x00, 0x00, 0xcf, 0xdc, 0xd4, 0xdb}), 2, 2);

  roboclaw_serial::Interface interface(device);
  roboclaw_serial::EncoderCounters request;

  interface.read(request);
  ASSERT_EQ(std::make_tuple(12528, 53212), request.fields);
  ASSERT_TRUE(device->writeBufferMatches());
}

TEST_F(TestExecutor, ReadThrowsForWriteOnlyRequest)
{
  auto device = std::make_shared<DummyTestDevice>(
    create_byte_vector({0xff}), create_byte_vector({0x80, 0xff}));

  roboclaw_serial::Interface interface(device);
  roboclaw_serial::DriveM1M2WithSignedSpeed request;

  ASSERT_THROW(interface.read(request), std::invalid_argument);
}

TEST_F(TestExecutor, WriteThrowsForReadOnlyRequest)
{
  auto device = std::make_shared<DummyTestDevice>(
    create_byte_vector({0xff}), create_byte_vector({0x80, 0xff, 0x43, 0x43}));

  roboclaw_serial::Interface interface(device);
  roboclaw_serial::MainBatteryVoltage request;

  ASSERT_THROW(interface.write(request), std::invalid_argument);
}
