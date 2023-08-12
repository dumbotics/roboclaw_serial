#pragma once

// Including necessary libraries and files.
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dummy_serial_device.hpp"
#include "roboclaw_serial/command.hpp"
#include "roboclaw_serial/interface.hpp"

struct Data
{
  std::string requestType;  // "read" or "write"
  const std::vector<std::byte> writeBytes;
  const std::vector<std::byte> readBytes;
};

template <typename RequestType>
struct TestData
{
  typename RequestType::ArgsTuple args;
  Data data;
};

class TestExecutor : public ::testing::Test
{
protected:
  // Method to execute test
  template <typename RequestType>
  void executeTest(const TestData<RequestType> & p)
  {
    // Create a dummy device
    auto device =
      std::make_shared<DummyTestDevice>(std::move(p.data.readBytes), std::move(p.data.writeBytes));

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

TEST_F(TestExecutor, WriteVelocityPIDConstantsM1SerializationTest)
{
  this->executeTest(TestData<roboclaw_serial::VelocityPIDConstantsM1>{
    {0, 8631, 878, 201562},  // request arguments
    {
      "write",  // requestType
      {std::byte{0x80}, std::byte{0x1c}, std::byte{0x00}, std::byte{0x00},
       std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
       std::byte{0x21}, std::byte{0xb7}, std::byte{0x00}, std::byte{0x00},
       std::byte{0x03}, std::byte{0x6e}, std::byte{0x00}, std::byte{0x03},
       std::byte{0x13}, std::byte{0x5a}, std::byte{0x7d}, std::byte{0x8e}},  // writeBytes
      {std::byte{0xff}}                                                      // readBytes
    }});
}

TEST_F(TestExecutor, ReadEncoderCountersSerializationTest)
{
  this->executeTest(TestData<roboclaw_serial::EncoderCounters>{
    {12528, 53212},  // request arguments
    {
      "read",                              // requestType
      {std::byte{0x80}, std::byte{0x4e}},  // writeBytes
      {std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
       std::byte{0x00}, std::byte{0x30}, std::byte{0xf0}, std::byte{0x00}, std::byte{0x00},
       std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0xcf},
       std::byte{0xdc}, std::byte{0xcf}, std::byte{0x2e}}  // readBytes
    }});
}
