#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>

#include "dummy_serial_device.hpp"
#include "roboclaw_serial/command.hpp"
#include "roboclaw_serial/serialization.hpp"

namespace roboclaw_serial
{
void RunSerializationTest(const nlohmann::json & test_case)
{
  // Instantiate the corresponding request from the JSON data
  auto request = json_to_request(test_case);

  // Create a dummy serial device and set the expected request/response
  DummySerialDevice serial_device;
  serial_device.setExpectedRequest(request.getExpectedRequest());
  serial_device.setResponse(request.getResponse());

  // Call the send() function of the Interface class with the dummy serial device and the request
  Interface interface(serial_device);
  auto result = interface.send(request);

  // Compare the result with the expected result from the test case
  EXPECT_EQ(result, request.getExpectedResult());
}

TEST(Serialization, TestCasesFromJson)
{
  // Read the test data from the JSON file
  std::ifstream test_data_file("test_data.json");
  nlohmann::json test_data;
  test_data_file >> test_data;

  // Loop through the test cases and run the serialization test for each
  for (const auto & test_case : test_data) {
    RunSerializationTest(test_case);
  }
}
}  // namespace roboclaw_serial
