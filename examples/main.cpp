#include "roboclaw_serial/roboclaw_serial.hpp"
#include <memory>
#include <iostream>
#include <cstdlib>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <device> [device_id = 128] [baudrate = 115200]" << std::endl;
        return 1;
    }

    std::string device_path = argv[1];
    int device_id = (argc > 2) ? std::stoi(argv[2]) : 128;
    int baud = (argc > 3) ? std::stoi(argv[3]) : 115200;

    std::cout << baud << std::endl;
    std::cout << device_path << std::endl;

    // Connect to device
    auto device = std::make_shared<roboclaw_serial::SerialDevice>(device_path);

    // Create device interface
    auto interface = std::make_shared<roboclaw_serial::Interface>(device);

    // Read firmware version
    const auto [fw_version] = interface->read<roboclaw_serial::FirmwareVersion>(device_id);
    std::cout << "Firmware version: " << fw_version << std::endl;

    // Read encoder counts until terminated
    while (device->connected())
    {
        const auto [left_count, right_count] = interface->read<roboclaw_serial::EncoderCounters>(device_id);

        std::cout << "Encoder counts: [" << int(left_count) << ", " << int(right_count) << "]" << std::endl;
    }

    return 0;
}
