#include "roboclaw_serial/roboclaw_serial.hpp"

#include <memory>

int main()
{
    auto device = std::make_shared<roboclaw_serial::SerialDevice>("/dev/ttyACM0");
    auto interface = std::make_shared<roboclaw_serial::Interface>(device);

    // Read firware version
    const auto [fw_version] = interface->read<roboclaw_serial::FirmwareVersion>();

    std::cout << "Firmware versions: " << fw_version << std::endl;

    // Drive motor 1 forward
    roboclaw_serial::DriveM1Forward m1_command;
    auto & [m1_speed] = m1_command.fields;
    m1_speed = 50;

    while (device->connected())
    {
        interface->write<roboclaw_serial::DriveM1Forward>(m1_command, 128);
        const auto [left_count, right_count] = interface->read<roboclaw_serial::EncoderCounters>();
        const auto [batt] = interface->read<roboclaw_serial::MainBatteryVoltage>();
        float batt_percent = float(batt / 255);
        printf("Encoder counts: [%i, %i]\n", left_count, right_count);
    }

    return 0;
}