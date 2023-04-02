#pragma once

#include <string>
#include <stdint.h>

namespace roboclaw_serial
{
    struct Config
    {
        std::string device;
        uint32_t baudrate;
    };
}