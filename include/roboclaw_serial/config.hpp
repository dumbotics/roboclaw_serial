#pragma once

#include <stdint.h>

#include <string>

namespace roboclaw_serial
{
struct Config
{
  std::string device;
  uint32_t baudrate;
};
}  // namespace roboclaw_serial
