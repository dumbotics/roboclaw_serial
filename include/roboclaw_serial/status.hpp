#pragma once

namespace roboclaw_serial
{
    enum Status
    {
        BAD_RESPONSE = -5,
        CRC_MISMATCH,
        WRITE_FAILED,
        READ_ERROR,
        BAD_CONNECTION,
        OK = 0
    };
}