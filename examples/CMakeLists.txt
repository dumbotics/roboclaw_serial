cmake_minimum_required(VERSION 3.10)

add_executable(roboclaw_serial_examples
   main.cpp
)

target_link_libraries(roboclaw_serial_examples
PRIVATE
    roboclaw_serial
)
