cmake_minimum_required(VERSION 3.10)

# Name your project and specify the required C++ standard
project(roboclaw_serial_tests LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Locate and enable the GoogleTest framework
find_package(GTest REQUIRED)
include(GoogleTest)

# Add the executable for the test suite
add_executable(test_roboclaw_serial
    test_main.cpp
)

# Link the necessary libraries to the test executable
target_link_libraries(test_roboclaw_serial
    PRIVATE
        GTest::gtest
        GTest::gtest_main
)

# Link the header-only library (roboclaw_serial) to the test executable
target_include_directories(test_roboclaw_serial PRIVATE ../include)

# Enable testing and add your test suite
enable_testing()
gtest_discover_tests(test_roboclaw_serial)
