#pragma once

#include <array>
#include <tuple>

namespace roboclaw_serial
{

    // Serialize a single parameter into a buffer
    template <typename T>
    std::size_t serializeArg(const T &arg, std::byte*& iter)
    {
        // Reinterpret the input argument as a byte array
        const std::byte* data_ptr = reinterpret_cast<const std::byte*>(&arg);

        // Serialize each byte into a big-endian array
        for (size_t i = 0; i < sizeof(T); ++i)
        {
            *iter++ = data_ptr[sizeof(T) - 1 - i];
        }

        return sizeof(T);
    }

    // Deserialize a single parameter from a buffer
    template <typename T>
    void deserializeArg(T &arg, const std::byte*& iter)
    {
        // Reinterpret the input argument as a byte array
        std::byte* data_ptr = reinterpret_cast<std::byte*>(&arg);

        // Deserialize each byte into a big-endian array
        for (size_t i = 0; i < sizeof(T); ++i)
        {
            data_ptr[sizeof(T) - 1 - i] = *iter++;
        }
    }

    template <typename... Args>
    std::size_t serialize(const std::tuple<Args...>& values, std::byte*& iter)
    {
        std::size_t bytes_serialized = 0;
        std::apply([&iter, &bytes_serialized](const auto&... args)
        {
            ((bytes_serialized += serializeArg(args, iter)), ...);
        }, values);
        return bytes_serialized;
    }

    // Deserialize data from a buffer into a tuple of values
    template <typename... Args>
    void deserialize(std::tuple<Args...> &values, const std::byte *iter)
    {
        // Loop over each of values, and deserialize it from the buffer
        std::apply([&iter](auto &...args)
        {
            (deserializeArg(args, iter), ...);
        }, values);
    }

} // namespace roboclaw_serial
