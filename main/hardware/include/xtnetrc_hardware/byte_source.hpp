#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
namespace xtnetrc::hardware {
class ByteSource {
public:
    virtual ~ByteSource() = default;
    virtual std::size_t read(std::uint8_t* bytes, std::size_t capacity,
                             std::chrono::milliseconds timeout) = 0;
};
std::unique_ptr<ByteSource> open_serial(const std::string& device, int baud);
}  // namespace xtnetrc::hardware
