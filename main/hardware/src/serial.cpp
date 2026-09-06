#include "xtnetrc_hardware/byte_source.hpp"
#include <cerrno>
#include <stdexcept>
#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif
namespace xtnetrc::hardware {
#if defined(__unix__) || defined(__APPLE__)
namespace {
class SerialPort final : public ByteSource {
public:
    SerialPort(const std::string& path, int baud) {
        speed_t speed;
        if (baud == 9600) speed = B9600;
        else if (baud == 115200) speed = B115200;
        else throw std::invalid_argument("unsupported serial baud rate");
        fd_ = ::open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw std::runtime_error("cannot open serial device: " + path);
        termios settings{};
        if (tcgetattr(fd_, &settings)) { ::close(fd_); throw std::runtime_error("tcgetattr failed"); }
        cfmakeraw(&settings);
        cfsetispeed(&settings, speed);
        cfsetospeed(&settings, speed);
        settings.c_cflag |= CLOCAL | CREAD;
        settings.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd_, TCSANOW, &settings) || tcflush(fd_, TCIFLUSH)) {
            ::close(fd_); throw std::runtime_error("serial configuration failed");
        }
    }
    ~SerialPort() override { ::close(fd_); }
    std::size_t read(std::uint8_t* bytes, std::size_t capacity,
                     std::chrono::milliseconds timeout) override {
        if (!bytes || !capacity || timeout.count() < 0 || timeout.count() > 500)
            throw std::invalid_argument("invalid bounded serial read");
        pollfd item{fd_, POLLIN, 0};
        const int status = poll(&item, 1, static_cast<int>(timeout.count()));
        if (status == 0 || (status < 0 && errno == EINTR)) return 0;
        if (status < 0 || (item.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw std::runtime_error("serial disconnected/poll failed");
        const auto count = ::read(fd_, bytes, capacity);
        if (count < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
        if (count <= 0) throw std::runtime_error("serial read failed/EOF");
        return static_cast<std::size_t>(count);
    }
private:
    int fd_{-1};
};
}
#endif
std::unique_ptr<ByteSource> open_serial(const std::string& device, int baud) {
#if defined(__unix__) || defined(__APPLE__)
    return std::make_unique<SerialPort>(device, baud);
#else
    (void)device; (void)baud;
    throw std::runtime_error("serial adapter unavailable on this platform");
#endif
}
}  // namespace xtnetrc::hardware
