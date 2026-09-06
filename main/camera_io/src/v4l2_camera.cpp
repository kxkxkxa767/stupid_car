#include "xtnetrc_camera/frame_source.hpp"
#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#ifdef __linux__
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#endif

namespace xtnetrc::camera {
#ifdef __linux__
namespace {
void checked_ioctl(int fd, unsigned long request, void* value) {
    int result;
    do { result = ioctl(fd, request, value); } while (result < 0 && errno == EINTR);
    if (result < 0) throw std::runtime_error("V4L2 ioctl failed: " + std::to_string(request));
}
class V4l2Camera final : public FrameSource {
public:
    V4l2Camera(const vision::GroundProjectionConfig& calibration, const IdentityCheck& identity)
        : size_(calibration.image_width, calibration.image_height) {
        try {
            fd_ = ::open(identity.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            if (fd_ < 0) throw std::runtime_error("cannot open fixed camera: " + identity.device);
            struct stat actual{}, expected{};
            if (fstat(fd_, &actual) || !S_ISCHR(actual.st_mode))
                throw std::runtime_error("camera must be a character device");
            // Revalidate the opened descriptor, not just a pathname checked
            // before open (USB enumeration can change between the two).
            if (identity.matched && (stat(identity.expected_device.c_str(), &expected) ||
                actual.st_rdev != expected.st_rdev))
                throw std::runtime_error("camera identity changed during open");
            v4l2_capability capabilities{};
            checked_ioctl(fd_, VIDIOC_QUERYCAP, &capabilities);
            const auto caps = (capabilities.capabilities & V4L2_CAP_DEVICE_CAPS)
                ? capabilities.device_caps : capabilities.capabilities;
            if (!(caps & V4L2_CAP_VIDEO_CAPTURE) || !(caps & V4L2_CAP_STREAMING))
                throw std::runtime_error("camera requires V4L2 capture + streaming");
            v4l2_format format{};
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            format.fmt.pix.width = size_.width;
            format.fmt.pix.height = size_.height;
            format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
            format.fmt.pix.field = V4L2_FIELD_ANY;
            checked_ioctl(fd_, VIDIOC_S_FMT, &format);
            if (format.fmt.pix.width != static_cast<unsigned>(size_.width) ||
                format.fmt.pix.height != static_cast<unsigned>(size_.height) ||
                format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
                throw std::runtime_error("camera did not accept native calibrated size/MJPEG; no resize fallback");
            v4l2_streamparm rate{};
            rate.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            rate.parm.capture.timeperframe = {1, 30};
            // Unsupported frame-rate control is not identity/calibration failure.
            ioctl(fd_, VIDIOC_S_PARM, &rate);
            v4l2_requestbuffers request{};
            request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            request.memory = V4L2_MEMORY_MMAP;
            request.count = 2;
            checked_ioctl(fd_, VIDIOC_REQBUFS, &request);
            if (request.count < 2 || request.count > 32) throw std::runtime_error("invalid camera buffer count");
            for (unsigned i = 0; i < request.count; ++i) {
                v4l2_buffer buffer{};
                buffer.type = request.type;
                buffer.memory = request.memory;
                buffer.index = i;
                checked_ioctl(fd_, VIDIOC_QUERYBUF, &buffer);
                void* address = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fd_, buffer.m.offset);
                if (address == MAP_FAILED) throw std::runtime_error("camera mmap failed");
                buffers_.push_back({address, buffer.length});
                checked_ioctl(fd_, VIDIOC_QBUF, &buffer);
            }
            auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            checked_ioctl(fd_, VIDIOC_STREAMON, &type);
            streaming_ = true;
        } catch (...) { close(); throw; }
    }
    ~V4l2Camera() override { close(); }
    bool read(Frame& frame, std::chrono::milliseconds timeout) override {
        if (timeout.count() < 0 || timeout > std::chrono::milliseconds(500))
            throw std::invalid_argument("camera wait must be 0..500 ms");
        pollfd item{fd_, POLLIN, 0};
        const int status = poll(&item, 1, static_cast<int>(timeout.count()));
        if (status == 0 || (status < 0 && errno == EINTR)) return false;
        if (status < 0 || (item.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw std::runtime_error("camera disconnected/poll failed");
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN || errno == EINTR) return false;
            throw std::runtime_error("camera dequeue failed");
        }
        if (buffer.index >= buffers_.size() || buffer.bytesused > buffers_[buffer.index].length)
            throw std::runtime_error("invalid camera buffer");
        // Kernel monotonic capture timestamps reject queued old frames. Unknown
        // timestamp semantics fail closed; never relabel old frames as fresh.
        if ((buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
            throw std::runtime_error("camera lacks monotonic capture timestamps");
        timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);
        const double age_s = now.tv_sec + now.tv_nsec * 1e-9 -
            (buffer.timestamp.tv_sec + buffer.timestamp.tv_usec * 1e-6);
        const auto captured = Clock::now() - std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(age_s));
        const auto* bytes = static_cast<const std::uint8_t*>(buffers_[buffer.index].address);
        std::vector<std::uint8_t> encoded(bytes, bytes + buffer.bytesused);
        const bool damaged = (buffer.flags & V4L2_BUF_FLAG_ERROR) != 0;
        checked_ioctl(fd_, VIDIOC_QBUF, &buffer);
        if (damaged || age_s < 0 || age_s > 0.25) return false;
        cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (decoded.empty() || decoded.size() != size_)
            throw std::runtime_error("decoded frame differs from calibrated resolution");
        frame = {std::move(decoded), captured};
        return true;
    }
private:
    void close() noexcept {
        if (streaming_) {
            auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
        }
        streaming_ = false;
        for (auto buffer : buffers_) munmap(buffer.address, buffer.length);
        buffers_.clear();
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }
    struct Buffer { void* address; std::size_t length; };
    std::vector<Buffer> buffers_;
    int fd_{-1};
    bool streaming_{false};
    cv::Size size_;
};
}
#endif
std::unique_ptr<FrameSource> open_fixed_camera(
    const vision::GroundProjectionConfig& calibration, const IdentityCheck& identity) {
#ifdef __linux__
    return std::make_unique<V4l2Camera>(calibration, identity);
#else
    (void)calibration; (void)identity;
    throw std::runtime_error("live fixed-camera capture requires Linux; use --replay-image offline");
#endif
}
}  // namespace xtnetrc::camera
