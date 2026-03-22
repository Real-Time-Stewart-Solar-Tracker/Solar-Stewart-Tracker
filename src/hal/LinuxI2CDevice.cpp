#include "hal/LinuxI2CDevice.hpp"

#if defined(__linux__)

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cstring>

namespace solar::hal {

namespace {
constexpr std::size_t kMaxWriteRegBytes = 32;
}

LinuxI2CDevice::LinuxI2CDevice(std::string dev_path, uint8_t address) noexcept {
    fd_ = ::open(dev_path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        fd_ = -1;
        return;
    }

    if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
        ::close(fd_);
        fd_ = -1;
        return;
    }
}

LinuxI2CDevice::~LinuxI2CDevice() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool LinuxI2CDevice::ok() const noexcept {
    return fd_ >= 0;
}

bool LinuxI2CDevice::write_reg_u8(uint8_t reg, uint8_t value) noexcept {
    if (fd_ < 0) return false;
    const uint8_t buf[2] = {reg, value};
    return ::write(fd_, buf, sizeof(buf)) == static_cast<ssize_t>(sizeof(buf));
}

bool LinuxI2CDevice::write_reg_bytes(uint8_t reg, const uint8_t* data, size_t len) noexcept {
    if (fd_ < 0) return false;
    if (len > 0 && data == nullptr) return false;
    if (len > kMaxWriteRegBytes) return false;

    std::array<uint8_t, 1 + kMaxWriteRegBytes> buf{};
    buf[0] = reg;

    if (len > 0) {
        std::memcpy(buf.data() + 1, data, len);
    }

    const size_t total = 1 + len;
    return ::write(fd_, buf.data(), total) == static_cast<ssize_t>(total);
}

bool LinuxI2CDevice::read_reg_bytes(uint8_t reg, uint8_t* data, size_t len) noexcept {
    if (fd_ < 0) return false;
    if (len > 0 && data == nullptr) return false;

    // Standard I2C register read: write register pointer, then read payload
    if (::write(fd_, &reg, 1) != 1) return false;
    return ::read(fd_, data, len) == static_cast<ssize_t>(len);
}

} // namespace solar::hal

#else  // non-Linux: compile-safe stub

namespace solar::hal {

LinuxI2CDevice::LinuxI2CDevice(std::string, uint8_t) noexcept {}
LinuxI2CDevice::~LinuxI2CDevice() = default;

bool LinuxI2CDevice::ok() const noexcept { return false; }

bool LinuxI2CDevice::write_reg_u8(uint8_t, uint8_t) noexcept { return false; }
bool LinuxI2CDevice::write_reg_bytes(uint8_t, const uint8_t*, size_t) noexcept { return false; }
bool LinuxI2CDevice::read_reg_bytes(uint8_t, uint8_t*, size_t) noexcept { return false; }

} // namespace solar::hal

#endif