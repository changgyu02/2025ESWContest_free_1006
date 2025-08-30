#include "sensor_pkg/bno055.hpp"
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cstring>
#include <fstream>

using json = nlohmann::json;

BNO055::BNO055(const std::string& i2c_device, uint8_t address)
    : i2c_device_(i2c_device), address_(address) {
  i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    throw std::runtime_error("I2C 디바이스 열기 실패");
  }
  if (ioctl(i2c_fd_, I2C_SLAVE, address_) < 0) {
    close(i2c_fd_);
    throw std::runtime_error("I2C 슬레이브 주소 설정 실패");
  }
}

BNO055::~BNO055() {
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

bool BNO055::initialize() {
  setMode(OPERATION_MODE_CONFIG);
  usleep(20000);

  if (!writeByte(BNO055_PWR_MODE_ADDR, 0x00)) return false;  // Normal mode
  usleep(10000);

  if (!writeByte(BNO055_SYS_TRIGGER_ADDR, 0x00)) return false;
  usleep(10000);

  setMode(OPERATION_MODE_NDOF);
  usleep(20000);

  return true;
}

bool BNO055::readQuaternion(float& w, float& x, float& y, float& z) {
  uint8_t buffer[8];
  if (!readBytes(QUATERNION_DATA_W_LSB, buffer, 8)) return false;

  int16_t qw = (buffer[1] << 8) | buffer[0];
  int16_t qx = (buffer[3] << 8) | buffer[2];
  int16_t qy = (buffer[5] << 8) | buffer[4];
  int16_t qz = (buffer[7] << 8) | buffer[6];

  w = qw * QUAT_SCALE;
  x = qx * QUAT_SCALE;
  y = qy * QUAT_SCALE;
  z = qz * QUAT_SCALE;

  return true;
}

bool BNO055::isFullyCalibrated() {
  uint8_t calib;
  if (!readBytes(CALIB_STAT_ADDR, &calib, 1)) return false;
  uint8_t sys = (calib >> 6) & 0x03;
  uint8_t gyro = (calib >> 4) & 0x03;
  uint8_t accel = (calib >> 2) & 0x03;
  uint8_t mag = calib & 0x03;

  return (sys == 3 && gyro == 3 && accel == 3 && mag == 3);
}

bool BNO055::loadCalibrationData(const std::string& json_file_path) {
  std::ifstream file(json_file_path);
  if (!file.is_open()) return false;

  json j;
  try {
    file >> j;
  } catch (...) {
    return false;
  }
  file.close();

  std::vector<uint8_t> calib;
  for (const auto& val : j["calibration_data"]) {
    calib.push_back(static_cast<uint8_t>(val));
  }
  if (calib.size() != CALIBRATION_DATA_LENGTH) return false;

  setMode(OPERATION_MODE_CONFIG);
  usleep(25000);

  if (!writeBytes(CALIBRATION_DATA_START_ADDR, calib.data(), calib.size()))
    return false;

  setMode(OPERATION_MODE_NDOF);
  usleep(20000);
  return true;
}

bool BNO055::writeByte(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  return write(i2c_fd_, buffer, 2) == 2;
}

bool BNO055::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
  if (write(i2c_fd_, &reg, 1) != 1) return false;
  return read(i2c_fd_, buffer, length) == static_cast<ssize_t>(length);
}

bool BNO055::writeBytes(uint8_t reg, const uint8_t* data, size_t length) {
  std::vector<uint8_t> buffer(length + 1);
  buffer[0] = reg;
  std::memcpy(&buffer[1], data, length);
  return write(i2c_fd_, buffer.data(), buffer.size()) == static_cast<ssize_t>(buffer.size());
}

void BNO055::setMode(uint8_t mode) {
  writeByte(BNO055_OPR_MODE_ADDR, mode);
  usleep(30000);
}
