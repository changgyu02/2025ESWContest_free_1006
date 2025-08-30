#ifndef IMU_PKG_BNO055_HPP
#define IMU_PKG_BNO055_HPP

#include <cstdint>
#include <string>

class BNO055 {
public:
  explicit BNO055(const std::string& i2c_device = "/dev/i2c-1", uint8_t address = 0x28);
  ~BNO055();

  bool initialize();                                         // 센서 초기화
  bool readQuaternion(float& w, float& x, float& y, float& z); // 쿼터니언 읽기
  bool isFullyCalibrated();                                 // 보정 상태 확인
  bool loadCalibrationData(const std::string& json_file_path); // 보정값 로드

private:
  int i2c_fd_;               // I2C 파일 디스크립터
  std::string i2c_device_;
  uint8_t address_;

  // 내부 레지스터 접근 함수들
  bool writeByte(uint8_t reg, uint8_t value);
  bool writeBytes(uint8_t reg, const uint8_t* data, size_t length);
  bool readBytes(uint8_t reg, uint8_t* buffer, size_t length);
  void setMode(uint8_t mode);

  // 레지스터 주소 정의
  static constexpr uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
  static constexpr uint8_t BNO055_PWR_MODE_ADDR = 0x3E;
  static constexpr uint8_t BNO055_SYS_TRIGGER_ADDR = 0x3F;
  static constexpr uint8_t BNO055_UNIT_SEL_ADDR = 0x3B;
  static constexpr uint8_t QUATERNION_DATA_W_LSB = 0x20;
  static constexpr uint8_t CALIB_STAT_ADDR = 0x35;

  static constexpr uint8_t OPERATION_MODE_CONFIG = 0x00;
  static constexpr uint8_t OPERATION_MODE_NDOF = 0x0C;

  static constexpr uint8_t CALIBRATION_DATA_START_ADDR = 0x55;
  static constexpr size_t CALIBRATION_DATA_LENGTH = 22;

  static constexpr float QUAT_SCALE = (1.0f / (1 << 14)); // 2^14 분해능
};

#endif  // IMU_PKG_BNO055_HPP

