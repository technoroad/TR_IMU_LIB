/*
The MIT License (MIT)
Copyright (c) 2019 Techno Road Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef ADIS_RCV_BIN_H_
#define ADIS_RCV_BIN_H_

#include <termios.h>

#include <cstdint>
#include <string>
#include <vector>

enum ImuBinError {
  kImuBinOk = 0,
  kImuBinErrCantRcvData = 1,
  kImuBinErrCouldNotFindPkt = 2,
  kImuBinErrInvalidData = 3,
  kImuBinErrChecksum = 4,
};

constexpr uint8_t kBinHeader = 0xAA;
constexpr int kBinRingBufSize = 4096;
constexpr int kBinPacketSize = 70;

constexpr double kGravity = 9.80665;
constexpr double kDeg2Rad = 0.01745329251;

// mpu_error bit masks (spec 5.4.1). bit4 is critical: when set, the IMU
// board cannot operate normally and the upper PC must stop the robot drive
// system and report the error to the developer.
constexpr uint8_t kMpuErrValueOutOfRange = 1 << 0;
constexpr uint8_t kMpuErrUnknownCommand  = 1 << 1;
constexpr uint8_t kMpuErrFlashWrite      = 1 << 2;
constexpr uint8_t kMpuErrWdtReboot       = 1 << 3;
constexpr uint8_t kMpuErrImuNotFound     = 1 << 4;

class AdisRcvBin {
 public:
  enum class State { INITIAL, READY, RUNNING };

  struct TelemetryData {
    uint8_t  mpu_error;
    uint32_t send_counter;
    double   quat[4];       // W, X, Y, Z (DATA / 32767.0)
    int32_t  acc_raw[3];    // X, Y, Z
    int32_t  gyro_raw[3];   // X, Y, Z
    int16_t  temperature;   // DATA / 10.0 = degrees
    uint16_t imu_counter;
    uint16_t imu_dropped;
    uint16_t computation_time_us;
    uint16_t spi_transaction_time_us;
    uint8_t  in0_port;
    uint64_t timestamp;     // MCU internal time [us] since boot (logging use only)
    uint8_t  response_id;
  };

  struct SettingsData {
    uint8_t  mpu_error;
    uint32_t send_counter;
    uint32_t build_date;
    uint8_t  peripheral_enable;
    uint8_t  read_32bit;
    uint8_t  filter_select;
    uint64_t accl_sensitivity;
    uint64_t gyro_sensitivity;
    uint16_t sample_rate;
    uint8_t  imu_maker;
    uint16_t product_id;
    uint8_t  model;
    uint8_t  board;
    uint8_t  grav_corr_en;
    uint8_t  in_pupd;
    uint8_t  in_trigger;
  };

  AdisRcvBin();
  ~AdisRcvBin();

  // Lifecycle
  bool Open(const std::string& device);
  void Close();

  // Commands
  bool SendCommand(uint8_t cmd_id, const uint8_t* data, size_t data_len);
  bool ReadSettings();
  bool StartTelemetry();
  bool StopTelemetry();
  bool ResetAttitude();

  // Data acquisition
  int UpdateTelemetry();

  // Accessors
  State GetState() const { return state_; }
  const TelemetryData& GetTelemetry() const { return telemetry_; }
  const SettingsData& GetSettings() const { return settings_; }

  // Unit conversion
  void GetAccSI(double ret[3]) const;
  void GetGyroSI(double ret[3]) const;
  void GetQuat(double ret[4]) const;
  double GetTemperature() const;
  uint64_t GetTimestamp() const { return telemetry_.timestamp; }

  std::string GetProductIdStr() const;

 private:
  int fd_;
  struct termios defaults_;
  State state_;

  TelemetryData telemetry_;
  SettingsData settings_;

  // Ring buffer
  uint8_t ring_buf_[kBinRingBufSize];
  int ring_write_pos_;
  int ring_data_count_;

  // Serial I/O
  int ReadSerial();
  int WriteBytes(const uint8_t* data, size_t len);
  void FlushSerial();

  // Packet operations
  std::vector<uint8_t> BuildPacket(uint8_t cmd_id, const uint8_t* data, size_t data_len);
  bool FindAndParsePacket();
  bool ParseTelemetryPayload(const uint8_t* payload, uint8_t response_id);
  bool ParseSettingsPayload(const uint8_t* payload);

  // Checksum
  static uint16_t CalcRFC1071(const uint8_t* data, size_t len);
  static bool VerifyRFC1071(const uint8_t* data, size_t len);

  // Helpers
  template <typename T>
  static T ReadLE(const uint8_t* p);

  bool WaitForResponse(uint8_t expected_id, int timeout_ms);

#if defined(UTEST)
 public:
  friend class AdisRcvBinTest;
#endif
};

#endif  // ADIS_RCV_BIN_H_
