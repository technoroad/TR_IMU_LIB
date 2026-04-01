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

#include "adis_rcv_bin.h"

#include <fcntl.h>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <thread>

// ============================================================
// Template specializations for little-endian reads
// ============================================================
template <>
uint8_t AdisRcvBin::ReadLE<uint8_t>(const uint8_t* p)
{
  return p[0];
}

template <>
int16_t AdisRcvBin::ReadLE<int16_t>(const uint8_t* p)
{
  uint16_t v = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
  return static_cast<int16_t>(v);
}

template <>
uint16_t AdisRcvBin::ReadLE<uint16_t>(const uint8_t* p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

template <>
int32_t AdisRcvBin::ReadLE<int32_t>(const uint8_t* p)
{
  uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  return static_cast<int32_t>(v);
}

template <>
uint32_t AdisRcvBin::ReadLE<uint32_t>(const uint8_t* p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

template <>
uint64_t AdisRcvBin::ReadLE<uint64_t>(const uint8_t* p)
{
  uint64_t v = 0;
  for (int i = 7; i >= 0; i--) {
    v = (v << 8) | p[i];
  }
  return v;
}

// ============================================================
// Constructor / Destructor
// ============================================================
AdisRcvBin::AdisRcvBin()
    : fd_(-1), state_(State::INITIAL), ring_write_pos_(0), ring_data_count_(0)
{
  memset(&defaults_, 0, sizeof(defaults_));
  memset(&telemetry_, 0, sizeof(telemetry_));
  memset(&settings_, 0, sizeof(settings_));
  memset(ring_buf_, 0, sizeof(ring_buf_));
}

AdisRcvBin::~AdisRcvBin()
{
  if (fd_ >= 0) {
    StopTelemetry();
    Close();
  }
}

// ============================================================
// Serial port management
// ============================================================
bool AdisRcvBin::Open(const std::string& device)
{
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    perror("AdisRcvBin::Open");
    return false;
  }

  if (tcgetattr(fd_, &defaults_) < 0) {
    perror("AdisRcvBin::Open tcgetattr");
    close(fd_);
    fd_ = -1;
    return false;
  }

  struct termios config;
  cfmakeraw(&config);
  config.c_cc[VMIN] = 0;
  config.c_cc[VTIME] = 10;  // 1 second read timeout

  if (tcsetattr(fd_, TCSANOW, &config) < 0) {
    perror("AdisRcvBin::Open tcsetattr");
    close(fd_);
    fd_ = -1;
    return false;
  }

  state_ = State::READY;
  return true;
}

void AdisRcvBin::Close()
{
  if (fd_ >= 0) {
    tcsetattr(fd_, TCSANOW, &defaults_);
    close(fd_);
    fd_ = -1;
  }
  state_ = State::INITIAL;
}

int AdisRcvBin::ReadSerial()
{
  constexpr int kReadBufSize = 1024;
  uint8_t buf[kReadBufSize];
  int rcv_cnt = read(fd_, buf, kReadBufSize);

  if (rcv_cnt <= 0) {
    return rcv_cnt;
  }

  for (int i = 0; i < rcv_cnt; i++) {
    ring_buf_[ring_write_pos_] = buf[i];
    ring_write_pos_ = (ring_write_pos_ + 1) % kBinRingBufSize;
    if (ring_data_count_ < kBinRingBufSize) {
      ring_data_count_++;
    }
  }

  return rcv_cnt;
}

int AdisRcvBin::WriteBytes(const uint8_t* data, size_t len)
{
  int written = write(fd_, data, len);
  if (written < 0) {
    perror("AdisRcvBin::WriteBytes");
  }
  return written;
}

void AdisRcvBin::FlushSerial()
{
  tcflush(fd_, TCIOFLUSH);
  ring_write_pos_ = 0;
  ring_data_count_ = 0;
  memset(ring_buf_, 0, sizeof(ring_buf_));
}

// ============================================================
// RFC1071 Checksum
// ============================================================
uint16_t AdisRcvBin::CalcRFC1071(const uint8_t* data, size_t len)
{
  // Sum individual bytes (matching device firmware implementation)
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }

  while (sum >> 16) {
    sum = (sum & 0xFFFF) + (sum >> 16);
  }

  return static_cast<uint16_t>(~sum & 0xFFFF);
}

bool AdisRcvBin::VerifyRFC1071(const uint8_t* data, size_t len)
{
  // Sum individual bytes for cmd + length + data,
  // then add checksum as uint16 LE (low byte + high byte << 8)
  if (len < 2) return false;

  uint32_t sum = 0;
  for (size_t i = 0; i < len - 2; i++) {
    sum += data[i];
  }
  // Add checksum bytes: low byte as-is, high byte shifted
  sum += data[len - 2];
  sum += static_cast<uint32_t>(data[len - 1]) << 8;

  while (sum >> 16) {
    sum = (sum & 0xFFFF) + (sum >> 16);
  }

  return (static_cast<uint16_t>(sum) == 0xFFFF);
}

// ============================================================
// Packet building
// ============================================================
std::vector<uint8_t> AdisRcvBin::BuildPacket(uint8_t cmd_id, const uint8_t* data, size_t data_len)
{
  // Header(2) + CmdID(1) + Length(1) + Data(data_len) + Checksum(2)
  std::vector<uint8_t> packet;
  packet.reserve(2 + 1 + 1 + data_len + 2);

  packet.push_back(kBinHeader);
  packet.push_back(kBinHeader);
  packet.push_back(cmd_id);
  packet.push_back(static_cast<uint8_t>(data_len));

  for (size_t i = 0; i < data_len; i++) {
    packet.push_back(data[i]);
  }

  // Checksum is computed over CmdID + Length + Data (bytes 2 onward, excluding header)
  uint16_t csum = CalcRFC1071(&packet[2], 1 + 1 + data_len);
  packet.push_back(static_cast<uint8_t>(csum & 0xFF));
  packet.push_back(static_cast<uint8_t>((csum >> 8) & 0xFF));

  return packet;
}

bool AdisRcvBin::SendCommand(uint8_t cmd_id, const uint8_t* data, size_t data_len)
{
  auto packet = BuildPacket(cmd_id, data, data_len);
  int written = WriteBytes(packet.data(), packet.size());
  return (written == static_cast<int>(packet.size()));
}

// ============================================================
// Packet finding and parsing from ring buffer
// ============================================================
bool AdisRcvBin::FindAndParsePacket()
{
  // Need at least kBinPacketSize (70) bytes
  while (ring_data_count_ >= kBinPacketSize) {
    int read_pos = (ring_write_pos_ - ring_data_count_ + kBinRingBufSize) % kBinRingBufSize;

    // Check header
    uint8_t h0 = ring_buf_[read_pos];
    uint8_t h1 = ring_buf_[(read_pos + 1) % kBinRingBufSize];

    if (h0 != kBinHeader || h1 != kBinHeader) {
      // Not a header, skip 1 byte
      ring_data_count_--;
      continue;
    }

    // Check if we have enough data for a full packet
    if (ring_data_count_ < kBinPacketSize) {
      return false;
    }

    // Extract 70 bytes
    uint8_t packet[kBinPacketSize];
    for (int i = 0; i < kBinPacketSize; i++) {
      packet[i] = ring_buf_[(read_pos + i) % kBinRingBufSize];
    }

    // Verify length field
    uint8_t length = packet[3];
    if (length != 64) {
      // Unexpected length, skip this header
      ring_data_count_--;
      continue;
    }

    // Verify checksum over bytes 2..69 (CmdID + Length + Data + Checksum)
    if (!VerifyRFC1071(&packet[2], kBinPacketSize - 2)) {
      // Checksum error, skip this header
      ring_data_count_--;
      continue;
    }

    // Valid packet found — consume it
    ring_data_count_ -= kBinPacketSize;

    uint8_t response_id = packet[2];
    const uint8_t* payload = &packet[4];  // Data starts at byte 4

    if (response_id >= 0x70 && response_id <= 0x77) {
      return ParseSettingsPayload(payload);
    } else {
      return ParseTelemetryPayload(payload, response_id);
    }
  }

  return false;
}

bool AdisRcvBin::ParseTelemetryPayload(const uint8_t* d, uint8_t response_id)
{
  telemetry_.response_id = response_id;
  telemetry_.mpu_warning = ReadLE<uint8_t>(&d[0]);
  telemetry_.send_counter = ReadLE<uint32_t>(&d[1]);

  // Quaternion: int16 -> double (DATA / 32767.0)
  telemetry_.quat[0] = static_cast<double>(ReadLE<int16_t>(&d[5])) / 32767.0;   // W
  telemetry_.quat[1] = static_cast<double>(ReadLE<int16_t>(&d[7])) / 32767.0;   // X
  telemetry_.quat[2] = static_cast<double>(ReadLE<int16_t>(&d[9])) / 32767.0;   // Y
  telemetry_.quat[3] = static_cast<double>(ReadLE<int16_t>(&d[11])) / 32767.0;  // Z

  // Acceleration: int32 raw
  telemetry_.acc_raw[0] = ReadLE<int32_t>(&d[13]);
  telemetry_.acc_raw[1] = ReadLE<int32_t>(&d[17]);
  telemetry_.acc_raw[2] = ReadLE<int32_t>(&d[21]);

  // Gyro: int32 raw
  telemetry_.gyro_raw[0] = ReadLE<int32_t>(&d[25]);
  telemetry_.gyro_raw[1] = ReadLE<int32_t>(&d[29]);
  telemetry_.gyro_raw[2] = ReadLE<int32_t>(&d[33]);

  // Temperature, counters, timing
  telemetry_.temperature = ReadLE<int16_t>(&d[37]);
  telemetry_.imu_counter = ReadLE<uint16_t>(&d[39]);
  telemetry_.imu_dropped = ReadLE<uint16_t>(&d[41]);
  telemetry_.computation_time_us = ReadLE<uint16_t>(&d[43]);
  telemetry_.spi_transaction_time_us = ReadLE<uint16_t>(&d[45]);
  telemetry_.in0_port = ReadLE<uint8_t>(&d[47]);

  return true;
}

bool AdisRcvBin::ParseSettingsPayload(const uint8_t* d)
{
  settings_.mpu_warning = ReadLE<uint8_t>(&d[0]);
  settings_.send_counter = ReadLE<uint32_t>(&d[1]);
  settings_.build_date = ReadLE<uint32_t>(&d[5]);
  settings_.peripheral_enable = ReadLE<uint8_t>(&d[9]);
  settings_.read_32bit = ReadLE<uint8_t>(&d[10]);
  settings_.filter_select = ReadLE<uint8_t>(&d[11]);
  settings_.accl_sensitivity = ReadLE<uint64_t>(&d[12]);
  settings_.gyro_sensitivity = ReadLE<uint64_t>(&d[20]);
  settings_.sample_rate = ReadLE<uint16_t>(&d[28]);
  settings_.imu_maker = ReadLE<uint8_t>(&d[30]);
  settings_.product_id = ReadLE<uint16_t>(&d[31]);
  settings_.model = ReadLE<uint8_t>(&d[33]);
  settings_.board = ReadLE<uint8_t>(&d[34]);
  settings_.grav_corr_en = ReadLE<uint8_t>(&d[35]);
  settings_.in_pupd = ReadLE<uint8_t>(&d[36]);
  settings_.in_trigger = ReadLE<uint8_t>(&d[37]);

  return true;
}

// ============================================================
// High-level commands
// ============================================================
bool AdisRcvBin::StartTelemetry()
{
  uint8_t data = 0x00;
  if (!SendCommand(0x31, &data, 1)) return false;
  state_ = State::RUNNING;
  return true;
}

bool AdisRcvBin::StopTelemetry()
{
  uint8_t data = 0x00;
  bool ret = SendCommand(0x32, &data, 1);
  if (state_ == State::RUNNING) {
    state_ = State::READY;
  }
  return ret;
}

bool AdisRcvBin::ResetAttitude()
{
  uint8_t data = 0x00;
  return SendCommand(0x33, &data, 1);
}

bool AdisRcvBin::ReadSettings()
{
  FlushSerial();

  uint8_t data = 0x00;
  if (!SendCommand(0x70, &data, 1)) return false;

  return WaitForResponse(0x70, 2000);
}

bool AdisRcvBin::WaitForResponse(uint8_t expected_id, int timeout_ms)
{
  auto start = std::chrono::steady_clock::now();

  while (true) {
    ReadSerial();

    // Try to find and parse a packet
    int saved_count = ring_data_count_;
    if (FindAndParsePacket()) {
      // Check if we got the expected response type
      if (expected_id >= 0x70 && expected_id <= 0x77) {
        // Settings response — always parsed into settings_
        return true;
      } else {
        // Telemetry response — check response_id
        if (telemetry_.response_id == expected_id) {
          return true;
        }
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= timeout_ms) {
      return false;
    }

    if (saved_count == ring_data_count_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

// ============================================================
// Main data acquisition
// ============================================================
int AdisRcvBin::UpdateTelemetry()
{
  int rcv = ReadSerial();
  if (rcv < 0) {
    return kImuBinErrCantRcvData;
  }

  if (ring_data_count_ < kBinPacketSize) {
    if (rcv == 0) {
      return kImuBinErrCantRcvData;
    }
    return kImuBinErrCouldNotFindPkt;
  }

  // Find the latest packet by consuming all available packets
  bool found = false;
  while (ring_data_count_ >= kBinPacketSize) {
    if (FindAndParsePacket()) {
      // Only count telemetry packets (not settings)
      if (telemetry_.response_id <= 0x35) {
        found = true;
      }
    } else {
      break;
    }
  }

  if (!found) {
    return kImuBinErrCouldNotFindPkt;
  }

  return kImuBinOk;
}

// ============================================================
// Unit conversion
// ============================================================
void AdisRcvBin::GetAccSI(double ret[3]) const
{
  double sensi = static_cast<double>(settings_.accl_sensitivity) * 1e-6;
  if (sensi <= 0.0) {
    ret[0] = ret[1] = ret[2] = 0.0;
    return;
  }
  for (int i = 0; i < 3; i++) {
    ret[i] = static_cast<double>(telemetry_.acc_raw[i]) / sensi * kGravity;
  }
}

void AdisRcvBin::GetGyroSI(double ret[3]) const
{
  double sensi = static_cast<double>(settings_.gyro_sensitivity) * 1e-6;
  if (sensi <= 0.0) {
    ret[0] = ret[1] = ret[2] = 0.0;
    return;
  }
  for (int i = 0; i < 3; i++) {
    ret[i] = static_cast<double>(telemetry_.gyro_raw[i]) / sensi * kDeg2Rad;
  }
}

void AdisRcvBin::GetQuat(double ret[4]) const
{
  for (int i = 0; i < 4; i++) {
    ret[i] = telemetry_.quat[i];
  }
}

double AdisRcvBin::GetTemperature() const
{
  return static_cast<double>(telemetry_.temperature) / 10.0;
}

std::string AdisRcvBin::GetProductIdStr() const
{
  char buf[64];
  uint16_t pid = settings_.product_id;
  uint8_t model = settings_.model;

  const char* model_str = "";
  switch (model) {
    case 0x03:
      model_str = "-1";
      break;
    case 0x07:
      model_str = "-2";
      break;
    case 0x0F:
      model_str = "-3";
      break;
    default:
      model_str = "";
      break;
  }

  snprintf(buf, sizeof(buf), "ADIS%u%s", pid, model_str);
  return std::string(buf);
}
