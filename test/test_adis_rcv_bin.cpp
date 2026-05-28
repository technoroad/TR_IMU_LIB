// Tests for AdisRcvBin.
// - Layer 1: pure-logic unit tests (no hardware required). Always runs.
// - Layer 2: integration tests against a real IMU. Gated by env var
//   IMU_DEVICE (e.g. IMU_DEVICE=/dev/ttyACM0). Skipped otherwise.
// See doc/test_plan_gen2.md for the overall plan.

#include <adis_rcv_bin.h>
#include <gtest/gtest.h>

#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

// Fixture is the friend; TEST_F-generated classes derive from it and reach
// private members only through these helper methods.
class AdisRcvBinTest : public ::testing::Test
{
 protected:
  AdisRcvBin imu_;

  // Static-method proxies.
  static uint16_t Calc(const uint8_t* data, size_t len)
  {
    return AdisRcvBin::CalcRFC1071(data, len);
  }
  static bool Verify(const uint8_t* data, size_t len)
  {
    return AdisRcvBin::VerifyRFC1071(data, len);
  }
  template <typename T>
  static T ReadLE(const uint8_t* p)
  {
    return AdisRcvBin::ReadLE<T>(p);
  }

  // Private-method proxies.
  std::vector<uint8_t> Build(uint8_t cmd, const std::vector<uint8_t>& data)
  {
    return imu_.BuildPacket(cmd, data.data(), data.size());
  }
  bool ParseTelemetry(const uint8_t* payload, uint8_t response_id)
  {
    return imu_.ParseTelemetryPayload(payload, response_id);
  }
  bool ParseSettings(const uint8_t* payload) { return imu_.ParseSettingsPayload(payload); }
  bool FindAndParse() { return imu_.FindAndParsePacket(); }

  // Accessors for private state.
  AdisRcvBin::TelemetryData& Telemetry() { return imu_.telemetry_; }
  AdisRcvBin::SettingsData& Settings() { return imu_.settings_; }
  int& RingDataCount() { return imu_.ring_data_count_; }
  int& RingWritePos() { return imu_.ring_write_pos_; }

  // Mimic ReadSerial() byte-by-byte injection into the ring buffer.
  void Inject(const uint8_t* data, size_t len)
  {
    for (size_t i = 0; i < len; i++) {
      imu_.ring_buf_[imu_.ring_write_pos_] = data[i];
      imu_.ring_write_pos_ = (imu_.ring_write_pos_ + 1) % kBinRingBufSize;
      if (imu_.ring_data_count_ < kBinRingBufSize) {
        imu_.ring_data_count_++;
      }
    }
  }

  // Build a complete 70-byte packet with a 64-byte payload.
  std::vector<uint8_t> BuildFullPacket(uint8_t cmd_id, const uint8_t payload[64])
  {
    std::vector<uint8_t> packet;
    packet.reserve(kBinPacketSize);
    packet.push_back(kBinHeader);
    packet.push_back(kBinHeader);
    packet.push_back(cmd_id);
    packet.push_back(64);
    for (int i = 0; i < 64; i++) packet.push_back(payload[i]);
    uint16_t csum = Calc(&packet[2], 2 + 64);
    packet.push_back(static_cast<uint8_t>(csum & 0xFF));
    packet.push_back(static_cast<uint8_t>((csum >> 8) & 0xFF));
    return packet;
  }
};

// ============================================================
// RFC1071 checksum
// ============================================================

TEST_F(AdisRcvBinTest, Rfc1071CalcEmpty)
{
  uint8_t d = 0x00;
  EXPECT_EQ(Calc(&d, 0), 0xFFFF);
}

TEST_F(AdisRcvBinTest, Rfc1071CalcSingleZero)
{
  uint8_t d = 0x00;
  EXPECT_EQ(Calc(&d, 1), 0xFFFF);
}

TEST_F(AdisRcvBinTest, Rfc1071CalcSingleFF)
{
  uint8_t d = 0xFF;
  EXPECT_EQ(Calc(&d, 1), 0xFF00);
}

TEST_F(AdisRcvBinTest, Rfc1071CalcCmdAndLength)
{
  // Sum = 0x70 + 0x40 = 0xB0, ~0xB0 & 0xFFFF = 0xFF4F.
  uint8_t d[] = {0x70, 0x40};
  EXPECT_EQ(Calc(d, 2), 0xFF4F);
}

TEST_F(AdisRcvBinTest, Rfc1071CalcAllFF4Bytes)
{
  // Sum = 0x3FC, ~0x3FC & 0xFFFF = 0xFC03.
  uint8_t d[] = {0xFF, 0xFF, 0xFF, 0xFF};
  EXPECT_EQ(Calc(d, 4), 0xFC03);
}

TEST_F(AdisRcvBinTest, Rfc1071CalcOverflowFold)
{
  // 1000 * 0xFF = 0x3E418. Fold: 0x3 + 0xE418 = 0xE41B. ~ & 0xFFFF = 0x1BE4.
  std::vector<uint8_t> d(1000, 0xFF);
  EXPECT_EQ(Calc(d.data(), d.size()), 0x1BE4);
}

TEST_F(AdisRcvBinTest, Rfc1071VerifyValid)
{
  uint8_t d[] = {0x70, 0x40, 0x4F, 0xFF};
  EXPECT_TRUE(Verify(d, 4));
}

TEST_F(AdisRcvBinTest, Rfc1071VerifyTampered)
{
  uint8_t d[] = {0x70, 0x41, 0x4F, 0xFF};
  EXPECT_FALSE(Verify(d, 4));
}

TEST_F(AdisRcvBinTest, Rfc1071VerifyTooShort)
{
  uint8_t d[] = {0xFF};
  EXPECT_FALSE(Verify(d, 1));
  EXPECT_FALSE(Verify(d, 0));
}

TEST_F(AdisRcvBinTest, Rfc1071BuildVerifyRoundtrip)
{
  std::vector<uint8_t> body{0x31, 0x40};
  for (int i = 0; i < 64; i++) body.push_back(static_cast<uint8_t>(i * 7));
  uint16_t csum = Calc(body.data(), body.size());
  body.push_back(static_cast<uint8_t>(csum & 0xFF));
  body.push_back(static_cast<uint8_t>((csum >> 8) & 0xFF));
  EXPECT_TRUE(Verify(body.data(), body.size()));
}

// ============================================================
// ReadLE template specializations
// ============================================================

TEST_F(AdisRcvBinTest, ReadLEUint8)
{
  uint8_t d[] = {0x42};
  EXPECT_EQ(ReadLE<uint8_t>(d), 0x42);
}

TEST_F(AdisRcvBinTest, ReadLEInt16Positive)
{
  uint8_t d[] = {0xFF, 0x7F};
  EXPECT_EQ(ReadLE<int16_t>(d), INT16_MAX);
}

TEST_F(AdisRcvBinTest, ReadLEInt16Negative)
{
  uint8_t d[] = {0x00, 0x80};
  EXPECT_EQ(ReadLE<int16_t>(d), INT16_MIN);
}

TEST_F(AdisRcvBinTest, ReadLEInt16MinusOne)
{
  uint8_t d[] = {0xFF, 0xFF};
  EXPECT_EQ(ReadLE<int16_t>(d), -1);
}

TEST_F(AdisRcvBinTest, ReadLEUint16)
{
  uint8_t d[] = {0x34, 0x12};
  EXPECT_EQ(ReadLE<uint16_t>(d), 0x1234u);
}

TEST_F(AdisRcvBinTest, ReadLEInt32Positive)
{
  uint8_t d[] = {0xFF, 0xFF, 0xFF, 0x7F};
  EXPECT_EQ(ReadLE<int32_t>(d), INT32_MAX);
}

TEST_F(AdisRcvBinTest, ReadLEInt32Negative)
{
  uint8_t d[] = {0x00, 0x00, 0x00, 0x80};
  EXPECT_EQ(ReadLE<int32_t>(d), INT32_MIN);
}

TEST_F(AdisRcvBinTest, ReadLEUint32)
{
  uint8_t d[] = {0x78, 0x56, 0x34, 0x12};
  EXPECT_EQ(ReadLE<uint32_t>(d), 0x12345678u);
}

TEST_F(AdisRcvBinTest, ReadLEUint64)
{
  uint8_t d[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
  EXPECT_EQ(ReadLE<uint64_t>(d), 0xEFCDAB8967452301ULL);
}

// ============================================================
// BuildPacket
// ============================================================

TEST_F(AdisRcvBinTest, BuildPacketLayout)
{
  uint8_t data[] = {0x12, 0x34, 0x56};
  auto pkt = Build(0x70, std::vector<uint8_t>(data, data + 3));
  ASSERT_EQ(pkt.size(), 2u + 1 + 1 + 3 + 2);
  EXPECT_EQ(pkt[0], 0xAA);
  EXPECT_EQ(pkt[1], 0xAA);
  EXPECT_EQ(pkt[2], 0x70);
  EXPECT_EQ(pkt[3], 3);
  EXPECT_EQ(pkt[4], 0x12);
  EXPECT_EQ(pkt[5], 0x34);
  EXPECT_EQ(pkt[6], 0x56);
  // Sum 0x70+3+0x12+0x34+0x56 = 0x10F → ~ & 0xFFFF = 0xFEF0.
  EXPECT_EQ(pkt[7], 0xF0);
  EXPECT_EQ(pkt[8], 0xFE);
}

TEST_F(AdisRcvBinTest, BuildPacketEmptyData)
{
  auto pkt = Build(0x30, {});
  ASSERT_EQ(pkt.size(), 2u + 1 + 1 + 0 + 2);
  EXPECT_EQ(pkt[0], 0xAA);
  EXPECT_EQ(pkt[1], 0xAA);
  EXPECT_EQ(pkt[2], 0x30);
  EXPECT_EQ(pkt[3], 0);
  // Sum 0x30 → ~ & 0xFFFF = 0xFFCF.
  EXPECT_EQ(pkt[4], 0xCF);
  EXPECT_EQ(pkt[5], 0xFF);
}

TEST_F(AdisRcvBinTest, BuildPacketSelfChecksum)
{
  std::vector<uint8_t> data(64, 0xAB);
  auto pkt = Build(0x31, data);
  EXPECT_TRUE(Verify(&pkt[2], pkt.size() - 2));
}

// ============================================================
// ParseTelemetryPayload
// ============================================================

TEST_F(AdisRcvBinTest, ParseTelemetryFieldByField)
{
  uint8_t d[64] = {0};
  d[0] = 0x05;
  d[1] = 0x78; d[2] = 0x56; d[3] = 0x34; d[4] = 0x12;  // send_counter
  d[5] = 0x00; d[6] = 0x40;                              // quat W = 16384
  d[7] = 0x00; d[8] = 0x00;                              // quat X = 0
  d[9] = 0xFF; d[10] = 0xFF;                             // quat Y = -1
  d[11] = 0x00; d[12] = 0x80;                            // quat Z = INT16_MIN
  d[13] = 0x78; d[14] = 0x56; d[15] = 0x34; d[16] = 0x12;
  d[17] = 0xFF; d[18] = 0xFF; d[19] = 0xFF; d[20] = 0xFF;
  d[21] = 0x01; d[22] = 0x00; d[23] = 0x00; d[24] = 0x00;
  d[25] = 100; d[26] = 0; d[27] = 0; d[28] = 0;
  d[29] = 200; d[30] = 0; d[31] = 0; d[32] = 0;
  d[33] = 44;  d[34] = 1; d[35] = 0; d[36] = 0;          // 300
  d[37] = 0x9C; d[38] = 0xFF;                            // temperature = -100
  d[39] = 0xCD; d[40] = 0xAB;                            // imu_counter
  d[41] = 5; d[42] = 0;
  d[43] = 250; d[44] = 0;
  d[45] = 0xE8; d[46] = 0x03;                            // 1000
  d[47] = 1;

  ASSERT_TRUE(ParseTelemetry(d, 0x31));

  const auto& t = Telemetry();
  EXPECT_EQ(t.response_id, 0x31);
  EXPECT_EQ(t.mpu_warning, 0x05);
  EXPECT_EQ(t.send_counter, 0x12345678u);
  EXPECT_NEAR(t.quat[0], 16384.0 / 32767.0, 1e-9);
  EXPECT_NEAR(t.quat[1], 0.0, 1e-9);
  EXPECT_NEAR(t.quat[2], -1.0 / 32767.0, 1e-9);
  EXPECT_NEAR(t.quat[3], -32768.0 / 32767.0, 1e-9);
  EXPECT_EQ(t.acc_raw[0], 0x12345678);
  EXPECT_EQ(t.acc_raw[1], -1);
  EXPECT_EQ(t.acc_raw[2], 1);
  EXPECT_EQ(t.gyro_raw[0], 100);
  EXPECT_EQ(t.gyro_raw[1], 200);
  EXPECT_EQ(t.gyro_raw[2], 300);
  EXPECT_EQ(t.temperature, -100);
  EXPECT_EQ(t.imu_counter, 0xABCDu);
  EXPECT_EQ(t.imu_dropped, 5u);
  EXPECT_EQ(t.computation_time_us, 250u);
  EXPECT_EQ(t.spi_transaction_time_us, 1000u);
  EXPECT_EQ(t.in0_port, 1);
}

// ============================================================
// ParseSettingsPayload
// ============================================================

TEST_F(AdisRcvBinTest, ParseSettingsFieldByField)
{
  uint8_t d[64] = {0};
  d[0] = 0x42;
  d[1] = 1;
  const uint32_t bd = 20260401u;
  for (int i = 0; i < 4; i++) d[5 + i] = static_cast<uint8_t>((bd >> (i * 8)) & 0xFF);
  d[9] = 0x07;
  d[10] = 0x01;
  d[11] = 0x02;
  const uint64_t accl = 52428800ULL;
  for (int i = 0; i < 8; i++) d[12 + i] = static_cast<uint8_t>((accl >> (i * 8)) & 0xFF);
  const uint64_t gyro = 10485760ULL;
  for (int i = 0; i < 8; i++) d[20 + i] = static_cast<uint8_t>((gyro >> (i * 8)) & 0xFF);
  d[28] = 0xD0; d[29] = 0x07;
  d[30] = 0x01;
  d[31] = 0x79; d[32] = 0x40;  // product_id = 16505
  d[33] = 0x07;
  d[34] = 0x02;
  d[35] = 0x01;
  d[36] = 0x03;
  d[37] = 0x01;

  ASSERT_TRUE(ParseSettings(d));

  const auto& s = Settings();
  EXPECT_EQ(s.mpu_warning, 0x42);
  EXPECT_EQ(s.send_counter, 1u);
  EXPECT_EQ(s.build_date, 20260401u);
  EXPECT_EQ(s.peripheral_enable, 0x07);
  EXPECT_EQ(s.read_32bit, 0x01);
  EXPECT_EQ(s.filter_select, 0x02);
  EXPECT_EQ(s.accl_sensitivity, 52428800ULL);
  EXPECT_EQ(s.gyro_sensitivity, 10485760ULL);
  EXPECT_EQ(s.sample_rate, 2000u);
  EXPECT_EQ(s.imu_maker, 0x01);
  EXPECT_EQ(s.product_id, 16505u);
  EXPECT_EQ(s.model, 0x07);
  EXPECT_EQ(s.board, 0x02);
  EXPECT_EQ(s.grav_corr_en, 0x01);
  EXPECT_EQ(s.in_pupd, 0x03);
  EXPECT_EQ(s.in_trigger, 0x01);
}

// ============================================================
// FindAndParsePacket
// ============================================================

TEST_F(AdisRcvBinTest, FindPacketHeaderResync)
{
  uint8_t noise[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
  Inject(noise, 5);

  uint8_t payload[64] = {0};
  payload[0] = 0xA5;
  auto pkt = BuildFullPacket(0x31, payload);
  Inject(pkt.data(), pkt.size());

  EXPECT_TRUE(FindAndParse());
  EXPECT_EQ(RingDataCount(), 0);
  EXPECT_EQ(Telemetry().response_id, 0x31);
  EXPECT_EQ(Telemetry().mpu_warning, 0xA5);
}

TEST_F(AdisRcvBinTest, FindPacketLengthFieldError)
{
  uint8_t bad[70];
  bad[0] = 0xAA; bad[1] = 0xAA; bad[2] = 0x31; bad[3] = 63;
  for (int i = 4; i < 70; i++) bad[i] = static_cast<uint8_t>(i);
  Inject(bad, 70);

  uint8_t payload[64] = {0};
  payload[0] = 0xBB;
  auto good = BuildFullPacket(0x32, payload);
  Inject(good.data(), good.size());

  EXPECT_TRUE(FindAndParse());
  EXPECT_EQ(Telemetry().response_id, 0x32);
  EXPECT_EQ(Telemetry().mpu_warning, 0xBB);
}

TEST_F(AdisRcvBinTest, FindPacketChecksumError)
{
  uint8_t payload[64] = {0};
  auto bad = BuildFullPacket(0x31, payload);
  bad[20] ^= 0xFF;
  Inject(bad.data(), bad.size());

  payload[0] = 0xCC;
  auto good = BuildFullPacket(0x33, payload);
  Inject(good.data(), good.size());

  EXPECT_TRUE(FindAndParse());
  EXPECT_EQ(Telemetry().response_id, 0x33);
  EXPECT_EQ(Telemetry().mpu_warning, 0xCC);
}

TEST_F(AdisRcvBinTest, FindPacketWraparound)
{
  RingWritePos() = kBinRingBufSize - 30;
  RingDataCount() = 0;

  uint8_t payload[64] = {0};
  payload[0] = 0xDD;
  auto pkt = BuildFullPacket(0x31, payload);
  Inject(pkt.data(), pkt.size());

  EXPECT_TRUE(FindAndParse());
  EXPECT_EQ(Telemetry().mpu_warning, 0xDD);
}

TEST_F(AdisRcvBinTest, FindPacketNotEnoughData)
{
  uint8_t pkt[50] = {0xAA, 0xAA};
  Inject(pkt, 50);
  EXPECT_FALSE(FindAndParse());
}

TEST_F(AdisRcvBinTest, FindPacketRoutesSettingsByResponseId)
{
  uint8_t payload[64] = {0};
  payload[0] = 0x99;
  payload[31] = 0x79; payload[32] = 0x40;
  auto pkt = BuildFullPacket(0x70, payload);
  Inject(pkt.data(), pkt.size());

  EXPECT_TRUE(FindAndParse());
  EXPECT_EQ(Settings().mpu_warning, 0x99);
  EXPECT_EQ(Settings().product_id, 16505u);
}

// ============================================================
// Unit conversion
// ============================================================

TEST_F(AdisRcvBinTest, GetAccSI_ZeroSensitivityReturnsZero)
{
  Settings().accl_sensitivity = 0;
  Telemetry().acc_raw[0] = 1000;
  Telemetry().acc_raw[1] = 2000;
  Telemetry().acc_raw[2] = 3000;
  double r[3];
  imu_.GetAccSI(r);
  EXPECT_EQ(r[0], 0.0);
  EXPECT_EQ(r[1], 0.0);
  EXPECT_EQ(r[2], 0.0);
}

TEST_F(AdisRcvBinTest, GetAccSI_Scaling)
{
  Settings().accl_sensitivity = 52428800ULL;
  Telemetry().acc_raw[0] = 5242880;
  Telemetry().acc_raw[1] = -5242880;
  Telemetry().acc_raw[2] = 0;
  double r[3];
  imu_.GetAccSI(r);
  const double s = 52428800.0 * 1e-6;
  EXPECT_NEAR(r[0],  5242880.0 / s * kGravity, 1e-6);
  EXPECT_NEAR(r[1], -5242880.0 / s * kGravity, 1e-6);
  EXPECT_NEAR(r[2], 0.0, 1e-9);
}

TEST_F(AdisRcvBinTest, GetGyroSI_ZeroSensitivityReturnsZero)
{
  Settings().gyro_sensitivity = 0;
  Telemetry().gyro_raw[0] = 100;
  Telemetry().gyro_raw[1] = 200;
  Telemetry().gyro_raw[2] = 300;
  double r[3];
  imu_.GetGyroSI(r);
  EXPECT_EQ(r[0], 0.0);
  EXPECT_EQ(r[1], 0.0);
  EXPECT_EQ(r[2], 0.0);
}

TEST_F(AdisRcvBinTest, GetGyroSI_Scaling)
{
  Settings().gyro_sensitivity = 10485760ULL;
  Telemetry().gyro_raw[0] = 1048576;
  Telemetry().gyro_raw[1] = -1048576;
  Telemetry().gyro_raw[2] = 0;
  double r[3];
  imu_.GetGyroSI(r);
  const double s = 10485760.0 * 1e-6;
  EXPECT_NEAR(r[0],  1048576.0 / s * kDeg2Rad, 1e-6);
  EXPECT_NEAR(r[1], -1048576.0 / s * kDeg2Rad, 1e-6);
  EXPECT_NEAR(r[2], 0.0, 1e-9);
}

TEST_F(AdisRcvBinTest, GetQuat_Passthrough)
{
  Telemetry().quat[0] = 0.5;
  Telemetry().quat[1] = -0.5;
  Telemetry().quat[2] = 0.5;
  Telemetry().quat[3] = -0.5;
  double r[4];
  imu_.GetQuat(r);
  EXPECT_DOUBLE_EQ(r[0], 0.5);
  EXPECT_DOUBLE_EQ(r[1], -0.5);
  EXPECT_DOUBLE_EQ(r[2], 0.5);
  EXPECT_DOUBLE_EQ(r[3], -0.5);
}

TEST_F(AdisRcvBinTest, GetTemperature_DivBy10)
{
  Telemetry().temperature = 256;
  EXPECT_DOUBLE_EQ(imu_.GetTemperature(), 25.6);
  Telemetry().temperature = -100;
  EXPECT_DOUBLE_EQ(imu_.GetTemperature(), -10.0);
}

// ============================================================
// GetProductIdStr
// ============================================================

TEST_F(AdisRcvBinTest, GetProductIdStr_Model03Suffix1)
{
  Settings().product_id = 16505;
  Settings().model = 0x03;
  EXPECT_EQ(imu_.GetProductIdStr(), "ADIS16505-1");
}

TEST_F(AdisRcvBinTest, GetProductIdStr_Model07Suffix2)
{
  Settings().product_id = 16505;
  Settings().model = 0x07;
  EXPECT_EQ(imu_.GetProductIdStr(), "ADIS16505-2");
}

TEST_F(AdisRcvBinTest, GetProductIdStr_Model0FSuffix3)
{
  Settings().product_id = 16505;
  Settings().model = 0x0F;
  EXPECT_EQ(imu_.GetProductIdStr(), "ADIS16505-3");
}

TEST_F(AdisRcvBinTest, GetProductIdStr_UnknownModelNoSuffix)
{
  Settings().product_id = 16470;
  Settings().model = 0xFF;
  EXPECT_EQ(imu_.GetProductIdStr(), "ADIS16470");
}

// ============================================================
// Layer 2: hardware integration tests
//
// Gated by the IMU_DEVICE environment variable. Tests named Hw_AtRest_*
// assume the device is sitting still; filter them out (--gtest_filter)
// when the IMU is being handled.
// ============================================================

class AdisRcvBinHwTest : public ::testing::Test
{
 protected:
  AdisRcvBin imu_;
  std::string device_;

  void SetUp() override
  {
    const char* env = std::getenv("IMU_DEVICE");
    if (env == nullptr || env[0] == '\0') {
      GTEST_SKIP() << "IMU_DEVICE not set; skipping hardware test.";
    }
    device_ = env;
  }

  void TearDown() override
  {
    if (imu_.GetState() != AdisRcvBin::State::INITIAL) {
      imu_.StopTelemetry();
      imu_.Close();
    }
  }

  // Open the device, stop any in-flight telemetry, and read settings.
  // Returns false (with a gtest failure) if any step fails.
  bool OpenAndPrepare()
  {
    if (!imu_.Open(device_)) {
      ADD_FAILURE() << "Open(" << device_
                    << ") failed. Check cable and dialout group membership.";
      return false;
    }
    imu_.StopTelemetry();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!imu_.ReadSettings()) {
      ADD_FAILURE() << "ReadSettings() failed; device may not be a TR-IMU-Platform2.";
      return false;
    }
    return true;
  }

  // Drive UpdateTelemetry() in a loop for the given duration and count
  // successful packets and hard errors. kImuBinErrCantRcvData (no data yet)
  // is treated as benign — it just means we polled faster than the device.
  void DrainTelemetry(std::chrono::milliseconds duration, int* packets, int* errors)
  {
    *packets = 0;
    *errors = 0;
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      const int r = imu_.UpdateTelemetry();
      if (r == kImuBinOk) {
        (*packets)++;
      } else if (r != kImuBinErrCantRcvData) {
        (*errors)++;
      }
    }
  }
};

TEST_F(AdisRcvBinHwTest, Hw_OpenClose)
{
  EXPECT_TRUE(imu_.Open(device_));
  EXPECT_EQ(imu_.GetState(), AdisRcvBin::State::READY);
  imu_.Close();
  EXPECT_EQ(imu_.GetState(), AdisRcvBin::State::INITIAL);
}

TEST_F(AdisRcvBinHwTest, Hw_OpenInvalidDevice)
{
  AdisRcvBin local;
  EXPECT_FALSE(local.Open("/dev/adis-bin-does-not-exist"));
  EXPECT_EQ(local.GetState(), AdisRcvBin::State::INITIAL);
}

TEST_F(AdisRcvBinHwTest, Hw_ReadSettings)
{
  ASSERT_TRUE(OpenAndPrepare());
  const auto& s = imu_.GetSettings();
  EXPECT_NE(s.product_id, 0);
  EXPECT_GT(s.accl_sensitivity, 0u);
  EXPECT_GT(s.gyro_sensitivity, 0u);
  EXPECT_GT(s.sample_rate, 0);
}

TEST_F(AdisRcvBinHwTest, Hw_NopCommand)
{
  ASSERT_TRUE(imu_.Open(device_));
  uint8_t data = 0;
  EXPECT_TRUE(imu_.SendCommand(0x30, &data, 1));
}

TEST_F(AdisRcvBinHwTest, Hw_StartStopTelemetry)
{
  ASSERT_TRUE(OpenAndPrepare());

  ASSERT_TRUE(imu_.StartTelemetry());
  EXPECT_EQ(imu_.GetState(), AdisRcvBin::State::RUNNING);

  // Verify telemetry is actually arriving.
  int packets = 0;
  int errors = 0;
  DrainTelemetry(std::chrono::milliseconds(500), &packets, &errors);
  EXPECT_GT(packets, 0) << "No packets received after StartTelemetry";

  ASSERT_TRUE(imu_.StopTelemetry());
  EXPECT_EQ(imu_.GetState(), AdisRcvBin::State::READY);
}

TEST_F(AdisRcvBinHwTest, Hw_TelemetryFlow)
{
  ASSERT_TRUE(OpenAndPrepare());
  ASSERT_TRUE(imu_.StartTelemetry());

  int packets = 0;
  int errors = 0;
  DrainTelemetry(std::chrono::seconds(1), &packets, &errors);

  // At the 100 Hz default sample rate we expect ~100; allow generous margin.
  EXPECT_GE(packets, 50) << "Too few packets in 1 second: " << packets;
  // Hard errors (checksum, invalid data) should be rare.
  EXPECT_LE(errors, packets / 100 + 1)
      << "Too many hard errors: " << errors << " of " << packets << " packets";
}

TEST_F(AdisRcvBinHwTest, Hw_QuaternionNorm)
{
  ASSERT_TRUE(OpenAndPrepare());
  ASSERT_TRUE(imu_.StartTelemetry());

  // Wait for at least one packet.
  bool got = false;
  for (int i = 0; i < 200 && !got; i++) {
    if (imu_.UpdateTelemetry() == kImuBinOk) got = true;
    else std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(got) << "No telemetry packet within 1 second";

  double q[4];
  imu_.GetQuat(q);
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  EXPECT_NEAR(norm, 1.0, 0.05) << "Quaternion norm out of range: " << norm;
}

TEST_F(AdisRcvBinHwTest, Hw_AtRest_GravityMagnitude)
{
  ASSERT_TRUE(OpenAndPrepare());
  ASSERT_TRUE(imu_.StartTelemetry());

  // Average over ~30 packets to smooth out noise.
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (n < 30 && std::chrono::steady_clock::now() < deadline) {
    if (imu_.UpdateTelemetry() == kImuBinOk) {
      double a[3];
      imu_.GetAccSI(a);
      sx += a[0]; sy += a[1]; sz += a[2];
      n++;
    }
  }
  ASSERT_GE(n, 10) << "Too few packets to compute average";

  const double ax = sx / n, ay = sy / n, az = sz / n;
  const double mag = std::sqrt(ax * ax + ay * ay + az * az);
  EXPECT_NEAR(mag, kGravity, 0.5) << "Gravity magnitude: " << mag << " m/s^2";
}

TEST_F(AdisRcvBinHwTest, Hw_AtRest_GyroSmall)
{
  ASSERT_TRUE(OpenAndPrepare());
  ASSERT_TRUE(imu_.StartTelemetry());

  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (n < 30 && std::chrono::steady_clock::now() < deadline) {
    if (imu_.UpdateTelemetry() == kImuBinOk) {
      double g[3];
      imu_.GetGyroSI(g);
      sx += g[0]; sy += g[1]; sz += g[2];
      n++;
    }
  }
  ASSERT_GE(n, 10) << "Too few packets to compute average";

  const double gx = sx / n, gy = sy / n, gz = sz / n;
  const double mag = std::sqrt(gx * gx + gy * gy + gz * gz);
  EXPECT_LT(mag, 0.05) << "At-rest gyro magnitude: " << mag << " rad/s";
}

TEST_F(AdisRcvBinHwTest, Hw_ResetAttitude)
{
  ASSERT_TRUE(OpenAndPrepare());
  EXPECT_TRUE(imu_.ResetAttitude());
}

TEST_F(AdisRcvBinHwTest, Hw_LongRunStability)
{
  ASSERT_TRUE(OpenAndPrepare());
  ASSERT_TRUE(imu_.StartTelemetry());

  int packets = 0;
  int errors = 0;
  DrainTelemetry(std::chrono::seconds(10), &packets, &errors);

  EXPECT_GE(packets, 500) << "Long run packet count too low: " << packets;
  EXPECT_LE(errors, packets / 100 + 1)
      << "Long run errors: " << errors << " of " << packets;
}

TEST_F(AdisRcvBinHwTest, Hw_SettingsCommandRoundtrip)
{
  // Change filter_select via 0x75 (without saving to flash via 0x71),
  // then ReadSettings (0x70) and verify the change is reflected.
  ASSERT_TRUE(OpenAndPrepare());
  const uint8_t original = imu_.GetSettings().filter_select;
  const uint8_t target = static_cast<uint8_t>(original == 2 ? 3 : 2);

  uint8_t data = target;
  ASSERT_TRUE(imu_.SendCommand(0x75, &data, 1));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_TRUE(imu_.ReadSettings());
  EXPECT_EQ(imu_.GetSettings().filter_select, target);

  // Best-effort restore (no flash save, so a reboot would revert anyway).
  data = original;
  imu_.SendCommand(0x75, &data, 1);
}
