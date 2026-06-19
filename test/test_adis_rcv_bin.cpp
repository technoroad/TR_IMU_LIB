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

// 空配列 (len=0) のチェックサム計算が 0xFFFF (NOT 0) を返すこと
TEST_F(AdisRcvBinTest, Rfc1071CalcEmpty)
{
  uint8_t d = 0x00;
  EXPECT_EQ(Calc(&d, 0), 0xFFFF);
}

// 単一バイト 0x00 のチェックサムは 0xFFFF
TEST_F(AdisRcvBinTest, Rfc1071CalcSingleZero)
{
  uint8_t d = 0x00;
  EXPECT_EQ(Calc(&d, 1), 0xFFFF);
}

// 単一バイト 0xFF のチェックサムは 0xFF00 (補数演算の最大値ケース)
TEST_F(AdisRcvBinTest, Rfc1071CalcSingleFF)
{
  uint8_t d = 0xFF;
  EXPECT_EQ(Calc(&d, 1), 0xFF00);
}

// cmd_id + length の典型的な 2 バイト入力に対する正しい計算
TEST_F(AdisRcvBinTest, Rfc1071CalcCmdAndLength)
{
  // Sum = 0x70 + 0x40 = 0xB0, ~0xB0 & 0xFFFF = 0xFF4F.
  uint8_t d[] = {0x70, 0x40};
  EXPECT_EQ(Calc(d, 2), 0xFF4F);
}

// 全 0xFF×4 バイトの計算 (sum が小さく折返し発生しないケース)
TEST_F(AdisRcvBinTest, Rfc1071CalcAllFF4Bytes)
{
  // Sum = 0x3FC, ~0x3FC & 0xFFFF = 0xFC03.
  uint8_t d[] = {0xFF, 0xFF, 0xFF, 0xFF};
  EXPECT_EQ(Calc(d, 4), 0xFC03);
}

// sum が 16bit を超えるとき、上位ビットを下位に畳む処理が正しく動くこと
TEST_F(AdisRcvBinTest, Rfc1071CalcOverflowFold)
{
  // 1000 * 0xFF = 0x3E418. Fold: 0x3 + 0xE418 = 0xE41B. ~ & 0xFFFF = 0x1BE4.
  std::vector<uint8_t> d(1000, 0xFF);
  EXPECT_EQ(Calc(d.data(), d.size()), 0x1BE4);
}

// 正しいチェックサムが付いたデータが verify を通ること
TEST_F(AdisRcvBinTest, Rfc1071VerifyValid)
{
  uint8_t d[] = {0x70, 0x40, 0x4F, 0xFF};
  EXPECT_TRUE(Verify(d, 4));
}

// データを 1 バイト改ざんすると verify が false を返すこと
TEST_F(AdisRcvBinTest, Rfc1071VerifyTampered)
{
  uint8_t d[] = {0x70, 0x41, 0x4F, 0xFF};
  EXPECT_FALSE(Verify(d, 4));
}

// 長さ 2 未満 (csum 自体が入らない) の入力は false を返すガード条件
TEST_F(AdisRcvBinTest, Rfc1071VerifyTooShort)
{
  uint8_t d[] = {0xFF};
  EXPECT_FALSE(Verify(d, 1));
  EXPECT_FALSE(Verify(d, 0));
}

// Calc で算出した csum を付加したデータが Verify を通る (ラウンドトリップ整合)
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

// uint8 は 1 バイトをそのまま返す
TEST_F(AdisRcvBinTest, ReadLEUint8)
{
  uint8_t d[] = {0x42};
  EXPECT_EQ(ReadLE<uint8_t>(d), 0x42);
}

// int16 最大値 (INT16_MAX = 0x7FFF) の境界
TEST_F(AdisRcvBinTest, ReadLEInt16Positive)
{
  uint8_t d[] = {0xFF, 0x7F};
  EXPECT_EQ(ReadLE<int16_t>(d), INT16_MAX);
}

// int16 最小値 (INT16_MIN = 0x8000) の境界、符号反転が正しく扱われる
TEST_F(AdisRcvBinTest, ReadLEInt16Negative)
{
  uint8_t d[] = {0x00, 0x80};
  EXPECT_EQ(ReadLE<int16_t>(d), INT16_MIN);
}

// int16 = -1 (全ビット 1) の解釈
TEST_F(AdisRcvBinTest, ReadLEInt16MinusOne)
{
  uint8_t d[] = {0xFF, 0xFF};
  EXPECT_EQ(ReadLE<int16_t>(d), -1);
}

// uint16 の little-endian バイト並び (下位バイトが先)
TEST_F(AdisRcvBinTest, ReadLEUint16)
{
  uint8_t d[] = {0x34, 0x12};
  EXPECT_EQ(ReadLE<uint16_t>(d), 0x1234u);
}

// int32 最大値 (INT32_MAX) の境界
TEST_F(AdisRcvBinTest, ReadLEInt32Positive)
{
  uint8_t d[] = {0xFF, 0xFF, 0xFF, 0x7F};
  EXPECT_EQ(ReadLE<int32_t>(d), INT32_MAX);
}

// int32 最小値 (INT32_MIN) の境界、符号反転が正しい
TEST_F(AdisRcvBinTest, ReadLEInt32Negative)
{
  uint8_t d[] = {0x00, 0x00, 0x00, 0x80};
  EXPECT_EQ(ReadLE<int32_t>(d), INT32_MIN);
}

// uint32 の little-endian 4 バイト読み出し
TEST_F(AdisRcvBinTest, ReadLEUint32)
{
  uint8_t d[] = {0x78, 0x56, 0x34, 0x12};
  EXPECT_EQ(ReadLE<uint32_t>(d), 0x12345678u);
}

// uint64 の little-endian 8 バイト読み出し
TEST_F(AdisRcvBinTest, ReadLEUint64)
{
  uint8_t d[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
  EXPECT_EQ(ReadLE<uint64_t>(d), 0xEFCDAB8967452301ULL);
}

// ============================================================
// BuildPacket
// ============================================================

// パケット全体のバイト配置検証: header(2) + cmd + len + data + csum(LE)
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

// data_len=0 の場合でもパケットが正しく構築される (csum はヘッダー除く先頭から計算)
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

// BuildPacket が生成したパケットの csum が、その同じパケット自身の Verify を通る
TEST_F(AdisRcvBinTest, BuildPacketSelfChecksum)
{
  std::vector<uint8_t> data(64, 0xAB);
  auto pkt = Build(0x31, data);
  EXPECT_TRUE(Verify(&pkt[2], pkt.size() - 2));
}

// ============================================================
// ParseTelemetryPayload
// ============================================================

// 既知の 64 バイトペイロード → TelemetryData 構造体の全フィールドが
// 仕様通り (オフセット・型・スケーリング) にパースされること
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
  // timestamp = 0x0123456789ABCDEF (uint64 LE)
  d[48] = 0xEF; d[49] = 0xCD; d[50] = 0xAB; d[51] = 0x89;
  d[52] = 0x67; d[53] = 0x45; d[54] = 0x23; d[55] = 0x01;

  ASSERT_TRUE(ParseTelemetry(d, 0x31));

  const auto& t = Telemetry();
  EXPECT_EQ(t.response_id, 0x31);
  EXPECT_EQ(t.mpu_error, 0x05);
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
  EXPECT_EQ(t.timestamp, 0x0123456789ABCDEFULL);
}

// GetTimestamp() アクセサがそのまま telemetry_.timestamp を返すこと
TEST_F(AdisRcvBinTest, GetTimestamp_Passthrough)
{
  Telemetry().timestamp = 0xDEADBEEFCAFE1234ULL;
  EXPECT_EQ(imu_.GetTimestamp(), 0xDEADBEEFCAFE1234ULL);
}

// mpu_error の bit 定義が仕様書 (5.4.1) と一致していること
TEST_F(AdisRcvBinTest, MpuErrorBitConstants)
{
  EXPECT_EQ(kMpuErrValueOutOfRange, 0x01);
  EXPECT_EQ(kMpuErrUnknownCommand,  0x02);
  EXPECT_EQ(kMpuErrFlashWrite,      0x04);
  EXPECT_EQ(kMpuErrWdtReboot,       0x08);
  // bit4: 駆動系停止 + 開発者報告が必要な重大エラー (仕様書 5.4.1)
  EXPECT_EQ(kMpuErrImuNotFound,     0x10);
}

// ============================================================
// ParseSettingsPayload
// ============================================================

// 既知の 64 バイトペイロード → SettingsData 構造体の全フィールド
// (product_id, sensitivity, filter 等) が正しくパースされること
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
  EXPECT_EQ(s.mpu_error, 0x42);
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

// ノイズバイトの後に正規ヘッダが続く場合、ノイズをスキップして
// 正規パケットに再同期できること
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
  EXPECT_EQ(Telemetry().mpu_error, 0xA5);
}

// length フィールドが 64 以外のヘッダはスキップして次のヘッダ候補を探すこと
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
  EXPECT_EQ(Telemetry().mpu_error, 0xBB);
}

// チェックサムエラーのパケットはスキップして次の有効なパケットを採用すること
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
  EXPECT_EQ(Telemetry().mpu_error, 0xCC);
}

// リングバッファの折返し境界をまたぐパケットも正しく抽出できること
TEST_F(AdisRcvBinTest, FindPacketWraparound)
{
  RingWritePos() = kBinRingBufSize - 30;
  RingDataCount() = 0;

  uint8_t payload[64] = {0};
  payload[0] = 0xDD;
  auto pkt = BuildFullPacket(0x31, payload);
  Inject(pkt.data(), pkt.size());

  EXPECT_TRUE(FindAndParse());
  EXPECT_EQ(Telemetry().mpu_error, 0xDD);
}

// 70 バイト未満しか溜まっていない場合は false を返し、パース処理を始めない
TEST_F(AdisRcvBinTest, FindPacketNotEnoughData)
{
  uint8_t pkt[50] = {0xAA, 0xAA};
  Inject(pkt, 50);
  EXPECT_FALSE(FindAndParse());
}

// response_id が 0x70-0x77 の範囲なら Settings として、それ以外は
// Telemetry としてパース先を振り分けること
TEST_F(AdisRcvBinTest, FindPacketRoutesSettingsByResponseId)
{
  uint8_t payload[64] = {0};
  payload[0] = 0x99;
  payload[31] = 0x79; payload[32] = 0x40;
  auto pkt = BuildFullPacket(0x70, payload);
  Inject(pkt.data(), pkt.size());

  EXPECT_TRUE(FindAndParse());
  EXPECT_EQ(Settings().mpu_error, 0x99);
  EXPECT_EQ(Settings().product_id, 16505u);
}

// ============================================================
// Unit conversion
// ============================================================

// accl_sensitivity = 0 のときはゼロ除算せず 0 を返す (ガード条件)
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

// sensitivity と raw 値から正しい m/s² (重力加速度倍) が計算されること
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

// gyro_sensitivity = 0 のときはゼロ除算せず 0 を返す (ガード条件)
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

// sensitivity と raw 値から正しい rad/s (deg→rad 換算含む) が計算されること
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

// 内部 telemetry_.quat[] の値がそのまま外部に返ること (パススルー)
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

// 温度は raw 値を 10 で割って摂氏に変換 (0.1℃ 単位 → ℃)
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

// model = 0x03 → "ADIS{pid}-1" サフィックス
TEST_F(AdisRcvBinTest, GetProductIdStr_Model03Suffix1)
{
  Settings().product_id = 16505;
  Settings().model = 0x03;
  EXPECT_EQ(imu_.GetProductIdStr(), "ADIS16505-1");
}

// model = 0x07 → "ADIS{pid}-2" サフィックス
TEST_F(AdisRcvBinTest, GetProductIdStr_Model07Suffix2)
{
  Settings().product_id = 16505;
  Settings().model = 0x07;
  EXPECT_EQ(imu_.GetProductIdStr(), "ADIS16505-2");
}

// model = 0x0F → "ADIS{pid}-3" サフィックス
TEST_F(AdisRcvBinTest, GetProductIdStr_Model0FSuffix3)
{
  Settings().product_id = 16505;
  Settings().model = 0x0F;
  EXPECT_EQ(imu_.GetProductIdStr(), "ADIS16505-3");
}

// 未知の model 値の場合はサフィックスなし "ADIS{pid}" を返す
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

  // Average the at-rest accelerometer magnitude (m/s^2) over up to 30 packets.
  // Manages telemetry start/stop internally; the device must be open, READY
  // and physically stationary. Returns -1.0 if too few packets arrived.
  double MeasureAccMagnitude()
  {
    if (!imu_.StartTelemetry()) return -1.0;
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
    imu_.StopTelemetry();
    if (n < 10) return -1.0;
    const double ax = sx / n, ay = sy / n, az = sz / n;
    return std::sqrt(ax * ax + ay * ay + az * az);
  }

  // Reboot the MCU (0xB0) and reopen the device once it re-enumerates on the
  // USB bus, leaving settings_ holding a stable post-reboot read. Does NOT
  // save first, so any unsaved settings changes revert. Returns false (with a
  // gtest failure) if the device does not come back.
  bool RebootReconnect()
  {
    // 0xB0 reboot: Length 1, Data 0x00. Fire-and-forget — the MCU resets
    // immediately and returns no response; the serial link drops. The reboot
    // takes ~3 s and periodic telemetry does NOT auto-resume.
    uint8_t reboot = 0x00;
    imu_.SendCommand(0xB0, &reboot, 1);
    imu_.Close();

    // Wait for USB re-enumeration, then reopen and confirm communication.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (!imu_.Open(device_)) continue;
      imu_.StopTelemetry();
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      if (!imu_.ReadSettings()) { imu_.Close(); continue; }
      // The first settings packet right after re-enumeration can be stale;
      // settle, then re-read so settings_ holds a trustworthy value.
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (imu_.ReadSettings()) return true;
      imu_.Close();
    }
    ADD_FAILURE() << "Device did not re-enumerate within 20s after reboot";
    return false;
  }

  // Persist current settings (0x71) with the spec-defined permission keyword,
  // then reboot + reconnect via RebootReconnect(). Required to make settings
  // commands 0x73-0x77 survive a reboot (platform2 spec §6.2.2, §6.4, §7.2.1).
  bool SaveRebootReconnect()
  {
    // 0x71 save: Length 2, Data = permission keyword {0x12, 0x34}. A wrong
    // keyword sets mpu_error bit0 and the save is silently skipped.
    const uint8_t save_key[2] = {0x12, 0x34};
    if (!imu_.SendCommand(0x71, save_key, sizeof(save_key))) {
      ADD_FAILURE() << "Save (0x71) command failed";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // Confirm the save was accepted (a bad key would set mpu_error bit0).
    if (imu_.ReadSettings() &&
        (imu_.GetSettings().mpu_error & kMpuErrValueOutOfRange)) {
      ADD_FAILURE() << "Save (0x71) rejected: mpu_error bit0 set (bad key?)";
      return false;
    }
    return RebootReconnect();
  }
};

// シリアル port の Open/Close と state 遷移 (INITIAL → READY → INITIAL)
TEST_F(AdisRcvBinHwTest, Hw_OpenClose)
{
  EXPECT_TRUE(imu_.Open(device_));
  EXPECT_EQ(imu_.GetState(), AdisRcvBin::State::READY);
  imu_.Close();
  EXPECT_EQ(imu_.GetState(), AdisRcvBin::State::INITIAL);
}

// 存在しないデバイスパスで Open すると false、state は INITIAL のまま
TEST_F(AdisRcvBinHwTest, Hw_OpenInvalidDevice)
{
  AdisRcvBin local;
  EXPECT_FALSE(local.Open("/dev/adis-bin-does-not-exist"));
  EXPECT_EQ(local.GetState(), AdisRcvBin::State::INITIAL);
}

// 0x70 (ReadSettings) で取得した product_id と sensitivity が妥当な値であること
TEST_F(AdisRcvBinHwTest, Hw_ReadSettings)
{
  ASSERT_TRUE(OpenAndPrepare());
  const auto& s = imu_.GetSettings();
  EXPECT_NE(s.product_id, 0);
  EXPECT_GT(s.accl_sensitivity, 0u);
  EXPECT_GT(s.gyro_sensitivity, 0u);
  EXPECT_GT(s.sample_rate, 0);
}

// 0x30 (NOP) コマンド送信成功 — 通信の死活確認
TEST_F(AdisRcvBinHwTest, Hw_NopCommand)
{
  ASSERT_TRUE(imu_.Open(device_));
  uint8_t data = 0;
  EXPECT_TRUE(imu_.SendCommand(0x30, &data, 1));
}

// 0x31 でテレメトリ開始 → state=RUNNING、データ到達確認、
// 0x32 で停止 → state=READY
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

// 100Hz 想定で 1 秒間に 50 パケット以上、ハードエラー率 < 1% であること
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

// 取得したクォータニオンのノルムが 1.0 ± 0.05 の範囲内 (正規化されている)
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

// 静置時、加速度ベクトルの大きさが ≈ 9.8 ± 0.5 m/s² (重力のみ検出)
TEST_F(AdisRcvBinHwTest, Hw_AtRest_GravityMagnitude)
{
  ASSERT_TRUE(OpenAndPrepare());
  const double mag = MeasureAccMagnitude();
  ASSERT_GE(mag, 0.0) << "Too few packets to compute average";
  EXPECT_NEAR(mag, kGravity, 0.5) << "Gravity magnitude: " << mag << " m/s^2";
}

// 静置時、ジャイロベクトルの大きさが < 0.05 rad/s (回転していない)
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

// 0x33 (姿勢リセット) コマンドが送信成功
TEST_F(AdisRcvBinHwTest, Hw_ResetAttitude)
{
  ASSERT_TRUE(OpenAndPrepare());
  EXPECT_TRUE(imu_.ResetAttitude());
}

// 10 秒間連続取得して、パケット数が十分でハードエラー率 < 1% であること
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

// 0x75 (filter 設定) でフィルタ値を変更し、0x70 (ReadSettings) 再取得で
// 変更が反映されていることを確認 (flash 保存はせず一時的)
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

// 0x76 (重力補正) の永続化仕様を実機で検証する。
// 重力補正 grav_corr_en は姿勢推定の 3軸(gyroのみ)/6軸 切替で (仕様書 §6.4.12)、
// 加速度の出力には現れない。よって判定は設定値のリードバックで行う。
// 仕様: 0x76 で設定値は即 RAM に入るが、不揮発化には 0x71(save) が必要で、
//       反映には再起動 (0xB0) が必要。save せずに再起動すると元へ戻る。
// 流れ:
//   1. 元の grav_corr_en を記録 (最後に必ず復元する)
//   2. 0x76 で値を反転 (save しない) → RAM には即反映されることを確認
//   3. save せずに reboot → 元の値へ戻る (未保存は不揮発化されない)
//   4. 0x76 で反転 + 0x71 save + reboot → 反転値が維持される
//   5. 元の値に戻して save + reboot
// 注意: flash へ書き込み、デバイスを 3 回リブートする破壊的テスト。
TEST_F(AdisRcvBinHwTest, Hw_GravityCorrectionPersistsOnlyWhenSaved)
{
  ASSERT_TRUE(OpenAndPrepare());

  // Step 1: record the current setting so we can restore it at the end.
  const uint8_t original = imu_.GetSettings().grav_corr_en;
  const uint8_t target = static_cast<uint8_t>(original ? 0 : 1);

  // Step 2: toggle WITHOUT saving — the change lands in RAM immediately.
  uint8_t data = target;
  ASSERT_TRUE(imu_.SendCommand(0x76, &data, 1));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_TRUE(imu_.ReadSettings());
  EXPECT_EQ(imu_.GetSettings().grav_corr_en, target)
      << "0x76 not reflected in settings RAM";

  // Step 3: reboot WITHOUT saving — the unsaved change must revert.
  ASSERT_TRUE(RebootReconnect());
  EXPECT_EQ(imu_.GetSettings().grav_corr_en, original)
      << "Unsaved 0x76 change persisted across reboot (should have reverted)";

  // Step 4: toggle, save (0x71), reboot — now the change must persist.
  data = target;
  ASSERT_TRUE(imu_.SendCommand(0x76, &data, 1));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_TRUE(SaveRebootReconnect());
  EXPECT_EQ(imu_.GetSettings().grav_corr_en, target)
      << "Saved 0x76 change did not persist across reboot";

  // Step 5: restore the original setting (save + reboot). Use EXPECT above so
  // we always reach this restore even if an assertion failed.
  data = original;
  ASSERT_TRUE(imu_.SendCommand(0x76, &data, 1));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_TRUE(SaveRebootReconnect());
  EXPECT_EQ(imu_.GetSettings().grav_corr_en, original)
      << "Failed to restore original gravity correction setting";
}
