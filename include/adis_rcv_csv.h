// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <termios.h>
#include <string>
#include <vector>

#ifndef RING_BUF_SIZE
#define RING_BUF_SIZE (2000)
#endif

#ifndef GRAVITY
#define GRAVITY (9.80655)
#endif

#ifndef DEG2RAD
#define DEG2RAD (0.01745329251)
#endif

#ifndef RAD2DEG
#define RAD2DEG (57.2957795131)
#endif

#ifndef IMU_OK
#define IMU_OK (0)
#endif
#ifndef IMU_ERR_CANT_RCV_DATA
#define IMU_ERR_CANT_RCV_DATA (1)
#endif
#ifndef IMU_ERR_COULDNOT_FIND_PACKET
#define IMU_ERR_COULDNOT_FIND_PACKET (2)
#endif
#ifndef IMU_ERR_INVALID_DATA
#define IMU_ERR_INVALID_DATA (3)
#endif
#ifndef IMU_ERR_CHECKSUM
#define IMU_ERR_CHECKSUM (4)
#endif

class AdisRcvCsv {
public:
  enum class State {
    READY,
    RUNNING,
    INITIAL
  };

  enum class Mode {
    ATTIUDE,
    REGISTER,
    INITIAL
  };

  enum class Product {
    ADIS16470,
    ADIS16500,
    ADIS16505_2
  };

  AdisRcvCsv();

  int UpdateRegMode();
  int UpdateYprMode();

  bool Prepare();
  bool Open(const std::string& device);

  void Close();
  void SetMode(const Mode& m);
  void SetState(const State& m);
  void Stop();
  void GetYPR(double ret[]);
  void GetAcc(double ret[]);
  void GetGyro(double ret[]);
  
  Mode GetMode();
  State GetState();
  std::string GetProductIdStr();
  std::string SendAndRetCmd(const std::string& cmd, const std::string& args = "", const bool& is_print = true);

private:
  State st_;
  Mode md_;
  Product pd_;

  //! File descripter for USB-ISS
  int fd_;
  bool SetSensi(const std::string& sensi_str);
  // Gyro sensor(x, y, z)
  double gyro_[3];
  // Acceleration sensor(x, y, z)
  double accl_[3];
  // estimated imu pose(Yaw, Pitch, Roll)[deg]
  double ypr_[3];

  //! Saved terminal config
  struct termios defaults_;

  double gyro_sensi_;
  double acc_sensi_;
  char ring_buf_[RING_BUF_SIZE];
  int ring_pointer_;

  std::string format_str_;
  std::string prod_id_;

  int ReadSerial();
  int WriteSerial(const std::string& cmd);
  int CalNextPointer(const int& src);
  int CalPrePointer(const int& src);
  int MakeCsum(const std::vector<int>& data);

  void PrintFirmVersion();
  void GetProductId();
  void ClearRingBuf();

  bool IsOpened();
  bool Prepared();
  bool SendCmd(const std::string& cmd);
  bool SetFormat();
  bool CheckFormat();
  bool CheckStatus();
  bool CheckSensitivity();

  std::string FindLastData();
  std::string GetHelpCmdReturn();
  std::string FindCmdReturnRow(const std::string& cmd);
  std::vector<std::string> Split(const std::string& str, const char& delm);
};
