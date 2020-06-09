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
#ifndef IMU_ERR_INVALID_DATA
#define IMU_ERR_INVALID_DATA (2)
#endif

class AdisRcvCsv
{
public:
  AdisRcvCsv();

  enum class State {
    Ready,
    Running,
    Unknown
  };

  enum class Mode {
    YPR,
    Register,
    Unknown
  };

  //! File descripter for USB-ISS
  int fd_;
  //! Saved terminal config
  struct termios defaults_;

  // Gyro sensor(x, y, z)
  double gyro_[3];
  // Acceleration sensor(x, y, z)
  double accl_[3];
  // estimated imu pose(Yaw, Pitch, Roll)[deg]
  double ypr_[3];

  double gyro_sensi_;
  double acc_sensi_;
  char ring_buf_[RING_BUF_SIZE];
  int ring_pointer_;

  int UpdateRegMode(void);
  int UpdateYprMode(void);

  int OpenPort(const std::string& device);
  void ClosePort();
  int ReadSerial();
  int WriteSerial(const std::string& cmd);
  
  bool SendCmd(const std::string& cmd);
  std::string SendAndRetCmd(const std::string& cmd);

  int CalNextPointer(const int& src);
  int CalPrePointer(const int& src);
  std::string FindLastData();
  std::string FindCmdReturnRow(const std::string& cmd);
  std::vector<std::string> Split(const std::string& str, const char& delm);
  bool SetSensi(const std::string& sensi_str);
  int MakeCsum(const std::vector<int>& data);
//  void clearBuf();

  State st_;
  Mode md_;
};
