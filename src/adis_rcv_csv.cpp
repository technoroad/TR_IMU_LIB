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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <stdexcept>
#include <sstream>

#include "adis_rcv_csv.h"

  
/**
 * @brief Constructor
 */
AdisRcvCsv::AdisRcvCsv() {
  fd_ = -1;
  ring_pointer_ = 0;
  st_ = State::INITIAL;
  md_ = Mode::INITIAL;
  format_str_ = "";
  prod_id_ = "";
}

/**
 * @brief Close device
 */
void AdisRcvCsv::Close() {
  if (tcsetattr(fd_, TCSANOW, &defaults_) < 0) {
    perror("closePort");
  }
  close(fd_);
}

int AdisRcvCsv::ReadSerial() {
  int rcv_cnt = -1;
  const int read_buf_size = 1000;
  char buf[read_buf_size];
  rcv_cnt = read(fd_, buf, read_buf_size);

  // write ring buffer
  for (int i = 0; i < rcv_cnt; i++) {
    ring_pointer_ = (ring_pointer_+1) % RING_BUF_SIZE;
    ring_buf_[ring_pointer_] = buf[i];
  }
 #if 0
//  printf("rcv_cnt: %d\n", rcv_cnt);
  ring_buf_[RING_BUF_SIZE-1] = '\0';
  printf("%s\n\n", ring_buf_);
 #endif
  return rcv_cnt;
}

int AdisRcvCsv::WriteSerial(const std::string& cmd) {
  int write_cnt = -1;
  write_cnt = write(fd_, cmd.c_str(), cmd.size());

  if (write_cnt < 0) {
    fprintf(stdout, "Could not write to serial port %d\n", write_cnt);
  }
  return write_cnt;
}

std::string AdisRcvCsv::SendAndRetCmd(const std::string& cmd, const std::string& args = "", const bool& is_print = true) {
  std::string ret_str = "";
  std::string tmp_str = "";
  std::string err_str = "";

  SendCmd(cmd + args);
  ReadSerial(); // store data to ringbuf

  tmp_str = FindCmdReturnRow(cmd);

  err_str = FindCmdReturnRow("ERROR");
  if (err_str != "") {
    memset(ring_buf_, '\0', RING_BUF_SIZE); // clear buffer
    return err_str;
  }

  auto splited = Split(tmp_str,  ',');

  if (splited.size() >= 2) {
    // is include cmd string?
    if (splited[0].find(cmd) != std::string::npos) {
      for (size_t i = 1; i < splited.size(); i++) {
        ret_str += splited[i] + ",";
      }
      ret_str.erase(ret_str.end()-1); // erase last comma
    }
  } else if (splited.size() == 1) {
    if (splited[0].find(cmd) != std::string::npos) {
      ret_str = splited[0];
    }
  }

  if (is_print)  {
    printf("%s = %s\n", cmd.c_str(), ret_str.c_str());
  }

  return ret_str;
}

bool AdisRcvCsv::SendCmd(const std::string& cmd) {
  auto success = WriteSerial(cmd + "\r\n");
  usleep(100000); // 100ms
  return success;
}

std::string AdisRcvCsv::FindCmdReturnRow(const std::string& cmd) {
  int wp = ring_pointer_;
  std::string ret_str = "";

  int ii = 0;
  for (; ii < RING_BUF_SIZE; ii++) {
    if (ring_buf_[wp] == cmd[0]) {
      bool find_flg = true;
      int wp2 = CalNextPointer(wp);
      for (size_t j = 1; j < cmd.size(); j++) {
        if (cmd[j] != ring_buf_[wp2]) {
          find_flg = false;
          break;  // inner for
        }
        wp2 = CalNextPointer(wp2);
      }
      if (find_flg) {
        break;  // outer for
      }
    }
    wp = CalPrePointer(wp);
  }

  if (ii == RING_BUF_SIZE) {
    if (cmd != "ERROR") {
      printf("Can not find cmd from ring buffer\n");
    }
    return ret_str;
  }

  for (int i = 0; i < RING_BUF_SIZE; i++) {
    ret_str += ring_buf_[wp];
    wp = CalNextPointer(wp);
    if (ring_buf_[wp] == '\r') {
      break;
    }
  }
  return ret_str;
}

std::string AdisRcvCsv::FindLastData() {
  int wp = ring_pointer_;
  int n_cnt = 0;
  int last_index = -1;
  int pre_last_index = -1;
  // 最後から探査して\r\nを2つ探す
  for (int i = 0; i < RING_BUF_SIZE; i++) {
    if (ring_buf_[wp] == '\r' 
      && ring_buf_[CalNextPointer(wp)] == '\n') {
      n_cnt++;
      if (n_cnt == 2) {
        pre_last_index = wp;
        break;
      } else {
        last_index = wp;
      }
    }
    wp = CalPrePointer(wp);
  }

  // 最後と次の間の文字を取得する  
  // \r\nの\rを飛ばす+1
  int index = CalNextPointer(pre_last_index+1);

  std::string ret_string;
  for (int i = 0; i < RING_BUF_SIZE; i++) {
    if (index == last_index) {
      break;
    } else {
      ret_string += ring_buf_[index];
    }
    index = (index+1) % RING_BUF_SIZE;
//    index = ++index % RING_BUF_SIZE;
  }
  return ret_string;
}


/**
 * @brief update gyro and accel in high-precision read
 */
int AdisRcvCsv::UpdateRegMode() {
  if (ReadSerial() <= 0) {
    printf("Can not read data\n");
    return IMU_ERR_CANT_RCV_DATA;
  }

  std::string row = FindLastData();
  if (row.size() == 0) {
    printf("Can not find packet\n");
    return IMU_ERR_INVALID_DATA;
  }

  auto splited_data = Split(row, ',');
  if (splited_data.size() != 7) {
    printf("Invalid data length\n");
    return IMU_ERR_INVALID_DATA;
  }

  std::vector<int> num_data(6, 0);
  int csum = 300;
  try {
    for (size_t i = 0; i < num_data.size(); i++) {
      num_data[i] = (int)std::stol(splited_data[i], nullptr, 16);
    }
    csum = (int)std::stol(splited_data.back(), nullptr, 16);
  } catch (std::invalid_argument e) {
    printf("%s\n", e.what());
  }

  if (MakeCsum(num_data) != csum) {
    printf("Invalid checksum!\n");
    return 1;
  }

  for (int i = 0; i < 3; i++) {
    gyro_[i] = (double)num_data[i]   * DEG2RAD / gyro_sensi_;
    accl_[i] = (double)num_data[i+3] * GRAVITY / acc_sensi_;
  } 
  return IMU_OK;
}

/**
 * @brief update YawPitchRoall in high-precision read
 */
int AdisRcvCsv::UpdateYprMode() {
  if (ReadSerial() <= 0) {
    printf("Can not read data\n");
    return IMU_ERR_CANT_RCV_DATA;
  }

  std::string row = FindLastData();
  if (row.size() == 0) {
    printf("Can not find packet\n");
    return IMU_ERR_INVALID_DATA;
  }

  auto splited_data = Split(row, ',');
  if (splited_data.size() != 3) {
    printf("Invalid data length\n");
    return IMU_ERR_INVALID_DATA;
  }

  try {
    for (int i = 0; i < 3; i++) {
      ypr_[i] = std::stof(splited_data[i]);
    }
  } catch (std::invalid_argument e) {
    printf("%s\n", e.what());
  }
  return IMU_OK;
}

int AdisRcvCsv::MakeCsum(const std::vector<int>& array) {
  int sum = 0;
  for (size_t i = 0; i < array.size(); i++) {
    sum += (array[i]>>24) & 0xff;
    sum += (array[i]>>16) & 0xff;
    sum += (array[i]>>8)  & 0xff;
    sum += (array[i])     & 0xff;
  }
  return (sum & 0xff); 
}

int AdisRcvCsv::CalNextPointer(const int& src) {
  return ((src+1) % RING_BUF_SIZE);
}

int AdisRcvCsv::CalPrePointer(const int& src) {
  return (src+RING_BUF_SIZE-1) % RING_BUF_SIZE;
}

std::vector<std::string> AdisRcvCsv::Split(const std::string& str, const char& delm) {
  std::vector<std::string> ret;
  std::stringstream stream(str);
  std::string tmp;

  while (getline(stream, tmp, delm)) {
    ret.push_back(tmp);
  }
  return ret;
}

bool AdisRcvCsv::SetSensi(const std::string& sensi_str) {
  auto splited = Split(sensi_str, ',');
  if (splited.size() != 2) {
    return false;
  }
  gyro_sensi_ = std::stod(splited[0]);
  acc_sensi_ = std::stod(splited[1]);
//  printf("%f, %f\n", gyro_sensi_, acc_sensi_);
  return true; 
}

void AdisRcvCsv::SetMode(const Mode& m) {
  md_ = m;

  if (md_ == Mode::ATTIUDE) {
    format_str_ = "YAW[deg],PITCH[deg],ROLL[deg]";
  } else if (md_ == Mode::REGISTER) {
    format_str_ = "X_GYRO_HEX,Y_GYRO_HEX,Z_GYRO_HEX,X_ACC_HEX,Y_ACC_HEX,Z_ACC_HEX,CSUM";
  }
}

AdisRcvCsv::Mode AdisRcvCsv::GetMode() {
  return md_;
}

void AdisRcvCsv::SetState(const State& s) {
  st_ = s;
}

AdisRcvCsv::State AdisRcvCsv::GetState() {
  return st_;
}

bool AdisRcvCsv::Prepare() {
  PrintFirmVersion();
  GetProductId();

  if (!SetFormat()) return false;

  // check imu state
  if (!CheckStatus()) return false;

  
  SetState(State::READY);

  // check data format
  if (!CheckFormat()) return false;

  // check sensitivity of gyro and acc
  if (GetMode() == AdisRcvCsv::Mode::REGISTER) {
    if (!CheckSensitivity()) {
      return false;
    }
  }

  auto ret_str = SendAndRetCmd("start", /* args */"", /* is_print */false);
  if (ret_str != "start") {
    printf("Send start cmd. But imu was not started.\n");
//    ROS_WARN("Send start cmd. But imu was not started.");
    return false;
  }

  printf("Start imu!\n");
  SetState(AdisRcvCsv::State::RUNNING);

  return true;
}

/**
 * @brief Open IMU device file
 */
bool AdisRcvCsv::Open(const std::string& device) {
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    perror("openPort");
    return false;
  }

  if (tcgetattr(fd_, &defaults_) < 0) {
    perror("openPort");
    return false;
  }

  struct termios config;
  cfmakeraw(&config);
  config.c_cc[VMIN] = 0;    // non block
  config.c_cc[VTIME] = 10;  // 1 second read timeout

  if (tcsetattr(fd_, TCSANOW, &config) < 0) {
    perror("openPort");
    return false;
  }
  return true;
}

/**
 * @brief Check if the device is opened
 */
bool AdisRcvCsv::IsOpened() {
  return (fd_ >= 0);
}

bool AdisRcvCsv::SetFormat() {
  std::string cmd = "SET_FORMAT";
  std::string args = "";
  if (GetMode() == AdisRcvCsv::Mode::ATTIUDE) {
    args += ",1";
  } else if (GetMode() == AdisRcvCsv::Mode::REGISTER) {
    args += ",3";
  }
  auto ret_str = SendAndRetCmd(cmd, args);
    
  if (ret_str != format_str_) {
    //ROS_ERROR("SET_FORMAT is failed.");
    printf("SET_FORMAT is failed.\n");
    return false;
  }
  return true;
}

bool AdisRcvCsv::CheckFormat() {
  auto ret_str = SendAndRetCmd("GET_FORMAT"); 

  if (ret_str != format_str_) {
//    ROS_ERROR("Ivalid format.");
    printf("Ivalid format.\n");
    return false;
  }
  return true;
}

void AdisRcvCsv::GetProductId() {
  prod_id_ = SendAndRetCmd("GET_PROD_ID");

  if (prod_id_ == "ADIS16470") {
    pd_ = Product::ADIS16470;
  } else if (prod_id_ == "ADIS16500") {
    pd_ = Product::ADIS16500;
  } else if (prod_id_ == "ADIS16505-2") {
    pd_ = Product::ADIS16505_2;
  } else {
    printf("Unknown product id\n");
  }
}

std::string AdisRcvCsv::GetProductIdStr() {
  return prod_id_;
}

void AdisRcvCsv::PrintFirmVersion() {
  auto ret_str = SendAndRetCmd("GET_VERSION"); 
}

bool AdisRcvCsv::CheckStatus() {
  auto ret_str = SendAndRetCmd("GET_STATUS"); 
  if (ret_str == "Running") {
    printf("Imu state is Running. Send stop command.\n");
//    ROS_WARN("Imu state is Running. Send stop command.");
    SendCmd("stop");
    return false;

  } else if (ret_str != "Ready") {
    printf("Invalid imu state.\n");
//    ROS_WARN("Invalid imu state.");
    return false;
  }
  return true;
}

bool AdisRcvCsv::CheckSensitivity() {
  std::string ret_str = "";
  ret_str = SendAndRetCmd("GET_SENSI"); 
  if (ret_str == "") {
    printf("Could not get sensitivities!\n");
//    ROS_WARN("Could not get sensitivities!");
    return false;

  } else {
    if (!SetSensi(ret_str)) {
      printf("Insufficient number of sensitivities.\n");
//    ROS_WARN("Insufficient number of sensitivities.");
    return false;
    }
  }
  return true;
}

void AdisRcvCsv::Stop() {
  SetState(State::READY);
  SendCmd("stop"); 
}

void AdisRcvCsv::GetYPR(double ret[]) {
  ret[0] = ypr_[0];
  ret[1] = ypr_[1];
  ret[2] = ypr_[2];
}

void AdisRcvCsv::GetAcc(double ret[]) {
  ret[0] = accl_[0];
  ret[1] = accl_[1];
  ret[2] = accl_[2];
  
  // convert unit m/s^2 to g
  if (pd_ == Product::ADIS16500 || pd_ == Product::ADIS16505_2) {
    ret[0] /= GRAVITY;
    ret[1] /= GRAVITY;
    ret[2] /= GRAVITY;
  }
}

void AdisRcvCsv::GetGyro(double ret[]) {
  ret[0] = gyro_[0];
  ret[1] = gyro_[1];
  ret[2] = gyro_[2];
}
