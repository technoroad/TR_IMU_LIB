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
AdisRcvCsv::AdisRcvCsv()
{
  fd_ = -1;
  ring_pointer_ = 0;
  st_ = State::Unknown;
  md_ = Mode::Unknown;
}

/**
 * @brief Open device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int AdisRcvCsv::openPort(const std::string& device)
{
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0)
  {
    perror("openPort");
    return -1;
  }
  if (tcgetattr(fd_, &defaults_) < 0)
  {
    perror("openPort");
    return -1;
  }
  struct termios config;
  cfmakeraw(&config);
  config.c_cc[VMIN] = 0;    // non block
  config.c_cc[VTIME] = 10;  // 1 second read timeout

  if (tcsetattr(fd_, TCSANOW, &config) < 0)
  {
    perror("openPort");
    return -1;
  }
  return 0;
}

/**
 * @brief Close device
 */
void AdisRcvCsv::closePort()
{
  if (tcsetattr(fd_, TCSANOW, &defaults_) < 0)
  {
    perror("closePort");
  }
  close(fd_);
}


int AdisRcvCsv::readSerial()
{
  int rcv_cnt = -1;
  int read_buf_size = 1000;
  char buf[read_buf_size];
  rcv_cnt = read(fd_, buf, read_buf_size);

  // write ring buffer
  for (int i = 0; i < rcv_cnt; i++) 
  {
    ring_pointer_ = ++ring_pointer_ % RING_BUF_SIZE;
    ring_buf_[ring_pointer_] = buf[i];
  }
 #if 0
//  printf("rcv_cnt: %d\n", rcv_cnt);
  ring_buf_[RING_BUF_SIZE-1] = '\0';
  printf("%s\n\n", ring_buf_);
 #endif
  return rcv_cnt;
}

int AdisRcvCsv::writeSerial(const std::string& cmd)
{
  int write_cnt = -1;
  write_cnt = write(fd_, cmd.c_str(), cmd.size());

  if (write_cnt < 0) 
  {
    fprintf(stdout, "Could not write to serial port %d\n", write_cnt);
  }
  return write_cnt;
}

std::string AdisRcvCsv::sendAndRetCmd(const std::string& cmd) 
{
  std::string ret_str = "";
  std::string tmp_str = "";
  std::string err_str = "";

  sendCmd(cmd);
  readSerial(); // store data to ringbuf
//  std::string tmp_str = findLastData();
  tmp_str = findCmdReturnRow(cmd);

  err_str = findCmdReturnRow("ERROR");
  if (err_str != "")
  {
    memset(ring_buf_, '\0', RING_BUF_SIZE); // clear buffer
    return err_str;
  }

  auto splited = split(tmp_str,  ',');

  if (splited.size() >= 2) 
  {
    // is include cmd string?
    if (splited[0].find(cmd) != std::string::npos)
    {
      for (int i = 1; i < splited.size(); i++) 
      {
        ret_str += splited[i] + ",";
      }
      ret_str.erase(ret_str.end()-1); // erase last comma
    }
  }
  else if (splited.size() == 1)
  {
    if (splited[0].find(cmd) != std::string::npos)
    {
      ret_str = splited[0];
    }
  }
  return ret_str;
}

bool AdisRcvCsv::sendCmd(const std::string& cmd) 
{
  auto success = writeSerial(cmd + "\r\n");
  usleep(100000); // 100ms
  return success;
}

std::string AdisRcvCsv::findCmdReturnRow(const std::string& cmd)
{
  int wp = ring_pointer_;
  std::string ret_str = "";

  int ii = 0;
  for (; ii < RING_BUF_SIZE; ii++) 
  {
    if (ring_buf_[wp] == cmd[0]) 
    {
      bool find_flg = true;
      int wp2 = calNextPointer(wp);
      for (int j = 1; j < cmd.size(); j++) 
      {
        if (cmd[j] != ring_buf_[wp2]) 
        {
          find_flg = false;
          break;  // inner for
        }
        wp2 = calNextPointer(wp2);
      }
      if (find_flg)
      {
        break;  // outer for
      }
    }    
    wp = calPrePointer(wp);
  }
  if (ii == RING_BUF_SIZE) 
  {
    if (cmd != "ERROR") 
    {
      printf("Can not find cmd from ring buffer\n");
    }
    return ret_str;
  }

  for (int i = 0; i < RING_BUF_SIZE; i++) 
  {
    ret_str += ring_buf_[wp];
    wp = calNextPointer(wp);
    if (ring_buf_[wp] == '\r')
    {
      break;
    }
  }
  return ret_str;
}

std::string AdisRcvCsv::findLastData() 
{
  int wp = ring_pointer_;
  int n_cnt = 0;
  int last_index = -1;
  int pre_last_index = -1;
  // 最後から探査して\r\nを2つ探す
  for (int i = 0; i < RING_BUF_SIZE; i++) 
  {
    if (ring_buf_[wp] == '\r' 
      && ring_buf_[calNextPointer(wp)] == '\n') 
    {
      n_cnt++;
      if (n_cnt == 2) 
      {
        pre_last_index = wp;
        break;
      }
      else
      {
        last_index = wp;
      }
    }
    wp = calPrePointer(wp);
  }

  // 最後と次の間の文字を取得する  
  // \r\nの\rを飛ばす+1
  int index = calNextPointer(pre_last_index+1);

  std::string ret_string;
  for (int i = 0; i < RING_BUF_SIZE; i++) 
  {
    if (index == last_index) 
    {
      break;
    }
    else 
    {
      ret_string += ring_buf_[index];
    }
    index = ++index % RING_BUF_SIZE;
  }
  return ret_string;
}


/**
 * @brief update gyro and accel in high-precision read
 */
int AdisRcvCsv::updateRegMode(void)
{
  if (readSerial() <= 0) 
  {
    printf("Can not read data\n");
    return IMU_ERR_CANT_RCV_DATA;
  }

  std::string row = findLastData();  
  if (row.size() == 0) 
  {
    printf("Can not find packet\n");
    return IMU_ERR_INVALID_DATA;
  }

  auto splited_data = split(row, ',');
  if (splited_data.size() != 7) 
  {
    printf("Invalid data length\n");
    return IMU_ERR_INVALID_DATA;
  }

  std::vector<int> num_data(6, 0);
  int csum = 300;
  try
  {
    for (int i = 0; i < num_data.size(); i++) 
    {
      num_data[i] = (int)std::stol(splited_data[i], nullptr, 16);
    }
    csum = (int)std::stol(splited_data.back(), nullptr, 16);
  } 
  catch (std::invalid_argument e) 
  {
    printf("%s\n", e.what());
  }

  if (makeCsum(num_data) != csum) 
  {
    printf("Invalid checksum!\n");
    return 1;
  }

  for (int i = 0; i < 3; i++)
  {
    gyro_[i] = (double)num_data[i]   * DEG2RAD / gyro_sensi_;
    accl_[i] = (double)num_data[i+3] * GRAVITY / acc_sensi_;
  } 
  return IMU_OK;
}

/**
 * @brief update YawPitchRoall in high-precision read
 */
int AdisRcvCsv::updateYprMode(void)
{
  if (readSerial() <= 0) 
  {
    printf("Can not read data\n");
    return IMU_ERR_CANT_RCV_DATA;
  }

  std::string row = findLastData();
  if (row.size() == 0) 
  {
    printf("Can not find packet\n");
    return IMU_ERR_INVALID_DATA;
  }

  auto splited_data = split(row, ',');
  if (splited_data.size() != 3) 
  {
    printf("Invalid data length\n");
    return IMU_ERR_INVALID_DATA;
  }

  try
  {
    for (int i = 0; i < 3; i++) 
    {
      ypr_[i] = std::stof(splited_data[i]);
    }
  } 
  catch (std::invalid_argument e) 
  {
    printf("%s\n", e.what());
  }
  return IMU_OK;
}

int AdisRcvCsv::makeCsum(const std::vector<int>& array) 
{
  int sum = 0;
  for (int i = 0; i < array.size(); i++)
  {
    sum += (array[i]>>24) & 0xff;
    sum += (array[i]>>16) & 0xff;
    sum += (array[i]>>8)  & 0xff;
    sum += (array[i])     & 0xff;
  }
  return (sum & 0xff); 
}

int AdisRcvCsv::calNextPointer(const int& src) 
{
  return ((src+1) % RING_BUF_SIZE);
}

int AdisRcvCsv::calPrePointer(const int& src) 
{
  return (src+RING_BUF_SIZE-1) % RING_BUF_SIZE;
}

std::vector<std::string> AdisRcvCsv::split(const std::string& str, const char& delm) 
{
  std::vector<std::string> ret;
  std::stringstream stream(str);
  std::string tmp;

  while (getline(stream, tmp, delm)) 
  {
    ret.push_back(tmp);
  }
  return ret;
}

bool AdisRcvCsv::setSensi(const std::string& sensi_str) 
{
  auto splited = split(sensi_str, ',');
  if (splited.size() != 2) 
  {
    return false;
  }
  gyro_sensi_ = std::stod(splited[0]);
  acc_sensi_ = std::stod(splited[1]);
//  printf("%f, %f\n", gyro_sensi_, acc_sensi_);
  return true; 
}
