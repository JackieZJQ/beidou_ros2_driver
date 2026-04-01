/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>
#include <set>
#include <iomanip>

#include "parser/parser.h"
#include "parser/novatel_messages.h"

namespace beidou {
namespace { 

//缓存字节数
constexpr size_t BUFFER_SIZE = 256;

/*--------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i) {
  int j;
  unsigned long ulCRC;
  ulCRC = i;
  for (j = 8; j > 0; j--) {
    if (ulCRC & 1)
      ulCRC = (ulCRC >> 1) ^ 0xEDB88320L;
    else
      ulCRC >>= 1;
  }

  return ulCRC;
}

/*--------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
ulCount- Number of bytes in the data block
ucBuffer- Data block
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer) {
  unsigned long ulTemp1;
  unsigned long ulTemp2;
  unsigned long ulCRC = 0;
  while (ulCount-- != 0) {
    ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
    ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xFF);
    ulCRC = ulTemp1 ^ ulTemp2;
  }

  return (ulCRC);
}

} // namespace

class NovatelParser : public Parser {
public:
  NovatelParser();
  virtual void GetMessages(MessageInfoVec* messages) override;
  virtual void GetMessages(MessageInfo* message) override;
  virtual MessageType GetMessage(MessagePtr* message_ptr) override;

private:
  bool check_crc();

  MessageType PrepareMessage(MessagePtr* message_ptr);

  //处理数据
  bool HandleCorrImuData(const novatel::CorrImuData* imu);
  bool HandleInsPva(const novatel::InsPva* pva);

  std::vector<uint8_t> buffer_;
  size_t header_length_ = 0;
  size_t total_length_ = 0;

  novatel::CorrImuData imu_;
  novatel::InsPva pva_;
};

Parser* Parser::CreateNovatel() {
  return new NovatelParser();
}

NovatelParser::NovatelParser() {
  buffer_.reserve(BUFFER_SIZE);
}

void NovatelParser::GetMessages(MessageInfoVec* messages) {
  std::set<MessageType> s;
  while (true) {
    MessageInfo message_info;
    message_info.type = GetMessage(&message_info.message_ptr); //处理[data_, data_end_)
    if (message_info.type == MessageType::NONE) {
      break;
    }

    //如果set中没找到对应种类，则push进messages(vector)中
    //在一组[data_, data_end_)中，每种类型只push一次？其余丢弃？
    if (s.find(message_info.type) == s.end()) {
      messages->push_back(std::move(message_info));
      s.insert(message_info.type);
    }
  }
}

void NovatelParser::GetMessages(MessageInfo* message) {
  message->type = GetMessage(&message->message_ptr);
}

MessageType NovatelParser::GetMessage(MessagePtr* message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  while (data_ < data_end_) {         //data_为第一个字节，data_end_-1为最后一个字节
    if (buffer_.empty()) {            //Looking for SYNC0
      if (*data_ == novatel::SYNC_0) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) { //Looking for SYNC1
      if (*data_ == novatel::SYNC_1) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 2) { //Looking for SYNC2
      switch (*data_) {
        case novatel::SYNC_2_LONG_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::LongHeader);
          break;
        case novatel::SYNC_2_SHORT_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::ShortHeader);
          break;
        default:
          buffer_.clear();
      }
    } else if (header_length_ > 0) {  //Working on header.
      if (buffer_.size() < header_length_) {
        buffer_.push_back(*data_++);
      } else {
        if (header_length_ == sizeof(novatel::LongHeader)) {
          total_length_ = header_length_ + novatel::CRC_LENGTH +
                          reinterpret_cast<novatel::LongHeader *>(buffer_.data())->message_length;
        } else if (header_length_ == sizeof(novatel::ShortHeader)) {
          total_length_ = header_length_ + novatel::CRC_LENGTH +
                          reinterpret_cast<novatel::ShortHeader *>(buffer_.data())->message_length;
        } else {
          std::cout << "incorrect header_length_. Should never reach here.\n";
          buffer_.clear();
        }

        header_length_ = 0;
      }
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) { //Working on body.
        buffer_.push_back(*data_++);
        continue;
      }

      MessageType type = PrepareMessage(message_ptr);
      buffer_.clear();
      total_length_ = 0;
      if (type != MessageType::NONE) {
        return type;
      }
    }
  }

  return MessageType::NONE;
}

MessageType NovatelParser::PrepareMessage(MessagePtr* message_ptr) {
  if (!check_crc()) {
    std::cout << "crc check failed.\n";
    return MessageType::NONE;
  }

  uint8_t* message = nullptr;
  novatel::MessageId message_id;
  uint16_t message_length;
  uint16_t gps_week;
  uint32_t gps_millisecs;
  if (buffer_[2] == novatel::SYNC_2_LONG_HEADER) {
    auto header = reinterpret_cast<const novatel::LongHeader*>(buffer_.data());
    message = buffer_.data() + sizeof(novatel::LongHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
  } else {
    auto header = reinterpret_cast<const novatel::ShortHeader*>(buffer_.data());
    message = buffer_.data() + sizeof(novatel::ShortHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
  }

  switch (message_id) {
    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
    case novatel::IMURATECORRIMUS:
      if (message_length != sizeof(novatel::CorrImuData)) {
        std::cout << "incorrect message_length\n";
        break;
      }
      if (HandleCorrImuData(reinterpret_cast<novatel::CorrImuData *>(message))) {
        *message_ptr = &imu_;
        return MessageType::IMU;
      }
      break;
    case novatel::INSPVA:
    case novatel::INSPVAS:
      if (message_length != sizeof(novatel::InsPva)) {
        std::cout << "Incorrent message_length\n";
        break;
      }
      if (HandleInsPva(reinterpret_cast<novatel::InsPva *>(message))) {
        *message_ptr = &pva_;
        return MessageType::INS;
      }
      break;
    default:
      break;
  }

  return MessageType::NONE;
}

bool NovatelParser::check_crc() {
  size_t l = buffer_.size() - novatel::CRC_LENGTH;
  return CalculateBlockCRC32(l, (unsigned char*)buffer_.data()) == *(uint32_t*)(buffer_.data() + l);
}

bool NovatelParser::HandleInsPva(const novatel::InsPva* pva) {
  //最简单的处理方式？
  pva_ = *pva;
  return true;
}

bool NovatelParser::HandleCorrImuData(const novatel::CorrImuData* imu) {
  //todo
  //数据处理都放在publish函数中，如 从 velocity_change 至 acceleration 的变换操作 
  imu_ = *imu;
  return true;
}
} // namespace beidou
