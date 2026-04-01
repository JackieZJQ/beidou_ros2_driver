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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tools/macros.h"

namespace beidou {

// A general pointer to a protobuf message.
using MessagePtr = void*;

enum class MessageType {
  NONE,
  GNSS,
  GNSS_RANGE,
  IMU,
  INS,
  INSSPEED,
  INS_STAT,
  WHEEL,
  EPHEMERIDES,
  OBSERVATION,
  GPGGA,
  BDSEPHEMERIDES,
  RAWIMU,
  GPSEPHEMERIDES,
  GLOEPHEMERIDES,
  BEST_GNSS_POS,
  HEADING,
};

struct MessageInfo {
  MessageType type;
  MessagePtr message_ptr;
};

using MessageInfoVec = std::vector<MessageInfo>;

// An abstract class of Parser.
// One should use the create_xxx() functions to create a Parser object.
class Parser {
public:
  // Return a pointer to a parser. The caller should take ownership.
  static Parser* CreateNovatel();

  static Parser* CreateParser() {
    return Parser::CreateNovatel();
  }

  virtual ~Parser() {}

  // Updates the parser with new data. The caller must keep the data valid until GetMessage()
  // returns NONE.
  void Update(const uint8_t* data, size_t length) {
    data_ = data;
    data_end_ = data + length;
  }

  void Update(const std::string& data) {
    Update(reinterpret_cast<const uint8_t*>(data.data()), data.size());
  }

  // Gets a parsed protobuf message. The caller must consume the message before calling another
  // GetMessage() or Update();
  virtual MessageType GetMessage(MessagePtr* message_ptr) { return MessageType::NONE; }
  virtual void GetMessages(MessageInfoVec* messages) {}
  virtual void GetMessages(MessageInfo* message) {}
  virtual bool GetInsStat(MessagePtr* message_ptr) { return false; }

protected:
  Parser() {}

  // Point to the beginning and end of data. Do not take ownership.
  const uint8_t* data_ = nullptr;
  const uint8_t* data_end_ = nullptr;

private:
   DISABLE_COPY_AND_ASSIGN(Parser);
};

}  // namespace beidou

