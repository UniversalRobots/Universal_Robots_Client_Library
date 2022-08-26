// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#include <gtest/gtest.h>
#include <ur_client_library/control/script_sender.h>
#include <ur_client_library/comm/tcp_socket.h>

using namespace urcl;

class ScriptSenderTest : public ::testing::Test
{
protected:
  class Client : public comm::TCPSocket
  {
  public:
    Client(const int& port)
    {
      std::string host = "127.0.0.1";
      TCPSocket::setup(host, port);
      timeval tv;
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      TCPSocket::setReceiveTimeout(tv);
    }

    void send(const std::string& text)
    {
      size_t len = text.size();
      const uint8_t* data = reinterpret_cast<const uint8_t*>(text.c_str());
      size_t written;
      TCPSocket::write(data, len, written);
    }

    std::string recv()
    {
      std::stringstream result;
      char character;
      size_t read_chars = 99;
      while (read_chars > 0)
      {
        if (!TCPSocket::read((uint8_t*)&character, 1, read_chars))
        {
          break;
        }
        result << character;
        if (character == '\n')
        {
          break;
        }
      }
      return result.str();
    }

  protected:
    virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
    {
      return ::connect(socket_fd, address, address_len) == 0;
    }
  };
};

TEST_F(ScriptSenderTest, wrong_request_command)
{
  std::string program = "test program\n";
  control::ScriptSender script_sender(50002, program);
  Client client(50002);

  std::string wrong_request = "request\n";
  client.send(wrong_request);

  // We shouldn't receive an answer when sending the wrong request to the server, the result should therefore be an
  // empty string
  std::string actual_anwser = client.recv();
  std::string expected_anwser = "";
  EXPECT_EQ(expected_anwser, actual_anwser);
}

TEST_F(ScriptSenderTest, request_program)
{
  std::string send_program = "test_program\n";
  control::ScriptSender script_sender(50002, send_program);
  Client client(50002);

  std::string request = "request_program\n";
  client.send(request);

  std::string received_program = client.recv();
  EXPECT_EQ(send_program, received_program);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
