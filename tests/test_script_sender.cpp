// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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

  void SetUp()
  {
    program_ = "test program\n";
    script_sender_.reset(new control::ScriptSender(50002, program_));
    client_.reset(new Client(50002));
  }

  void TearDown()
  {
    if (client_->getState() == comm::SocketState::Connected)
    {
      client_->close();
    }
  }

  std::unique_ptr<control::ScriptSender> script_sender_;
  std::unique_ptr<Client> client_;
  std::string program_;
};

TEST_F(ScriptSenderTest, wrong_request_command)
{
  std::string wrong_request = "request\n";
  client_->send(wrong_request);

  // We shouldn't receive an answer when sending the wrong request to the server, the result should therefore be an
  // empty string
  std::string actual_answer = client_->recv();
  std::string expected_answer = "";
  EXPECT_EQ(expected_answer, actual_answer);
}

TEST_F(ScriptSenderTest, request_program)
{
  std::string request = "request_program\n";
  client_->send(request);

  std::string received_program = client_->recv();
  EXPECT_EQ(program_, received_program);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
