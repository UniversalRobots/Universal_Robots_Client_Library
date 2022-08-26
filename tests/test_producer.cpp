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
#include <condition_variable>

#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/comm/tcp_server.h>
#include <ur_client_library/rtde/rtde_parser.h>

using namespace urcl;

class ProducerTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    server_.reset(new comm::TCPServer(60002));
    server_->setConnectCallback(std::bind(&ProducerTest::connectionCallback, this, std::placeholders::_1));
    server_->start();
  }

  void Teardown()
  {
    // Clean up
    server_.reset();
  }

  void connectionCallback(const int filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    client_fd_ = filedescriptor;
    connect_cv_.notify_one();
    connection_callback_ = true;
  }

  bool waitForConnectionCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(connect_mutex_);
    if (connect_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        connection_callback_ == true)
    {
      connection_callback_ = false;
      return true;
    }
    return false;
  }

  std::unique_ptr<comm::TCPServer> server_;
  int client_fd_;

private:
  std::condition_variable connect_cv_;
  std::mutex connect_mutex_;

  bool connection_callback_ = false;
};

TEST_F(ProducerTest, get_data_package)
{
  comm::URStream<rtde_interface::RTDEPackage> stream("127.0.0.1", 60002);
  std::vector<std::string> recipe = { "timestamp" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  comm::URProducer<rtde_interface::RTDEPackage> producer(stream, parser);

  producer.setupProducer();
  waitForConnectionCallback();
  producer.startProducer();

  // RTDE package with timestamp
  uint8_t data_package[] = { 0x00, 0x0c, 0x55, 0x01, 0x40, 0xbb, 0xbf, 0xdb, 0xa5, 0xe3, 0x53, 0xf7 };
  size_t written;
  server_->write(client_fd_, data_package, sizeof(data_package), written);

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  EXPECT_EQ(producer.tryGet(products), true);

  if (rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(products[0].get()))
  {
    double timestamp;
    data->getData("timestamp", timestamp);
    EXPECT_FLOAT_EQ(timestamp, 7103.86);
  }
  else
  {
    std::cout << "Failed to get data package" << std::endl;
    GTEST_FAIL();
  }

  producer.stopProducer();
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}