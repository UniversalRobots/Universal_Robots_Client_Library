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

#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/tcp_server.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/rtde/rtde_package.h>
#include <ur_client_library/rtde/rtde_parser.h>

using namespace urcl;

class PipelineTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    server_.reset(new comm::TCPServer(60002));
    server_->setConnectCallback(std::bind(&PipelineTest::connectionCallback, this, std::placeholders::_1));
    server_->start();

    // Setup pipeline
    stream_.reset(new comm::URStream<rtde_interface::RTDEPackage>("127.0.0.1", 60002));
    std::vector<std::string> recipe = { "timestamp" };
    parser_.reset(new rtde_interface::RTDEParser(recipe));
    parser_->setProtocolVersion(2);
    producer_.reset(new comm::URProducer<rtde_interface::RTDEPackage>(*stream_.get(), *parser_.get()));

    pipeline_.reset(new comm::Pipeline<rtde_interface::RTDEPackage>(*producer_.get(), "RTDE_PIPELINE", notifier_));
    pipeline_->init();
  }

  void Teardown()
  {
    // Clean up
    pipeline_->stop();
    pipeline_.reset();
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

  std::unique_ptr<comm::URStream<rtde_interface::RTDEPackage>> stream_;
  std::unique_ptr<rtde_interface::RTDEParser> parser_;
  std::unique_ptr<comm::URProducer<rtde_interface::RTDEPackage>> producer_;
  std::unique_ptr<comm::Pipeline<rtde_interface::RTDEPackage>> pipeline_;
  comm::INotifier notifier_;

  // Consumer class
  class TestConsumer : public comm::IConsumer<rtde_interface::RTDEPackage>
  {
  public:
    TestConsumer() = default;
    virtual ~TestConsumer() = default;

    bool waitForConsumer(int milliseconds = 100)
    {
      std::unique_lock<std::mutex> lk(consumed_mutex_);
      if (consumed_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
          consumed_callback_ == true)
      {
        consumed_callback_ = true;
        return true;
      }
      return false;
    }

    // Consume a package
    virtual bool consume(std::shared_ptr<rtde_interface::RTDEPackage> product)
    {
      std::lock_guard<std::mutex> lk(consumed_mutex_);
      if (rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(product.get()))
      {
        data->getData("timestamp", timestamp_);
      }
      consumed_cv_.notify_one();
      consumed_callback_ = true;
      return true;
    }

    double timestamp_ = 0.0;
    std::condition_variable consumed_cv_;
    std::mutex consumed_mutex_;
    bool consumed_callback_ = false;
  };

private:
  std::condition_variable connect_cv_;
  std::mutex connect_mutex_;

  bool connection_callback_ = false;
};

TEST_F(PipelineTest, get_product_from_stopped_pipeline)
{
  std::unique_ptr<rtde_interface::RTDEPackage> urpackage;
  std::chrono::milliseconds timeout{ 100 };
  EXPECT_EQ(pipeline_->getLatestProduct(urpackage, timeout), false);
}

TEST_F(PipelineTest, get_product_from_running_pipeline)
{
  waitForConnectionCallback();
  pipeline_->run();

  // RTDE package with timestamp
  uint8_t data_package[] = { 0x00, 0x0c, 0x55, 0x01, 0x40, 0xbb, 0xbf, 0xdb, 0xa5, 0xe3, 0x53, 0xf7 };
  size_t written;
  server_->write(client_fd_, data_package, sizeof(data_package), written);

  std::unique_ptr<rtde_interface::RTDEPackage> urpackage;
  std::chrono::milliseconds timeout{ 500 };
  EXPECT_EQ(pipeline_->getLatestProduct(urpackage, timeout), true);
  if (rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(urpackage.get()))
  {
    double timestamp;
    double expected_timestamp = 7103.8579;
    data->getData("timestamp", timestamp);
    EXPECT_FLOAT_EQ(timestamp, expected_timestamp);
  }
  else
  {
    std::cout << "Failed to get data package data" << std::endl;
    GTEST_FAIL();
  }
}

TEST_F(PipelineTest, stop_pipeline)
{
  waitForConnectionCallback();
  pipeline_->run();

  // RTDE package with timestamp
  uint8_t data_package[] = { 0x00, 0x0c, 0x55, 0x01, 0x40, 0xbb, 0xbf, 0xdb, 0xa5, 0xe3, 0x53, 0xf7 };
  size_t written;
  server_->write(client_fd_, data_package, sizeof(data_package), written);

  std::unique_ptr<rtde_interface::RTDEPackage> urpackage;
  std::chrono::milliseconds timeout{ 500 };
  // Ensure that pipeline is running
  EXPECT_EQ(pipeline_->getLatestProduct(urpackage, timeout), true);

  pipeline_->stop();

  // We shouldn't be able to fetch a package when the pipeline has been stopped
  EXPECT_EQ(pipeline_->getLatestProduct(urpackage, timeout), false);
}

TEST_F(PipelineTest, consumer_pipeline)
{
  stream_.reset(new comm::URStream<rtde_interface::RTDEPackage>("127.0.0.1", 60002));
  producer_.reset(new comm::URProducer<rtde_interface::RTDEPackage>(*stream_.get(), *parser_.get()));
  TestConsumer consumer;
  pipeline_.reset(
      new comm::Pipeline<rtde_interface::RTDEPackage>(*producer_.get(), &consumer, "RTDE_PIPELINE", notifier_));
  pipeline_->init();
  waitForConnectionCallback();
  pipeline_->run();

  // RTDE package with timestamp
  uint8_t data_package[] = { 0x00, 0x0c, 0x55, 0x01, 0x40, 0xbb, 0xbf, 0xdb, 0xa5, 0xe3, 0x53, 0xf7 };
  size_t written;
  server_->write(client_fd_, data_package, sizeof(data_package), written);

  // Wait for data to be consumed
  int max_retries = 3;
  int count = 0;
  while (consumer.waitForConsumer(500) == false)
  {
    if (count >= max_retries)
    {
      break;
    }
    server_->write(client_fd_, data_package, sizeof(data_package), written);
    count++;
  }
  EXPECT_LT(count, max_retries);

  // Test that the package was consumed
  double expected_timestamp = 7103.8579;
  EXPECT_FLOAT_EQ(consumer.timestamp_, expected_timestamp);

  pipeline_->stop();
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
