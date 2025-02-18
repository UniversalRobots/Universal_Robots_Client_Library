// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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

#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/ur/calibration_checker.h>
#include <memory>

using namespace urcl;

std::string g_ROBOT_IP = "192.168.56.101";

class PrimaryClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_ = std::make_unique<primary_interface::PrimaryClient>(g_ROBOT_IP, notifier_);
  }

  std::unique_ptr<primary_interface::PrimaryClient> client_;
  comm::INotifier notifier_;
};

TEST_F(PrimaryClientTest, start_communication_succeeds)
{
  EXPECT_NO_THROW(client_->start());
}

TEST_F(PrimaryClientTest, add_and_remove_consumer)
{
  auto calibration_consumer = std::make_shared<urcl::CalibrationChecker>("test");

  client_->addPrimaryConsumer(calibration_consumer);

  EXPECT_NO_THROW(client_->start());

  auto start_time = std::chrono::system_clock::now();
  const auto timeout = std::chrono::seconds(5);
  while (!calibration_consumer->isChecked() && std::chrono::system_clock::now() - start_time < timeout)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(calibration_consumer->isChecked());

  client_->removePrimaryConsumer(calibration_consumer);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
