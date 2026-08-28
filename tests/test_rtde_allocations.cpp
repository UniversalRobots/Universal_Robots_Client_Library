// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

// All memory an RTDE connection needs for exchanging data is allocated while the recipes are set
// up, using the data types the robot reports in its acknowledgements. These tests pin that down by
// counting allocations around a few hundred receive cycles against the fake RTDE server.

#include <gtest/gtest.h>

#include <cstdlib>
#include <new>

#include <ur_client_library/log.h>
#include <ur_client_library/rtde/rtde_client.h>

#include "fake_rtde_server.h"

using namespace urcl;

namespace
{
// Counting is per-thread: the fake server and, in the background-read case, the client's read
// thread run in the same process, and their allocations are none of this test's business.
thread_local std::size_t g_allocation_count = 0;
thread_local bool g_count_allocations = false;

constexpr int g_FAKE_RTDE_PORT = 60005;
constexpr double g_RTDE_FREQUENCY = 125.0;
constexpr int g_WARMUP_CYCLES = 10;
constexpr int g_MEASURED_CYCLES = 50;

/*!
 * \brief Counts the allocations made on the current thread for as long as it is alive.
 */
class AllocationCounter
{
public:
  AllocationCounter()
  {
    g_allocation_count = 0;
    g_count_allocations = true;
  }

  ~AllocationCounter()
  {
    g_count_allocations = false;
  }

  std::size_t count() const
  {
    return g_allocation_count;
  }
};
}  // namespace

// These replace the global allocation functions, so pairing malloc with free is correct here even
// though GCC cannot see across the replacement and flags it.
#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

void* operator new(std::size_t size)
{
  if (g_count_allocations)
  {
    ++g_allocation_count;
  }
  void* memory = std::malloc(size == 0 ? 1 : size);
  if (memory == nullptr)
  {
    throw std::bad_alloc();
  }
  return memory;
}

void* operator new[](std::size_t size)
{
  return operator new(size);
}

void operator delete(void* memory) noexcept
{
  std::free(memory);
}

void operator delete[](void* memory) noexcept
{
  std::free(memory);
}

void operator delete(void* memory, std::size_t) noexcept
{
  std::free(memory);
}

void operator delete[](void* memory, std::size_t) noexcept
{
  std::free(memory);
}

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic pop
#endif

// Guards the tests below: if the counter stopped seeing allocations, they would pass vacuously.
TEST(AllocationCounterTest, counts_allocations)
{
  std::size_t allocations = 0;
  std::vector<double> values;
  {
    AllocationCounter counter;
    values.resize(1024);
    allocations = counter.count();
  }
  EXPECT_GT(allocations, 0);
}

class RTDEAllocationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
    // Skip the client's bootup check, which would otherwise read data for a second
    server_->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(42));

    client_ = std::make_unique<rtde_interface::RTDEClient>("localhost", notifier_, output_recipe_, input_recipe_,
                                                           g_RTDE_FREQUENCY, false, g_FAKE_RTDE_PORT);
    ASSERT_TRUE(client_->init());
  }

  void TearDown() override
  {
    client_.reset();
    server_.reset();
  }

  // A recipe covering all data types that appear on the receiving side
  std::vector<std::string> output_recipe_{ "timestamp",         "actual_q",
                                           "actual_TCP_force",  "runtime_state",
                                           "robot_status_bits", "actual_digital_input_bits",
                                           "joint_mode",        "payload_cog",
                                           "tool_mode",         "output_int_register_24" };
  std::vector<std::string> input_recipe_{ "speed_slider_mask", "speed_slider_fraction" };

  comm::INotifier notifier_;
  std::unique_ptr<RTDEServer> server_;
  std::unique_ptr<rtde_interface::RTDEClient> client_;
};

TEST_F(RTDEAllocationTest, blocking_receive_does_not_allocate)
{
  ASSERT_TRUE(client_->start(false));
  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());

  // The first cycles let every buffer along the way reach its final capacity
  for (int i = 0; i < g_WARMUP_CYCLES; ++i)
  {
    ASSERT_TRUE(client_->getDataPackageBlocking(data_pkg));
  }

  // Deliberately no gtest macros inside the measured section, as those allocate themselves
  int received = 0;
  bool all_data_read = true;
  double timestamp = 0.0;
  vector6d_t actual_q{};
  std::bitset<18> digital_input_bits;
  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    for (int i = 0; i < g_MEASURED_CYCLES; ++i)
    {
      if (!client_->getDataPackageBlocking(data_pkg))
      {
        continue;
      }
      ++received;
      all_data_read &= data_pkg->getData("timestamp", timestamp);
      all_data_read &= data_pkg->getData("actual_q", actual_q);
      all_data_read &= data_pkg->getData<uint64_t>("actual_digital_input_bits", digital_input_bits);
    }
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(all_data_read);
  EXPECT_GT(received, 0);
  EXPECT_GT(timestamp, 0.0);
}

TEST_F(RTDEAllocationTest, background_receive_does_not_allocate)
{
  ASSERT_TRUE(client_->start(true));
  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  const std::chrono::milliseconds read_timeout{ 100 };

  for (int i = 0; i < g_WARMUP_CYCLES; ++i)
  {
    ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));
  }

  int received = 0;
  bool all_data_read = true;
  double timestamp = 0.0;
  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    for (int i = 0; i < g_MEASURED_CYCLES; ++i)
    {
      if (!client_->getDataPackage(data_pkg, read_timeout))
      {
        continue;
      }
      ++received;
      all_data_read &= data_pkg.getData("timestamp", timestamp);
    }
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(all_data_read);
  EXPECT_GT(received, 0);
  EXPECT_GT(timestamp, 0.0);

  client_->pause();
}

TEST_F(RTDEAllocationTest, sending_input_data_does_not_allocate)
{
  ASSERT_TRUE(client_->start(true));
  rtde_interface::DataPackage input_pkg(client_->getInputRecipe());
  ASSERT_TRUE(input_pkg.setData("speed_slider_mask", static_cast<uint32_t>(1)));

  for (int i = 0; i < g_WARMUP_CYCLES; ++i)
  {
    ASSERT_TRUE(client_->getWriter().sendSpeedSlider(0.5));
  }

  bool all_sent = true;
  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    for (int i = 0; i < g_MEASURED_CYCLES; ++i)
    {
      all_sent &= input_pkg.setData("speed_slider_fraction", 0.5);
      all_sent &= client_->getWriter().sendPackage(input_pkg);
    }
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(all_sent);

  client_->pause();
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  // Logging allocates, and a log statement inside a measured section would rightfully be counted.
  // Keep the routine chatter out of the way so the tests measure the data exchange itself.
  setLogLevel(LogLevel::ERROR);

  return RUN_ALL_TESTS();
}
