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

// These tests count calls to the replaced allocation functions on the calling thread.
// Package construction, recipe negotiation and test assertions stay outside measured sections.
// Unit tests isolate typing, parsing, copying and serialization; fake-server tests exercise
// steady-state client reads and input submission after warm-up. Separate first-read tests measure
// applying negotiated types to untyped and incorrectly typed application packages.
// Background-thread allocations, direct malloc calls and aligned allocation overloads are not
// counted. Error handling and logging are not generally allocation-free.

#include <gtest/gtest.h>

#include <cstdlib>
#include <new>

#include <ur_client_library/log.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/comm/bin_parser.h>

#include "fake_rtde_server.h"
#include "rtde_test_helpers.h"

using namespace urcl;

namespace
{
// Counting is per-thread so the fake server's allocations are not attributed to the client.
// Blocking receive covers parsing on this thread. Background receive and input submission measure
// the calling thread (copying the latest sample / copying into the store buffer); parsing and
// serialization themselves are covered by the same-thread tests below.
thread_local std::size_t g_allocation_count = 0;
thread_local bool g_count_allocations = false;
// Stores the pointer from the guard allocation so the compiler cannot prove the new/delete pair
// is unused and omit the call to the replaced operator new (GCC's allocation DCE at -O2).
void* volatile g_allocation_sink = nullptr;

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

void* operator new(std::size_t size, const std::nothrow_t&) noexcept
{
  if (g_count_allocations)
  {
    ++g_allocation_count;
  }
  return std::malloc(size == 0 ? 1 : size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept
{
  return operator new(size, std::nothrow);
}

void operator delete(void* memory, const std::nothrow_t&) noexcept
{
  std::free(memory);
}

void operator delete[](void* memory, const std::nothrow_t&) noexcept
{
  std::free(memory);
}

void operator delete(void* memory, std::size_t, const std::nothrow_t&) noexcept
{
  std::free(memory);
}

void operator delete[](void* memory, std::size_t, const std::nothrow_t&) noexcept
{
  std::free(memory);
}

#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic pop
#endif

// Positive control: require a nonzero count for an explicit allocation, so a broken counter cannot
// make the allocation-free tests pass without observing any allocations.
// Call operator new directly rather than writing `new int`: a new-expression may be omitted even
// when the pointer escapes, which is what Alpine's gcc 15 does at -O2. Allocate with operator new
// rather than a container: on some libstdc++ / musl builds std::allocator uses malloc and would
// never hit the replaced operator new that the RTDE tests count.
TEST(AllocationCounterTest, counts_allocations)
{
  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    g_allocation_sink = ::operator new(sizeof(int));
    allocations = counter.count();
    ::operator delete(g_allocation_sink);
    g_allocation_sink = nullptr;
  }
  EXPECT_GT(allocations, 0);
}

// Construct the recipe storage and type vector before counting, then measure only setTypes().
// Applying valid types must reuse that storage; check a resulting type after measurement.
TEST(DataPackageAllocationTest, applying_types_does_not_allocate)
{
  rtde_interface::DataPackage package({ "timestamp", "actual_q" });
  const std::vector<std::string> types{ "DOUBLE", "VECTOR6D" };

  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    package.setTypes(types);
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_EQ(package.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
}

// Serialize a valid frame before counting, then parse it 100 times into the same typed package.
// The borrowed parser must succeed on every iteration without allocating temporary packages.
TEST(DataPackageAllocationTest, borrowed_parser_reuses_storage)
{
  auto output = test::typedPackage({ "timestamp" }, { "DOUBLE" });
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(output);
  uint8_t bytes[64];
  const auto size = output.serializePackage(bytes);
  bool success = true;
  size_t allocations = 0;
  {
    AllocationCounter counter;
    for (size_t i = 0; i < 100; ++i)
    {
      comm::BinParser bp(bytes, size);
      success = parser.parseDataPackage(bp, output) && success;
    }
    allocations = counter.count();
  }
  EXPECT_TRUE(success);
  EXPECT_EQ(allocations, 0u);
}

// Deliberately pass a foreign layout with DEBUG logging enabled. Parsing must fail and emit one
// diagnostic. Even a non-allocating handler cannot avoid the logger's formatting allocation,
// demonstrating why enabled failure diagnostics are outside the allocation-free guarantee.
TEST(DataPackageAllocationTest, enabled_failure_diagnostics_are_not_allocation_free)
{
  struct DiagnosticHandler : LogHandler
  {
    explicit DiagnosticHandler(size_t& calls) : calls_(calls)
    {
    }
    void log(const char*, int, LogLevel, const char*) override
    {
      ++calls_;
    }
    size_t& calls_;
  };
  struct RestoreLogger
  {
    ~RestoreLogger()
    {
      unregisterLogHandler();
      setLogLevel(LogLevel::INFO);
    }
  } restore;
  size_t messages = 0;
  registerLogHandler(std::make_unique<DiagnosticHandler>(messages));
  setLogLevel(LogLevel::DEBUG);
  auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
  auto foreign = test::typedPackage({ "other" }, { "DOUBLE" });
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(expected);
  uint8_t bytes[64];
  const auto size = expected.serializePackage(bytes);
  size_t allocations = 0;
  bool parsed = true;
  {
    AllocationCounter counter;
    comm::BinParser bp(bytes, size);
    parsed = parser.parseDataPackage(bp, foreign);
    allocations = counter.count();
  }
  EXPECT_FALSE(parsed);
  EXPECT_EQ(messages, 1u);
  // urcl::log allocates its formatting buffer even with a non-allocating handler.
  EXPECT_GT(allocations, 0u);
}

// Parse a known wire frame through the unique-pointer overload into an already typed package.
// Require zero allocations and verify the decoded timestamp, so simply rejecting the frame
// cannot satisfy the allocation check.
TEST(DataPackageAllocationTest, parsing_a_preallocated_package_does_not_allocate)
{
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  const std::vector<std::string> types = { "DOUBLE", "DOUBLE" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(test::typedPackage(recipe, types).layoutHash());
  // Same as after the handshake: the package already has the negotiated layout, so parse must not
  // allocate a replacement.
  std::unique_ptr<rtde_interface::RTDEPackage> product =
      std::make_unique<rtde_interface::DataPackage>(test::typedPackage(recipe, types));

  std::size_t allocations = 0;
  bool parsed = false;
  {
    AllocationCounter counter;
    comm::BinParser bp(raw_data, sizeof(raw_data));
    try
    {
      parsed = parser.parse(bp, product);
    }
    catch (const urcl::UrException&)
    {
      parsed = false;
    }
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(parsed);
  rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
  ASSERT_NE(data, nullptr);
  double timestamp = 0.0;
  ASSERT_TRUE(data->getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 16412.206);
}

// Unlike RTDEClient, the parser does not apply negotiated types to an untyped destination.
// Register only the expected layout hash, then require rejection without allocation or pointer
// replacement. The layout-mismatch DEBUG diagnostic is disabled for this measurement.
TEST(DataPackageAllocationTest, parsing_into_an_untyped_package_is_rejected)
{
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }).layoutHash());
  std::unique_ptr<rtde_interface::RTDEPackage> product = std::make_unique<rtde_interface::DataPackage>(recipe);
  const rtde_interface::RTDEPackage* package_address = product.get();

  std::size_t allocations = 0;
  bool parsed = false;
  {
    AllocationCounter counter;
    comm::BinParser bp(raw_data, sizeof(raw_data));
    parsed = parser.parse(bp, product);
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_FALSE(parsed);
  EXPECT_EQ(product.get(), package_address);
}

// Copy a partially initialized source into a typed destination with a previously nonzero mask.
// Measure copyFrom() only, then verify the supplied fraction is copied and the unset mask becomes
// a typed zero. INFO logging remains enabled to catch unexpected allocating chatter on this path.
TEST(DataPackageAllocationTest, copying_a_partial_package_does_not_allocate)
{
  auto destination = test::typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  ASSERT_TRUE(destination.setData("speed_slider_mask", uint32_t{ 1 }));
  rtde_interface::DataPackage source({ "speed_slider_mask", "speed_slider_fraction" });
  ASSERT_TRUE(source.setData("speed_slider_fraction", 0.5));

  setLogLevel(LogLevel::INFO);
  std::size_t allocations = 0;
  bool copied = false;
  {
    AllocationCounter counter;
    copied = destination.copyFrom(source);
    allocations = counter.count();
  }
  setLogLevel(LogLevel::ERROR);

  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(copied);
  uint32_t mask = 1;
  double fraction = 0.0;
  ASSERT_TRUE(destination.getData("speed_slider_mask", mask));
  ASSERT_TRUE(destination.getData("speed_slider_fraction", fraction));
  EXPECT_EQ(mask, 0u);
  EXPECT_DOUBLE_EQ(fraction, 0.5);
}

// Serialize an already typed UINT32 field into caller-owned storage while counting allocations.
// Require zero allocations and the expected eight-byte frame size: header, recipe ID and payload.
TEST(DataPackageAllocationTest, serializing_a_typed_package_does_not_allocate)
{
  auto package = test::typedPackage({ "speed_slider_mask" }, { "UINT32" });
  ASSERT_TRUE(package.setData("speed_slider_mask", static_cast<uint32_t>(1)));
  package.setRecipeID(1);
  uint8_t buffer[4096];

  std::size_t allocations = 0;
  size_t size = 0;
  {
    AllocationCounter counter;
    size = package.serializePackage(buffer);
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_EQ(size, 8);
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

  // Representative output recipe covering scalar, vector and digital-bit fields.
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

// After warm-up, measure synchronous socket reads, parsing and selected scalar/vector/bitset
// accessors on the calling thread. Require zero allocations, successful field reads and at least
// one received package. First-read typing and wrong-type repair have separate allocation tests.
TEST_F(RTDEAllocationTest, blocking_receive_does_not_allocate)
{
  ASSERT_TRUE(client_->start(false));
  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());

  // The first warm-up read applies the negotiated types to this recipe-only package.
  // The remaining cycles let every buffer along the way reach its final capacity.
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

// Warm up the connection using a different package, leaving the measured destination untyped.
// Count its first read, including application of negotiated types, then require successful
// receipt, a typed destination and a readable timestamp without allocation.
TEST_F(RTDEAllocationTest, typing_a_package_on_the_first_read_does_not_allocate)
{
  ASSERT_TRUE(client_->start(false));
  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  ASSERT_FALSE(data_pkg->isTyped());

  // Warm up on a package of its own so the measured read is the first one for data_pkg
  auto warmup_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  for (int i = 0; i < g_WARMUP_CYCLES; ++i)
  {
    ASSERT_TRUE(client_->getDataPackageBlocking(warmup_pkg));
  }

  bool received = false;
  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    received = client_->getDataPackageBlocking(data_pkg);
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0);
  EXPECT_TRUE(received);
  EXPECT_TRUE(data_pkg->isTyped());

  double timestamp = 0.0;
  ASSERT_TRUE(data_pkg->getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);
}

// A same-recipe package may already carry incorrect types. The client must replace them with
// the negotiated layout before parsing, without allocating or replacing the package object.
// Warm up using a separate package so the measured read includes the incorrect layout's repair.
TEST_F(RTDEAllocationTest, repairing_wrong_types_on_the_first_read_does_not_allocate)
{
  ASSERT_TRUE(client_->start(false));

  auto warmup_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  for (int i = 0; i < g_WARMUP_CYCLES; ++i)
  {
    ASSERT_TRUE(client_->getDataPackageBlocking(warmup_pkg));
  }
  ASSERT_EQ(warmup_pkg->getDataType("joint_mode"), rtde_interface::DataType::VECTOR6INT32);

  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  // Deliberately wrong: joint_mode is a six-element integer vector, not an INT32 scalar.
  // Do not correct this entry: it is the mismatch whose repair this test measures.
  data_pkg->setTypes(
      { "DOUBLE", "VECTOR6D", "VECTOR6D", "UINT32", "UINT32", "UINT64", "INT32", "VECTOR3D", "UINT32", "INT32" });

  ASSERT_TRUE(data_pkg->isTyped());
  ASSERT_EQ(data_pkg->getDataType("joint_mode"), rtde_interface::DataType::INT32);
  ASSERT_NE(data_pkg->layoutHash(), warmup_pkg->layoutHash());
  const auto* package_address = data_pkg.get();

  bool received = false;
  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    received = client_->getDataPackageBlocking(data_pkg);
    allocations = counter.count();
  }

  EXPECT_EQ(allocations, 0u);
  ASSERT_TRUE(received);
  ASSERT_EQ(data_pkg.get(), package_address);
  EXPECT_EQ(data_pkg->layoutHash(), warmup_pkg->layoutHash());
  ASSERT_EQ(data_pkg->getDataType("joint_mode"), rtde_interface::DataType::VECTOR6INT32);

  // Verify that the repaired layout supports reading the vector and that data was received.
  vector6int32_t joint_mode{};
  EXPECT_TRUE(data_pkg->getData("joint_mode", joint_mode));
  double timestamp = 0.0;
  ASSERT_TRUE(data_pkg->getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);
}

// Warm up a reusable destination, then measure copying the latest background sample and reading
// its timestamp on the calling thread. Require successful receipt without allocation; parsing
// on the background reader thread is outside this counter and is tested separately.
TEST_F(RTDEAllocationTest, copying_the_latest_background_package_does_not_allocate)
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

// Create a negotiated input package before counting. Repeatedly update its fraction and submit
// it to the writer, requiring every operation to succeed without calling-thread allocations.
// This measures store-buffer updates, not background serialization or delivery to the server.
TEST_F(RTDEAllocationTest, copying_input_data_into_the_store_buffer_does_not_allocate)
{
  ASSERT_TRUE(client_->start(true));
  rtde_interface::DataPackage input_pkg = client_->createInputDataPackage();
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

// Leave the input mask untyped and set only the fraction, then repeatedly submit that partial
// package after warm-up. Submission must accept its compatible fields and fill unset fields
// with typed zeros without calling-thread allocations. Wire delivery is not checked here.
TEST_F(RTDEAllocationTest, sending_a_partial_package_does_not_allocate)
{
  ASSERT_TRUE(client_->start(true));
  rtde_interface::DataPackage input_pkg(client_->getInputRecipe());
  ASSERT_TRUE(input_pkg.setData("speed_slider_fraction", 0.5));

  for (int i = 0; i < g_WARMUP_CYCLES; ++i)
  {
    ASSERT_TRUE(client_->getWriter().sendPackage(input_pkg));
  }

  bool all_sent = true;
  std::size_t allocations = 0;
  {
    AllocationCounter counter;
    for (int i = 0; i < g_MEASURED_CYCLES; ++i)
    {
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
