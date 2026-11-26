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

#include <gtest/gtest.h>
#include <urcl_3rdparty/portable_endian.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include <ur_client_library/comm/tcp_socket.h>
#include <ur_client_library/control/reverse_interface.h>
#include <ur_client_library/control/script_command_interface.h>
#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/helpers.h>

using namespace urcl;

namespace
{
// State owned by the std::function the interface stores. Its destructor records when the last copy
// is gone, which is when that callback member is destroyed. The callback copies the observers onto
// its stack first, so it can report that fact after the member is gone without reading freed memory
// except for `token`, which is the use AddressSanitizer and ThreadSanitizer should report.
struct CallbackLifetime
{
  std::shared_ptr<std::atomic<int>> copies = std::make_shared<std::atomic<int>>(0);
  std::shared_ptr<std::atomic<bool>> last_copy_destroyed = std::make_shared<std::atomic<bool>>(false);
  std::shared_ptr<std::atomic<bool>> entered = std::make_shared<std::atomic<bool>>(false);
  std::shared_ptr<std::atomic<bool>> release = std::make_shared<std::atomic<bool>>(false);
  std::shared_ptr<std::atomic<bool>> saw_destroyed = std::make_shared<std::atomic<bool>>(false);
  std::shared_ptr<std::atomic<bool>> token_ok = std::make_shared<std::atomic<bool>>(false);

  struct State
  {
    static constexpr int TOKEN = 0x5A5A5A5A;

    std::shared_ptr<std::atomic<int>> copies;
    std::shared_ptr<std::atomic<bool>> last_copy_destroyed;
    std::shared_ptr<std::atomic<bool>> entered;
    std::shared_ptr<std::atomic<bool>> release;
    std::shared_ptr<std::atomic<bool>> saw_destroyed;
    std::shared_ptr<std::atomic<bool>> token_ok;
    int token = TOKEN;

    explicit State(const CallbackLifetime& owner)
      : copies(owner.copies)
      , last_copy_destroyed(owner.last_copy_destroyed)
      , entered(owner.entered)
      , release(owner.release)
      , saw_destroyed(owner.saw_destroyed)
      , token_ok(owner.token_ok)
    {
      copies->fetch_add(1, std::memory_order_relaxed);
    }

    State(const State& other)
      : copies(other.copies)
      , last_copy_destroyed(other.last_copy_destroyed)
      , entered(other.entered)
      , release(other.release)
      , saw_destroyed(other.saw_destroyed)
      , token_ok(other.token_ok)
      , token(other.token)
    {
      copies->fetch_add(1, std::memory_order_relaxed);
    }

    State(State&& other) noexcept
      : copies(std::move(other.copies))
      , last_copy_destroyed(std::move(other.last_copy_destroyed))
      , entered(std::move(other.entered))
      , release(std::move(other.release))
      , saw_destroyed(std::move(other.saw_destroyed))
      , token_ok(std::move(other.token_ok))
      , token(other.token)
    {
    }

    State& operator=(const State&) = delete;
    State& operator=(State&&) = delete;

    ~State()
    {
      if (copies && copies->fetch_sub(1, std::memory_order_acq_rel) == 1 && last_copy_destroyed)
      {
        last_copy_destroyed->store(true, std::memory_order_release);
      }
    }

    void run() const
    {
      auto destroyed = last_copy_destroyed;
      auto is_entered = entered;
      auto may_finish = release;
      auto saw = saw_destroyed;
      auto ok = token_ok;
      is_entered->store(true);
      while (!may_finish->load())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      saw->store(destroyed->load());
      ok->store(token == TOKEN);
    }
  };

  template <typename... Args>
  std::function<void(Args...)> pausingCallback()
  {
    State state(*this);
    return [state](Args...) { state.run(); };
  }
};

// Sets the most-derived destructor as started and does not stop the worker. These probes have no
// callback-owned members; stopping here would hide a base destructor that joins too late.
class ProbeReverseInterface : public control::ReverseInterface
{
public:
  ProbeReverseInterface(const control::ReverseInterfaceConfig& config,
                        std::shared_ptr<std::atomic<bool>> destructor_started)
    : ReverseInterface(config), destructor_started_(std::move(destructor_started))
  {
  }

  ~ProbeReverseInterface() override
  {
    destructor_started_->store(true);
  }

private:
  std::shared_ptr<std::atomic<bool>> destructor_started_;
};

class ProbeScriptCommandInterface : public control::ScriptCommandInterface
{
public:
  ProbeScriptCommandInterface(const control::ReverseInterfaceConfig& config,
                              std::shared_ptr<std::atomic<bool>> destructor_started)
    : ScriptCommandInterface(config), destructor_started_(std::move(destructor_started))
  {
  }

  ~ProbeScriptCommandInterface() override
  {
    destructor_started_->store(true);
  }

private:
  std::shared_ptr<std::atomic<bool>> destructor_started_;
};

class ProbeTrajectoryPointInterface : public control::TrajectoryPointInterface
{
public:
  ProbeTrajectoryPointInterface(uint32_t port, std::shared_ptr<std::atomic<bool>> destructor_started)
    : TrajectoryPointInterface(port), destructor_started_(std::move(destructor_started))
  {
  }

  ~ProbeTrajectoryPointInterface() override
  {
    destructor_started_->store(true);
  }

private:
  std::shared_ptr<std::atomic<bool>> destructor_started_;
};

class Client : public comm::TCPSocket
{
public:
  explicit Client(int port)
  {
    if (!TCPSocket::connect("127.0.0.1", port, 20, std::chrono::milliseconds(50)))
    {
      throw std::runtime_error("Failed to connect test client");
    }
  }

  void sendInt32(int32_t value)
  {
    const int32_t encoded = htobe32(value);
    uint8_t bytes[sizeof(encoded)];
    std::memcpy(bytes, &encoded, sizeof(encoded));
    size_t written = 0;
    if (!TCPSocket::write(bytes, sizeof(bytes), written) || written != sizeof(bytes))
    {
      throw std::runtime_error("Failed to write test message");
    }
  }
};

template <typename Destroy>
class ReleaseAndJoin
{
public:
  ReleaseAndJoin(std::atomic<bool>& release, Destroy destroy) : release_(release), thread_(std::move(destroy))
  {
  }

  ~ReleaseAndJoin()
  {
    release_.store(true);
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  ReleaseAndJoin(const ReleaseAndJoin&) = delete;
  ReleaseAndJoin& operator=(const ReleaseAndJoin&) = delete;

private:
  std::atomic<bool>& release_;
  std::thread thread_;
};

// Give a destructor that joins too late enough time to destroy the callback member while the
// callback is paused. A destructor that joins first stays blocked here with the member alive.
// Waiting only for the destructor to return would also pass the old code, because ~TCPServer
// joined the worker after the callback state was already gone.
void waitWhileCallbackIsPaused(const std::atomic<bool>& last_copy_destroyed)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (std::chrono::steady_clock::now() < deadline && !last_copy_destroyed.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

template <typename Interface>
void destroyWhileCallbackIsPaused(std::unique_ptr<Interface>& interface, CallbackLifetime& lifetime,
                                  const std::shared_ptr<std::atomic<bool>>& destructor_started)
{
  waitFor([&]() { return lifetime.entered->load(); }, std::chrono::seconds(2));
  {
    ReleaseAndJoin join(*lifetime.release, [&interface]() { interface.reset(); });
    waitFor([&]() { return destructor_started->load(); }, std::chrono::seconds(2));
    waitWhileCallbackIsPaused(*lifetime.last_copy_destroyed);
    EXPECT_FALSE(lifetime.last_copy_destroyed->load());
  }
  EXPECT_FALSE(lifetime.saw_destroyed->load());
  EXPECT_TRUE(lifetime.token_ok->load());
}
}  // namespace

TEST(ControlInterfaceTeardown, reverse_interface_callback_state_outlives_callback)
{
  CallbackLifetime lifetime;
  control::ReverseInterfaceConfig config;
  config.port = 0;
  // The constructor invokes this callback with false on the calling thread, before the worker
  // exists. Pausing there would block startup. Only the later connected notification runs on the
  // worker and is the one that must stay alive across destruction.
  config.handle_program_state = [state = CallbackLifetime::State(lifetime)](bool running) {
    if (running)
    {
      state.run();
    }
  };
  ASSERT_EQ(lifetime.copies->load(), 1);

  auto destructor_started = std::make_shared<std::atomic<bool>>(false);
  auto interface = std::make_unique<ProbeReverseInterface>(config, destructor_started);
  config.handle_program_state = nullptr;
  ASSERT_EQ(lifetime.copies->load(), 1);

  Client client(interface->getPort());
  destroyWhileCallbackIsPaused(interface, lifetime, destructor_started);
}

TEST(ControlInterfaceTeardown, script_command_interface_callback_state_outlives_callback)
{
  CallbackLifetime lifetime;
  control::ReverseInterfaceConfig config;
  config.port = 0;
  auto destructor_started = std::make_shared<std::atomic<bool>>(false);
  auto interface = std::make_unique<ProbeScriptCommandInterface>(config, destructor_started);
  interface->setToolContactResultCallback(lifetime.pausingCallback<control::ToolContactResult>());
  ASSERT_EQ(lifetime.copies->load(), 1);

  Client client(interface->getPort());
  client.sendInt32(static_cast<int32_t>(control::ToolContactResult::UNTIL_TOOL_CONTACT_RESULT_SUCCESS));
  destroyWhileCallbackIsPaused(interface, lifetime, destructor_started);
}

TEST(ControlInterfaceTeardown, trajectory_point_interface_callback_state_outlives_callback)
{
  CallbackLifetime lifetime;
  auto destructor_started = std::make_shared<std::atomic<bool>>(false);
  auto interface = std::make_unique<ProbeTrajectoryPointInterface>(0, destructor_started);
  interface->setTrajectoryEndCallback(lifetime.pausingCallback<control::TrajectoryResult>());
  ASSERT_EQ(lifetime.copies->load(), 1);

  Client client(interface->getPort());
  client.sendInt32(static_cast<int32_t>(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS));
  destroyWhileCallbackIsPaused(interface, lifetime, destructor_started);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
