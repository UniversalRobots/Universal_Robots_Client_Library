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
#include <type_traits>

#include <ur_client_library/comm/unique_fd.h>

using namespace urcl::comm;

// UniqueFd is a thin RAII wrapper around a socket descriptor. On POSIX platforms a socket
// descriptor is an ordinary file descriptor, so we can allocate real descriptors with dup() and
// verify that they get closed by querying fcntl(F_GETFD), which fails with EBADF once the
// descriptor is closed. On Windows sockets are not fcntl-able, so the closing-behaviour tests are
// POSIX-only; the ownership semantics are identical across platforms.
#ifndef _WIN32
#  include <fcntl.h>
#  include <unistd.h>

namespace
{
// Allocates a fresh, valid descriptor for testing by duplicating stdin.
socket_t makeFd()
{
  socket_t fd = ::dup(0);
  EXPECT_GE(fd, 0) << "Could not allocate a test file descriptor";
  return fd;
}

bool isFdOpen(socket_t fd)
{
  return ::fcntl(fd, F_GETFD) != -1;
}
}  // namespace

TEST(UniqueFdTest, default_constructed_is_invalid)
{
  UniqueFd fd;
  EXPECT_EQ(fd.get(), INVALID_SOCKET);
}

TEST(UniqueFdTest, reset_adopts_descriptor)
{
  socket_t raw = makeFd();
  UniqueFd fd;
  fd.reset(raw);
  EXPECT_EQ(fd.get(), raw);
  EXPECT_TRUE(isFdOpen(raw));
}

TEST(UniqueFdTest, reset_with_new_fd_closes_previous)
{
  socket_t first = makeFd();
  socket_t second = makeFd();

  UniqueFd fd;
  fd.reset(first);
  fd.reset(second);

  // The previously held descriptor must have been closed, and the new one adopted.
  EXPECT_FALSE(isFdOpen(first));
  EXPECT_TRUE(isFdOpen(second));
  EXPECT_EQ(fd.get(), second);
}

TEST(UniqueFdTest, reset_without_argument_closes_and_invalidates)
{
  socket_t raw = makeFd();

  UniqueFd fd;
  fd.reset(raw);
  fd.reset();

  EXPECT_FALSE(isFdOpen(raw));
  EXPECT_EQ(fd.get(), INVALID_SOCKET);
}

TEST(UniqueFdTest, reset_with_same_fd_is_a_noop)
{
  socket_t raw = makeFd();

  UniqueFd fd;
  fd.reset(raw);
  // Resetting with the descriptor we already hold must not close it.
  fd.reset(raw);

  EXPECT_TRUE(isFdOpen(raw));
  EXPECT_EQ(fd.get(), raw);
}

TEST(UniqueFdTest, destructor_closes_descriptor)
{
  socket_t raw = makeFd();
  {
    UniqueFd fd;
    fd.reset(raw);
    EXPECT_TRUE(isFdOpen(raw));
  }
  EXPECT_FALSE(isFdOpen(raw));
}

TEST(UniqueFdTest, destructor_on_invalid_is_safe)
{
  // Destroying a UniqueFd that never held a descriptor must not attempt to close anything.
  UniqueFd fd;
  SUCCEED();
}
#endif  // _WIN32

TEST(UniqueFdTest, is_non_copyable)
{
  static_assert(!std::is_copy_constructible<UniqueFd>::value, "UniqueFd must not be copy constructible");
  static_assert(!std::is_copy_assignable<UniqueFd>::value, "UniqueFd must not be copy assignable");
  SUCCEED();
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
