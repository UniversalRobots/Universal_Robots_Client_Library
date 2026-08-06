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

#pragma once

#include <atomic>

#include "ur_client_library/comm/socket_t.h"

namespace urcl
{
namespace comm
{
/*!
 * \brief RAII owner for a single socket descriptor.
 *
 * Owns the lifecycle of exactly one socket descriptor, closing it on destruction. To replace the
 * held descriptor, call reset(new_fd), which closes the currently held descriptor (if any) before
 * adopting the new one. This makes it impossible to accidentally leak a descriptor by overwriting
 * the handle, e.g. when retrying a connection attempt.
 *
 * The descriptor is stored atomically, so a concurrent close/reset (as issued by a deliberate
 * disconnect() from another thread) is safe against the connect path reading or replacing it. This
 * matches the concurrency model TCPSocket relied on when the descriptor was a plain
 * std::atomic<socket_t>.
 *
 * The class is non-copyable, mirroring std::unique_ptr's unique-ownership semantics.
 */
class UniqueFd
{
public:
  UniqueFd() = default;
  ~UniqueFd()
  {
    reset();
  }

  UniqueFd(const UniqueFd&) = delete;
  UniqueFd& operator=(const UniqueFd&) = delete;

  /*!
   * \brief Closes the currently held descriptor (if valid) and adopts new_fd.
   *
   * \param new_fd The descriptor to adopt. Defaults to INVALID_SOCKET, which simply closes the
   * currently held descriptor.
   */
  void reset(socket_t new_fd = INVALID_SOCKET)
  {
    socket_t old_fd = fd_.exchange(new_fd);
    if (old_fd >= 0 && old_fd != new_fd)
    {
      ::ur_close(old_fd);
    }
  }

  /*!
   * \brief Returns the currently held descriptor without transferring ownership.
   *
   * \returns The held descriptor, or INVALID_SOCKET if none is held.
   */
  socket_t get() const
  {
    return fd_.load();
  }

private:
  std::atomic<socket_t> fd_{ INVALID_SOCKET };
};
}  // namespace comm
}  // namespace urcl
