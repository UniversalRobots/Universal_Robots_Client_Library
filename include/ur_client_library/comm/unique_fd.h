/*
 * Copyright 2026, Universal Robots A/S
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
