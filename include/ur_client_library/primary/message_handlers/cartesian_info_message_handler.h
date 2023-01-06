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

#ifndef UR_CLIENT_LIBRARY_CARTESIAN_INFO_HANDLER_H_INCLUDED
#define UR_CLIENT_LIBRARY_CARTESIAN_INFO_HANDLER_H_INCLUDED

#include <ur_client_library/log.h>
#include <ur_client_library/primary/primary_package_handler.h>
#include <ur_client_library/primary/robot_state/cartesian_info.h>

namespace urcl
{
namespace primary_interface
{
class CartesianInfoHandler : public IPrimaryPackageHandler<CartesianInfo>
{
public:
  CartesianInfoHandler() = default;
  virtual ~CartesianInfoHandler() = default;

  /*!
   * \brief Actual worker function
   *
   * \param pkg package that should be handled
   */
  virtual void handle(CartesianInfo& pkg) override
  {
    // URCL_LOG_INFO("---CartesianInfo---\n%s", pkg.toString().c_str());
    data_.reset(new CartesianInfo(pkg));
  }

  /*!
   * \brief Get latest CartesianInfo
   *
   * \return latest CartesianInfo
   */
  virtual std::shared_ptr<CartesianInfo> getData()
  {
    if (data_ == nullptr)
      throw UrException("A CartesianInfo package has not been received yet");
    return data_;
  }

private:
  std::shared_ptr<CartesianInfo> data_;
};
}  // namespace primary_interface
}  // namespace urcl
#endif  // ifndef UR_CLIENT_LIBRARY_CARTESIAN_INFO_HANDLER_H_INCLUDED
