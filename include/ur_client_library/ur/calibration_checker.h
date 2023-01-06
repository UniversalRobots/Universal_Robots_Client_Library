// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-14
 *
 */
//----------------------------------------------------------------------
#ifndef UR_CLIENT_LIBRARY_UR_CALIBRATION_CHECKER_H_INCLUDED
#define UR_CLIENT_LIBRARY_UR_CALIBRATION_CHECKER_H_INCLUDED

#include <ur_client_library/primary/primary_package_handler.h>
#include <ur_client_library/primary/robot_state/kinematics_info.h>

namespace urcl
{
/*!
 * \brief The CalibrationChecker checks a received KinematicsInfo package against a registered calibration hash
 * value. This way we know whether the robot that sent the KinematicsInfo package matches the
 * expected calibration.
 */
class CalibrationChecker : public primary_interface::IPrimaryPackageHandler<primary_interface::KinematicsInfo>
{
public:
  /*!
   * \brief Creates a new CalibrationChecker object with an expected hash calculated from the used
   * kinematics.
   *
   * \param expected_hash The expected kinematics hash
   */
  CalibrationChecker(const std::string& expected_hash);
  virtual ~CalibrationChecker() = default;

  /*!
   * \brief Consumes a package, checking its hash if it is a KinematicsInfo package. If the hash
   * does not match the expected hash, an error is logged.
   *
   * \param kin_info The package to consume
   *
   * \returns True, if the package was consumed correctly
   */
  virtual void handle(primary_interface::KinematicsInfo& kin_info) override;

  /*!
   * \brief Used to make sure the calibration check is not performed several times.
   *
   * \returns True, if the calibration was already checked, false otherwise
   */
  bool isChecked()
  {
    return checked_;
  }

  /*!
   * \brief Returns whether the calibration check was successful.
   *
   * \returns True if the robot's calibration checksum matches the one given to the checker. False
   * if it doesn't match or the check was not yet performed.
   */
  bool checkSuccessful()
  {
    return matches_;
  }

  /*!
   * \brief Get latest KinematicsInfo
   *
   * \return latest KinematicsInfo
   */
  virtual std::shared_ptr<primary_interface::KinematicsInfo> getData()
  {
    if (data_ == nullptr)
      throw UrException("A KinematicsInfo package has not been received yet");
    return data_;
  }

  /*!
   * \brief Returns whether the calibration check was successful.
   *
   * \param expected_hash The hash expected to match the hash on the robot
   *
   * \returns True if the robot's expected_hash matches the one given to the checker. False
   * if it doesn't match or the check was not yet performed.
   */
  bool checkCalibration(const std::string& expected_hash);

private:
  std::string expected_hash_;
  bool checked_;
  bool matches_;
  std::shared_ptr<primary_interface::KinematicsInfo> data_;
};
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_UR_CALIBRATION_CHECKER_H_INCLUDED
