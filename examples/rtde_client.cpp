// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-08-12
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/rtde/rtde_client.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <ctime>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a better configuration
// system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

// Preallocation of string to avoid allocation in main loop
const std::string TARGET_SPEED_FRACTION = "target_speed_fraction";

void printFraction(const double fraction, const std::string& label, const size_t width = 20)
{
  std::cout << "\r" << label << ": [";
  for (size_t i = 0; i < std::ceil(fraction * width); i++)
  {
    std::cout << "#";
  }
  for (size_t i = 0; i < std::floor((1.0 - fraction) * width); i++)
  {
    std::cout << "-";
  }
  std::cout << "]" << std::flush;
}

int main(int argc, char* argv[])
{
  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // Parse how may seconds to run
  int second_to_run = -1;
  if (argc > 2)
  {
    second_to_run = std::stoi(argv[2]);
  }

  comm::INotifier notifier;
  const double rtde_frequency = 50;  // Hz
  rtde_interface::RTDEClient my_client(robot_ip, notifier, OUTPUT_RECIPE, INPUT_RECIPE, rtde_frequency);
  my_client.init();

  // We will use the speed_slider_fraction as an example how to write to RTDE
  double speed_slider_fraction = 1.0;
  double target_speed_fraction = 1.0;
  double speed_slider_increment = 0.01;

  std::unique_ptr<rtde_interface::DataPackage> data_pkg =
      std::make_unique<rtde_interface::DataPackage>(my_client.getOutputRecipe());
  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  my_client.start(false);

  auto start_time = std::chrono::steady_clock::now();
  while (second_to_run <= 0 ||
         std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() <
             second_to_run)
  {
    // Wait for a DataPackage. In a real-world application this thread should be scheduled with real-time priority in
    // order to ensure that this is called in time.
    bool success = my_client.getDataPackageBlocking(data_pkg);
    if (success)
    {
      // Data fields in the data package are accessed by their name. Only names present in the
      // output recipe can be accessed. Otherwise this function will return false.
      // We preallocated the string TARGET_SPEED_FRACTION to avoid allocations in the main loop.
      data_pkg->getData(TARGET_SPEED_FRACTION, target_speed_fraction);
      printFraction(target_speed_fraction, TARGET_SPEED_FRACTION);
    }
    else
    {
      // The client isn't connected properly anymore / doesn't receive any data anymore. Stop the
      // program.
      std::cout << "Could not get fresh data package from robot" << std::endl;
      return 1;
    }

    // Change the speed slider so that it will move between 0 and 1 all the time. This is for
    // demonstration purposes only and gains no real value.
    if (speed_slider_increment > 0)
    {
      if (speed_slider_fraction + speed_slider_increment > 1.0)
      {
        speed_slider_increment *= -1;
      }
    }
    else if (speed_slider_fraction + speed_slider_increment < 0.0)
    {
      speed_slider_increment *= -1;
    }
    speed_slider_fraction += speed_slider_increment;

    if (!my_client.getWriter().sendSpeedSlider(speed_slider_fraction))
    {
      // This will happen for example, when the required keys are not configured inside the input
      // recipe.
      std::cout << "\033[1;31mSending RTDE data failed." << "\033[0m\n" << std::endl;
      return 1;
    }
  }

  // Resetting the speedslider back to 100%
  my_client.getWriter().sendSpeedSlider(1);

  URCL_LOG_INFO("Exiting RTDE read/write example.");

  return 0;
}
