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

#include <ur_client_library/exceptions.h>
#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/rtde/rtde_client.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";

// RTDE recipes as argument lists, so this example needs no recipe files. RTDEClient also takes two
// filenames instead; see examples/rtde_client.cpp. The general purpose register ranges used here
// are the ones the RTDE guide reserves for external clients: bit registers 64..127, integer and
// double registers 24..47. Register fields need no companion "_mask" key.
const std::vector<std::string> INPUT_RECIPE = { "input_bit_register_64", "input_int_register_24",
                                                "input_double_register_24" };
const std::vector<std::string> OUTPUT_RECIPE = { "timestamp", "runtime_state", "output_bit_register_64",
                                                 "output_int_register_24", "output_double_register_24" };

// We write the inputs, the robot program below writes the outputs.
const std::string INPUT_BIT_REGISTER = "input_bit_register_64";
const std::string INPUT_INT_REGISTER = "input_int_register_24";
const std::string INPUT_DOUBLE_REGISTER = "input_double_register_24";
const std::string OUTPUT_BIT_REGISTER = "output_bit_register_64";
const std::string OUTPUT_INT_REGISTER = "output_int_register_24";
const std::string OUTPUT_DOUBLE_REGISTER = "output_double_register_24";

// All three values we send are derived from the cycle counter, so the integer the robot returns
// identifies which cycle an answer belongs to.
const double SINE_INCREMENT = 0.01;  // rad per cycle

// Robot program processing the general purpose inputs and writing the results to the outputs.
// Input registers cannot be written from URScript and output registers cannot be written through
// RTDE, so getting values back requires a program on the robot. The program does not copy the
// values: it inverts the bit, adds one to the integer and negates the double. RTDE also offers the
// input registers as outputs, so a copy would be indistinguishable from that read-back, while a
// value satisfying this relation can only have been produced by this program. sync() runs the loop
// once per control cycle.
const std::string MIRROR_PROGRAM = R"(def rtde_register_mirror():
  while (True):
    write_output_boolean_register(64, not read_input_boolean_register(64))
    write_output_integer_register(24, read_input_integer_register(24) + 1)
    write_output_float_register(24, -1.0 * read_input_float_register(24))
    sync()
  end
end)";

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

  // Start the robot program that processes the registers
  primary_interface::PrimaryClient primary_client(robot_ip, notifier);
  primary_client.start();
  try
  {
    primary_client.commandBrakeRelease();
  }
  catch (const UrException& e)
  {
    URCL_LOG_WARN("Could not release the brakes: %s", e.what());
  }
  if (!primary_client.sendScript(MIRROR_PROGRAM))
  {
    URCL_LOG_WARN("Could not upload the register-processing program. Output registers will stay at "
                  "zero until a matching program is running on the robot.");
  }
  // The program keeps running until we stop it later.

  // RTDE client at the robot's maximum frequency
  rtde_interface::RTDEClient my_client(robot_ip, notifier, OUTPUT_RECIPE, INPUT_RECIPE);
  my_client.init();
  URCL_LOG_INFO("RTDE target frequency: %f Hz", my_client.getTargetFrequency());

  // An input package carrying the data types the robot reported for the input recipe. Those types
  // are only known once the RTDE handshake has run, which is why this is created after init().
  rtde_interface::DataPackage input_pkg = my_client.createInputDataPackage();
  // The output package is still untyped; the first read applies the robot's types to it in place.
  auto output_pkg = std::make_unique<rtde_interface::DataPackage>(my_client.getOutputRecipe());

  my_client.start(false);

  int32_t counter = 0;
  size_t verified = 0;
  size_t mismatches = 0;
  int32_t last_lag_cycles = 0;
  auto start_time = std::chrono::steady_clock::now();
  auto last_print = start_time;

  while (second_to_run <= 0 ||
         std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() <
             second_to_run)
  {
    // The blocking read is this loop's clock
    if (!my_client.getDataPackageBlocking(output_pkg))
    {
      URCL_LOG_ERROR("Could not get a fresh data package from the robot.");
      return 1;
    }
    const auto now = std::chrono::steady_clock::now();

    // Reading what the robot made of the previous package
    bool echoed_bit = false;
    int32_t echoed_int = 0;
    double echoed_double = 0.0;
    uint32_t runtime_state = 0;
    if (!output_pkg->getData(OUTPUT_BIT_REGISTER, echoed_bit) ||
        !output_pkg->getData(OUTPUT_INT_REGISTER, echoed_int) ||
        !output_pkg->getData(OUTPUT_DOUBLE_REGISTER, echoed_double) ||
        !output_pkg->getData("runtime_state", runtime_state))
    {
      URCL_LOG_ERROR("Could not read the output registers from the received package.");
      return 1;
    }

    bool verified_this_cycle = false;
    if (echoed_int > 1)
    {
      const int32_t origin = echoed_int - 1;  // the counter value the robot processed
      const bool expected_bit = !((origin % 2) == 0);
      const double expected_double = -std::sin(origin * SINE_INCREMENT);
      last_lag_cycles = counter - origin;
      // The robot's double register is a 64-bit value, so the negated sine has to come back bit
      // for bit. Together with the inverted bit that is the proof the robot processed this cycle.
      if (echoed_bit == expected_bit && echoed_double == expected_double)
      {
        ++verified;
        verified_this_cycle = true;
      }
      else
      {
        ++mismatches;
      }
    }

    // Writing several general purpose inputs in one package
    ++counter;
    const bool sent_bit = (counter % 2) == 0;
    const double sent_double = std::sin(counter * SINE_INCREMENT);
    bool write_ok = input_pkg.setData(INPUT_BIT_REGISTER, sent_bit);
    write_ok = write_ok && input_pkg.setData(INPUT_INT_REGISTER, counter);
    write_ok = write_ok && input_pkg.setData(INPUT_DOUBLE_REGISTER, sent_double);
    if (!write_ok || !my_client.getWriter().sendPackage(input_pkg))
    {
      URCL_LOG_ERROR("Sending RTDE data failed.");
      return 1;
    }

    if (now - last_print >= std::chrono::seconds(1))
    {
      const double elapsed_s = std::chrono::duration<double>(now - start_time).count();
      const double measured_hz = elapsed_s > 0.0 ? static_cast<double>(counter) / elapsed_s : 0.0;
      const bool program_playing =
          static_cast<rtde_interface::RUNTIME_STATE>(runtime_state) == rtde_interface::RUNTIME_STATE::PLAYING;
      std::cout << "sent: bit=" << sent_bit << " int=" << counter << " double=" << sent_double
                << " | robot: bit=" << echoed_bit << " int=" << echoed_int << " double=" << echoed_double
                << " | verified=" << verified_this_cycle << " lag_cycles=" << last_lag_cycles << " freq=" << measured_hz
                << " Hz playing=" << program_playing << std::endl;
      if (echoed_int == 0)
      {
        std::cout << "No processed values yet. Is the register-processing program running on the robot?" << std::endl;
      }
      last_print = now;
    }
  }

  const double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  const double average_hz = elapsed_s > 0.0 ? static_cast<double>(counter) / elapsed_s : 0.0;
  URCL_LOG_INFO("Cycles: %d, average frequency: %f Hz, verified: %zu, mismatches: %zu, last lag: %d cycles", counter,
                average_hz, verified, mismatches, last_lag_cycles);

  // Reset the input registers before leaving
  input_pkg.setData(INPUT_BIT_REGISTER, false);
  input_pkg.setData(INPUT_INT_REGISTER, static_cast<int32_t>(0));
  input_pkg.setData(INPUT_DOUBLE_REGISTER, 0.0);
  my_client.getWriter().sendPackage(input_pkg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  try
  {
    primary_client.commandStop(false);
  }
  catch (const UrException& e)
  {
    URCL_LOG_WARN("Could not stop the robot program: %s", e.what());
  }

  return 0;
}
