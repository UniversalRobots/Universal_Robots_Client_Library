// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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

#include <chrono>
#include <cstdio>
#include <mutex>

#include "ur_client_library/example_robot_wrapper.h"
#include "ur_client_library/types.h"

// Platform-specific implementation of getChar()
#if defined(_WIN32) || defined(_WIN64)
#  include <conio.h>
char getChar()
{
  char ch = '\0';
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 1)
  {
    if (_kbhit())
    {
      ch = _getch();
    }
  }
  return ch;
}
#else
#  include <termios.h>
#  include <unistd.h>
#  include <fcntl.h>
#  include <sys/select.h>
char getChar()
{
  termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  fd_set set;
  FD_ZERO(&set);
  FD_SET(STDIN_FILENO, &set);

  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // Wait for input with timeout
  int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
  if (rv > 0)
  {
    std::ignore = read(STDIN_FILENO, &ch, 1);
  }
  else
  {
    ch = '\0';  // No input
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  return ch;
}
#endif

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_only_ft.txt";

std::unique_ptr<urcl::ExampleRobotWrapper> g_my_robot;
std::atomic<bool> g_RUNNING = true;
urcl::vector6d_t g_FT_VEC{ 0, 0, 0, 0, 0, 0 };
std::mutex g_FT_VEC_MUTEX;

using namespace urcl;

void ftInputTui()
{
  const std::string instructions = "Press x,y,z to increase the respective translational axes, a,b,c to increase the "
                                   "rotational axes, 0 for reset and q for quit.";
  urcl::vector6d_t local_ft_vec = g_FT_VEC;
  while (g_RUNNING)
  {
    std::cout << instructions << std::endl;
    char ch = getChar();

    if (ch == '\0')
    {
      continue;
    }
    std::cout << "<'" << ch << "' pressed>" << std::endl;

    switch (ch)
    {
      case 'x':
      {
        local_ft_vec[0] += 10;
        break;
      }
      case 'y':
      {
        local_ft_vec[1] += 10;
        break;
      }
      case 'z':
      {
        local_ft_vec[2] += 10;
        break;
      }
      case 'a':
      {
        local_ft_vec[3] += 10;
        break;
      }
      case 'b':
      {
        local_ft_vec[4] += 10;
        break;
      }
      case 'c':
      {
        local_ft_vec[5] += 10;
        break;
      }
      case 'X':
      {
        local_ft_vec[0] -= 10;
        break;
      }
      case 'Y':
      {
        local_ft_vec[1] -= 10;
        break;
      }
      case 'Z':
      {
        local_ft_vec[2] -= 10;
        break;
      }
      case 'A':
      {
        local_ft_vec[3] -= 10;
        break;
      }
      case 'B':
      {
        local_ft_vec[4] -= 10;
        break;
      }
      case 'C':
      {
        local_ft_vec[5] -= 10;
        break;
      }
      case '0':
      {
        local_ft_vec = { 0, 0, 0, 0, 0, 0 };
        break;
      }
      case 'q':
      {
        g_RUNNING = false;
      }
      default:
        break;
    }
    if (g_RUNNING)
    {
      std::scoped_lock<std::mutex> lock(g_FT_VEC_MUTEX);
      g_FT_VEC = local_ft_vec;
    }
    std::cout << "Artificial FT input: " << local_ft_vec << std::endl;
  }
  std::cout << "FT input TUI thread finished." << std::endl;
}

void rtdeWorker(const int second_to_run)
{
  g_my_robot->startRTDECommununication(false);

  vector6d_t actual_tcp_force;
  auto start_time = std::chrono::steady_clock::now();
  while (g_RUNNING)
  {
    urcl::vector6d_t local_ft_vec = g_FT_VEC;
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_robot->getUrDriver()->getDataPackage();
    if (data_pkg)
    {
      // Data fields in the data package are accessed by their name. Only names present in the
      // output recipe can be accessed. Otherwise this function will return false.
      data_pkg->getData("actual_TCP_force", actual_tcp_force);
      // Throttle output to once per second
      if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() %
              1000 <
          2)
        std::cout << "Force-torque reported by robot: " << actual_tcp_force << std::endl;
    }
    else
    {
      // The client isn't connected properly anymore / doesn't receive any data anymore. Stop the
      // program.
      std::cout << "Could not get fresh data package from robot" << std::endl;
      return;
    }

    if (g_FT_VEC_MUTEX.try_lock())
    {
      local_ft_vec = g_FT_VEC;
      g_FT_VEC_MUTEX.unlock();
    }
    if (!g_my_robot->getUrDriver()->getRTDEWriter().sendExternalForceTorque(local_ft_vec))
    {
      // This will happen for example, when the required keys are not configured inside the input
      // recipe.
      std::cout << "\033[1;31mSending RTDE data failed." << "\033[0m\n" << std::endl;
      return;
    }
    if (second_to_run > 0)
    {
      g_RUNNING =
          std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() <
          second_to_run;
    }
  }
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

  g_my_robot = std::make_unique<urcl::ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, true);
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }

  // Enable using the force-torque input through RTDE in the robot controller
  g_my_robot->getUrDriver()->ftRtdeInputEnable(true);
  // The RTDE thread sends the force-torque data to the robot and receives the wrench data from the
  // robot.
  std::thread rtde_thread(rtdeWorker, second_to_run);

  // Modify the artificial force-torque input through keyboard input
  ftInputTui();

  g_RUNNING = false;
  if (rtde_thread.joinable())
  {
    rtde_thread.join();
  }

  return 0;
}
