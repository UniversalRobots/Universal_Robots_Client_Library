#include <ur_client_library/primary/primary_client.h>
#include <thread>

int main()
{
  using namespace std::chrono_literals;
  auto notif = urcl::comm::INotifier();
  auto client = urcl::primary_interface::PrimaryClient("192.168.56.101", notif);
  client.start(10);
  std::cout << "Client connected" << std::endl;

  // Make sure the robot is running
  client.commandBrakeRelease();

  if (!client.safetyModeAllowsExecution())
  {
    std::cout << "Robot is not in a safety state where script execution is possible. Exiting." << std::endl;
    return 0;
  }

  const std::string fully_defined_script = R"""(
# This is a fully defined script function definition and all
# All comments in this script will be stripped before sending the script to the robot

# Any whitespace-only lines will also be removed
def example_fun():
  movej([0,-0.75,0,0,0,0])
  sleep(0.1)
  movel([0,0,-1.5,0,0,0], t=5)
end)""";

  if (client.sendScript(fully_defined_script))
  {
    // The function definition can also be omitted
    // A function name will then be auto generated
    client.sendScript(R"(textmsg("Successful program execution"))");
  }
  // A script-function name can also be passed to the method as well as whether it should be a function or secondary
  // program A timeout can also be given to limit the wait for the passed function to start. If timeout = 0, it will
  // wait indefinitely.
  client.sendScript(R"(textmsg("Named primary program"))", "cool_function_name", urcl::primary_interface::DEF,
                    std::chrono::milliseconds(2000));
  // There is however no feedback on secondary programs, so if will return successful as soon as the code is sent to the
  // robot
  client.sendScript(R"(textmsg("Named secondary program"))", "cool_secondary_name", urcl::primary_interface::SEC);
}