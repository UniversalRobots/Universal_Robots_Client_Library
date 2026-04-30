#include <ur_client_library/primary/primary_client.h>
#include <thread>
#include <chrono>

using namespace urcl;

std::string DEFAULT_ROBOT_IP = "192.168.56.101";

int main(int argc, char* argv[])
{
  // Set the loglevel to info to print info logs
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }
  auto notif = comm::INotifier();
  auto client = primary_interface::PrimaryClient(robot_ip, notif);
  client.start(10);
  std::cout << "Client connected" << std::endl;

  // --------------- INITIALIZATION END -------------------

  // Make sure the robot is running
  client.commandBrakeRelease();

  if (!client.safetyModeAllowsExecution())
  {
    std::cout << "Robot is not in a safety state where script execution is possible. Exiting." << std::endl;
    return 0;
  }

  // The sendScriptBlocking accepts script code, and will return true or false,
  // depending on whether the script is successfully executed
  const std::string fully_defined_script = R"""(
# This is a fully defined script, function definition and all
# All comments in this script will be stripped before sending the script to the robot

# Any whitespace-only lines will also be removed
def example_fun():
  movej([0,-0.75,0,0,0,0])
  sleep(0.1)
  movel([0,0,-1.5,0,0,0], t=5)
end)""";

  if (client.sendScriptBlocking(fully_defined_script))
  {
    // The function definition can also be omitted
    // A function name will then be auto generated
    client.sendScriptBlocking(R"(textmsg("Successful program execution"))");
  }
  // A script-function name can also be passed to the method
  // A timeout can also be given to limit the wait for the passed function to start. If timeout = 0, it will
  // wait indefinitely.
  client.sendScriptBlocking(R"(textmsg("hello"))", "cool_function_name", std::chrono::milliseconds(0));
  // There is no feedback on secondary programs, so it will return successful as soon as the script is sent to the
  // robot (Behavior is the same the sendScript function, except that robot state is checked before script is sent)
  // Note that secondary scripts have to be "fully defined" by the user.
  std::string secondary_script = R"(
sec sec_script():
  textmsg("Named secondary program")
end
)";
  client.sendScriptBlocking(secondary_script);
}