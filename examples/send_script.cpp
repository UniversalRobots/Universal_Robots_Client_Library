#include <ur_client_library/primary/primary_client.h>
#include <chrono>

using namespace urcl;

std::string g_DEFAULT_ROBOT_IP = "192.168.56.101";

int main(int argc, char* argv[])
{
  // Set the loglevel to info to print info logs
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = g_DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }
  auto notif = comm::INotifier();
  auto client = primary_interface::PrimaryClient(robot_ip, notif);
  client.start(10);

  // --------------- INITIALIZATION END -------------------

  // Make sure the robot is running
  client.commandBrakeRelease();

  if (!client.safetyModeAllowsExecution())
  {
    URCL_LOG_ERROR("Robot is not in a safety state where script execution is possible. Exiting.");
    return 1;
  }

  // The sendScriptBlocking accepts script code, and will return true or false,
  // depending on whether the script is successfully executed
  const std::string fully_defined_script = R"""(
# This is a fully defined script, function definition and all
# All comments in this script will be stripped before sending the script to the robot

# Any whitespace-only lines will also be removed
def example_fun():
  movej([0,-1.2,1.2,-0.1,1.57,0])
  sleep(0.1)
  current_pose = get_target_tcp_pose()
  relative_move = p[0,-0.1,0,0,0,0]
  movel(pose_trans(current_pose, relative_move), t=1)
end)""";
  client.sendScriptBlocking(fully_defined_script);
  // The function definition can also be omitted
  // A function name will then be auto generated
  client.sendScriptBlocking(R"(textmsg("Successful program execution"))");
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

  // Sending wrong script code will result in an exception with a clear explanation
  const std::string bad_script_code = R"""(
def bad_code():
  current_pose = get_target_tcp_pose()
  movel(current_pos) # note pose vs pos
end)""";
  URCL_LOG_INFO("Sending bad script code...");
  try
  {
    client.sendScriptBlocking(bad_script_code);
  }
  catch (const RobotRuntimeException& exc)
  {
    URCL_LOG_INFO("Caught expected runtime exception from sendScriptBlocking");
    URCL_LOG_INFO(exc.what());
  }
  catch (const UrException& exc)
  {
    URCL_LOG_ERROR("Caught unexpected exception from sendScriptBlocking");
    URCL_LOG_ERROR(exc.what());
  }

  // We can also send script code without any checks
  URCL_LOG_INFO("Executing motion without feedback");
  client.sendScript("movej([0.1,-0.9,0.9,0,0,0])");
  // But we won't know when that is done or even if our code was correct.
  // E.g. sending the bad script here will not give us any information
  // The return value will only tell us that the script code has been sent to the robot.
  URCL_LOG_INFO("Sending bad script code without feedback...");
  bool success = client.sendScript(bad_script_code);
  {
    std::stringstream ss;
    ss << "Bad code sent to robot successfully? " << std::boolalpha << success;
    URCL_LOG_INFO("%s", ss.str().c_str());
  }
}
