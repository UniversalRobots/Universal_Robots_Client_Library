#include <ur_client_library/primary/primary_client.h>
#include <thread>

int main()
{
  auto notif = urcl::comm::INotifier();
  auto client = urcl::primary_interface::PrimaryClient("192.168.56.101", notif);
  client.start(10);
  // std::cout << "Robot mode: " << int(client.getRobotMode()) << std::endl;
  // client.commandPowerOff();
  // std::cout << "Robot mode: " << int(client.getRobotMode()) << std::endl;
  // client.commandBrakeRelease();
  // std::cout << "Robot mode: " << int(client.getRobotMode()) << std::endl;
  auto s = "textmsg(\"hello\") \n #hhelllooe \n \n #helloe \n #jejfef";
  client.sendScript(s, "hello_world", urcl::primary_interface::ScriptTypes::SEC);

  client.sendScript(s, "", urcl::primary_interface::ScriptTypes::DEF);
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(2000ms);
}