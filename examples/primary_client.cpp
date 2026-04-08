#include <ur_client_library/primary/primary_client.h>
#include <thread>

int main()
{
  using namespace std::chrono_literals;
  auto notif = urcl::comm::INotifier();
  auto client = urcl::primary_interface::PrimaryClient("192.168.56.101", notif);
  client.start(10);
  std::cout << "Client connected" << std::endl;
  // std::this_thread::sleep_for(3000ms);
  //  std::cout << "Robot mode: " << int(client.getRobotMode()) << std::endl;
  //  client.commandPowerOff();
  //  std::cout << "Robot mode: " << int(client.getRobotMode()) << std::endl;
  //  client.commandBrakeRelease();
  //  std::cout << "Robot mode: " << int(client.getRobotMode()) << std::endl;
  auto s = "movej([1,0,0,0,0,0], t=5)";
  client.sendScript(s, "");

  // client.sendScript(s, "", urcl::primary_interface::ScriptTypes::DEF);

  // std::this_thread::sleep_for(2000ms);
}