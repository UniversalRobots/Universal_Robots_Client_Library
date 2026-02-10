#include "fake_rtde_server.h"
#include <ur_client_library/log.h>

int main()
{
  urcl::RTDEServer server(30004);

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
