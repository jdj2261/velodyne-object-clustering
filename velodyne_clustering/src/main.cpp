/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data main
*/

#include <iostream>
#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include "velodyne_driver/driver.h"
#include "main.h"
#include <chrono>
#include <thread>
using namespace velodyne_driver;


int main(int argc, char** argv)
{
  VelodyneDriver dvr;
  while(true)
  {
    dvr.poll();
//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}

