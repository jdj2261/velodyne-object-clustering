/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data main
*/

#include "main.h"

using namespace velodyne_driver;
using namespace velodyne_pcl;
int main(int argc, char** argv)
{
  show_help_consol(argc, argv);
  std::string address("192.168.1.21");
  std::string port("2368");
  std::string pcap;
  bool is_saved(false);
  set_param(argc, argv, address, port, pcap, is_saved);

  std::cout << "-ipadress : " << address << std::endl;
  std::cout << "-port : " << port << std::endl;
  std::cout << "-pcap : " << pcap << std::endl;
  std::cout << "-saveframes : " << is_saved << std::endl;

  VelodyneDriver dvr;
  pcl::visualization::PCLVisualizer::Ptr pcl_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  while(!pcl_viewer->wasStopped())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl_viewer->removeAllPointClouds();
    pcl_viewer->removeAllShapes();
    pcl_viewer->removeText3D();
    auto startTime = std::chrono::high_resolution_clock::now();
    dvr.poll(cloud);
    pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud, "test");
    pcl_viewer->spinOnce();
    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Main " << elapsedTime.count() << " milliseconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(elapsedTime.count()));
  }

  return 0;
}

