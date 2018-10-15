/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw RSLIDAR LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include "std_msgs/String.h"
#include <sstream>
#include <thread>

std::string userInput = "0 360 0 16";
bool running = true;

void userInputThread() {
  std::string buffer;
  while(::running) {
    std::getline(std::cin, buffer);
    if (::userInput == "q") {
      ::userInput = "0 0 0 0";
      ::running = false;
    } else {
      ::userInput = buffer;
    }
  }
}

void mytread() {
  ros::NodeHandle node2;
  ros::Publisher chatter_pub = node2.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  std::thread user_thread(userInputThread);

  while(::running) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << ::userInput;
    msg.data=ss.str();
    chatter_pub.publish(msg);
    loop_rate.sleep();
  }
  user_thread.join();
}

/** Main node entry point. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  rslidar_pointcloud::Convert conv(node, priv_nh);
  std::thread t(mytread);

  // handle callbacks until shut down
  ros::spin();
  t.join();


  return 0;
}
