/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "iostream"

std::string angle_laser_selection;

namespace rslidar_pointcloud
{
std::string model;

void Convert::chatterCallback(const std_msgs::String::ConstPtr& msg) {
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  ::angle_laser_selection = msg->data.c_str();
}

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new rslidar_rawdata::RawData())
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("RS16"));

  // advertise output point cloud (before subscribing to input data)
  output_ = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe to rslidarScan packets
  rslidar_scan_ = node.subscribe("rslidar_packets", 10, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));
  message = node.subscribe("chatter", 1000, &Convert::chatterCallback, (Convert*)this, ros::TransportHints().tcpNoDelay(true));
  std::cout << "done subscribing" << std::endl;
}

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model == "RS32")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }

  // process each packet provided by the driver

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints, ::angle_laser_selection);
  }
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);

  // print time
  // std::cout << outMsg.header.stamp << std::endl;

  output_.publish(outMsg);
}
}  // namespace rslidar_pointcloud
