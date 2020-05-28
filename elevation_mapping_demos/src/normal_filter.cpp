//
// Created by gzy on 5/27/20.
//

#include "normal_filter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "normal_filter_node");
  ros::NodeHandle nodeHandle("~");

  NormalFilter nf(nodeHandle);

  ros::spin();
  return 0;
}