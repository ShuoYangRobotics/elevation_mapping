//
// Created by gzy on 5/27/20.
//

#ifndef SRC_ELEVATION_MAPPING_ELEVATION_MAPPING_DEMOS_SRC_NORMAL_FILTER_H_
#define SRC_ELEVATION_MAPPING_ELEVATION_MAPPING_DEMOS_SRC_NORMAL_FILTER_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.h>

#include <string>

class NormalFilter {
 public:
  NormalFilter(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle), filterChain_("grid_map::GridMap") {

    subscriber_ = nodeHandle_.subscribe("/elevation_mapping/elevation_map", 1, &NormalFilter::callback, this);
    publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/normal_map", 1, true);
    filterChainParametersName_ = std::string("grid_map_filters");

    // Setup filter chain.
    if (!filterChain_.configure(filterChainParametersName_, nodeHandle_)) {
      ROS_ERROR("Could not configure the filter chain!");
      return;
    }
  }

  virtual ~NormalFilter() {}

  void callback(const grid_map_msgs::GridMap& message) {
    // Convert message to map.
    grid_map::GridMap inputMap;
    grid_map::GridMapRosConverter::fromMessage(message, inputMap);

    // Apply filter chain.
    grid_map::GridMap outputMap;
    if (!filterChain_.update(inputMap, outputMap)) {
      ROS_ERROR("Could not update the grid map filter chain!");
      return;
    }

    // Publish filtered output grid map.
    grid_map_msgs::GridMap outputMessage;
    grid_map::GridMapRosConverter::toMessage(outputMap, outputMessage);
    publisher_.publish(outputMessage);
  }

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  filters::FilterChain<grid_map::GridMap> filterChain_;
  std::string filterChainParametersName_;
};

#endif //SRC_ELEVATION_MAPPING_ELEVATION_MAPPING_DEMOS_SRC_NORMAL_FILTER_H_
