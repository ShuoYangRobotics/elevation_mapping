//
// Created by gzy on 4/6/20.
//

#include "elevation_mapping/ElevationMap.hpp"
#include "grid_map_core/GridMap.hpp"

#include <ros/ros.h>

class ConstantElevationMap {
 public:
  ConstantElevationMap(ros::NodeHandle& nodeHandle)
  : nh_(nodeHandle), map_(nodeHandle)
  {
    const std::string mapFrameId_("map");
    map_.setFrameId(mapFrameId_);
    map_.setGeometry(grid_map::Length(2, 2), 0.01, grid_map::Position(0.0,
        0.0));
    // Set Zero for both layers: [elevation] and [variance].
    map_.setRawSubmapHeight(0, 2, 2, 0);
  }
  void update() {
    ROS_INFO("Update the raw map.");
    grid_map::GridMap& grid_map_raw = map_.getRawGridMap();
    ros::Time time = ros::Time::now();
    for (grid_map::GridMapIterator it(grid_map_raw); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      grid_map_raw.getPosition(*it, position);
      grid_map_raw.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
      grid_map_raw.at("variance", *it) = 0;
    }
  }
  void pubElevationCB(const ros::TimerEvent&) {
    update();
    map_.publishRawElevationMap();
    map_.publishFusedElevationMap();
  }
  void run() {
    pub_elevation_timer_ = nh_.createTimer(ros::Duration(1.0/5),
                                           &ConstantElevationMap::pubElevationCB, this);
    ros::spin();
  }
 private:
  ros::NodeHandle nh_;
  elevation_mapping::ElevationMap map_;
  ros::Timer pub_elevation_timer_;
};


int main (int argc, char **argv) {
  ros::init(argc, argv, "elevation_map_server");
  ros::NodeHandle node_handle;
  ConstantElevationMap cem(node_handle);
  cem.run();
  return 0;
}
