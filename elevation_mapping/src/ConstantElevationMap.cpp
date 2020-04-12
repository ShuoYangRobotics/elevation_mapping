//
// Created by gzy on 4/6/20.
//

#include "elevation_mapping/ElevationMap.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_costmap_2d/grid_map_costmap_2d.hpp"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

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
    ROS_INFO("Map Initialized.");
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
  void pubCostCB(const ros::TimerEvent&) {
//    grid_map::GridMap& grid_map_raw = map_.getRawGridMap();
//    costmap_2d::Costmap2D outputCostMap;
//    grid_map::Costmap2DConverter<grid_map::GridMap> costmap2dConverter_;
//    costmap2dConverter_.initializeFromGridMap(map_.getRawGridMap(), outputCostMap);
//    costmap2dConverter_.setCostmap2DFromGridMap(map_.getRawGridMap(), "elevation", outputCostMap);
    nav_msgs::OccupancyGrid occupancyGrid;
    const float minHeight = 0, maxHeight = 0.1;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_.getRawGridMap(),
        "elevation", minHeight, maxHeight, occupancyGrid);
    pub_occupancy_.publish(occupancyGrid);
  }
  void run() {
    pub_elevation_timer_ = nh_.createTimer(ros::Duration(1.0/5),
                                           &ConstantElevationMap::pubElevationCB, this);
    pub_costmap_timer_ = nh_.createTimer(ros::Duration(1.0/5),
                                           &ConstantElevationMap::pubCostCB,
                                           this);
    pub_occupancy_ = nh_.advertise<nav_msgs::OccupancyGrid>
        ("map", 10);
    ros::spin();
  }
 private:
  ros::NodeHandle nh_;
  elevation_mapping::ElevationMap map_;
  ros::Timer pub_elevation_timer_;
  ros::Timer pub_costmap_timer_;
  ros::Publisher pub_occupancy_;
};


int main (int argc, char **argv) {
  ros::init(argc, argv, "elevation_map_server");
  ros::NodeHandle node_handle;
  ConstantElevationMap cem(node_handle);
  cem.run();
  return 0;
}
