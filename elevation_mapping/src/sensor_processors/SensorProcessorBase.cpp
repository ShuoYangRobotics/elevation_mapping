/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

//PCL
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

//TF
#include <tf_conversions/tf_eigen.h>

// STL
#include <limits>
#include <math.h>
#include <vector>

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : nodeHandle_(nodeHandle),
      transformListener_(transformListener),
      ignorePointsUpperThreshold_(std::numeric_limits<double>::infinity()),
      ignorePointsLowerThreshold_(-std::numeric_limits<double>::infinity())
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  transformationSensorToMap_.setIdentity();
  transformListenerTimeout_.fromSec(1.0);
}

SensorProcessorBase::~SensorProcessorBase() {}

bool SensorProcessorBase::readParameters()
{
  nodeHandle_.param("sensor_frame_id", sensorFrameId_, std::string("/velodyne")); // TODO Fail if parameters are not found.
  nodeHandle_.param("robot_base_frame_id", robotBaseFrameId_, std::string("/base_link"));
  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("/world"));

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  transformListenerTimeout_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!transformListenerTimeout_.isZero());

  nodeHandle_.param("sensor_processor/ignore_points_above", ignorePointsUpperThreshold_, std::numeric_limits<double>::infinity());
  nodeHandle_.param("sensor_processor/ignore_points_below", ignorePointsLowerThreshold_, -std::numeric_limits<double>::infinity());

  nodeHandle_.param("sensor_processor/apply_voxelgrid_filter", applyVoxelGridFilter_, false);
  nodeHandle_.param("sensor_processor/apply_leg_box", applyLegBoxFilter_, true);
  nodeHandle_.param("sensor_processor/voxelgrid_filter_size", sensorParameters_["voxelgrid_filter_size"], 0.0);
  return true;
}

bool SensorProcessorBase::process(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudMapFrame,
    Eigen::VectorXf& variances)
{
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloudInput->header.stamp);
  if (!updateTransformations(timeStamp)) return false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudSensorFrame(new pcl::PointCloud<pcl::PointXYZRGB>);
  transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);
  filterPointCloud(pointCloudSensorFrame);
  filterPointCloudSensorType(pointCloudSensorFrame);

  if (!transformPointCloud(pointCloudSensorFrame, pointCloudMapFrame, mapFrameId_)) return false;  

  // remove leg
  // if (pointCloudInput->header.frame_id == "camera_downward_depth_optical_frame") {
    // should tranform it to map and then do the compare
    filterPointCloudLegBox(pointCloudMapFrame);
    // transform filtered map points back to sensor so their size align with each other
    if (!transformPointCloud(pointCloudMapFrame, pointCloudSensorFrame, sensorFrameId_)) return false; 
  // }


  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);
  if (!computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances)) return false;

  return true;
}

bool SensorProcessorBase::updateTransformations(const ros::Time& timeStamp)
{
  try {
    transformListener_.waitForTransform(sensorFrameId_, mapFrameId_, timeStamp, ros::Duration(1.0));

    tf::StampedTransform transformTf;
    transformListener_.lookupTransform(mapFrameId_, sensorFrameId_, timeStamp, transformTf);
    poseTFToEigen(transformTf, transformationSensorToMap_);

    transformListener_.lookupTransform(robotBaseFrameId_, sensorFrameId_, timeStamp, transformTf);  // TODO Why wrong direction?
    Eigen::Affine3d transform;
    poseTFToEigen(transformTf, transform);
    rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
    translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

    transformListener_.lookupTransform(mapFrameId_, robotBaseFrameId_, timeStamp, transformTf);  // TODO Why wrong direction?
    poseTFToEigen(transformTf, transform);
    rotationMapToBase_.setMatrix(transform.rotation().matrix());
    translationMapToBaseInMapFrame_.toImplementation() = transform.translation();

    return true;
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SensorProcessorBase::transformPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTransformed,
    const std::string& targetFrame)
{
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloud->header.stamp);
  const std::string inputFrameId(pointCloud->header.frame_id);

  tf::StampedTransform transformTf;
  try {
    transformListener_.waitForTransform(targetFrame, inputFrameId, timeStamp, ros::Duration(1.0));
    transformListener_.lookupTransform(targetFrame, inputFrameId, timeStamp, transformTf);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Eigen::Affine3d transform;
  poseTFToEigen(transformTf, transform);
  pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
  pointCloudTransformed->header.frame_id = targetFrame;

  ROS_DEBUG_THROTTLE(5, "Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
      ros::Time(pointCloudTransformed->header.stamp).toSec());
  return true;
}

void SensorProcessorBase::removePointsOutsideLimits(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr reference, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& pointClouds)
{
  if (!std::isfinite(ignorePointsLowerThreshold_) && !std::isfinite(ignorePointsUpperThreshold_)) return;
  ROS_DEBUG("Limiting point cloud to the height interval of [%f, %f] relative to the robot base.", ignorePointsLowerThreshold_, ignorePointsUpperThreshold_);

  pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z"); // TODO: Should this be configurable?
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndeces(new std::vector<int>);
  passThroughFilter.filter(*insideIndeces);

  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<pcl::PointXYZRGB> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndeces);
    pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }

  ROS_DEBUG("removePointsOutsideLimits() reduced point cloud to %i points.", (int) pointClouds[0]->size());
}

bool SensorProcessorBase::filterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;

  // remove nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices);
  tempPointCloud.is_dense = true;
  pointCloud->swap(tempPointCloud);

  // reduce points using VoxelGrid filter
  if(applyVoxelGridFilter_) {
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (pointCloud);
    double filter_size = sensorParameters_.at("voxelgrid_filter_size");
    sor.setLeafSize (filter_size, filter_size, filter_size);
    sor.filter (tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }
  ROS_DEBUG_THROTTLE(2, "cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
  return true;
}

bool SensorProcessorBase::filterPointCloudSensorType(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) {
    return true;
}

bool SensorProcessorBase::filterPointCloudLegBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) {

  // remove points close to leg
  if (applyLegBoxFilter_) {
    // define two boxes around foot0 and foot1
    Eigen::Affine3d foot0_pose_in_map, foot1_pose_in_map;
    try {  
      ros::Time timeStamp;
      tf::StampedTransform transformTf;
      timeStamp.fromNSec(1000 * pointCloud->header.stamp);


      transformListener_.waitForTransform(mapFrameId_, "foot_link0", timeStamp, ros::Duration(1.0));
      transformListener_.lookupTransform(mapFrameId_, "foot_link0", timeStamp, transformTf);
      poseTFToEigen(transformTf, foot0_pose_in_map);

      transformListener_.waitForTransform(mapFrameId_, "foot_link1", timeStamp, ros::Duration(1.0));
      transformListener_.lookupTransform(mapFrameId_, "foot_link1", timeStamp, transformTf);
      poseTFToEigen(transformTf, foot1_pose_in_map);

    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return true;
    }
    // we can safely assume we have foot pose in sensor now
    // create two cropbox filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> foot1BoxFilter;
    pcl::ConditionalRemoval<pcl::PointXYZRGB> foot2BoxFilter;
    pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;

    Eigen::Matrix3f cropbox_rotation1;
    Eigen::Vector3f cropbox_translation1, crop_min1, crop_max1;
    Eigen::Matrix3f cropbox_rotation2;
    Eigen::Vector3f cropbox_translation2, crop_min2, crop_max2;
 
    // apply first filter
    cropbox_rotation1 = foot0_pose_in_map.rotation().cast<float>();
    cropbox_translation1 = foot0_pose_in_map.translation().cast<float>();
    crop_min1 = cropbox_translation1 + cropbox_rotation1* Eigen::Vector3f(-0.12,-0.12,0.02); 
    crop_max1 = cropbox_translation1 + cropbox_rotation1* Eigen::Vector3f(0.12,0.12,2);

    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond1 (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond1_or_outside_z (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    range_cond1_or_outside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, crop_min1(2))));
    range_cond1_or_outside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, crop_max1(2))));
    range_cond1 ->addCondition(range_cond1_or_outside_z);
    
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond12 (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond1_and_inside_z (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    range_cond1_and_inside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, crop_min1(2))));
    range_cond1_and_inside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, crop_max1(2))));
    range_cond12 ->addCondition(range_cond1_and_inside_z);
    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond1_or_outside_x (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    range_cond1_or_outside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, crop_min1(0))));
    range_cond1_or_outside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, crop_max1(0))));
    range_cond12 ->addCondition(range_cond1_or_outside_x);
    range_cond1 ->addCondition(range_cond12);

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond13 (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    range_cond13 ->addCondition(range_cond1_and_inside_z);    
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond1_and_inside_x (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    range_cond1_and_inside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, crop_min1(0))));
    range_cond1_and_inside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, crop_max1(0))));
    range_cond13 ->addCondition(range_cond1_and_inside_x);
    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond1_or_outside_y (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    range_cond1_or_outside_y->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, crop_min1(1))));
    range_cond1_or_outside_y->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, crop_max1(1))));
    range_cond13 ->addCondition(range_cond1_or_outside_y);
    range_cond1 ->addCondition(range_cond13);

    foot1BoxFilter.setCondition(range_cond1);
    foot1BoxFilter.setInputCloud (pointCloud);
    foot1BoxFilter.filter (tempPointCloud);
    pointCloud->swap(tempPointCloud);

    // apply second filter    
    cropbox_rotation2 = foot1_pose_in_map.rotation().cast<float>();
    cropbox_translation2 = foot1_pose_in_map.translation().cast<float>(); 
    crop_min2 = cropbox_translation2 + Eigen::Vector3f(-0.12,-0.12,0.02); 
    crop_max2 = cropbox_translation2 + Eigen::Vector3f(0.12,0.12,2);
    std::cout << crop_min2.transpose() << std::endl;
    std::cout << crop_max2.transpose() << std::endl;

    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond2 (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond2_or_outside_z (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    range_cond2_or_outside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, crop_min2(2))));
    range_cond2_or_outside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, crop_max2(2))));
    range_cond2 ->addCondition(range_cond2_or_outside_z);
    
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond22 (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond2_and_inside_z (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    range_cond2_and_inside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, crop_min2(2))));
    range_cond2_and_inside_z->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, crop_max2(2))));
    range_cond22 ->addCondition(range_cond2_and_inside_z);
    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond2_or_outside_x (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    range_cond2_or_outside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, crop_min2(0))));
    range_cond2_or_outside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, crop_max2(0))));
    range_cond22 ->addCondition(range_cond2_or_outside_x);
    range_cond2 ->addCondition(range_cond22);

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond23 (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    range_cond23 ->addCondition(range_cond2_and_inside_z);    
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond2_and_inside_x (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    range_cond2_and_inside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, crop_min2(0))));
    range_cond2_and_inside_x->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, crop_max2(0))));
    range_cond23 ->addCondition(range_cond2_and_inside_x);
    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond2_or_outside_y (new pcl::ConditionOr<pcl::PointXYZRGB> ()); 
    range_cond2_or_outside_y->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, crop_min2(1))));
    range_cond2_or_outside_y->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, crop_max2(1))));
    range_cond23 ->addCondition(range_cond2_or_outside_y);
    range_cond2 ->addCondition(range_cond23);

    foot2BoxFilter.setCondition(range_cond2);
    foot2BoxFilter.setInputCloud (pointCloud);
    foot2BoxFilter.filter (tempPointCloud);
    pointCloud->swap(tempPointCloud);
    return true;
  }
  return true;
}

} /* namespace elevation_mapping */

