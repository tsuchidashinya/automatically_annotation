#pragma once
#include "cloud_common.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_msgs/PointIndices.h>

class CloudPlanarSegmenter : public CloudOperator
{
public:
    CloudPlanarSegmenter(ros::NodeHandle &nh);
    virtual void operate();
    virtual void publish();
    
private:
    void segment(pcl::PointIndices::Ptr);
    void extract(pcl::PointIndices::Ptr);
    
protected:
    ros::Publisher cloud_segment_pub_;
    ros::Publisher cloud_without_segment_pub_;
    ros::Publisher indice_pub_;
    ros::Publisher coefficients_pub_;
    sensor_msgs::PointCloud2 cloud_segmented_ros_;
    sensor_msgs::PointCloud2 cloud_without_segmented_ros_;
    pcl_msgs::PointIndices indices_ros_;
    pcl_msgs::ModelCoefficients coefficients_ros_;
    pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl_;
    ros::NodeHandle *pnh_;
    float distance_threshold_;
};