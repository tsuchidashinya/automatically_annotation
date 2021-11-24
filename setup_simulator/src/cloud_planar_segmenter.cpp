#include <setup_simulator/cloud_planar_segmenter.hpp>

CloudPlanarSegmenter::CloudPlanarSegmenter(ros::NodeHandle &nh) :
    cloud_segment_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_segmented", 1)),
    cloud_without_segment_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_without_segmented", 1)),
    indice_pub_(nh.advertise<pcl_msgs::PointIndices>("indices", 1)),
    coefficients_pub_(nh.advertise<pcl_msgs::ModelCoefficients>("coefficients", 1)),
    distance_threshold_(0.001)
{
    pnh_ = new ros::NodeHandle("~");
    pnh_->getParam("distance_threshold", distance_threshold_);
}

void CloudPlanarSegmenter::operate()
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    segment(inliers);
    extract(inliers);
}

void CloudPlanarSegmenter::publish()
{
    cloud_segment_pub_.publish(cloud_segmented_ros_);
    cloud_without_segment_pub_.publish(cloud_without_segmented_ros_);
    indice_pub_.publish(indices_ros_);
    coefficients_pub_.publish(coefficients_ros_);
}

void CloudPlanarSegmenter::segment(pcl::PointIndices::Ptr inliers)
{
    pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl_);
    pcl::ModelCoefficients coefficent_pcl_1;
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(1000);
    segmentation.setDistanceThreshold(distance_threshold_);
    segmentation.setInputCloud(cloud_input_pcl_.makeShared());
    segmentation.segment(*inliers, coefficent_pcl_1);

    pcl_conversions::fromPCL(*inliers, indices_ros_);
    pcl_conversions::fromPCL(coefficent_pcl_1, coefficients_ros_);

}

void CloudPlanarSegmenter::extract(pcl::PointIndices::Ptr inliers)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_segmented_pcl;
    pcl::PointCloud<pcl::PointXYZ> cloud_without_segmented_pcl;
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud_input_pcl_.makeShared());
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(cloud_segmented_pcl);

    extract.setNegative(true);
    extract.filter(cloud_without_segmented_pcl);

    pcl::toROSMsg(cloud_segmented_pcl, cloud_segmented_ros_);
    pcl::toROSMsg(cloud_without_segmented_pcl, cloud_without_segmented_ros_);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_planar");
    ros::NodeHandle nh;
    ros::WallDuration(10.0).sleep();
    ros::NodeHandle pnh("~");
    std::string sub_topic_name = "/photoneo_center/sensor/points";
    pnh.getParam("sub_topic_name", sub_topic_name);

    CloudOperationHandler hander(nh, new CloudPlanarSegmenter(nh), sub_topic_name);
    ros::spin();
    return 0;
}