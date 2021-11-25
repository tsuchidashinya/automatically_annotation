#pragma once
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <denso_msgs/out_segmentation.h>
#include <denso_msgs/object_kiriwake.h>

class Ano_and_Exec
{
public:
    Ano_and_Exec(ros::NodeHandle&);
    void tf_get(std::string, std::string, geometry_msgs::TransformStamped&);
    void InputCallback(sensor_msgs::CameraInfoConstPtr, sensor_msgs::ImageConstPtr);
    void parameter_set();
    void box_get(sensor_msgs::CameraInfo, sensor_msgs::Image, std::vector<cv::Point3d>, cv::Mat&, std::vector<std::vector<cv::Point2d>>&);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> Sync_Sub_type;
    cv::Point2d project3d_to_pixel(cv::Point3d, sensor_msgs::CameraInfo);
    cv::Point2d project3d_to_pixel_origin(cv::Point3d, sensor_msgs::CameraInfo);
    void paramter_set_bara();
    void rotation_convert(geometry_msgs::TransformStamped, std::vector<geometry_msgs::TransformStamped>, std::vector<cv::Point3d>&);
    void get_original_image(sensor_msgs::Image, cv::Mat&);
    template <typename T>
    void get_one_message(T &final_message, std::string message_name, ros::NodeHandle nh, int timespan)
    {
        boost::shared_ptr<const T> share;
        share = ros::topic::waitForMessage<T>(message_name, nh, ros::Duration(timespan));
        if (share != NULL) {
            final_message = *share;
        }
        share.reset();
    }
    void write_instance(std::vector<std::vector<cv::Point2d>>, std::vector<std::vector<int>> &);
    std::vector<std::vector<int>> write_instance(std::vector<std::vector<cv::Point2d>>, cv::Mat draw_IMG);
    template <class T>
    void swap(T &yes, T &we)
    {
        T t = yes;
        yes = we;
        we = t;
    }

    void hurui(pcl::PointCloud<pcl::PointXYZ>, std::vector<std::vector<int>>, sensor_msgs::Image, sensor_msgs::CameraInfo, pcl::PointCloud<pcl::PointXYZRGB>&);
    void hurui(pcl::PointCloud<pcl::PointXYZ>, std::vector<std::vector<int>>, sensor_msgs::Image, sensor_msgs::CameraInfo, pcl::PointCloud<pcl::PointXYZ>&);

private:
    message_filters::Synchronizer<Sync_Sub_type> *sensor_sync_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    std::vector<std::vector<int>> image_instance_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string source_frame_, target_frame_;
    ros::Publisher output_pub_;
    std::string world_frame_;
    std::string inputcloud_topic_name_;
    std::string output_topic_name_;
    std::string oculuder_topic_name_;
    std::vector<std::string> target_frames_;
    std::string camera_topic_name_, image_topic_name_;
    tf2_ros::TransformListener lister_;
    tf2_ros::Buffer buffer_;
    cv::Mat draw_image_;
    sensor_msgs::PointCloud2 output_cloud_msgs_;
    double f_scale_, cx_scale_, cy_scale_;
    double fx_, fy_, tx_, ty_, cx_, cy_;
    double dulation_;
    float radious_;
    bool write_is_ok_;
    std::string image_dir_name_, filebasename_, model_name_, label_dir_name_, boxes_dir_name_;
    int save_count_;
    int the_number_of_data;
    int timespan_;
};