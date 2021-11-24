#pragma once
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <denso_msgs/object_kiriwake.h>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <sys/stat.h>
#include <denso_srvs/object_occuluder_service.h>

struct target_frame_and_state
{
    std::string frame_name;
    std::string oculution_state;
    geometry_msgs::TransformStamped tf_pose;
    cv::Point3d point_3d;
    std::vector<cv::Point2d> points_2d;
};

class Annotation_yolo
{
public:
    Annotation_yolo(ros::NodeHandle&);
    void tf_get(std::string, std::string, geometry_msgs::TransformStamped&);
    void InputCallback(sensor_msgs::CameraInfoConstPtr, sensor_msgs::ImageConstPtr);
    void parameter_set();
    void frame_set();
    void box_get(sensor_msgs::CameraInfo, sensor_msgs::Image, std::vector<cv::Point3d>, cv::Mat&, std::vector<std::vector<cv::Point2d>>&);
    void box_get(cv::Mat&, std::vector<target_frame_and_state>&);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> Sync_Sub_type;
    cv::Point2d project3d_to_pixel(cv::Point3d, sensor_msgs::CameraInfo);
    void rotation_convert(geometry_msgs::TransformStamped, std::vector<geometry_msgs::TransformStamped>, std::vector<cv::Point3d>&);
    void rotation_convert(geometry_msgs::TransformStamped, std::vector<target_frame_and_state>&);
    
    void get_original_image(sensor_msgs::Image, cv::Mat&);
    cv::Mat get_original_image(sensor_msgs::Image);

    template <typename T>
    void get_one_message(std::string topic_name, ros::NodeHandle nh, double timeout, T &final)
    {
        boost::shared_ptr<const T> share;
        share = ros::topic::waitForMessage<T>(topic_name, nh, ros::Duration(timeout));
        if (share != NULL) {
            final = *share;
        }
    }
private:
    message_filters::Synchronizer<Sync_Sub_type> *sensor_sync_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    ros::NodeHandle pnh_;
    std::string source_frame_, target_frame_;
    std::string world_frame_;
    std::vector<std::string> target_frames_;
    std::string camera_topic_name_, image_topic_name_, occulution_mes_topic_name_;
    sensor_msgs::CameraInfo cinfo_;
    tf2_ros::TransformListener *lister_;
    tf2_ros::Buffer *buffer_;
    cv::Mat draw_image_;
    double f_scale_, cx_scale_, cy_scale_;
    double fx_, fy_, tx_, ty_, cx_, cy_;
    float radious_;
    bool write_is_ok_;
    std::string image_dir_name_, filebasename_, model_name_, label_dir_name_, boxes_dir_name_;
    int save_count_;
    int work_count_;
    int the_number_of_data;
    std::vector<target_frame_and_state> target_frames_and_states_;

};