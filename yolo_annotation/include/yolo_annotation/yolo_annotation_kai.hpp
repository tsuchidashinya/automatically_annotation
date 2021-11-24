#pragma once
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <denso_msgs/object_kiriwake.h>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <sys/stat.h>
#include <denso_srvs/object_occuluder_service.h>
#include <denso_srvs/sensor_data_service.h>
#include <gazebo_msgs/ModelState.h>
#include <random>
#include <gazebo_msgs/SetModelState.h>

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
    void Annotation_exec();
    void parameter_set();
    void frame_set();
    void box_get(cv::Mat&, std::vector<target_frame_and_state>&);
    cv::Point2d project3d_to_pixel(cv::Point3d, sensor_msgs::CameraInfo);
    void rotation_convert(geometry_msgs::TransformStamped, std::vector<target_frame_and_state>&);
    void get_original_image(sensor_msgs::Image, cv::Mat&);
    void photoneo_move();
    void object_box_remove();
    void object_box_set();
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
    ros::NodeHandle nh_;
    const tm* localtime_;
    ros::ServiceClient occuluder_client_, message_client_, gazebo_model_client_;
    ros::NodeHandle pnh_;
    std::string source_frame_, target_frame_;
    std::string occuluder_service_name_, message_service_name_;
    std::string world_frame_;
    std::vector<std::string> target_frames_;
    sensor_msgs::CameraInfo cinfo_;
    tf2_ros::TransformListener *lister_;
    tf2_ros::Buffer *buffer_;
    tf2_ros::StaticTransformBroadcaster st_br_;
    cv::Mat draw_image_;
    gazebo_msgs::SetModelState gazebo_service_;
    double f_scale_, cx_scale_, cy_scale_;
    double fx_, fy_, tx_, ty_, cx_, cy_;
    double z_position_;
    float radious_;
    bool box_is_set_;
    bool write_is_ok_;
    std::string image_dir_name_, filebasename_, model_name_, label_dir_name_, boxes_dir_name_;
    int save_count_;
    int work_count_;
    int the_number_of_data;
    std::vector<target_frame_and_state> target_frames_and_states_;
};