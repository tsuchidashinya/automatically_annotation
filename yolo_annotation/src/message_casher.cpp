#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <denso_srvs/sensor_data_service.h>

class Sensor_Data_Service
{
public:
    Sensor_Data_Service(ros::NodeHandle nh):
    pnh_("~"),
    nh_(nh)
    {
        get_topic_name();
        server_ = nh_.advertiseService(service_name, &Sensor_Data_Service::service_callback, this);
        camera_sub_ = nh_.subscribe(camera_topic, 10, &Sensor_Data_Service::camera_callback, this);
        image_sub_ = nh_.subscribe(image_topic, 10, &Sensor_Data_Service::image_callback, this);

    }
    void get_topic_name()
    {
        pnh_.getParam("image_topic", image_topic);
        pnh_.getParam("camera_info_topic", camera_topic);
        pnh_.getParam("service_name", service_name);
    }
    bool service_callback(denso_srvs::sensor_data_service::Request &req, denso_srvs::sensor_data_service::Response &response)
    {
        response.image = image_;
        response.camera_info = camera_;
        return true;
    }

    void image_callback(const sensor_msgs::ImageConstPtr msg)
    {
        image_ = *msg;
    }

    void camera_callback(const sensor_msgs::CameraInfoConstPtr msg)
    {
        camera_ = *msg;
    }
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string image_topic, camera_topic, service_name;
    ros::ServiceServer server_;
    ros::Subscriber image_sub_;
    ros::Subscriber camera_sub_;
    sensor_msgs::Image image_;
    sensor_msgs::CameraInfo camera_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_cache");
    ros::NodeHandle nh;
    Sensor_Data_Service sen(nh);
    ros::spin();
}