#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <denso_msgs/out_segmentation.h>


std::string pcd_dir_name;
std::string pcd_file_name;
std::string receive_topic_name;
std::string output_topic_name;
ros::Publisher pub;
std::string frame_id_;

void callback(const denso_msgs::out_segmentationConstPtr msg)
{
    denso_msgs::out_segmentation out_pcl = *msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    int count = 0;
    for (int i = 0; i < out_pcl.x.size(); i++) {
        pcl::PointXYZRGB xyzrgb;
        xyzrgb.x = out_pcl.x[i];
        // if (i % 1000 == 0) {
            // std::cout << out_pcl.instance[i] << " ";
        // }/
        xyzrgb.y = out_pcl.y[i];
        xyzrgb.z = out_pcl.z[i];
        if (out_pcl.instance[i] >= 0 && out_pcl.instance[i] < 50) {
            xyzrgb.r = 255;
            xyzrgb.b = 0;
            xyzrgb.g = 0;
        }
        else if (out_pcl.instance[i] >= 50 && out_pcl.instance[i] < 200) {
            xyzrgb.r = 0;
            xyzrgb.b = 255;
            xyzrgb.g = 0;
        }
        
        else {
            xyzrgb.r = 255;
            xyzrgb.b = 255;
            xyzrgb.g  =255;
            count++;
        }
        
        cloud.push_back(xyzrgb);
    }
    // ROS_INFO_STREAM("shiro kazu: " << count);
    // ROS_INFO_STREAM("start save");
    // ROS_INFO_STREAM("cloud size is " << cloud.points.size());
    // ROS_INFO_STREAM("finish save");
    sensor_msgs::PointCloud2 ros_msgs;
    pcl::toROSMsg(cloud, ros_msgs);
    ros_msgs.header.frame_id = frame_id_;
    pub.publish(ros_msgs);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_save_2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("receive_topic_name", receive_topic_name);
    pnh.getParam("output_topic_name", output_topic_name);
    pnh.getParam("frame_id", frame_id_);
   
    ros::Subscriber sub;
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic_name, 10);
    sub = nh.subscribe(receive_topic_name, 10, callback);
    ros::spin();
    
}