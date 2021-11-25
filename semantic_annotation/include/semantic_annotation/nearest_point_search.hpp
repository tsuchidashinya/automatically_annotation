#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <denso_msgs/out_segmentation.h>
#include <denso_msgs/object_kiriwake.h>

namespace nearest_point_extractor
{
    struct mesh_and_instance {
        pcl::PointCloud<pcl::PointXYZ> mesh_pcl_one;
        int instance;
    };
    class NearestPointExtractor
    {
    public:
        NearestPointExtractor(ros::NodeHandle &nh);
        void InputCallback(const denso_msgs::object_kiriwakeConstPtr &);
        void exect();
        void mesh_callback(const sensor_msgs::PointCloud2ConstPtr&, int);
        void extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud, denso_msgs::object_kiriwake mes, double radius, pcl::PointCloud<pcl::PointXYZ> before_cloud, denso_msgs::out_segmentation &out);
        template <typename T>
        void print_parameter(T para)
        {
            std::cout << para << std::endl;
        }

        template <typename T>
        void get_one_message(T &msg, std::string topic_name, int timeout)
        {
            boost::shared_ptr<const T> share;
            share = ros::topic::waitForMessage<T>(topic_name, nh_, ros::Duration(timeout));
            if (share != NULL) {
                msg = *share;
            }
            share.reset();
        }
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_;
        denso_msgs::out_segmentation output_cloud_;
        std::string frame_id_;
        std::string occuluder_kiriwake_topic_name_;
        std::vector<int> cloud_index_;
        sensor_msgs::PointCloud2 sensor_cloud_msgs_;
        std::vector<sensor_msgs::PointCloud2> mesh_clouds_msgs_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>*> mesh_clouds_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_, sensor_before_cloud_;

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher dummy_pub_;
        ros::Subscriber mesh_topic_name_sub_;
        tf::StampedTransform transform_;
        tf::TransformListener listener_;
        int num_of_nearest_points_;
        int timeout_;
        double dulation_;
        bool flag_;
        std::string mesh_topic_name_;
        std::string sensor_topic_name_;
        std::string output_topic_name_;
        std::string source_frame_;
        double radius_;
        int background_instance_;
          ros::Subscriber mesh_sub_1_;
        ros::Subscriber mesh_sub_2_;
        ros::Subscriber mesh_sub_3_;
        ros::Subscriber mesh_sub_4_;
        ros::Subscriber mesh_sub_5_;
        ros::Subscriber mesh_sub_6_;
        ros::Subscriber mesh_sub_7_;
        ros::Subscriber mesh_sub_8_;
        ros::Subscriber mesh_sub_9_;
        ros::Subscriber mesh_sub_10_;
        ros::Subscriber mesh_sub_11_;
        ros::Subscriber mesh_sub_12_;
        ros::Subscriber mesh_sub_13_;
        ros::Subscriber mesh_sub_14_;
        ros::Subscriber mesh_sub_15_;
        ros::Subscriber mesh_sub_16_;
        ros::Subscriber mesh_sub_17_;
        ros::Subscriber mesh_sub_18_;
        ros::Subscriber mesh_sub_19_;
        ros::Subscriber mesh_sub_20_;
        ros::Subscriber mesh_sub_21_;
        ros::Subscriber mesh_sub_22_;
        ros::Subscriber mesh_sub_23_;
        ros::Subscriber mesh_sub_24_;
        ros::Subscriber mesh_sub_25_;
        ros::Subscriber mesh_sub_26_;
        ros::Subscriber mesh_sub_27_;
        ros::Subscriber mesh_sub_28_;
        ros::Subscriber mesh_sub_29_;
        ros::Subscriber mesh_sub_30_;
        ros::Subscriber mesh_sub_31_;
        ros::Subscriber mesh_sub_0_;
        std::string mesh_base_topic_name_;

    };
}