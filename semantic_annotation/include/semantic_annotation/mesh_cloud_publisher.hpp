#pragma once
#include <ros/ros.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <semantic_annotation/mesh_sampler.hpp>
namespace mesh_cloud
{
    class MeshCloud
    {
    public:
            /*
        MeshCloud(nodehandle, object_name, mesh_topic_name)
        */
        MeshCloud(ros::NodeHandle& nh, std::string, std::string);
        ~MeshCloud(){}
        void getMesh(const std::string dir_path);
        void transformMesh();
        void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        void publishCloud();
        void frame_set();
        void stl_file_set();
        void get_tf();
        int OBJECT_QUANTITY;
        template <typename T>
        void print_parameter(T para)
        {
            std::cout << para << std::endl;
        }
        

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle *pnh_;
        tf::TransformListener tf_;
        sensor_msgs::PointCloud2 mesh_cloud_ros_;
        ros::Publisher mesh_point_pub_;
        pcl::PointCloud<pcl::PointXYZ> mesh_point_pcl_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_mesh_ptr_pcl_;

        std::vector<pcl::PointCloud<pcl::PointXYZ>> parts_clouds_;
        std::vector<pcl::PolygonMesh> meshes_cluster_;
        std::vector<std::string> link_names_;
        std::vector<std::string> frame_names_;
        tf::StampedTransform transform_;
        std::string frame_;
        std::string object_name_;
        std::string link_;
        std::string mesh_path_;
        std::string mesh_topic_name_;
        std::string source_frame_;
        float LEAF_SIZE;
        int sample_points;
        int RETRY_COUNT_LIMIT;
        float DURATION_TIME;
    };
}
