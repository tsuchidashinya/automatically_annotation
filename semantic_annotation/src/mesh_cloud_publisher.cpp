#include <semantic_annotation/mesh_cloud_publisher.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

using mesh_cloud::MeshCloud;
using mesh_sampler::uniform_sampling;

/*
MeshCloud(nodehandle, object_name, mesh_topic_name)
*/
MeshCloud::MeshCloud(ros::NodeHandle &nh, std::string object_name, std::string mesh_topic_name)
    : nh_(nh), downsampled_mesh_ptr_pcl_(new pcl::PointCloud<pcl::PointXYZ>())
{
    pnh_ = new ros::NodeHandle("~");
    pnh_->getParam("sample_points", sample_points);
    pnh_->getParam("LEAF_SIZE", LEAF_SIZE);
    pnh_->getParam("RETRY_COUNT_LIMIT", RETRY_COUNT_LIMIT);
    pnh_->getParam("DURATION_TIME", DURATION_TIME);
    pnh_->getParam("OBJECT_QUANTITY", OBJECT_QUANTITY);
    pnh_->getParam("source_frame", source_frame_);
    //pnh_->getParam("object_name", object_name);
    //pnh_->getParam("mesh_topic_name", mesh_topic_name_);
    mesh_topic_name_ = mesh_topic_name;
    object_name_ = object_name;
    OBJECT_QUANTITY = 1;
    // print_parameter("sample_point is" + std::to_string(sample_points));
    
    frame_set();
    stl_file_set();
    // this->get_tf();
    // this->transformMesh();
   /* for (int i = 0; i < parts_clouds_.size(); i++) {
        mesh_point_pcl_ += parts_clouds_[i];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_point_ptr_pcl(
                new pcl::PointCloud<pcl::PointXYZ>(mesh_point_pcl_));
    this->downSample(mesh_point_ptr_pcl, downsampled_mesh_ptr_pcl_);*/
    mesh_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(mesh_topic_name_, 1);
}

void MeshCloud::frame_set()
{
    if (OBJECT_QUANTITY == 1) {
        frame_names_.push_back(object_name_);
    }
    else {
        for (int i = 0; i < OBJECT_QUANTITY; i++) {
            frame_names_.push_back("HV8_" + std::to_string(i));
            //frame_names_.push_back(object_name);
            // print_parameter(frame_names_[i]);
        }
    }
    
}

void MeshCloud::stl_file_set()
{
    pnh_->getParam("mesh_path", mesh_path_);
    for (int i = 0; i < 1; i++) {
        link_names_.push_back(mesh_path_);
    }
    // print_parameter(link_names_[0]);
    // print_parameter(OBJECT_QUANTITY);
    // print_parameter(LEAF_SIZE);
    for (int i = 0; i < OBJECT_QUANTITY; i++) {
        this->getMesh(link_names_[i]);
    }

}

void MeshCloud::getMesh(const std::string dir_name)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(dir_name, mesh);
    vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh, polydata1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr parts_clou(new pcl::PointCloud<pcl::PointXYZ>);
    uniform_sampling(polydata1, sample_points, *parts_clou);
    parts_clouds_.push_back(*parts_clou);
}

void MeshCloud::get_tf()
{
    while (true) 
    {
        try
            {
                tf_.lookupTransform(source_frame_, frame_names_[0], ros::Time(0), transform_);
                //ROS_INFO_STREAM("Get transform : " << frame_names_[0]);
                break;
            }
            catch (tf2::LookupException e)
            {
                // ROS_ERROR("pcl::ros %s", e.what());
                ros::Duration(DURATION_TIME).sleep();
                mesh_point_pcl_.clear();
                continue;
            }
            catch (tf2::ExtrapolationException e)
            {
                // ROS_ERROR("pcl::ros %s", e.what());
                ros::Duration(DURATION_TIME).sleep();
                mesh_point_pcl_.clear();
                continue;
            }
            catch (...)
            {
                ros::Duration(DURATION_TIME).sleep();
                mesh_point_pcl_.clear();
                continue;
            }
    }
}
void MeshCloud::transformMesh()
{

    pcl_ros::transformPointCloud(parts_clouds_[0], mesh_point_pcl_, transform_);
    if (mesh_point_pcl_.points.size() == (sample_points * parts_clouds_.size())) {
        // std::cout << "0) mesh : " << mesh_point_pcl_.points.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_point_ptr_pcl(
            new pcl::PointCloud<pcl::PointXYZ>(mesh_point_pcl_));
        downSample(mesh_point_ptr_pcl, downsampled_mesh_ptr_pcl_);           
    }
    
    //ROS_WARN_STREAM("try if transform 5times, but failed");
    //exit(-1);
   // return;
}

void MeshCloud::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    sor.filter(*cloud_filtered);
    // std::cout << "------------------------------------" << std::endl;
    // std::cout << "DownSampled cloud point size : " << cloud_filtered->points.size() << std::endl;
}

void MeshCloud::publishCloud()
{
    pcl::toROSMsg(*downsampled_mesh_ptr_pcl_, mesh_cloud_ros_);
    mesh_cloud_ros_.header.stamp = ros::Time::now();
    mesh_cloud_ros_.header.frame_id = source_frame_;
    mesh_point_pub_.publish(mesh_cloud_ros_);
    // std::cout << "========================" << std::endl;

}