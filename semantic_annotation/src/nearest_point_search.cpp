#include <semantic_annotation/nearest_point_search.hpp>

namespace nearest_point_extractor
{
    NearestPointExtractor::NearestPointExtractor(ros::NodeHandle &nh)
    : nh_(nh)
    , flag_(false)
    , pnh_("~")
    , sensor_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , sensor_before_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pnh_.getParam("sensor_topic_name", sensor_topic_name_);
        pnh_.getParam("output_topic_name", output_topic_name_);
        pnh_.getParam("mesh_base_topic_name", mesh_base_topic_name_);
        pnh_.getParam("radius", radius_);
        pnh_.getParam("num_of_nearest_points", num_of_nearest_points_);
        pnh_.getParam("message_timeout", timeout_);
        pnh_.getParam("background_instance", background_instance_);
        pnh_.getParam("dulation", dulation_);
        pnh_.getParam("oculuder_kiriwake_topic_name", occuluder_kiriwake_topic_name_);
        pnh_.getParam("source_frame", source_frame_);
        for (int i = 0; i < 32; i++) {
            mesh_clouds_.push_back(new pcl::PointCloud<pcl::PointXYZ>());
        }
        exect();
    }

    /*Refecence the publisher and subscriber*/
    void NearestPointExtractor::exect()
    {
        ROS_INFO_STREAM(output_topic_name_);
        ROS_INFO_STREAM(sensor_topic_name_);
        dummy_pub_ = nh_.advertise<denso_msgs::out_segmentation>(output_topic_name_, 10);
        mesh_topic_name_sub_ = nh_.subscribe(occuluder_kiriwake_topic_name_, 10, &NearestPointExtractor::InputCallback, this);
        mesh_sub_0_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_0", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 0));
        mesh_sub_1_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_1", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 1));
        mesh_sub_2_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_2", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 2));
        mesh_sub_3_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_3", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 3));
        mesh_sub_4_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_4", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 4));
        mesh_sub_5_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_5", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 5));
        mesh_sub_6_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_6", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 6));
        mesh_sub_7_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_7", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 7));
        mesh_sub_8_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_8", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 8));
        mesh_sub_9_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_9", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 9));
        mesh_sub_10_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_10", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 10));
        mesh_sub_11_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_11", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 11));
        mesh_sub_12_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_12", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 12));
        mesh_sub_13_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_13", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 13));
        mesh_sub_14_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_14", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 14));
        mesh_sub_15_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_15", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 15));
        mesh_sub_16_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_16", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 16));
        mesh_sub_17_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_17", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 17));
        mesh_sub_18_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_18", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 18));
        mesh_sub_19_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_19", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 19));
        mesh_sub_20_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_20", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 20));
        mesh_sub_21_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_21", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 21));
        mesh_sub_22_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_22", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 22));
        mesh_sub_23_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_23", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 23));
        mesh_sub_24_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_24", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 24));
        mesh_sub_25_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_25", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 25));
        mesh_sub_26_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_26", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 26));
        mesh_sub_27_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_27", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 27));
        mesh_sub_28_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_28", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 28));
        mesh_sub_29_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_29", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 29));
        mesh_sub_30_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_30", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 30));
        mesh_sub_31_ = nh_.subscribe<sensor_msgs::PointCloud2>(mesh_base_topic_name_ + "_31", 10, boost::bind(&NearestPointExtractor::mesh_callback, this, _1, 31));
    }



    void NearestPointExtractor::InputCallback(const denso_msgs::object_kiriwakeConstPtr &kiriwake_msgs)
    {   
        sensor_msgs::PointCloud2 sensor_pc_msgs;
        // std::cout << "tsuchida_1" << std::endl;
        get_one_message<sensor_msgs::PointCloud2>(sensor_pc_msgs, sensor_topic_name_, timeout_);
        try
        {
            listener_.lookupTransform(source_frame_, sensor_pc_msgs.header.frame_id, ros::Time(0), transform_);
            // ROS_INFO_ONCE("I got a transfomr");

        }
        catch (tf::TransformException ex)
        {
            // ROS_ERROR("%s", ex.what());
            ros::Duration(dulation_).sleep();
            return;
        }
        // source_frame_ = sensor_pc_msgs.header.frame_id;
        
        sensor_msgs::PointCloud2 msg_transformed;
        // print_parameter(sensor_pc_msgs->header.frame_id);
        pcl::fromROSMsg(sensor_pc_msgs, *sensor_before_cloud_);
        pcl_ros::transformPointCloud(source_frame_, transform_, sensor_pc_msgs, msg_transformed);
        // std::cout << "tsuchida_5" << std::endl;
        
        pcl::fromROSMsg(msg_transformed, *sensor_cloud_);
        // std::cout << "tsuchida_6" << std::endl;
        flag_ = true;
        denso_msgs::out_segmentation output;
       
        extract_cloud(*sensor_cloud_, *kiriwake_msgs, radius_, *sensor_before_cloud_, output);
        if (output.instance.size() == 0) {
            return;
        }
        output.header.frame_id = sensor_cloud_msgs_.header.frame_id;
        
        // print_parameter(std::to_string(output.x.size()) + "output_point");
        dummy_pub_.publish(output);
        
    }

    // void NearestPointExtractor::publish()
    // {
    //     if (!flag_)
    //         return;
    //     denso_msgs::out_segmentation output;
    //     denso_msgs::object_kiriwake kiri_msg;
    //     get_one_message(kiri_msg, occuluder_kiriwake_topic_name_, timeout_);
    //     output = extract_cloud(*sensor_cloud_, kiri_msg, radius_);
    //     output.header.frame_id = sensor_cloud_msgs_.header.frame_id;
    //     print_parameter(std::to_string(output.x.size()) + "output_point");
    //     dummy_pub_.publish(output);
    // }

    void NearestPointExtractor::extract_cloud(pcl::PointCloud<pcl::PointXYZ> sensor_cloud, denso_msgs::object_kiriwake kiri, double radisu_arg, pcl::PointCloud<pcl::PointXYZ> before_cloud, denso_msgs::out_segmentation &out_cloud)
    {
        // denso_msgs::out_segmentation out_cloud;
        for (int i = 0; i < sensor_cloud.size(); i++) {
            out_cloud.x.push_back(before_cloud.points[i].x);
            out_cloud.y.push_back(before_cloud.points[i].y);
            out_cloud.z.push_back(before_cloud.points[i].z);
            out_cloud.instance.push_back(background_instance_);
        }
        // std::cout << "input kdtree: " << sensor_cloud.size() <<std::endl;
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        if (sensor_cloud.points.size() == 0) {
            // ROS_WARN_STREAM("The number of point is zero");
            return;
        }
        kdtree.setInputCloud(sensor_cloud.makeShared());

        std::vector<int> pointIndices, list_pointIndices;
        std::vector<float> squaredDistances;
        double c2c_distance = 0.0;
        int point_size = 0;
        for (int i = 0; i < kiri.occuludy_instance_number.size(); i++) {
            for (auto mesh : mesh_clouds_[kiri.occuludy_instance_number[i]]->points)
            {
                if (kdtree.nearestKSearch(mesh, num_of_nearest_points_, pointIndices, squaredDistances) > 0) {
                    c2c_distance += squaredDistances[0];
                    for (int j = 0; j < pointIndices.size(); j++) {
                        out_cloud.instance[pointIndices[j]] = kiri.occuludy_instance_number[i] + 50;
                    }
                }
            }
        }
        ros::WallTime start = ros::WallTime::now();
        for (int i = 0; i < kiri.occuluder_instance_numbers.size(); i++) {
            // std::cout << kiri.occuluder_instance_numbers[i] << ": kazuha : ";
            int count = 0;
            for (auto mesh : mesh_clouds_[kiri.occuluder_instance_numbers[i]]->points)
            {
                if (kdtree.nearestKSearch(mesh, num_of_nearest_points_, pointIndices, squaredDistances) > 0) {
                    c2c_distance += squaredDistances[0];
                    point_size++;
                    for (int j = 0; j < pointIndices.size(); j++) {
                        ros::WallTime end = ros::WallTime::now();
                        ros::WallDuration shori = end - start;
                        if (shori.toSec() >= 3) {
                            // ROS_WARN_STREAM("fail");
                            return;
                        }
                        out_cloud.instance[pointIndices[j]] = kiri.occuluder_instance_numbers[i];
                        list_pointIndices.push_back(pointIndices[j]);
                    }

                }
                pointIndices.clear();
                squaredDistances.clear();
                count++;
            }
            
            // std::cout << count << std::endl;
        }
        
        // ("sensor all size: " << senROS_INFO_STREAMsor_cloud_->points.size());
        // ROS_INFO_STREAM("color point size: " << list_pointIndices.size());
        list_pointIndices.clear();
        
    }

    void NearestPointExtractor::mesh_callback(const sensor_msgs::PointCloud2ConstPtr &msg, int num)
    {
        pcl::fromROSMsg(*msg, *mesh_clouds_[num]);
    }


}