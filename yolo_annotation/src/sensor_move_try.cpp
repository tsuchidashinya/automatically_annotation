#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

tf2_ros::Buffer *buf;
tf2_ros::TransformListener *listener;

void tf_get(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &trans)
{
    while (1) {
        try 
        {
            trans = buf->lookupTransform(source_frame, target_frame, ros::Time(0));
            ROS_INFO_ONCE("I got transfer");
            break;
        }
        catch (tf2::TransformException &e)
        {
            ROS_WARN_STREAM(e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
}

tf2::Quaternion convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
{
    tf2::Quaternion q_after, q_final;
    q_after = q_moto * q_ori * q_moto.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    q_final.setRotation(vec, angle);
    return q_final;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchdia_");
    ros::NodeHandle nh;
    buf = new tf2_ros::Buffer();
    listener = new tf2_ros::TransformListener(*buf);
    tf2_ros::TransformBroadcaster br;
    ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    gazebo_msgs::ModelState model_state;
    model_state.model_name = "phoxi_camera";
    double x, y, z;
    std::random_device rd;
    std::default_random_engine eng(rd());
    geometry_msgs::TransformStamped origin_pose;
    tf_get("base_link", "phoxi_camera", origin_pose);
    std::cout << "origin_pose.transform.rotation" << "  x: " << origin_pose.transform.rotation.x << "  y: " << origin_pose.transform.rotation.y
        << "  z: " << origin_pose.transform.rotation.z << "  w: " << origin_pose.transform.rotation.w << std::endl;
    tf2::Quaternion source_quat, q_x(1, 0, 0, 0), q_y(0, 1, 0, 0), q_z(0, 0, 1, 0);

    tf2::convert(origin_pose.transform.rotation, source_quat);
    
    int count = 0;
    while (count <= 10) {
        geometry_msgs::TransformStamped tf_pose;
        double r = 1;
        std::uniform_real_distribution<> distr(-M_PI/6, M_PI/6);
        double angle = distr(eng);
        
        model_state.pose.position.x = r*sin(angle);
        model_state.pose.position.y = 0;
        model_state.pose.position.z = r * cos(angle);

        tf2::Quaternion source(0, 0, 0, 1);
        tf2::Quaternion qu_optical = convert_quat(q_y, source_quat, -angle) * source_quat;
        tf2::Quaternion qu = convert_quat(q_y, source, angle) * source;
        //  std::cout << "source_quat" << "  x: " << source_quat.x() << "  y: " << source_quat.y()
        // << "  z: " << source_quat.z() << "  w: " << source_quat.w() << std::endl;
        // std::cout << "qu" << "  x: " << qu.x() << "  y: " << qu.y()
        // << "  z: " << qu.z() << "  w: " << qu.w() << std::endl;
        tf2::convert(qu, model_state.pose.orientation);
        tf2::convert(qu_optical, tf_pose.transform.rotation);
        std::cout << "model_state_pose_ori" << "  x: " << model_state.pose.orientation.x << "  y: " << model_state.pose.orientation.y
        << "  z: " << model_state.pose.orientation.z << "  w: " << model_state.pose.orientation.w << std::endl;
        tf_pose.transform.translation.x = model_state.pose.position.x;
        tf_pose.transform.translation.y = model_state.pose.position.y;
        tf_pose.transform.translation.z = model_state.pose.position.z;
        tf2::convert(qu * convert_quat(q_y, qu, M_PI) * convert_quat(q_y, qu, -M_PI_2), tf_pose.transform.rotation);
        tf_pose.child_frame_id = "photoneo_center";
        tf_pose.header.frame_id = "base_link";
        tf_pose.header.stamp = ros::Time::now();
        
        ros::Rate loop(15);
        int naka_count = 0;
        while (naka_count <= 10) {
            tf_pose.header.stamp = ros::Time::now();
            br.sendTransform(tf_pose);
            pub.publish(model_state);
            naka_count++;
            loop.sleep();
        }
        pub.publish(model_state);
        ros::Duration(0.2).sleep();
        count++;
    }
    return 0;

    
}