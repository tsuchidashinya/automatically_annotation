#include <yolo_annotation/yolo_annotation_kai.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <iomanip>
#include <tf2/utils.h>



#define PI 3.141592

tf2::Quaternion convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
{
    tf2::Quaternion q_after, q_final;
    q_after = q_moto * q_ori * q_moto.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    q_final.setRotation(vec, angle);
    return q_final;
}

tf2::Vector3 get_vec(tf2::Quaternion quat_pose, tf2::Quaternion q_moto_jiku)
{
    tf2::Quaternion q_after;
    q_after = quat_pose * q_moto_jiku * quat_pose.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    return vec;
}


std::vector<std::string> split(std::string str, char del) {
    int first = 0;
    int last = str.find_first_of(del);
    std::vector<std::string> result;
    
    while (first < str.size()) {
        
        std::string subStr(str, first, last - first);
        // std::cout << subStr << std::endl;
    
        result.push_back(subStr);
    
        first = last + 1;
        last = str.find_first_of(del, first);
    
        if (last == std::string::npos) {
            last = str.size();
        }
    }
    return result;
}

std::string create_directory_and_get_path(std::string path)
{
    const char *dir = path.c_str();
    struct stat statBuf;
    if (stat(dir, &statBuf) == 0) {
        return path;
    }
    else {
        char del = '/';
        const char *tmp = std::getenv("HOME");
        std::string env_var(tmp ? tmp : "");
        if (env_var.empty()) {
            std::cerr << "[ERROR] No such variable found!" << std::endl;
            exit(EXIT_FAILURE);
        }
        std::string ss;
        ss = env_var + "/";
        for (int i = 0; i < split(path, del).size() - 3; i++) {
            if (i == split(path, del).size() - 4) {
                ss = ss + split(path, del)[i+3];
                if (ss.find(".") != std::string::npos) {
                    break;
                }
                else {
                    std::filesystem::create_directories(ss);
                    break;
                }
                // std::cout << split(path, del)[i+3] << std::endl;
            }
            ss = ss + split(path, del)[i + 3] + "/";
            std::filesystem::create_directory(ss);
            // std::cout << split(path, del)[i + 3] << "/";
        }
        return ss;
    }
}

std::mutex m;
Annotation_yolo::Annotation_yolo(ros::NodeHandle &nh) :
    nh_(nh),
    radious_(0.05),
    save_count_(0),
    pnh_("~")
{
    buffer_ = new tf2_ros::Buffer();
    lister_ = new tf2_ros::TransformListener(*buffer_);
    ros::WallDuration(7).sleep();
    parameter_set();
    occuluder_client_ = nh_.serviceClient<denso_srvs::object_occuluder_service>(occuluder_service_name_);
    message_client_ = nh_.serviceClient<denso_srvs::sensor_data_service>(message_service_name_);
    gazebo_model_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Rate loop(20);
    while (ros::ok()) {
        // std::cout << "loop in" << std::endl;
        Annotation_exec();
        loop.sleep();
    }
}

void Annotation_yolo::parameter_set()
{
    pnh_.getParam("source_frame", source_frame_);
    pnh_.getParam("target_frame", target_frame_);
    pnh_.getParam("f_scale", f_scale_);
    pnh_.getParam("cx_scale", cx_scale_);
    pnh_.getParam("cy_scale", cy_scale_);
    pnh_.getParam("radious", radious_);
    pnh_.getParam("image_dir_name", image_dir_name_);
    pnh_.getParam("filebasename", filebasename_);
    pnh_.getParam("model_name", model_name_);
    pnh_.getParam("world_frame", world_frame_);
    pnh_.getParam("label_dir_name", label_dir_name_);
    pnh_.getParam("boxes_dir_name", boxes_dir_name_);
    pnh_.getParam("the_number_of_data", the_number_of_data);
    pnh_.getParam("occuluder_service_name", occuluder_service_name_);
    pnh_.getParam("message_service_name", message_service_name_);
    pnh_.getParam("z_position", z_position_);
}

void Annotation_yolo::photoneo_move()
{
    std::random_device rd;
    std::default_random_engine eng(rd());
    geometry_msgs::TransformStamped tf_pose;
    double angle;
    std::uniform_int_distribution<> box_distr(0, 100);
    std::uniform_real_distribution<> *distr;
    if (box_distr(eng) > 50) {
        // object_box_set();
        box_is_set_ = true;
        distr = new std::uniform_real_distribution<>(-M_PI/12, M_PI/12);
        angle = distr->operator()(eng);
    }
    else {
        // object_box_remove();
        box_is_set_ = false;
        distr = new std::uniform_real_distribution<>(-M_PI/3, M_PI/3);
        angle = distr->operator()(eng);
    }
    std::uniform_real_distribution<> gabga(0.47, 1.2), delta(-M_PI/14, M_PI/14);
    double r = gabga(eng);
    double x_zure = r * delta(eng), y_zure = r * delta(eng);
    gazebo_service_.request.model_state.model_name = "phoxi_camera";
    gazebo_service_.request.model_state.pose.position.x = r*sin(angle) + x_zure;
    gazebo_service_.request.model_state.pose.position.y = y_zure;
    gazebo_service_.request.model_state.pose.position.z = r * cos(angle) + z_position_;
    // std::cout << r << std::endl;
    // std::cout << x_zure << "  y: " << y_zure << std::endl;
    // std::cout << gazebo_service_.request.model_state.pose.position.z << std::endl;
    // std::cout << box_is_set_ << std::endl;
    tf2::Quaternion source_quat, q_x(1, 0, 0, 0), q_y(0, 1, 0, 0), q_z(0, 0, 1, 0);
    tf2::Quaternion source(0, 0, 0, 1);
    tf2::Quaternion qu_optical = convert_quat(q_y, source_quat, -angle) * source_quat;
    tf2::Quaternion qu = convert_quat(q_y, source, angle) * source;
    tf2::convert(qu, gazebo_service_.request.model_state.pose.orientation);
    tf2::convert(qu_optical, tf_pose.transform.rotation);
    tf_pose.transform.translation.x = gazebo_service_.request.model_state.pose.position.x;
    tf_pose.transform.translation.y = gazebo_service_.request.model_state.pose.position.y;
    tf_pose.transform.translation.z = gazebo_service_.request.model_state.pose.position.z;
    tf2::convert(qu * convert_quat(q_y, qu, M_PI) * convert_quat(q_y, qu, -M_PI_2), tf_pose.transform.rotation);
    tf_pose.child_frame_id = "photoneo_center";
    tf_pose.header.frame_id = world_frame_;
    tf_pose.header.stamp = ros::Time::now();
    gazebo_model_client_.call(gazebo_service_);
    ros::Duration(0.6).sleep();
    tf_pose.header.stamp = ros::Time::now();
    st_br_.sendTransform(tf_pose);
}

void Annotation_yolo::object_box_remove()
{
    gazebo_service_.request.model_state.model_name = "object_box";
    gazebo_service_.request.model_state.pose.position.x = -12;
    gazebo_service_.request.model_state.pose.position.y = -15;
    gazebo_service_.request.model_state.pose.position.z = z_position_;
    tf2::Quaternion quat(0, 0, 0, 1);
    tf2::convert(quat, gazebo_service_.request.model_state.pose.orientation);
    gazebo_model_client_.call(gazebo_service_);
    ros::Duration(0.6).sleep();
}

void Annotation_yolo::object_box_set()
{
    gazebo_service_.request.model_state.model_name = "object_box";
    gazebo_service_.request.model_state.pose.position.x = 0;
    gazebo_service_.request.model_state.pose.position.y = 0;
    gazebo_service_.request.model_state.pose.position.z = z_position_;
    tf2::Quaternion quat(0, 0, 0, 1);
    tf2::convert(quat, gazebo_service_.request.model_state.pose.orientation);
    gazebo_model_client_.call(gazebo_service_);
    ros::Duration(0.6).sleep();
}

void Annotation_yolo::Annotation_exec()
{
    photoneo_move();
    frame_set();
    if (save_count_ == the_number_of_data) {
        ROS_INFO_STREAM("the number of data: " << the_number_of_data);
        nh_.setParam("not_finish", false);
        ros::shutdown();
        return;
    }
    // std::cout << "size: " << target_frames_and_states_.size() << std::endl;
    if (target_frames_and_states_.size() == 0) {
        return;
    }
    denso_srvs::sensor_data_service mes_srv;
    mes_srv.request.call = true;
    message_client_.call(mes_srv);
    cinfo_ = mes_srv.response.camera_info;
    sensor_msgs::Image image1 = mes_srv.response.image;
    geometry_msgs::TransformStamped trans_source;
    tf_get(world_frame_, source_frame_, trans_source);
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int i = 0; i < target_frames_and_states_.size(); i++) {
        geometry_msgs::TransformStamped trans;
        tf_get(world_frame_, target_frames_and_states_[i].frame_name, target_frames_and_states_[i].tf_pose);
        transforms.push_back(trans);
    }
    for (int i = 0; i < target_frames_and_states_.size(); i++) {
        geometry_msgs::Vector3 trans = target_frames_and_states_[i].tf_pose.transform.translation;
        geometry_msgs::Quaternion quat = target_frames_and_states_[i].tf_pose.transform.rotation;
    }
    rotation_convert(trans_source, target_frames_and_states_);
    get_original_image(image1, draw_image_);
    box_get(draw_image_, target_frames_and_states_);
    cv::resize(draw_image_, draw_image_, cv::Size(), 0.7, 0.7);
    if (save_count_ < the_number_of_data) {
        ROS_INFO_STREAM("process rate: " << save_count_ + 1 << "/" << the_number_of_data);
        time_t t_ = time(nullptr);
        localtime_ = localtime(&t_);
        std::string filebasename_shin;
        filebasename_shin = filebasename_ + "_" + std::to_string(localtime_->tm_mon + 1)
                        + "_" + std::to_string(localtime_->tm_mday) + "_" + std::to_string(localtime_->tm_hour)
                        + "_" + std::to_string(localtime_->tm_min) + "_" + std::to_string(localtime_->tm_sec);
        image_dir_name_ = create_directory_and_get_path(image_dir_name_);
        std::string img_path = image_dir_name_ + "/" + filebasename_shin  + ".jpg";
        cv::Mat origin_img; 
        get_original_image(image1, origin_img);
        cv::imwrite(img_path, origin_img);
        boxes_dir_name_ = create_directory_and_get_path(boxes_dir_name_);
        std::string box_path = boxes_dir_name_ + "/" + filebasename_shin + ".jpg";
        cv::imwrite(box_path, draw_image_);
        label_dir_name_ = create_directory_and_get_path(label_dir_name_);
        std::string label_path = label_dir_name_ + "/" + filebasename_shin + ".txt";
        std::ofstream file(label_path);
        for (int i = 0; i < target_frames_and_states_.size(); i++) {
            if (target_frames_and_states_[i].oculution_state == "occuluder")
                
                file << model_name_ << "_" << target_frames_and_states_[i].oculution_state << " " << target_frames_and_states_[i].points_2d[0].x << " " << target_frames_and_states_[i].points_2d[0].y << " " << target_frames_and_states_[i].points_2d[1].x << " " << target_frames_and_states_[i].points_2d[1].y << std::endl;
        }
        file.close();
        save_count_++;
    }
}

void Annotation_yolo::tf_get(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &trans)
{
    try
    {
        // trans = buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
        trans = buffer_->lookupTransform(source_frame, target_frame, ros::Time(0));
        
        ROS_INFO_ONCE("I got a transfomr");
    }
    catch (tf2::TransformException &e)
    {
        ROS_WARN_STREAM(e.what());
        ros::Duration(0.1).sleep();
        return;
    }
}

void Annotation_yolo::box_get(cv::Mat& draw_image, std::vector<target_frame_and_state>& all_msgs)
{
    int count = 0;
    int aida = 100;
    for (int i = 0; i < all_msgs.size(); i++) {
        double x = all_msgs[i].point_3d.x;
        double y = all_msgs[i].point_3d.y;
        double z = all_msgs[i].point_3d.z;
        // std::cout << "x: " <<  x << "  y: " << y << "    z:  "  << z << std::endl;
        cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
        cv::Point2d uv, uv_x1, uv_x2;
        uv = project3d_to_pixel(pt_cv, cinfo_);
        uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo_);
        uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo_);
        double scale = 1;
        // ROS_INFO_STREAM(uv.x << "  " << uv.y << "   imgasize" << draw_image.cols << "  " << draw_image.rows);
        // std::cout << "box_mae" << std::endl;
        if (uv.x > (-draw_image.cols / scale) && uv.x < (draw_image.cols / scale) && uv.y > (-draw_image.rows / scale) &&
        uv.y < (draw_image.rows / scale))
        {
            
            if (all_msgs[i].oculution_state == "occuluder") {
                cv::circle(draw_image, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
                cv::rectangle(draw_image, cv::Point(uv_x1.x, uv_x2.y), cv::Point(uv_x2.x, uv_x1.y), cv::Scalar(0, 255, 0), 4);
                cv::circle(draw_image, cv::Point(uv_x1.x, uv_x2.y), 8, cv::Scalar(255, 255, 255), -1, 1);
                cv::circle(draw_image, cv::Point(uv_x2.x, uv_x1.y), 8, cv::Scalar(0, 0, 0), -1, 1);
            }
            else {
                // cv::rectangle(draw_image, cv::Point(uv_x1.x, uv_x2.y), cv::Point(uv_x2.x, uv_x1.y), cv::Scalar(0, 100, 100), 4);
                // cv::circle(draw_image, cv::Point(uv_x1.x, uv_x2.y), 8, cv::Scalar(255, 255, 255), -1, 1);
                // cv::circle(draw_image, cv::Point(uv_x2.x, uv_x1.y), 8, cv::Scalar(0, 0, 0), -1, 1);
                ;
            }
            
            std::vector<cv::Point2d> uv_s;
            cv::Point2d new_x1(uv_x1.x, uv_x2.y), new_x2(uv_x2.x, uv_x1.y);
            uv_s.push_back(new_x1);
            uv_s.push_back(new_x2);
            // ROS_INFO_STREAM("new.x: " << new_x1.x << " x1.y: " << new_x1.y << "    x2.x: " << new_x2.x << " x2.y: " << new_x2.y);
            all_msgs[i].points_2d = uv_s;
        }
    }
    // std::cout << "uv_points size " << uv_points.size() << std::endl;
}
    

cv::Point2d Annotation_yolo::project3d_to_pixel(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg)
{
    // std::cout << cinfo_msg.K[0] << " " << cinfo_msg.K[1] << " " << cinfo_msg.K[2] << " " << cinfo_msg.K[3] << " " << cinfo_msg.K[4] << " " << cinfo_msg.K[5] << std::endl;
    cv::Point2d uv_rect;
    fx_ = cinfo_msg.K[0] * f_scale_;
    tx_ = cinfo_msg.K[1];
    cx_ = cinfo_msg.K[2] * cx_scale_;
    fy_ = cinfo_msg.K[4] * f_scale_;
    ty_ = cinfo_msg.K[3];
    cy_ = cinfo_msg.K[5] * cy_scale_;
    uv_rect.x = (fx_ * xyz.x + tx_) / xyz.z + cx_;
    uv_rect.y = (fy_ * -xyz.y + ty_) / xyz.z + cy_;
    return uv_rect;
}



void Annotation_yolo::frame_set()
{
    denso_srvs::object_occuluder_service srv;
    srv.request.box_is_set = box_is_set_;
    occuluder_client_.call(srv);

    denso_msgs::object_kiriwake kiri;
    kiri = srv.response.output;
    target_frames_and_states_.clear();
    
    for (int i = 0; i < kiri.occuluder_name.size(); i++) {
        target_frame_and_state obj1;
        obj1.frame_name = kiri.occuluder_name[i];
        obj1.oculution_state = "occuluder";
        target_frames_and_states_.push_back(obj1);
    }

    for (int i = 0; i < kiri.occuludy_name.size(); i++) {
        target_frame_and_state obj2;
        obj2.frame_name = kiri.occuludy_name[i];
        obj2.oculution_state = "occuludy";
        target_frames_and_states_.push_back(obj2);
    }
}


void Annotation_yolo::rotation_convert(geometry_msgs::TransformStamped source_tf, std::vector<target_frame_and_state> &all_msgs)
{
    geometry_msgs::Vector3 source_trans = source_tf.transform.translation;
    tf2::Quaternion source_quat, source_hontou;
    tf2::convert(source_tf.transform.rotation, source_quat);
    tf2::Quaternion q_z(0, 0, 1, 0), q_x(1, 0, 0, 0), q_y(0, 1, 0, 0), q_ken, q_ken_2, q_ken_3;
    tf2::Vector3 vec;
    // q_ken = convert_quat(q_z, source_quat, 0.001) * source_quat;
    // int count = 0;
    // do  {
    //     q_ken = convert_quat(q_z, source_quat, 0.002) * q_ken;
    //     vec = get_vec(q_ken, q_y);
    //     // ROS_INFO_STREAM("*******");
    //     // ROS_INFO_STREAM("x: " << vec.x() << "   y: " << vec.y() << "   z: " << vec.z());
    //     count++;
    // } while (!(vec.x() <= 0.004 && vec.x() >= -0.004 && vec.y() > 0));
    // count = 0;
    // q_ken_2 = convert_quat(q_y, source_quat, 0.001) * q_ken;
    // do  {
    //     q_ken_2 = convert_quat(q_y, source_quat, 0.002) * q_ken_2;
    //     vec = get_vec(q_ken_2, q_x);
    //     // ROS_INFO_STREAM("*******");
    //     // ROS_INFO_STREAM("x: " << vec.x() << "   y: " << vec.y() << "   z: " << vec.z());
    //     count++;
    // } while (!(vec.z() <= 0.004 && vec.z() >= -0.004 && vec.x() < 0));
    // count = 0;
    // q_ken_3 = convert_quat(q_x, source_quat, 0.001) * q_ken_2;
    // do  {
    //     q_ken_3 = convert_quat(q_x, source_quat, 0.002) * q_ken_3;
    //     vec = get_vec(q_ken_3, q_z);
    //     // ROS_INFO_STREAM("*******");
    //     // ROS_INFO_STREAM("x: " << vec.x() << "   y: " << vec.y() << "   z: " << vec.z());
    //     count++;
    // } while (!(vec.y() <= 0.004 && vec.y() >= -0.004 && vec.z() > 0));
    source_hontou = source_quat;
    // source_quat = q_ken_3;
    for (int i = 0; i < all_msgs.size(); i++) {

        geometry_msgs::Vector3 target_trans = all_msgs[i].tf_pose.transform.translation;
        tf2::Quaternion target_q_before(target_trans.x, target_trans.y, target_trans.z, 0), target_rot, target_after;
        tf2::convert(all_msgs[i].tf_pose.transform.rotation, target_rot);
        target_after = target_rot * target_q_before * target_rot.inverse();
        // std::cout << "source:  x: " << source_trans.x << "    y: " << source_trans.y << "    z: " << source_trans.z << std::endl;
        // std::cout << "target:  x: " << target_trans.x << "    y: " << target_trans.y << "    z: " << target_trans.z << std::endl;
        // double x = target_after.x() - source_trans.x;
        // double y = target_after.y() - source_trans.y;
        // double z = target_after.z() - source_trans.z;
        double x = target_trans.x - source_trans.x;
        double y = target_trans.y - source_trans.y;
        double z = target_trans.z - source_trans.z;
        tf2::Quaternion q_before(x, y, z, 0), q_rotation;
        q_rotation = convert_quat(q_z, source_quat, -M_PI / 2) * source_quat;
        //  tf2::Quaternion q_after = source_quat * q_before * source_quat.inverse();
        tf2::Quaternion q_after = q_rotation * q_before * q_rotation.inverse();
        // tf2::Quaternion q_rotation, q_tigai, q_tigai_2, q_tigai_3;
        // q_rotation = convert_quat(q_z, source_quat, M_PI);
        // q_after = q_rotation * source_quat;
        // q_tigai = q_ken.inverse() * source_hontou;
        // q_tigai_2 = q_ken_2.inverse() * q_ken;
        // q_tigai_3 = q_ken_3.inverse() * q_ken_2;

        
        // q_rotation = convert_quat(q_z, q_ken, 0);
        // q_after = q_rotation * q_after * q_rotation.inverse();
        // double roll, pitch, yaw;
        // tf2::getEulerYPR(source_quat, yaw, pitch, roll);
        // std::cout << "q.x: " << source_quat.getX() << "  q.y: " << source_quat.getY() << "  q.z: " << source_quat.getZ() << "  z.w: " << source_quat.getW() << std::endl;
        
        // std::cout << "roll: " << roll << std::endl;
        // std::cout << "pitch: " << pitch << std::endl;
        // std::cout << "yaw: " << yaw - M_PI << std::endl;
        // q_rotation.setEuler()
        
        // q_rotation = convert_quat(q_z, q_ken,  q_tigai.getAngle());
        // q_after = q_rotation * q_after * q_rotation.inverse();
        // q_rotation = convert_quat(q_x, q_ken_2, q_tigai_2.getAngle() + M_PI/16.8);
        // q_rotation = convert_quat(q_x, q_ken_2, q_tigai_2.getAngle());
        // q_after = q_rotation * q_after * q_rotation.inverse();
        // q_rotation = convert_quat(q_y, q_ken_3, -q_tigai_3.getAngle() - M_PI/11.5);
        // q_rotation = convert_quat(q_y, q_ken_3, q_tigai_3.getAngle() - M_PI/2);
        // q_after = q_rotation * q_after * q_rotation.inverse();

        //  std::cout << "angle_ken_2: " << angle_rot << std::endl;
        // double distance = sqrt(source_trans.x*source_trans.x + source_trans.y*source_trans.y);
        // std::cout << "distance: " << distance << std::endl;
        // std::cout << "source_z: " << source_trans.z << std::endl;
        double x_plus, y_plus;
        // angle_rot = angle_rot;
        // std::cout << "angle_rot: " << angle_rot << std::endl;
        
        // x_plus = (distance * sin(angle_rot) + source_trans.x) * 1.3;
        // y_plus = (distance * cos(angle_rot) + source_trans.y) * 1.3;
        x_plus = 0;
        y_plus = 0;
        cv::Point3d out(q_after.x() + x_plus, q_after.y() + y_plus, q_after.z());
        // /std::cout << "outcome:  x: " << out.x << "    y: " << out.y << "    z: " << out.z << std::endl;
        all_msgs[i].point_3d = out;
    }
    
}

void Annotation_yolo::get_original_image(sensor_msgs::Image image1, cv::Mat &original_img)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        return;
    }
    original_img = cv_img_ptr->image;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "colored");
    // ros::WallDuration(10).sleep();
    // ROS_INFO_STREAM("feei");
    ros::NodeHandle nh;
    bool not_finish = true;
    nh.setParam("not_finish", true);
    Annotation_yolo yolo(nh);
    ros::Rate loop(10);
    while (not_finish) {
        ros::spinOnce();
        nh.getParam("not_finish", not_finish);
        loop.sleep();
    }
}