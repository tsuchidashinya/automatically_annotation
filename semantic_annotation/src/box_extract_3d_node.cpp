#include <semantic_annotation/box_extract_3d.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tuschda_inti");
    ros::WallDuration(20).sleep();
    ros::NodeHandle nh;
    Ano_and_Exec ano_and(nh);
    ros::spin();
}