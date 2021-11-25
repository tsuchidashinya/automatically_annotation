#include <semantic_annotation/nearest_point_search.hpp>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <fstream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bara_nearest");
    ros::NodeHandle nh;
    nearest_point_extractor::NearestPointExtractor loader(nh);
    ros::spin();
    return 0;
    
}