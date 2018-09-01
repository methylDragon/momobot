#include <ros/ros.h>
#include "momo_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "momo_base_node");
    MomoBase momo;
    ros::spin();
    return 0;
}
