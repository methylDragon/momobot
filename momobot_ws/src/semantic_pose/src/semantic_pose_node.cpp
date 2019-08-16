
#include <ros/ros.h> 
#include <tf/transform_listener.h> 
#include <std_msgs/String.h>

#include "places.h"

int main(int argc, char * * argv) 
{
    ros::init(argc, argv, "semantic_pose");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string base_frame_id;
    std::string map_frame_id;
    float publish_rate;

    nh_priv.param<std::string>("base_frame_id", base_frame_id, "base_link");
    nh_priv.param<std::string>("map_frame_id", map_frame_id, "map");
    nh_priv.param<float >("publish_rate", publish_rate, 10.0);

    ROS_INFO("base_frame_id: %s", base_frame_id.c_str());
    ROS_INFO("map_frame_id: %s", map_frame_id.c_str());
    ROS_INFO("publish_rate: %f", publish_rate);

    ros::Publisher location_pub = nh.advertise < std_msgs::String > ("location", 10);

    tf::TransformListener listener;

    Places places(nh, nh, "places");

    ros::Rate rate(publish_rate);

    while (nh.ok()) 
    {
        tf::StampedTransform transform;
        ros::Time t = ros::Time(0);

        try 
        {
            listener.waitForTransform(map_frame_id, base_frame_id, t, ros::Duration(10.0));
            listener.lookupTransform(map_frame_id, base_frame_id, t, transform);
        } 
        catch (tf::TransformException ex) 
        {
            ROS_ERROR("%s", ex.what());
        }

        geometry_msgs::Point pt;

        pt.x = transform.getOrigin().getX();
        pt.y = transform.getOrigin().getY();

        std_msgs::String loc_msg;
        loc_msg.data = places.where_am_i(pt);
        location_pub.publish(loc_msg);

        rate.sleep();
    }
    return 0;
};