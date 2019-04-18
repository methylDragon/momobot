#ifndef _PLACES_H_
#define _PLACES_H_

#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Point.h>

class Places
{
    public:
        Places(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, const std::string& param_name);
        std::vector<std::string> getPlaces();
        std::vector<std::vector<geometry_msgs::Point> > getBoundary();
        std::string where_am_i(geometry_msgs::Point pt);

    private:
        std::vector<std::string> places_;
        std::vector<std::vector<geometry_msgs::Point> > boundary_;
};

#endif