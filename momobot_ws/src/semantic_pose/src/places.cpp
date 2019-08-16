#include "places.h"
#include "xmlrpc_helpers.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

Places::Places(ros::NodeHandle & nh, ros::NodeHandle & nh_priv,
    const std::string & param_name) {

    xh::Array output;
    xh::fetchParam(nh_priv, param_name, output);
    xh::Struct output_i;

    for (int i = 0; i < output.size(); ++i) 
    {
        XmlRpc::XmlRpcValue name_list;
        XmlRpc::XmlRpcValue pos_list;
        std::vector<geometry_msgs::Point> pos;

        xh::getArrayItem(output, i, output_i);
        xh::getStructMember(output_i, "boundary", pos_list);
        xh::getStructMember(output_i, "name", name_list);

        for(int j=0; j < pos_list.size(); ++j)
        {
            geometry_msgs::Point pt;

            pt.x = pos_list[j][0];
            pt.y = pos_list[j][1];

            pos.push_back(pt);
        }

        boundary_.push_back(pos);
        places_.push_back(name_list);
    }
}

std::vector<std::string> Places::getPlaces()
{
    return places_;
}

std::vector<std::vector<geometry_msgs::Point> > Places::getBoundary()
{
    return boundary_;
}

std::string Places::where_am_i(geometry_msgs::Point pt)
{
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;

    for(int i = 0; i < boundary_.size() ; i++)
    {
        std::string polygon = "POLYGON((";

        for(int j=0; j < boundary_[i].size(); j++)
        {
            polygon.append(std::to_string(boundary_[i][j].y));
            polygon.append(" ");
            polygon.append(std::to_string(boundary_[i][j].x));
            polygon.append(",");
        }

        polygon.append(std::to_string(boundary_[i][0].y));
        polygon.append(" ");
        polygon.append(std::to_string(boundary_[i][0].x));
        polygon.append("))");

        polygon_type poly;
        boost::geometry::read_wkt(polygon, poly);

        point_type p(pt.y, pt.x);

        if(boost::geometry::within(p, poly))
            return places_[i];
    }
    return "";
}