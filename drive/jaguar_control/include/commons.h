//
// Created by ahmet on 1.12.2020.
//

#ifndef SRC_COMMONS_H
#define SRC_COMMONS_H

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "path_segment.h"




class commons {
public:
    static geometry_msgs::Point toEulerAngle(geometry_msgs::Quaternion q);
    static geometry_msgs::Point line_direction(PathSegment a);
    static geometry_msgs::Point projection_on_line(PathSegment l, geometry_msgs::Point a);
    static double distance_to_line(PathSegment l, geometry_msgs::Point a);
    static geometry_msgs::Point intersect_lines(PathSegment l1, PathSegment l2);
    static double angle_between_lines(PathSegment l1, PathSegment l2);
    static double distance_between_points(geometry_msgs::Point p1, geometry_msgs::Point p2);
    static geometry_msgs::Point global_to_local (geometry_msgs::Point target_global, geometry_msgs::Point vehicle_global, geometry_msgs::Point vehicle_local);
};


#endif //SRC_COMMONS_H
