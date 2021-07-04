//
// Created by ahmet on 18.01.2021.
//

#ifndef SRC_PATH_SEGMENT_H
#define SRC_PATH_SEGMENT_H
#include <geometry_msgs/Point.h>



class PathSegment {
public:
    geometry_msgs::Point point_begin;
    geometry_msgs::Point point_end;
    geometry_msgs::Point center;
    bool turn_direction;  //1:clockwise, 0:counterclockwise
    uint8_t type;   //1: line, else: arc
    virtual double lenght();
    virtual geometry_msgs::Point direction();
    virtual geometry_msgs::Point direction(geometry_msgs::Point);

};


#endif //SRC_PATH_SEGMENT_H
