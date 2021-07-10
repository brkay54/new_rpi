//
// Created by ahmet on 6.01.2021.
//

#ifndef SRC_SURVEY_PATH_GENERATOR_H
#define SRC_SURVEY_PATH_GENERATOR_H

#include <cmath>
#include <geometry_msgs/Point.h>
#include <fly_msgs/Line.h>
#include "commons.h"
#include <array>
#include <fstream>
#include "path_segment.h"
#include <bits/stdc++.h>
#include <ros/package.h>





class survey_path_generator {
public:
    std::vector <PathSegment> survey_points;
    std::vector <geometry_msgs::Point> Polygon;
    geometry_msgs::Point sweep_line_direction;
    double d;
    void read_from_file(std::string file_name);  //works if local coordinates are written in the file. Not suitable for real vehicles
    void read_from_file(std::string file_name, geometry_msgs::Point vehicle_local, geometry_msgs::Point vehicle_global);  //transforms global coordinates to positions in local frame of the vehicle
    void standardize();   //looking from upside, generate_path works only if points are in clockwise order. This function orders the input
    void generate_path();  //generates survey points in local coordinates
    void add_arcs(int side);  //adds arcs to the path to connect lines
    int choose_side(geometry_msgs::Point vehicle_pose);  //chooses the direction and the beginning point of the survey path to minimize distance to beginning point

};


#endif //SRC_SURVEY_PATH_GENERATOR_H
