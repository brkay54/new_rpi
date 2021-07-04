//
// Created by ahmet on 16.01.2021.
//

#ifndef SRC_PATH_PLOTTER_H
#define SRC_PATH_PLOTTER_H

#include "matplotlibcpp.h"
#include "path_segment.h"
#include <string.h>
#include <array>


class path_plotter {
public:
    static void plot(std::vector<PathSegment>& ppath);
    static void plot(std::vector<PathSegment>& ppath, std::vector<geometry_msgs::Point> Points);
    static void plot(std::vector<PathSegment>& ppath, double step_size );
    static void plot(std::vector<PathSegment>& ppath, double step_size, std::string clour );
    static std::array<std::vector<double>, 2> generate_points (std::vector<PathSegment>& ppath);

};


#endif //SRC_PATH_PLOTTER_H
