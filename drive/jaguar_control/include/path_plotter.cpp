//
// Created by ahmet on 16.01.2021.
//

#include "path_plotter.h"


namespace plt = matplotlibcpp;

void path_plotter::plot (std::vector<PathSegment>& ppath){

    double s_size= 0.1;
    for(int i=0; i<ppath.size(); i++){
        std::vector <double> x,y;

        if (ppath[i].type==1) {
            for (int j = 0; double(j) * s_size < ppath[i].lenght(); j++) {
                x.push_back(ppath[i].point_begin.x + double(j) * s_size * ppath[i].direction().x);
                y.push_back(ppath[i].point_begin.y + double(j) * s_size * ppath[i].direction().y);
            }
        }

        else if(ppath[i].type==2){
            double radius = sqrt(pow(ppath[i].point_end.y- ppath[i].center.y,2.0)+pow(ppath[i].point_end.x- ppath[i].center.x,2.0));
            double angle_begin=atan2((ppath[i].point_begin.y-ppath[i].center.y), (ppath[i].point_begin.x-ppath[i].center.x));
            double angle_end=atan2((ppath[i].point_end.y-ppath[i].center.y), (ppath[i].point_end.x-ppath[i].center.x));
            double d_angle = s_size/radius;
            double angle = ppath[i].turn_direction ? angle_end-angle_begin : angle_begin-angle_end;

            angle = angle<0 ?  angle+ 2*3.14159265358979323846 :  angle;
            for(int j=0; double(j)*d_angle<angle; j++) {
                if(ppath[i].turn_direction){
                    x.push_back(ppath[i].center.x + radius * cos(-double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(-double(j) * d_angle + angle_begin));
                }
                else {
                    x.push_back(ppath[i].center.x + radius * cos(double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(double(j) * d_angle + angle_begin));
                }


            }

        }
        plt::plot(x, y);

    }
    plt::axis("equal");
    plt::show();

}

void path_plotter::plot (std::vector<PathSegment>& ppath, std::vector<geometry_msgs::Point> Points){

    double s_size= 0.1;
    for(int i=0; i<ppath.size(); i++){
        std::vector <double> x,y;

        if (ppath[i].type==1) {
            for (int j = 0; double(j) * s_size < ppath[i].lenght(); j++) {
                x.push_back(ppath[i].point_begin.x + double(j) * s_size * ppath[i].direction().x);
                y.push_back(ppath[i].point_begin.y + double(j) * s_size * ppath[i].direction().y);
            }
        }

        else if(ppath[i].type==2){
            double radius = sqrt(pow(ppath[i].point_end.y- ppath[i].center.y,2.0)+pow(ppath[i].point_end.x- ppath[i].center.x,2.0));
            double angle_begin=atan2((ppath[i].point_begin.y-ppath[i].center.y), (ppath[i].point_begin.x-ppath[i].center.x));
            double angle_end=atan2((ppath[i].point_end.y-ppath[i].center.y), (ppath[i].point_end.x-ppath[i].center.x));
            double d_angle = s_size/radius;
            double angle = ppath[i].turn_direction ? angle_end-angle_begin : angle_begin-angle_end;

            angle = angle<0 ?  angle+ 2*3.14159265358979323846 :  angle;
            for(int j=0; double(j)*d_angle<angle; j++) {
                if(ppath[i].turn_direction){
                    x.push_back(ppath[i].center.x + radius * cos(-double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(-double(j) * d_angle + angle_begin));
                }
                else {
                    x.push_back(ppath[i].center.x + radius * cos(double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(double(j) * d_angle + angle_begin));
                }


            }

        }
        if(ppath[i].type==1){
            plt::plot(x, y , "b");
        }
        else if(ppath[i].type==2){
            plt::plot(x, y , "r");
        }

    }
    std::vector <double> Xp, Yp;
    for(int i=0; i<Points.size(); i++){
        Xp.push_back(Points[i].x);
        Yp.push_back(Points[i].y);

    }
    Xp.push_back(Points[0].x);
    Yp.push_back(Points[0].y);

    plt::plot(Xp,Yp, "g");
    plt::axis("equal");
    plt::show();

}

std::array<std::vector<double> ,2> path_plotter::generate_points (std::vector<PathSegment>& ppath){

    double s_size= 0.1;
    std::array<std::vector<double> ,2> points;
    std::vector <double> x,y;

    for(int i=0; i<ppath.size(); i++){

        if (ppath[i].type==1) {
            for (int j = 0; double(j) * s_size < ppath[i].lenght(); j++) {
                x.push_back(ppath[i].point_begin.x + double(j) * s_size * ppath[i].direction().x);
                y.push_back(ppath[i].point_begin.y + double(j) * s_size * ppath[i].direction().y);
            }
        }

        else if(ppath[i].type==2){
            double radius = sqrt(pow(ppath[i].point_end.y- ppath[i].center.y,2.0)+pow(ppath[i].point_end.x- ppath[i].center.x,2.0));
            double angle_begin=atan2((ppath[i].point_begin.y-ppath[i].center.y), (ppath[i].point_begin.x-ppath[i].center.x));
            double angle_end=atan2((ppath[i].point_end.y-ppath[i].center.y), (ppath[i].point_end.x-ppath[i].center.x));
            double d_angle = s_size/radius;
            double angle = ppath[i].turn_direction ? angle_end-angle_begin : angle_begin-angle_end;

            angle = angle<0 ?  angle+ 2*3.14159265358979323846 :  angle;
            for(int j=0; double(j)*d_angle<angle; j++) {
                if(ppath[i].turn_direction){
                    x.push_back(ppath[i].center.x + radius * cos(-double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(-double(j) * d_angle + angle_begin));
                }
                else {
                    x.push_back(ppath[i].center.x + radius * cos(double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(double(j) * d_angle + angle_begin));
                }


            }

        }

    }

    for(int i=0; i<x.size(); i++){
        points[0].push_back(x[i]);
        points[1].push_back(y[i]);

    }
   return points;

}

void path_plotter::plot (std::vector<PathSegment>& ppath, double s_s){

    double s_size= s_s;
    for(int i=0; i<ppath.size(); i++){
        std::vector <double> x,y;

        if (ppath[i].type==1) {
            for (int j = 0; double(j) * s_size < ppath[i].lenght(); j++) {
                x.push_back(ppath[i].point_begin.x + double(j) * s_size * ppath[i].direction().x);
                y.push_back(ppath[i].point_begin.y + double(j) * s_size * ppath[i].direction().y);
            }
        }

        else if(ppath[i].type==2){
            double radius = sqrt(pow(ppath[i].point_end.y- ppath[i].center.y,2.0)+pow(ppath[i].point_end.x- ppath[i].center.x,2.0));
            double angle_begin=atan2((ppath[i].point_begin.y-ppath[i].center.y), (ppath[i].point_begin.x-ppath[i].center.x));
            double angle_end=atan2((ppath[i].point_end.y-ppath[i].center.y), (ppath[i].point_end.x-ppath[i].center.x));
            double d_angle = s_size/radius;
            double angle = ppath[i].turn_direction ? angle_end-angle_begin : angle_begin-angle_end;

            angle = angle<0 ?  angle+ 2*3.14159265358979323846 :  angle;
            for(int j=0; double(j)*d_angle<angle; j++) {
                if(ppath[i].turn_direction){
                    x.push_back(ppath[i].center.x + radius * cos(-double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(-double(j) * d_angle + angle_begin));
                }
                else {
                    x.push_back(ppath[i].center.x + radius * cos(double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(double(j) * d_angle + angle_begin));
                }


            }

        }
        plt::plot(x, y);
    }
    plt::axis("equal");
    plt::show();

}

void path_plotter::plot (std::vector<PathSegment>& ppath, double s_s, std::string colour ){

    double s_size= s_s;
    for(int i=0; i<ppath.size(); i++){
        std::vector <double> x,y;
        std::cout<<ppath[i].lenght()<<std::endl;
        if (ppath[i].type==1) {
            for (int j = 0; double(j) * s_size < ppath[i].lenght(); j++) {
                x.push_back(ppath[i].point_begin.x + double(j) * s_size * ppath[i].direction().x);
                y.push_back(ppath[i].point_begin.y + double(j) * s_size * ppath[i].direction().y);
            }
        }

        else if(ppath[i].type==2){
            double radius = sqrt(pow(ppath[i].point_end.y- ppath[i].center.y,2.0)+pow(ppath[i].point_end.x- ppath[i].center.x,2.0));
            double angle_begin=atan2((ppath[i].point_begin.y-ppath[i].center.y), (ppath[i].point_begin.x-ppath[i].center.x));
            double angle_end=atan2((ppath[i].point_end.y-ppath[i].center.y), (ppath[i].point_end.x-ppath[i].center.x));
            double d_angle = s_size/radius;
            double angle = ppath[i].turn_direction ? angle_begin-angle_end : angle_end-angle_begin;
            std::cout<<radius<<std::endl;

            angle = angle<0 ?  angle+ 2*3.14159265358979323846 :  angle;
            for(int j=0; double(j)*d_angle<angle; j++) {
                if(ppath[i].turn_direction){
                    x.push_back(ppath[i].center.x + radius * cos(-double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(-double(j) * d_angle + angle_begin));
                }
                else {
                    x.push_back(ppath[i].center.x + radius * cos(double(j) * d_angle + angle_begin));
                    y.push_back(ppath[i].center.y + radius * sin(double(j) * d_angle + angle_begin));
                }


            }

        }
        plt::plot(x, y, colour);

    }
    plt::axis("equal");
    plt::show();

}