

#include "commons.h"

//WGS84 constants for earth
const double _a_ = 6378137.0;
const double _e2_ = 0.00669437999014;
const double _k_ = 0.9933056199957391;

const double pi = 3.14159265358979323846;


geometry_msgs::Point commons::toEulerAngle(geometry_msgs::Quaternion q){  //from wikipedia
    geometry_msgs::Point euler_angles;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler_angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1)
        euler_angles.y = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        euler_angles.y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler_angles.z = std::atan2(siny_cosp, cosy_cosp);

    return euler_angles;
}

geometry_msgs::Point commons::line_direction(PathSegment a){
    geometry_msgs::Point direction_of;

    double len=sqrt((a.point_end.y-a.point_begin.y)*(a.point_end.y-a.point_begin.y)+(a.point_end.x-a.point_begin.x)*(a.point_end.x-a.point_begin.x));
    direction_of.y=(a.point_end.y-a.point_begin.y)/len;
    direction_of.x=(a.point_end.x-a.point_begin.x)/len;
    direction_of.z=0;
    return direction_of;
}

geometry_msgs::Point commons::projection_on_line(PathSegment l, geometry_msgs::Point a){

    PathSegment l2;
    l2.point_begin=a;

    l2.point_end.x= a.x - line_direction(l).y;
    l2.point_end.y= a.y + line_direction(l).x;


    return intersect_lines(l, l2);
}

geometry_msgs::Point commons::intersect_lines (PathSegment l1, PathSegment l2){

    double m1, n1, m2, n2;
    geometry_msgs::Point intersection;

    m1= (l1.point_end.y - l1.point_begin.y) / (l1.point_end.x - l1.point_begin.x);
    n1= l1.point_begin.y - m1*l1.point_begin.x;

    m2= (l2.point_end.y - l2.point_begin.y) / (l2.point_end.x - l2.point_begin.x);
    n2= l2.point_begin.y - m2*l2.point_begin.x;

    if(std::isinf(m1)){
        if(std::isinf(m2)){
            std::cout<<"no intersection for lines"<<std::endl;
        }
        else{
            intersection.x= l1.point_end.x;
            intersection.y= m2*intersection.x + n2;
        }
    }
    else{
        if(std::isinf(m2)){
            intersection.x= l2.point_end.x;
            intersection.y= m1*intersection.x + n1;
        }
        else{
            intersection.x= (n2-n1) / (m1-m2);
            intersection.y= m1*intersection.x + n1;
        }
    }

    return intersection;
}

double commons::angle_between_lines(PathSegment l1, PathSegment l2){
    double angle1 = atan2((l1.point_end.y- l1.point_begin.y),(l1.point_end.x-l1.point_begin.x));
    double angle2 = atan2((l2.point_end.y- l2.point_begin.y),(l2.point_end.x-l2.point_begin.x));
    return(angle2-angle1);

}

double commons::distance_between_points(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return(sqrt(pow(p2.y-p1.y , 2) + pow(p2.x-p1.x , 2)));
}

double commons::distance_to_line(PathSegment l, geometry_msgs::Point a){
    return distance_between_points(projection_on_line(l, a), a);

}

geometry_msgs::Point commons::global_to_local(geometry_msgs::Point target_global, geometry_msgs::Point vehicle_global, geometry_msgs::Point vehicle_local){

    double Xv, Yv, Zv, Xt, Yt, Zt; //ecef coords
    double N;

    vehicle_global.x = vehicle_global.x*pi/180.0;  //default global coordinates are in degrees. Transform them to radians.
    vehicle_global.y = vehicle_global.y*pi/180.0;

    target_global.x = target_global.x*pi/180;
    target_global.y = target_global.y*pi/180;

    //vehicle ecef coords
    N = _a_ / sqrt(1.0 - _e2_ * sin(vehicle_global.x) * sin(vehicle_global.x));

    Xv = (N + vehicle_global.z) * cos(vehicle_global.x) * cos(vehicle_global.y);
    Yv = (N + vehicle_global.z) * cos(vehicle_global.x) * sin(vehicle_global.y);
    Zv = (N * _k_ + vehicle_global.z) * sin(vehicle_global.x);

    //target ecef coords
    N = _a_ / sqrt(1.0 - _e2_ * sin(target_global.x) * sin(target_global.x));

    Xt = (N + target_global.z) * cos(target_global.x) * cos(target_global.y);
    Yt = (N + target_global.z) * cos(target_global.x) * sin(target_global.y);
    Zt = (N * _k_ + target_global.z) * sin(target_global.x);

    geometry_msgs::Point relative_local;

    //ecef to relative ENU
    relative_local.x = (Xt - Xv) * -sin(vehicle_global.y) + (Yt - Yv) * cos(vehicle_global.y) ;
    relative_local.y = (Xt - Xv) * -sin(vehicle_global.x)*cos(vehicle_global.y) + (Yt - Yv) * -sin(vehicle_global.x)*sin(vehicle_global.y) + (Zt - Zv) * cos(vehicle_global.x);
    relative_local.z = (Xt - Xv) * cos(vehicle_global.x)*cos(vehicle_global.y) + (Yt - Yv) * cos(vehicle_global.x)*sin(vehicle_global.y) + (Zt - Zv) * sin(vehicle_global.x);


    geometry_msgs::Point target_local;

    //ecef to absolute ENU
    target_local.x = relative_local.x + vehicle_local.x;
    target_local.y = relative_local.y + vehicle_local.y;
    target_local.z = relative_local.z + vehicle_local.z;


    return target_local;
}
