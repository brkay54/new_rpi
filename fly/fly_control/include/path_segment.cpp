
#include "path_segment.h"


double PathSegment::lenght() {
    if (this-> type == 1){
        return sqrt(pow(this->point_end.y- this->point_begin.y,2.0)+pow(this->point_end.x- this->point_begin.x,2.0));
    }
    else if (this-> type ==2){
        double R = sqrt(pow(this->point_end.y- this->center.y,2.0)+pow(this->point_end.x- this->center.x,2.0));
        double angle_begin=atan2((this->point_begin.y-this->center.y), (this->point_begin.x-this->center.x));
        double angle_end=atan2((this->point_end.y-this->center.y), (this->point_end.x-this->center.x));

        return (angle_end-angle_begin)<0.0 ? R*(angle_end-angle_begin+2*3.14159265358979323846) : R*(angle_end-angle_begin);

    }
}

geometry_msgs::Point PathSegment::direction(geometry_msgs::Point pos) {

        geometry_msgs::Point direction_of;
        double angle = atan2(pos.y-this->center.y , pos.x-this->center.x);
        double offset = 0.5;

        if(this->turn_direction){
            direction_of.x = sin(angle-offset);
            direction_of.y = -cos(angle-offset);
        }
        else{
            direction_of.x = -sin(angle+offset);
            direction_of.y = cos(angle+offset);
        }
        return direction_of;
}
geometry_msgs::Point PathSegment::direction() {
    geometry_msgs::Point direction_of;

    double len=sqrt((this->point_end.y-this->point_begin.y)*(this->point_end.y-this->point_begin.y)+(this->point_end.x-this->point_begin.x)*(this->point_end.x-this->point_begin.x));
    direction_of.y=(this->point_end.y-this->point_begin.y)/len;
    direction_of.x=(this->point_end.x-this->point_begin.x)/len;
    direction_of.z=0;
    return direction_of;

}
