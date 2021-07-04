#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "../include/commons.h"
#include <fly_msgs/Line.h>
#include <fly_msgs/Arc.h>
#include <nav_msgs/Odometry.h>
#include <fly_msgs/Int.h>

const double pi = 3.14159265358979;





geometry_msgs::PoseStamped vehicle_pose;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vehicle_pose = *msg;
}

geometry_msgs::TwistStamped BodyVel;
void body_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    BodyVel = *msg;
}

geometry_msgs::Point target_point;
void target_point_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    target_point = *msg;
}




float look_ahead_dist =5;  //in meters. for carrot guidance.
float cruise_vel = 2;  //in meters/sec. maximum and straight line velocity.
float vehicle_width = 0.71;   //in meters. distance between right-left wheels.
float Pgain_direction= 1;
float Dgain_direction= -0.05;//controller gains for reaching commanded direction.


geometry_msgs::Twist line_follower(PathSegment l, geometry_msgs::PoseStamped vehicle_pose){    //implements the carrot guidance algorithm. output is either thrust + angular velocity or left wheel velocity + right wheel velocity.


    double theta_command;
    double theta_current = commons::toEulerAngle(vehicle_pose.pose.orientation).z;
    double theta_rate_current = BodyVel.twist.angular.z;
    double theta_error;
    float vel_right_wheel;
    float vel_left_wheel;
    bool turn_direction;  //0:clockwise, 1:counter clockwise



    if((l.point_end.y==l.point_begin.y && l.point_end.x ==l.point_begin.x) &&(l.point_end.y!=0 || l.point_end.x!=0)){

        theta_command = atan2(l.point_end.y, l.point_end.x);

    }
    else{
        geometry_msgs::Point projected_point;
        projected_point.x = commons::projection_on_line(l, vehicle_pose.pose.position).x;
        projected_point.y = commons::projection_on_line(l, vehicle_pose.pose.position).y;

        geometry_msgs::Point ahead_point;
        ahead_point.x = projected_point.x + commons::line_direction(l).x*look_ahead_dist;
        ahead_point.y = projected_point.y + commons::line_direction(l).y*look_ahead_dist;

        theta_command = atan2(ahead_point.y - vehicle_pose.pose.position.y , ahead_point.x - vehicle_pose.pose.position.x );

    }

    if(theta_command<0){theta_command=theta_command + 2*pi;}

    if(theta_current<0){theta_current=theta_current + 2*pi;}


    if(theta_command>theta_current){    //chooses the turn direction
        if(theta_command-theta_current<pi)  {turn_direction=1;}
        else                                {turn_direction=0;}
    }
    else{
        if(theta_current-theta_command<pi)  {turn_direction=0; }
        else                                {turn_direction=1;}
    }

    if(turn_direction)  {theta_error=theta_command-theta_current;   }
    else                {theta_error=theta_current-theta_command;   theta_rate_current=-theta_rate_current;}

    if(theta_error<0) {theta_error=theta_error + 2*pi;}

    if(l.point_end.y==l.point_begin.y && l.point_end.x ==l.point_begin.x){
        if(l.point_end.y==0 && l.point_end.x==0){
            vel_right_wheel=0;
            vel_left_wheel=0;
        }
        else {
            if (turn_direction) {
                vel_right_wheel = vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);  //thera_rate_current is also equal to d(theta_error)/d(t) and has less noise.
                vel_left_wheel = -vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);
            }
            else {
                vel_right_wheel = -vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);
                vel_left_wheel = vehicle_width * (Pgain_direction * theta_error + Dgain_direction * theta_rate_current);
            }
        }
    }
    else{
        if(turn_direction){
            vel_right_wheel = cruise_vel*((pi-theta_error)/pi) + vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);  //thera_rate_current is also equal to d(theta_error)/d(t) and has less noise.
            vel_left_wheel   = cruise_vel*((pi-theta_error)/pi) - vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);
        }
        else{
            vel_right_wheel = cruise_vel*((pi-theta_error)/pi) - vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);
            vel_left_wheel   = cruise_vel*((pi-theta_error)/pi) + vehicle_width*(Pgain_direction*theta_error + Dgain_direction*theta_rate_current);
        }
    }





    geometry_msgs::Twist output;

    output.linear.x = (vel_right_wheel+vel_left_wheel)/2.0;
    output.angular.z= (vel_right_wheel-vel_left_wheel)/vehicle_width;

    return output;


}

int main(int argc, char **argv){

    ros::init(argc, argv, "path_folower");
    ros::NodeHandle nh;


    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber body_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_body", 10, body_velocity_cb);

    ros::Publisher auto_vel_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
    ros::ServiceClient client ;
    std::string modelName = (std::string)"jaguar" ;
    std::string relativeEntityName = (std::string)"world" ;


    PathSegment stop_line;
    stop_line.point_end.x=0;
    stop_line.point_end.y=0;
    stop_line.point_begin.x=0;
    stop_line.point_begin.y=0;

    ros::Duration(30.0).sleep();

    while (ros::ok()){

        PathSegment current_line;

        //current_line.point_begin.x = x;
        //current_line.point_begin.y = y;
        current_line.point_end.x = target_point.x;
        current_line.point_end.y = target_point.y;


        if(commons::distance_between_points(vehicle_pose.pose.position, current_line.point_end) >1){
            auto_vel_pub.publish(line_follower(current_line,vehicle_pose));
        }
        else{
            auto_vel_pub.publish(line_follower(stop_line,vehicle_pose));
        }

        ros::spinOnce();
        rate.sleep();

    }


}
