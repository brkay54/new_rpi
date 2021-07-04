
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "../include/commons.h"
#include "../include/path_plotter.h"
#include "../include/survey_path_generator.h"
#include <array>

double cruise_height;
double camera_vertical_fov;
double camera_horizontal_fov;
double overlap;
double max_distance_to_path;

namespace plt = matplotlibcpp;

geometry_msgs::PoseStamped vehicle_pose;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vehicle_pose = *msg;
}

geometry_msgs::Point global_pose;
void global_position_cb(const sensor_msgs::NavSatFix ::ConstPtr& msg)
{
    global_pose.x = msg->latitude;
    global_pose.y = msg->longitude;
    global_pose.z = msg->altitude;

}


geometry_msgs::TwistStamped BodyVel;
void body_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& twiststamped)
{
    BodyVel = *twiststamped;
}

geometry_msgs::Twist cmd_vel;
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& cmd_vell)
{
    cmd_vel = *cmd_vell;
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_plotter");
    ros::NodeHandle nh;


    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber body_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_body", 10, body_velocity_cb);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10, cmd_vel_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/global", 10, global_position_cb);

    ros::Rate rate(20.0);


    ros::param::get("drone/cruise_height", cruise_height);
    ros::param::get("camera_parameters/camera_vertical_fov", camera_vertical_fov);
    ros::param::get("camera_parameters/camera_horizontal_fov", camera_horizontal_fov);
    ros::param::get("camera_parameters/overlap", overlap);
    ros::param::get("drone/max_distance_to_path", max_distance_to_path);

    ros::Duration(10).sleep();
    survey_path_generator survey;
    //survey.d= (cruise_height * sin(camera_horizontal_fov/2.0) )*(1.0-overlap);
    survey.d=5;
    survey.read_from_file(vehicle_pose.pose.position, global_pose);
    survey.standardize();
    survey.generate_path();
    survey.add_arcs(survey.choose_side(vehicle_pose.pose.position));


    std::array<std::vector <double> , 2> path_points = path_plotter::generate_points(survey.survey_points);
    std::array<std::vector <double > ,2> local_poses;
    std::array<std::vector <double > ,4> cmd_vels;
    std::array<std::vector <double > ,4> body_vels;
    ROS_INFO("%d survey size   %f   %f", survey.survey_points.size(), survey.survey_points[0].point_begin.x, survey.survey_points[0].point_begin.y);

    double a=1;
    for(int i=0; i<4; i++){
        cmd_vels[i].push_back(a);
    }

    for(int i=0; i<4; i++){
        body_vels[i].push_back(a);
    }

    while (ros::ok()) {

        cmd_vels[0][0]=vehicle_pose.pose.position.x;
        cmd_vels[1][0]=vehicle_pose.pose.position.y;
        cmd_vels[2][0]=cmd_vel.linear.x;
        cmd_vels[3][0]=cmd_vel.linear.y;

        body_vels[0][0]=vehicle_pose.pose.position.x;
        body_vels[1][0]=vehicle_pose.pose.position.y;
        body_vels[2][0]=BodyVel.twist.linear.x;
        body_vels[3][0]=BodyVel.twist.linear.y;

        local_poses[0].push_back(vehicle_pose.pose.position.x);
        local_poses[1].push_back(vehicle_pose.pose.position.y);

        std::map <std::string, std::string> col = {{"color", "g"}};

        plt::clf();
        plt::plot(local_poses[0], local_poses[1], "r");
        plt::plot(path_points [0], path_points[1]);
        plt::quiver(cmd_vels[0],cmd_vels[1], cmd_vels[2], cmd_vels[3], col);
        plt::quiver(body_vels[0],body_vels[1], body_vels[2], body_vels[3] );


        plt::axis("equal");
        plt::pause(0.05);
        rate.sleep();
        ros::spinOnce();

    }

    return 0;
}
