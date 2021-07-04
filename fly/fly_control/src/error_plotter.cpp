
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "../include/commons.h"
#include "../include/path_plotter.h"
#include "../include/survey_path_generator.h"
#include <fly_msgs/Int.h>
#include <array>

const double pi = 3.14159265358979323846;


namespace plt = matplotlibcpp;

geometry_msgs::PoseStamped vehicle_pose;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    vehicle_pose = *pose;
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

fly_msgs::Int current_segment;
void current_segment_cb(const fly_msgs::Int::ConstPtr& current)
{
    current_segment = *current;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "error_plotter");
    ros::NodeHandle nh;


    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber body_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_body", 10, body_velocity_cb);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10, cmd_vel_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/global", 10, global_position_cb);

    ros::Subscriber segment_sub = nh.subscribe<fly_msgs::Int>
            ("current_segment_index", 10, current_segment_cb);
    ros::Rate rate(20.0);




    survey_path_generator survey;
    survey.d = 5;

    survey.read_from_file(vehicle_pose.pose.position, global_pose);
    survey.generate_path();
    survey.add_arcs(0);

    std::vector <double> error;
    while(commons::distance_between_points(vehicle_pose.pose.position, survey.survey_points[current_segment.index].point_begin) >2){
        rate.sleep();
        ros::spinOnce();
    }

    while (ros::ok()) {

        plt::clf();

        if(survey.survey_points[current_segment.index].type==1){
            error.push_back(commons::distance_between_points(commons::projection_on_line(survey.survey_points[current_segment.index],vehicle_pose.pose.position),  vehicle_pose.pose.position));
            plt::plot(error, "g");

        }
        else{
            error.push_back(commons::distance_between_points(vehicle_pose.pose.position, survey.survey_points[current_segment.index].center) - commons::distance_between_points(survey.survey_points[current_segment.index].center, survey.survey_points[current_segment.index].point_begin));
            plt::plot(error, "b");

        }
        if(error.size()>100){
            error.erase(error.begin());
        }

        //plt::axis("equal");
        plt::pause(0.05);
        rate.sleep();
        ros::spinOnce();

    }

    return 0;
}
