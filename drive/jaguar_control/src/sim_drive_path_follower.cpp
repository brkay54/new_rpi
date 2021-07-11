
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/commons.h"
#include <fly_msgs/Line.h>
#include "../include/survey_path_generator.h"
#include <sensor_msgs/NavSatFix.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>


const double pi = 3.14159265358979;


double camera_vertical_fov;
double camera_horizontal_fov;
double overlap;

double look_ahead_distance;
double cruise_velocity;
double P_gain_direction;
double D_gain_direction;
double mission_reach_treshold;
double vehicle_width;
double cruise_height;
double max_distance_to_path;
std::string coordinate_file_name;


int mode = 2;  //1:manual, 2:autonomous
int a_button = 0;
int y_button = 0;
int prev_y_button, prev_a_button;

sensor_msgs::Joy joystick;
void joy_val_cb(const sensor_msgs::Joy::ConstPtr &msg) {
    joystick = *msg;

    prev_a_button = a_button;
    prev_y_button = y_button;
    a_button = joystick.buttons[0];
    y_button = joystick.buttons[3];

    if (prev_a_button != a_button) {
        mode = 1;
    }
    if (prev_y_button != y_button) {
        mode = 2;
    }

}
geometry_msgs::PoseStamped vehicle_pose;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    vehicle_pose = *msg;
}

geometry_msgs::TwistStamped BodyVel;

void body_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    BodyVel = *msg;
}

geometry_msgs::Point global_pose;
void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    global_pose.x = msg->latitude;
    global_pose.y = msg->longitude;
    global_pose.z = msg->altitude;

}

gazebo::msgs::Twist line_follower(PathSegment l,
                                   geometry_msgs::PoseStamped vehicle_pose) {    //implements the carrot guidance algorithm. output is either thrust + angular velocity or left wheel velocity + right wheel velocity.


    double theta_command;
    double theta_current = commons::toEulerAngle(vehicle_pose.pose.orientation).z;
    double theta_rate_current = BodyVel.twist.angular.z;
    double theta_error;
    float vel_right_wheel;
    float vel_left_wheel;
    bool turn_direction;  //0:clockwise, 1:counter clockwise



    if ((l.point_end.y == l.point_begin.y && l.point_end.x == l.point_begin.x) &&
        (l.point_end.y != 0 || l.point_end.x != 0)) {

        theta_command = atan2(l.point_end.y, l.point_end.x);

    } else {
        geometry_msgs::Point projected_point;
        projected_point.x = commons::projection_on_line(l, vehicle_pose.pose.position).x;
        projected_point.y = commons::projection_on_line(l, vehicle_pose.pose.position).y;

        geometry_msgs::Point ahead_point;
        ahead_point.x = projected_point.x + commons::line_direction(l).x * look_ahead_distance;
        ahead_point.y = projected_point.y + commons::line_direction(l).y * look_ahead_distance;

        theta_command = atan2(ahead_point.y - vehicle_pose.pose.position.y,
                              ahead_point.x - vehicle_pose.pose.position.x);

    }

    if (theta_command < 0) { theta_command = theta_command + 2 * pi; }

    if (theta_current < 0) { theta_current = theta_current + 2 * pi; }


    if (theta_command > theta_current) {    //chooses the turn direction
        if (theta_command - theta_current < pi) { turn_direction = 1; }
        else { turn_direction = 0; }
    } else {
        if (theta_current - theta_command < pi) { turn_direction = 0; }
        else { turn_direction = 1; }
    }

    if (turn_direction) { theta_error = theta_command - theta_current; }
    else {
        theta_error = theta_current - theta_command;
        theta_rate_current = -theta_rate_current;
    }

    if (theta_error < 0) { theta_error = theta_error + 2 * pi; }

    if (l.point_end.y == l.point_begin.y && l.point_end.x == l.point_begin.x) {
        if (l.point_end.y == 0 && l.point_end.x == 0) {
            vel_right_wheel = 0;
            vel_left_wheel = 0;
        } else {
            if (turn_direction) {
                vel_right_wheel = vehicle_width * (P_gain_direction * theta_error + D_gain_direction *
                                                                                    theta_rate_current);  //thera_rate_current is also equal to d(theta_error)/d(t) and has less noise.
                vel_left_wheel =
                        -vehicle_width * (P_gain_direction * theta_error + D_gain_direction * theta_rate_current);
            } else {
                vel_right_wheel =
                        -vehicle_width * (P_gain_direction * theta_error + D_gain_direction * theta_rate_current);
                vel_left_wheel =
                        vehicle_width * (P_gain_direction * theta_error + D_gain_direction * theta_rate_current);
            }
        }
    } else {
        if (turn_direction) {
            vel_right_wheel =
                    cruise_velocity * ((pi - theta_error) / pi) + vehicle_width * (P_gain_direction * theta_error +
                                                                                   D_gain_direction *
                                                                                   theta_rate_current);  //thera_rate_current is also equal to d(theta_error)/d(t) and has less noise.
            vel_left_wheel = cruise_velocity * ((pi - theta_error) / pi) -
                             vehicle_width * (P_gain_direction * theta_error + D_gain_direction * theta_rate_current);
        } else {
            vel_right_wheel = cruise_velocity * ((pi - theta_error) / pi) -
                              vehicle_width * (P_gain_direction * theta_error + D_gain_direction * theta_rate_current);
            vel_left_wheel = cruise_velocity * ((pi - theta_error) / pi) +
                             vehicle_width * (P_gain_direction * theta_error + D_gain_direction * theta_rate_current);
        }
    }

    gazebo::msgs::Twist output;
    output.mutable_linear()->set_x((vel_right_wheel + vel_left_wheel) / 2.0);
    output.mutable_linear()->set_z(0);
    output.mutable_linear()->set_y(0);
    output.mutable_angular()->set_x(0);
    output.mutable_angular()->set_y(0);
    output.mutable_angular()->set_z((vel_right_wheel - vel_left_wheel) / vehicle_width);

    return output;


}


int main(int argc, char **argv) {

    ros::init(argc, argv, "sim_drive_path_follower");
    ros::NodeHandle nh;

    ros::Subscriber joy_val_sub = nh.subscribe<sensor_msgs::Joy>("j0", 10, joy_val_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/ugv/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber body_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/ugv/mavros/local_position/velocity_body", 10, body_velocity_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/ugv/mavros/global_position/global", 10, global_position_cb);
    ros::Rate rate(50);

    gazebo::client::setup(argc, argv);

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::PublisherPtr pub =
            node->Advertise<gazebo::msgs::Twist>("~/jaguar_0/cmd_vel_twist");
    ROS_INFO("asd");
    pub->WaitForConnection();
    ROS_INFO("asd2");
    ros::param::get("camera_parameters/camera_vertical_fov", camera_vertical_fov);
    ros::param::get("camera_parameters/camera_horizontal_fov", camera_horizontal_fov);
    ros::param::get("camera_parameters/overlap", overlap);
    ros::param::get("rover/look_ahead_distance", look_ahead_distance);
    ros::param::get("rover/cruise_velocity", cruise_velocity);
    ros::param::get("rover/P_gain_direction", P_gain_direction);
    ros::param::get("rover/D_gain_direction", D_gain_direction);
    ros::param::get("rover/mission_reach_treshold", mission_reach_treshold);
    ros::param::get("rover/vehicle_width", vehicle_width);
    ros::param::get("drone/cruise_height", cruise_height);
    ros::param::get("drone/max_distance_to_path", max_distance_to_path);
    ros::param::get("coordinate_file_name", coordinate_file_name);


    ros::Duration(2.0).sleep();
    ros::spinOnce();
    rate.sleep();

    while (ros::ok() && mode == 0) {
        ROS_INFO("Waiting for button to generate path");
        rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Generating Survey Path");

    std::string address = ros::package::getPath("fly_control");
    address = address + "/config/" + coordinate_file_name;

    char file_name[1024];
    strcpy(file_name, address.c_str());
    ROS_INFO("reading coordinates from: %s", file_name);

    survey_path_generator survey;
    survey.d = (cruise_height * sin(camera_horizontal_fov / 2.0)) * (1.0 - overlap);
    //survey.d=5;

    survey.read_from_file(file_name, vehicle_pose.pose.position, global_pose);

    ROS_INFO("Number of coordinates read: %d", survey.Polygon.size());

    survey.standardize();

    for (int i = 0; i < survey.Polygon.size(); i++) {
        ROS_INFO("%f  %f", survey.Polygon[i].x, survey.Polygon[i].y);

    }
    survey.generate_path();
    ROS_INFO("Path generated, containing %d lines", survey.survey_points.size());

    survey.add_arcs(survey.choose_side(vehicle_pose.pose.position));

    ROS_INFO("Arcs added, total path contains %d segments", survey.survey_points.size());
    ROS_INFO("Beginning point:   %f   %f", survey.survey_points[0].point_begin.x,
             survey.survey_points[0].point_begin.y);


    if (commons::distance_between_points(vehicle_pose.pose.position, survey.survey_points[0].point_begin) >
        max_distance_to_path) {
        ROS_INFO("Survey path is further than allowed distance , aborting mission %f",
                 commons::distance_between_points(vehicle_pose.pose.position, survey.survey_points[0].point_begin));
        //return 0;

    }
    ROS_INFO("Survey path generated succesfuly, starting mission");

    geometry_msgs::Point target_point;
/*   to determine a position target manually.
    target_point.x = 41.0854138;
    target_point.y = 29.041259;
    target_point.z = global_pose.z;
*/

    PathSegment target_line;
    target_line.point_begin = vehicle_pose.pose.position;
    //target_line.point_end = commons::global_to_local(target_point, global_pose, vehicle_pose.pose.position);
    target_line.point_end = survey.survey_points[survey.survey_points.size()-1].point_end;


    PathSegment stop_line; //line follower stops the vehicle when 0,0,0  0,0,0 line is sent
    stop_line.point_begin.x=0;
    stop_line.point_begin.y=0;
    stop_line.point_begin.z=0;
    stop_line.point_end=stop_line.point_begin;


    gazebo::msgs::Twist output;
        while (ros::ok()) {
        if (mode == 1) {
                output.mutable_linear()->set_x(0.5);
                output.mutable_linear()->set_z(0);
                output.mutable_linear()->set_y(0);
                output.mutable_angular()->set_x(0);
                output.mutable_angular()->set_y(0);
                output.mutable_angular()->set_z(0.5);
                pub->Publish(output);



            ROS_INFO("Manual");
            std::cout << output.linear().x() << ", " << output.angular().z() <<std::endl;


        } else if (mode == 2) {

            output = line_follower(target_line, vehicle_pose);
            pub->Publish(output);
            ROS_INFO("Autonomous");
            if (commons::distance_between_points(commons::projection_on_line(target_line, vehicle_pose.pose.position),
                                                 target_line.point_end) < mission_reach_treshold) {
                target_line=stop_line;
            }

        }

        rate.sleep();
        ros::spinOnce();
    }
    gazebo::client::shutdown();

    return 0;


}

//
// Created by ahmet on 10.07.2021.
//

