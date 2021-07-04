
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs//NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <fly_msgs/Line.h>
#include <fly_msgs/Arc.h>
#include <fly_msgs/Int.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <fiducial_msgs/FiducialTransform.h>

#include "../include/survey_path_generator.h"
#include "../include/commons.h"

const double pi = 3.14159265358979323846;
const int frequency = 50;

int simulation;
double look_ahead_distance;
double cruise_velocity;
double cruise_height;
double Pz_gain;
double Dz_gain;
double Pa_gain;
double Da_gain;
double Pp_gain;
double Dp_gain;
double Ip_gain;
double integration_limit;
double descent_rate;
double max_position_control_velocity;
double camera_vertical_fov;
double camera_horizontal_fov;
double overlap;
double max_distance_to_path;

double prev_x_err, prev_y_err, x_err_integrated, y_err_integrated;

int current_segment = 0;
fly_msgs::Int pub_segment;


geometry_msgs::Point rover_pose;


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped vehicle_pose;

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    vehicle_pose = *msg;
}

geometry_msgs::TwistStamped vehicle_velocity;

void body_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    vehicle_velocity = *msg;
}

geometry_msgs::Point global_pose;

void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    global_pose.x = msg->latitude;
    global_pose.y = msg->longitude;
    global_pose.z = msg->altitude;

}

geometry_msgs::Point euler;
geometry_msgs::Point rotate_point (geometry_msgs::Point point ){

    euler = commons::toEulerAngle(vehicle_pose.pose.orientation);

    geometry_msgs::Point output;

    double cx=cos(-euler.x);
    double sx=sin(-euler.x);
    double cy=cos(-euler.y);
    double sy=sin(-euler.y);
    double cz=cos(-euler.z);
    double sz=sin(-euler.z);

    output.x = point.x * (cz*cy) + point.y * (cz*sy*sx - sz*cx) + point.z * (cz*sy*cx + sz*sx);
    output.y = point.x * (sz*cy) + point.y * (sz*sy*sx + cz*cx) + point.z * (sz*sy*cx - cz*sx);
    output.z = point.x * (-sy) + point.y * (cy*sx) + point.z * (cy*cx);

    return output;
}
geometry_msgs::Point camera_relative;
bool rover_seen = false;

void arucoCallback(fiducial_msgs::FiducialTransformArray_<std::allocator<void>> msg) {
    if(msg.transforms.size() !=0){

        rover_seen = true;
        camera_relative.x = msg.transforms[0].transform.translation.x;
        camera_relative.y = -msg.transforms[0].transform.translation.y;
        camera_relative.z = -msg.transforms[0].transform.translation.z;


        rover_pose = rotate_point(camera_relative);
    }
    else{
        rover_seen = false;
    }
}


double altitude_control(double cruise_height) {
    return (Pz_gain * (cruise_height - vehicle_pose.pose.position.z) + Dz_gain * vehicle_velocity.twist.linear.z) > 2
           ? 2 : (Pz_gain * (cruise_height - vehicle_pose.pose.position.z) + Dz_gain * vehicle_velocity.twist.linear.z);
    //check direction of vel.z
}

geometry_msgs::Twist arc_follower(PathSegment arc2follow) {

    geometry_msgs::Twist output;

    double r_error = commons::distance_between_points(arc2follow.center, vehicle_pose.pose.position) -
                     commons::distance_between_points(arc2follow.point_begin, arc2follow.center);

    output.linear.x = arc2follow.direction(vehicle_pose.pose.position).x * cruise_velocity;
    output.linear.y = arc2follow.direction(vehicle_pose.pose.position).y * cruise_velocity;

    double angle_to_center = atan2(arc2follow.center.y - vehicle_pose.pose.position.y,
                                   arc2follow.center.x - vehicle_pose.pose.position.x);
    double angle_vel = atan2(vehicle_velocity.twist.linear.y, vehicle_velocity.twist.linear.x);
    double mag_vel = sqrt(pow(vehicle_velocity.twist.linear.y, 2) + pow(vehicle_velocity.twist.linear.x, 2));
    mag_vel = mag_vel * cos(angle_vel - angle_to_center);
    output.linear.x = output.linear.x + (Pa_gain * r_error + Da_gain * mag_vel) * cos(angle_to_center);
    output.linear.y = output.linear.y + (Pa_gain * r_error + Da_gain * mag_vel) * sin(angle_to_center);
    output.linear.z = altitude_control(cruise_height);

    return output;

}

geometry_msgs::Twist line_follower(PathSegment line2follow) {

    double theta_command;

    geometry_msgs::Point projected_point;
    projected_point.x = commons::projection_on_line(line2follow, vehicle_pose.pose.position).x;
    projected_point.y = commons::projection_on_line(line2follow, vehicle_pose.pose.position).y;

    geometry_msgs::Point ahead_point;
    ahead_point.x = projected_point.x + commons::line_direction(line2follow).x * look_ahead_distance;
    ahead_point.y = projected_point.y + commons::line_direction(line2follow).y * look_ahead_distance;

    theta_command = atan2(ahead_point.y - vehicle_pose.pose.position.y, ahead_point.x - vehicle_pose.pose.position.x);

    geometry_msgs::Twist output;

    output.linear.x = cruise_velocity * cos(theta_command);
    output.linear.y = cruise_velocity * sin(theta_command);
    output.linear.z = altitude_control(cruise_height);

    return output;
}

geometry_msgs::Twist path_follower(PathSegment following) {
    if (following.type == 1) {  //current segment is a line
        if (commons::distance_between_points(commons::projection_on_line(following, vehicle_pose.pose.position),
                                             following.point_end) < 0.4 &&
            commons::distance_between_points(vehicle_pose.pose.position, following.point_end) < 2) {
            current_segment++;
        }
        return line_follower(following);

    } else {   //current segment is an arc
        double ang_current = atan2(vehicle_pose.pose.position.y - following.center.y,
                                   vehicle_pose.pose.position.x - following.center.x);
        double ang_final = atan2(following.point_end.y - following.center.y,
                                 following.point_end.x - following.center.x);
        if (abs(ang_final - ang_current) < pi / 30) {
            current_segment++;
        }
        return arc_follower(following);

    }
}

geometry_msgs::Twist land_position_controller(geometry_msgs::Point target) {
    double x_err, y_err;

    prev_x_err = x_err;
    prev_y_err = y_err;

    double x_err_derivative = (x_err - prev_x_err) * frequency;
    double y_err_derivative = (y_err - prev_y_err) * frequency;

    x_err = target.x - vehicle_pose.pose.position.x;
    y_err = target.y - vehicle_pose.pose.position.y;

    x_err_integrated = x_err_integrated + x_err / frequency;
    y_err_integrated = y_err_integrated + y_err / frequency;

    if (x_err_integrated > integration_limit) { x_err_integrated = integration_limit; }
    else if (x_err_integrated < -integration_limit) { x_err_integrated = -integration_limit; }

    if (y_err_integrated > integration_limit) { y_err_integrated = integration_limit; }
    else if (y_err_integrated < -integration_limit) { y_err_integrated = -integration_limit; }


    geometry_msgs::Twist output;

    output.linear.x = Pp_gain * x_err + Dp_gain * x_err_derivative + Ip_gain * x_err_integrated;
    output.linear.y = Pp_gain * y_err + Dp_gain * y_err_derivative + Ip_gain * y_err_integrated;
    output.linear.z = -descent_rate;

    if (pow(output.linear.x, 2) + pow(output.linear.y, 2) > pow(max_position_control_velocity, 2)) {
        output.linear.x = output.linear.x * sqrt(pow(output.linear.x, 2) + pow(output.linear.y, 2)) / max_position_control_velocity;
        output.linear.y = output.linear.y * sqrt(pow(output.linear.x, 2) + pow(output.linear.y, 2)) / max_position_control_velocity;
    }

    return output;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "fly_path_follower");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber body_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_body", 10, body_velocity_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/global", 10, global_position_cb);
    ros::Subscriber aruco_transform_sub = nh.subscribe<fiducial_msgs::FiducialTransformArray>
            ("/fiducial_transforms", 10, arucoCallback);

    ros::Publisher set_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher segment_pub = nh.advertise<fly_msgs::Int>
            ("current_segment_index", 10);
    ros::Publisher raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10); //body frame pub
    ros::Publisher final_point_pub = nh.advertise<geometry_msgs::Point>
            ("/final_point", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(frequency);

    ros::param::get("simulation", simulation);
    ros::param::get("drone/look_ahead_distance", look_ahead_distance);
    ros::param::get("drone/cruise_velocity", cruise_velocity);
    ros::param::get("drone/cruise_height", cruise_height);
    ros::param::get("drone/Pz_gain", Pz_gain);
    ros::param::get("drone/Dz_gain", Dz_gain);
    ros::param::get("drone/Pa_gain", Pa_gain);
    ros::param::get("drone/Da_gain", Da_gain);
    ros::param::get("drone/Pp_gain", Pp_gain);
    ros::param::get("drone/Dp_gain", Dp_gain);
    ros::param::get("drone/Ip_gain", Ip_gain);
    ros::param::get("drone/integration_limit", integration_limit);
    ros::param::get("drone/descent_rate", descent_rate);
    ros::param::get("drone/max_position_control_velocity", max_position_control_velocity);
    ros::param::get("drone/max_distance_to_path", max_distance_to_path);
    ros::param::get("camera_parameters/camera_vertical_fov", camera_vertical_fov);
    ros::param::get("camera_parameters/camera_horizontal_fov", camera_horizontal_fov);
    ros::param::get("camera_parameters/overlap", overlap);


    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to Px4 sitl");

    rover_pose.x = 0;
    rover_pose.y = 0;

    geometry_msgs::PoseStamped take_off_p;
    take_off_p.pose.position.x = 0;
    take_off_p.pose.position.y = 0;
    take_off_p.pose.position.z = cruise_height;


    if (simulation == 1) {

        for (int i = 0; i < 100; i++) {
            local_pos_pub.publish(take_off_p);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        arm_cmd.request.value = true;

        if (current_state.mode != "OFFBOARD") {
            set_mode_client.call(offb_set_mode);
            if (offb_set_mode.response.mode_sent)
                ROS_INFO("Offboard enabled");
            else
                ROS_WARN("Switching to OFFBOARD failed!");
        }
        if (!current_state.armed) {
            arming_client.call(arm_cmd);
            if (arm_cmd.response.success)
                ROS_INFO("Vehicle armed");
            else
                ROS_WARN("Arming failed!");
        }
    }
    //wait gps position to get accurate before generating the path.
    while (ros::ok() && !current_state.armed) {

        pub_segment.index = -4;  //if waiting for arming, publish segment as -4
        segment_pub.publish(pub_segment); //publish current segment of the path to inform other nodes/vehicle

        local_pos_pub.publish(take_off_p);
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("Waiting for arming to generate path");

    }

    ROS_INFO("Generating Survey Path");

    survey_path_generator survey;
    survey.d = (cruise_height * sin(camera_horizontal_fov / 2.0)) * (1.0 - overlap);
    //survey.d=5;

    survey.read_from_file(vehicle_pose.pose.position, global_pose);

    ROS_INFO("%d", survey.Polygon.size());

    survey.standardize();

    for (int i = 0; i < survey.Polygon.size(); i++) {
        ROS_INFO("%f  %f", survey.Polygon[i].x, survey.Polygon[i].y);

    }
    survey.generate_path();
    ROS_INFO("paht generated, containing %d lines", survey.survey_points.size());

    survey.add_arcs(survey.choose_side(vehicle_pose.pose.position));

    ROS_INFO("arcs added, total path contains %d segments", survey.survey_points.size());
    ROS_INFO("Beginning point:   %f   %f",  survey.survey_points[0].point_begin.x,
             survey.survey_points[0].point_begin.y);


    if (commons::distance_between_points(vehicle_pose.pose.position, survey.survey_points[0].point_begin) >
        max_distance_to_path) {
        ROS_INFO("Survey path is further than allowed distance , aborting mission %f",
                 commons::distance_between_points(vehicle_pose.pose.position, survey.survey_points[0].point_begin));
        return 0;

    }
    ROS_INFO("Survey path generated succesfuly, starting mission");


    //wait for offboard mode to take off
    while (ros::ok() && current_state.mode != "OFFBOARD") {

        pub_segment.index = -3;  //if waiting for offboard,, publish segment as -3
        segment_pub.publish(pub_segment); //publish current segment of the path to inform other nodes/vehicle

        local_pos_pub.publish(take_off_p);
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("Waiting for offboard to take off");

    }


    ROS_INFO("Take off");

    take_off_p = vehicle_pose;
    take_off_p.pose.position.z=cruise_height;


    geometry_msgs::Point final_point;
    final_point = survey.survey_points[survey.survey_points.size() - 1].point_end;

    //reach take off height
    while (ros::ok() && vehicle_pose.pose.position.z < cruise_height - 0.5) {

        pub_segment.index = -2;  //if the take off has not finished yet, publish segment as -2
        segment_pub.publish(pub_segment); //publish current segment of the path to inform other nodes/vehicle

        local_pos_pub.publish(take_off_p);
        final_point_pub.publish(final_point);
        ros::spinOnce();
        rate.sleep();

    }

    ROS_INFO("Going to the beginning survey path");

    //go to the beginning of the survey path

    PathSegment line_to_path;
    line_to_path.type = 1;
    line_to_path.point_begin = vehicle_pose.pose.position;
    line_to_path.point_end = survey.survey_points[0].point_begin;

    while (ros::ok() &&
           commons::distance_between_points(vehicle_pose.pose.position, survey.survey_points[0].point_begin) > 1) {

        set_vel_pub.publish(path_follower(line_to_path));  //publish velocity for mavros

        pub_segment.index = -1;  //if survey path is not started yet, publish segment as -1
        segment_pub.publish(pub_segment); //publish current segment of the path to inform other nodes/vehicle
        final_point_pub.publish(final_point);
        ros::spinOnce();
        rate.sleep();

    }


    ROS_INFO("Starting surveying the path");

    //start following the path
    while (ros::ok() && !(current_segment == survey.survey_points.size() - 1 &&
                          commons::distance_between_points(vehicle_pose.pose.position,
                                                           survey.survey_points[current_segment].point_begin) < 2)) {
        set_vel_pub.publish(path_follower(survey.survey_points[current_segment]));  //publish velocity for mavros

        pub_segment.index = current_segment;
        segment_pub.publish(pub_segment); //publish current segment of the path to inform other nodes/vehicles
        final_point_pub.publish(final_point);  //publish end of the path to inform rover.
        ros::spinOnce();
        rate.sleep();

    }

    geometry_msgs::PoseStamped wait_p;
    wait_p = vehicle_pose;

    while(ros::ok() && !rover_seen){
        ROS_INFO("Waiting to detect rover");

        pub_segment.index = -7;  //wainting for rover
        segment_pub.publish(pub_segment); //publish current segment of the path to inform other nodes/vehicle

        local_pos_pub.publish(wait_p);
        final_point_pub.publish(final_point);

        ros::spinOnce;
        rate.sleep();

    }

    ROS_INFO("Starting landing");
    while (ros::ok() && current_state.mode == "OFFBOARD") {

        set_vel_pub.publish(land_position_controller(rover_pose));

        pub_segment.index = -6;
        segment_pub.publish(pub_segment);
        final_point_pub.publish(final_point);  //publish end of the path to inform rover.

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Ended");

    return 0;
}
