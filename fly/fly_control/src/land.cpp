
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

fiducial_msgs::FiducialTransformArray aruco_transform;

void arucoCallback(fiducial_msgs::FiducialTransformArray_<std::allocator<void>> msg) {
    aruco_transform = msg;
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

    rover_pose.x = 5;
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
        ROS_INFO("Waiting for arming ");

    }



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

    //reach take off height
    while (ros::ok() && vehicle_pose.pose.position.z < cruise_height - 0.5) {

        pub_segment.index = -2;  //if the take off has not finished yet, publish segment as -2
        segment_pub.publish(pub_segment); //publish current segment of the path to inform other nodes/vehicle

        local_pos_pub.publish(take_off_p);
        ros::spinOnce();
        rate.sleep();

    }



    ROS_INFO("Starting landing");
    while (ros::ok() && current_state.mode == "OFFBOARD") {

        set_vel_pub.publish(land_position_controller(rover_pose));

        pub_segment.index = -6;
        segment_pub.publish(pub_segment);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Ended");

    return 0;
}
