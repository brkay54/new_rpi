#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <fly_msgs/Int.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/commons.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"


double camera_vertical_fov;
double camera_horizontal_fov;
double overlap;
double cruise_height;
std::string output_folder_name;

int prev_segment;
int current_segment = -1;

void current_segment_cb(const fly_msgs::Int::ConstPtr &msg) {
    prev_segment = current_segment;
    current_segment = msg->index;

}

geometry_msgs::PoseStamped vehicle_pose;

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    vehicle_pose = *msg;
}

sensor_msgs::NavSatFix global_pose;

void vehicle_global_cb(const sensor_msgs::NavSatFix::ConstPtr &global) {
    global_pose = *global;
}


cv::Mat imgCallback;

static void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imgCallback = cv_ptr_compressed->image;

    }
    catch (cv_bridge::Exception &e) {
        //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void trig_cam(int frame_nm, sensor_msgs::NavSatFix global_pos) {

    std::string name = output_folder_name + "/" + std::to_string(frame_nm) + "_" + std::to_string(global_pos.latitude)
                       + "_" + std::to_string(global_pos.longitude) + ".jpg";
    cv::imwrite(name, imgCallback);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_trigger");
    ros::NodeHandle nh;
    ros::Subscriber vehicle_global_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, vehicle_global_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber global_position_sub = nh.subscribe<fly_msgs::Int>
            ("current_segment_index", 10, current_segment_cb);

    ros::Subscriber image_sub;
    std::string image_topic = "image/compressed";
    image_sub = nh.subscribe(image_topic, 1, ImageCallback);
    ros::Rate rate(20.0);

    ros::param::get("drone/cruise_height", cruise_height);
    ros::param::get("camera_parameters/camera_vertical_fov", camera_vertical_fov);
    ros::param::get("camera_parameters/camera_horizontal_fov", camera_horizontal_fov);
    ros::param::get("camera_parameters/overlap", overlap);
    ros::param::get("output_folder_name", output_folder_name);


    while (ros::ok() && current_segment < 0) {

        ros::spinOnce();
        rate.sleep();
    }


    double image_distance;
    //image_distance = 5;
    image_distance = (cruise_height * sin(camera_vertical_fov / 2.0)) * (1.0 - overlap);

    int frame_num = 0;
    geometry_msgs::Point last_image_pose;

    while (ros::ok() && current_segment > 0) {

        if (current_segment != prev_segment) { //line segment has started or overed
            frame_num++;
            last_image_pose = vehicle_pose.pose.position;
            trig_cam(frame_num, global_pose);
            ROS_INFO("Image %d captured", frame_num);

        }
        if (current_segment % 2 == 0) { //current segment is a line, where to take images.

            if (commons::distance_between_points(last_image_pose, vehicle_pose.pose.position) > image_distance) {
                frame_num++;
                last_image_pose = vehicle_pose.pose.position;
                trig_cam(frame_num, global_pose);
                ROS_INFO("Image %d captured", frame_num);

            }
        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
