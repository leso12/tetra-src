#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h> 
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
//#include "robot_pose_saver/SavePose.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_pose_saver");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0); 

    std::ofstream output_file("/home/ani/tetra/catkin_ws/src/robot_pose_saver/robot_positions.txt");

    if (!output_file.is_open()) {
        ROS_ERROR("Could not open output file at specified path!");
        return -1;
    }

    while (ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("my_robot/odom", "my_robot/base_link", ros::Time(0));

            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;
            double z = transformStamped.transform.translation.z;

            double yaw_rad = tf2::getYaw(transformStamped.transform.rotation); 
            double yaw_deg = yaw_rad * (180.0 / M_PI);

            ROS_INFO("Current Robot Position (x, y, z, yaw_deg): (%.2f, %.2f, %.2f, %.2f)", x, y, z, yaw_deg);

            output_file << x << "," << y << "," << z << "," << yaw_deg << std::endl;

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ros::spinOnce();
        rate.sleep();
    }

    output_file.close();
    return 0;
}