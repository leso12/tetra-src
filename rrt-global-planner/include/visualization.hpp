#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

/** LINE_LIST 마커 초기화 */
inline void init_line(visualization_msgs::Marker* line_msg)
{
  line_msg->header.frame_id = "map";
  line_msg->id = 0;
  line_msg->ns = "tree";
  line_msg->type = visualization_msgs::Marker::LINE_LIST;
  line_msg->action = visualization_msgs::Marker::ADD;
  line_msg->pose.orientation.w = 1.0;
  line_msg->scale.x = 0.05;  // 선 두께(m)
}

/** 두 점을 잇는 선분을 마커로 퍼블리시 */
inline void pub_line(visualization_msgs::Marker* line_msg, ros::Publisher* line_pub,
                     double x1, double y1, double x2, double y2)
{
  line_msg->header.stamp = ros::Time::now();

  geometry_msgs::Point p1, p2;
  std_msgs::ColorRGBA c1, c2;

  p1.x = x1; p1.y = y1; p1.z = 1.0;
  p2.x = x2; p2.y = y2; p2.z = 1.0;

  c1.r = 0.0; c1.g = 1.0; c1.b = 0.0; c1.a = 0.5;
  c2 = c1;

  line_msg->points.push_back(p1);
  line_msg->points.push_back(p2);
  line_msg->colors.push_back(c1);
  line_msg->colors.push_back(c2);

  line_pub->publish(*line_msg);
}
