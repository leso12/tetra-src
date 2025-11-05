#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

#include <angles/angles.h>
#include <ros/ros.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "2d_space.hpp"
#include "math_funcs.hpp"
#include "visualization.hpp"

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner
{
class RRTGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  RRTGlobalPlanner();
  RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  ~RRTGlobalPlanner() {}

private:
  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  costmap_2d::Costmap2D* costmap_{nullptr};
  std::vector<geometry_msgs::Point> footprint_;
  double goal_tol_{0.05}, d_{0.2}, robot_radius_{0.3};
  int K_in_{4000};
  bool viz_tree_{false}, initialized_{false};
  ros::Publisher plan_pub_, tree_pub_;

  // 후처리 파라미터 (언더바 표기!)
  double resample_step_{0.15};
  bool do_shortcut_{true};
  int  shortcut_iters_{200};
};
};  // namespace global_planner

#endif  // GLOBAL_PLANNER_CPP


// -------------------- RRT 자료구조 & 유틸 --------------------

struct tree_node
{
  int parent_id{};
  geometry_msgs::Point vertex{};
};

class rrt
{
public:
  geometry_msgs::Point x_initial{};
  std::vector<tree_node> tree_nodes{};
  std::vector<std::vector<geometry_msgs::Point>> edges{};
  costmap_2d::Costmap2D* X_space{nullptr};   // 스냅샷 Costmap2D*
  bool success{false};

  rrt(geometry_msgs::Point x_init, costmap_2d::Costmap2D* costmap)
  {
    x_initial = x_init;
    X_space = costmap;
    tree_node initial_node;
    initial_node.parent_id = 0;
    initial_node.vertex = x_init;
    add_vertex(initial_node);
  }

  void add_vertex(const tree_node new_node) { this->tree_nodes.push_back(new_node); }

  void add_edge(geometry_msgs::Point p1, geometry_msgs::Point p2)
  {
    this->edges.push_back({p1, p2});
  }

  ~rrt() = default;
};

// 최근접 노드
inline tree_node getNearestNeighbor(const geometry_msgs::Point point1, const rrt* T)
{
  tree_node nearest{};
  nearest.parent_id = 0;
  double nearest_distance = HUGE_VAL;

  for (int ii = 0; ii < static_cast<int>(T->tree_nodes.size()); ++ii)
  {
    const auto& v = T->tree_nodes.at(ii).vertex;
    if (point1.x == v.x && point1.y == v.y) continue;

    const double d = getDistance(point1, v);
    if (d < nearest_distance)
    {
      nearest_distance = d;
      nearest.vertex = v;
      nearest.parent_id = ii;
    }
  }
  nearest.vertex.z = 0.0;
  return nearest;
}

// 트리 확장
inline tree_node extendTree(const tree_node near, const geometry_msgs::Point rand, const double d)
{
  tree_node out{};
  out.vertex.z = 0.0;
  double th = std::atan2(rand.y - near.vertex.y, rand.x - near.vertex.x);
  out.vertex.x = near.vertex.x + d * std::cos(th);
  out.vertex.y = near.vertex.y + d * std::sin(th);
  out.parent_id = near.parent_id;
  return out;
}

// RRT 생성
inline rrt generateRRT(geometry_msgs::PoseStamped x_init,
                       geometry_msgs::PoseStamped x_final,
                       costmap_2d::Costmap2D* costmap,  // 스냅샷
                       double robot_radius, double goal_tol, int K, double d)
{
  rrt T(x_init.pose.position, costmap);

  for (int k = 1; k <= K; ++k)
  {
    const auto x_rand = getRandomState(T.X_space, robot_radius);
    const auto x_near = getNearestNeighbor(x_rand, &T);
    const auto x_new  = extendTree(x_near, x_rand, d);

    std::vector<geometry_msgs::Point> edge = { x_new.vertex, x_near.vertex };
    if (!edgeInFreeSpace(edge, T.X_space, robot_radius))
      continue;

    T.add_vertex(x_new);
    T.add_edge(x_near.vertex, x_new.vertex);

    if (getDistance(x_new.vertex, x_final.pose.position) <= goal_tol)
    {
      ROS_INFO("Found solution with %i/%i RRT vertices.", k, K);
      T.success = true;
      break;
    }
  }
  return T;
}

// 로봇 반경(footprint 외접)
inline double getRobotRadius(const std::vector<geometry_msgs::Point>& footprint)
{
  double maxd = 0.0;
  geometry_msgs::Point origin{};
  for (const auto& pt : footprint)
  {
    const double dx = pt.x - origin.x;
    const double dy = pt.y - origin.y;
    maxd = std::max(maxd, std::sqrt(dx*dx + dy*dy));
  }
  return maxd;
}

// 트리 → 전역경로
inline bool getGlobalPath(const rrt* tree, std::vector<geometry_msgs::PoseStamped>* plan,
                          const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
  plan->clear();
  if (tree->tree_nodes.empty()) return false;

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = start.header.frame_id;

  int current_id = static_cast<int>(tree->tree_nodes.size()) - 1;
  ps.pose.orientation = goal.pose.orientation;

  while (current_id != 0)
  {
    ps.pose.position = tree->tree_nodes.at(current_id).vertex;
    plan->push_back(ps);

    const int prev_id = current_id;
    current_id = tree->tree_nodes.at(current_id).parent_id;

    const double dy = tree->tree_nodes.at(prev_id).vertex.y - tree->tree_nodes.at(current_id).vertex.y;
    const double dx = tree->tree_nodes.at(prev_id).vertex.x - tree->tree_nodes.at(current_id).vertex.x;
    tf2::Quaternion q; q.setRPY(0,0,std::atan2(dy,dx));
    ps.pose.orientation = tf2::toMsg(q);
  }

  ps.pose.position  = tree->tree_nodes.at(0).vertex;
  ps.pose.orientation = start.pose.orientation;
  plan->push_back(ps);

  std::reverse(plan->begin(), plan->end());
  return true;
}
