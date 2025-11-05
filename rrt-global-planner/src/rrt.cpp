// rrt.cpp — rrt-global-planner/src/rrt.cpp
// RRT 전역 플래너: 코스트맵 스냅샷 기반 플래닝 + RPP 친화 후처리(리샘플/스무딩)

#include <pluginlib/class_list_macros.h>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// Melodic: Costmap2D::mutex_t는 boost 계열
#include <boost/thread/locks.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "rrt.hpp"  // 클래스 선언 및 인라인 유틸

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::RRTGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace {  // ---- 보조 유틸 ------------------------------------------------------------------

// 선분 a->b 의 yaw
inline double segYaw(const geometry_msgs::PoseStamped& a,
                     const geometry_msgs::PoseStamped& b) {
  const double dx = b.pose.position.x - a.pose.position.x;
  const double dy = b.pose.position.y - a.pose.position.y;
  return std::atan2(dy, dx);
}

// 등간격 리샘플 + 진행방향 yaw 채우기
static void resamplePlan(std::vector<geometry_msgs::PoseStamped>& plan, double step /*m*/ = 0.15)
{
  if (plan.size() < 2) return;

  std::vector<geometry_msgs::PoseStamped> out;
  out.reserve(plan.size() * 2);
  out.push_back(plan.front());

  for (size_t i = 1; i < plan.size(); ++i)
  {
    const auto& a = out.back();
    const auto& b = plan[i];
    const double dx = b.pose.position.x - a.pose.position.x;
    const double dy = b.pose.position.y - a.pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) continue;

    const int n = std::max(1, static_cast<int>(std::floor(dist / step)));
    for (int k = 1; k <= n; ++k)
    {
      geometry_msgs::PoseStamped p = a;
      const double t = std::min(1.0, (k * step) / dist);
      p.pose.position.x = a.pose.position.x + dx * t;
      p.pose.position.y = a.pose.position.y + dy * t;

      const double yaw = std::atan2(dy, dx);
      tf2::Quaternion q; q.setRPY(0, 0, yaw);
      p.pose.orientation = tf2::toMsg(q);

      out.push_back(p);
    }
  }

  // 마지막 포즈의 orientation은 goal 값 유지
  out.back().pose.orientation = plan.back().pose.orientation;
  plan.swap(out);
}

// 간단 쇼트카팅 스무딩: 두 점 i,j 사이 직선이 free면 중간 제거
static void shortcutSmoothing(std::vector<geometry_msgs::PoseStamped>& plan,
                              costmap_2d::Costmap2D* cm,
                              double robot_radius,
                              int max_iterations = 200)
{
  if (!cm || plan.size() < 3) return;

  for (int iter = 0; iter < max_iterations; ++iter)
  {
    if (plan.size() < 3) break;
    const int N = static_cast<int>(plan.size());
    int i = 1 + std::rand() % (N - 2);
    int j = i + 1 + std::rand() % (N - i - 1);
    if (j >= N) continue;

    geometry_msgs::Point p1, p2;
    p1.x = plan[i].pose.position.x; p1.y = plan[i].pose.position.y;
    p2.x = plan[j].pose.position.x; p2.y = plan[j].pose.position.y;

    std::vector<geometry_msgs::Point> edge{p1, p2};
    if (edgeInFreeSpace(edge, cm, robot_radius))
    {
      plan.erase(plan.begin() + i + 1, plan.begin() + j);
    }
  }

  // 스무딩 후 yaw 재채움
  for (size_t k = 1; k < plan.size(); ++k)
  {
    const double yaw = segYaw(plan[k-1], plan[k]);
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    plan[k-1].pose.orientation = tf2::toMsg(q);
  }
  // 마지막은 goal yaw 유지
}

} // anonymous namespace

namespace global_planner
{

RRTGlobalPlanner::RRTGlobalPlanner() {}

RRTGlobalPlanner::RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void RRTGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) return;

  ROS_INFO("Initializing RRTGlobalPlanner.");
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  footprint_ = costmap_ros_->getRobotFootprint();  // padding 반영 footprint
  robot_radius_ = getRobotRadius(footprint_);

  ros::NodeHandle nh("~/" + name);

  // 기본 파라미터
  nh.param("goal_tol", goal_tol_, 0.05);     // goal 위치 허용 오차(m)
  nh.param("K_in",     K_in_,     4000);     // RRT 최대 반복
  nh.param("d",        d_,        0.2);      // 한 스텝 확장 길이(m)
  nh.param("viz_tree", viz_tree_, false);    // 트리 시각화 on/off

  // 후처리 파라미터 (언더바 표기! . 사용하면 크래시)
  nh.param("postproc_resample_step",  resample_step_,  0.15); // 리샘플 간격
  nh.param("postproc_shortcut",       do_shortcut_,    true); // 쇼트카팅 스무딩 on/off
  nh.param("postproc_shortcut_iters", shortcut_iters_, 200);

  plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
  if (viz_tree_) {
    tree_pub_ = nh.advertise<visualization_msgs::Marker>("tree", 1);
  }

  initialized_ = true;
}

bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) {
    ROS_ERROR("Global planner was not initialized, performing initialization.");
    initialize(name_, costmap_ros_);
    if (!initialized_) return false;
  }

  // 프레임 체크 (간단 보호)
  if (start.header.frame_id != goal.header.frame_id) {
    ROS_ERROR("RRTGlobalPlanner: start frame (%s) != goal frame (%s)",
              start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  ROS_INFO("Generating Global Plan with RRT.");

  // ★★★ 코스트맵 스냅샷(깊은 복사)으로 계획: 업데이트 쓰레드(blocking) 방지
  std::unique_ptr<costmap_2d::Costmap2D> snapshot;
  {
    costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
    auto& mtx = *(cm->getMutex());
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lk(mtx);  // boost 락
    snapshot.reset(new costmap_2d::Costmap2D(*cm));              // 깊은 복사
  } // 락 즉시 해제

  // 스냅샷으로 RRT 생성 (헤더의 generateRRT는 Costmap2D* 기반)
  rrt T_out = generateRRT(start, goal, snapshot.get(), robot_radius_, goal_tol_, K_in_, d_);

  // 시각화(옵션)
  if (viz_tree_) {
    visualization_msgs::Marker tree_msg;
    init_line(&tree_msg);
    for (const auto& edge : T_out.edges) {
      pub_line(&tree_msg, &tree_pub_, edge.at(0).x, edge.at(0).y, edge.at(1).x, edge.at(1).y);
    }
  }

  if (!T_out.success) {
    ROS_INFO("Failed to find Global Plan with RRT.");
    return false;
  }

  // 1) 기본 경로 생성
  getGlobalPath(&T_out, &plan, start, goal);

  // 2) (옵션) 쇼트카팅 스무딩
  if (do_shortcut_) {
    shortcutSmoothing(plan, snapshot.get(), robot_radius_, shortcut_iters_);
  }

  // 3) 등간격 리샘플 + yaw 채움 (RPP 친화)
  resamplePlan(plan, resample_step_);

  // 4) 퍼블리시
  publishPlan(plan);
  return true;
}

void RRTGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  nav_msgs::Path rviz_path;
  rviz_path.poses.resize(plan.size());

  if (plan.empty()) {
    rviz_path.header.frame_id = "map";
    rviz_path.header.stamp = ros::Time::now();
  } else {
    rviz_path.header.frame_id = plan[0].header.frame_id;
    rviz_path.header.stamp = ros::Time::now();
  }

  for (unsigned int i = 0; i < plan.size(); ++i) {
    rviz_path.poses[i] = plan[i];
  }

  plan_pub_.publish(rviz_path);
}

} // namespace global_planner
