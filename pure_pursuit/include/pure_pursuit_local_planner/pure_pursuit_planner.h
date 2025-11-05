#ifndef PURE_PURSUIT_LOCAL_PLANNER_PURE_PURSUIT_PLANNER_H_
#define PURE_PURSUIT_LOCAL_PLANNER_PURE_PURSUIT_PLANNER_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

// nav_core plugin base
#include <nav_core/base_local_planner.h>

namespace pure_pursuit_local_planner
{

class PurePursuitPlanner : public nav_core::BaseLocalPlanner
{
public:
  PurePursuitPlanner();
  ~PurePursuitPlanner() override;

  // nav_core API (Melodic)
  void initialize(std::string name,
                  tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  bool isGoalReached() override;

private:
  // helpers
  void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& path) const;

  // 유지: 인터페이스는 있지만 내부에서 사용 최소화
  bool getGoalLocalCoordinates(std::vector<double>& local_xy,
                               const geometry_msgs::PoseStamped& robot_pose,
                               double lookahead) const;

  // heading-error P control
  void setControls(const std::vector<double>& lookahead_xy,
                   geometry_msgs::Twist& cmd_vel,
                   double max_lin, double max_ang) const;

  double getEuclideanDistance(double x1, double y1, double x2, double y2) const;

private:
  // state
  bool initialized_;
  bool goal_reached_;

  costmap_2d::Costmap2DROS*  costmap_ros_;
  tf2_ros::Buffer*           tf2_buffer_;

  // params
  int    lookahead_index_;
  double max_linear_x_;
  double max_angular_z_;
  double goal_tolerance_xy_;

  // TIP(turn-in-place) 관련 파라미터
  double tip_angle_rad_;
  double tip_release_angle_;
  double tip_near_dist_;
  double tip_ang_kp_;
  double tip_min_angular_;
  bool   tip_mode_;

  // watchdog
  int         no_motion_ticks_;
  ros::Time   last_motion_stamp_;
  geometry_msgs::Pose last_pose_;

  // plan & pub
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  ros::Publisher local_plan_pub_;
};

} // namespace pure_pursuit_local_planner

#endif // PURE_PURSUIT_LOCAL_PLANNER_PURE_PURSUIT_PLANNER_H_
