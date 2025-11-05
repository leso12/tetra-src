/* #ifndef PURE_PURSUIT_LOCAL_PLANNER_JOIN_COSTMAP_H_
#define PURE_PURSUIT_LOCAL_PLANNER_JOIN_COSTMAP_H_

#include <vector>
#include <costmap_2d/costmap_2d_ros.h>
 #include <tf2_ros/buffer.h>

namespace pure_pursuit_local_planner
{
class JoinCostmap
{
public:
  JoinCostmap();

  // tf2로 통일
  void initialize(costmap_2d::Costmap2DROS* local_costmap_ros,
                  costmap_2d::Costmap2DROS* global_costmap_ros,
                  tf2_ros::Buffer* tf2_buffer);

  // global costmap에 local costmap을 max로 병합
  void joinMaps();

private:
  costmap_2d::Costmap2DROS* global_costmap_ros_;
  costmap_2d::Costmap2DROS* local_costmap_ros_;
  tf2_ros::Buffer* tf2_buffer_;

  std::vector<std::vector<unsigned char> > global_cache_;
  bool initialized_;
};
} // namespace pure_pursuit_local_planner */

// #endif
