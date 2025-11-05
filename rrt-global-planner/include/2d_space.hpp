#pragma once

#include <cmath>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include "math_funcs.hpp"

// 괄호 보강
#define TWO_M_PI (2.0 * M_PI)
#define M_PI_10  (M_PI / 10.0)

/** L2 거리 */
inline double getDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
  const double dx = p2.x - p1.x;
  const double dy = p2.y - p1.y;
  return std::sqrt(dx * dx + dy * dy);
}

/** Free-space 체크 (Costmap2D* 버전: 핵심 구현) */
inline bool inFreeSpace(const geometry_msgs::Point point,
                        costmap_2d::Costmap2D* costmap,
                        const double robot_radius_max)
{
  if (!costmap) return false;

  bool result = true;
  double theta = 0.0;
  std::vector<costmap_2d::MapLocation> map_polygon, polygon_cells;

  // 외접 원형 footprint를 다각형 근사
  while (theta <= TWO_M_PI)
  {
    costmap_2d::MapLocation map_loc;
    if (!costmap->worldToMap(point.x + robot_radius_max * std::cos(theta),
                             point.y + robot_radius_max * std::sin(theta),
                             map_loc.x, map_loc.y))
    {
      return false; // 맵 밖
    }
    map_polygon.push_back(map_loc);
    theta += M_PI_10;
  }

  costmap->convexFillCells(map_polygon, polygon_cells);

  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    const unsigned char c = costmap->getCost(polygon_cells[i].x, polygon_cells[i].y);
    if (c >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      result = false;
      break;
    }
  }
  return result;
}

/** const Costmap2DROS* 오버로드 (내부 const_cast 제한 사용) */
inline bool inFreeSpace(const geometry_msgs::Point point,
                        const costmap_2d::Costmap2DROS* costmap_ros,
                        const double robot_radius_max)
{
  auto* cm_ros_nc = const_cast<costmap_2d::Costmap2DROS*>(costmap_ros);
  return inFreeSpace(point, cm_ros_nc->getCostmap(), robot_radius_max);
}

/** Edge free-space (Costmap2D* 버전) */
inline bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge,
                            costmap_2d::Costmap2D* costmap,
                            const double robot_radius)
{
  if (!costmap || edge.size() < 2) return false;

  const double dist = getDistance(edge[0], edge[1]);
  const double num_points = (robot_radius > 0.0) ? (dist / robot_radius) : 1.0;

  geometry_msgs::Point pt{};
  for (double ii = 0.0; ii <= num_points; ii += 1.0)
  {
    pt.x = edge[0].x + ii * (edge[1].x - edge[0].x) / num_points;
    pt.y = edge[0].y + ii * (edge[1].y - edge[0].y) / num_points;

    if (!inFreeSpace(pt, costmap, robot_radius))
      return false;
  }
  return true;
}

/** const Costmap2DROS* 오버로드 */
inline bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge,
                            const costmap_2d::Costmap2DROS* costmap_ros,
                            const double robot_radius)
{
  auto* cm_ros_nc = const_cast<costmap_2d::Costmap2DROS*>(costmap_ros);
  return edgeInFreeSpace(edge, cm_ros_nc->getCostmap(), robot_radius);
}

/** 랜덤 상태 샘플링 (Costmap2D* 버전) */
inline geometry_msgs::Point getRandomState(costmap_2d::Costmap2D* costmap,
                                           const double robot_radius)
{
  geometry_msgs::Point randomState{};
  randomState.z = 0.0;
  if (!costmap) return randomState;

  const double ox = costmap->getOriginX();
  const double oy = costmap->getOriginY();
  const double sx = costmap->getSizeInMetersX();
  const double sy = costmap->getSizeInMetersY();

  bool ok = false;
  int guard = 0;
  while (!ok && ++guard < 10000)
  {
    randomState.x = randomDouble(ox, ox + sx);
    randomState.y = randomDouble(oy, oy + sy);
    ok = inFreeSpace(randomState, costmap, robot_radius);
  }
  return randomState;
}

/** Costmap2DROS* 오버로드 */
inline geometry_msgs::Point getRandomState(costmap_2d::Costmap2DROS* costmap_ros,
                                           const double robot_radius)
{
  return getRandomState(costmap_ros->getCostmap(), robot_radius);
}
