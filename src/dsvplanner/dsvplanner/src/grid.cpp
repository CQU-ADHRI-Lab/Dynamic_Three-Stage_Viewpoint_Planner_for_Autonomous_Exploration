/*
grid.cpp
Implementation of OccupancyGrid class. Occupancy grid is used to get do the
collision check
based on terrain points in
local area.

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
pub_grid_points_topic_  occpancy_grid_map
*/

#ifndef GRID_HPP_
#define GRID_HPP_

#include "dsvplanner/grid.h"

namespace dsvplanner_ns
{
OccupancyGrid::OccupancyGrid(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  initialize();
}

OccupancyGrid::~OccupancyGrid()
{
}

bool OccupancyGrid::readParameters()
{
  nh_private_.getParam("/grid/world_frame_id", world_frame_id_);
  nh_private_.getParam("/grid/odomSubTopic", sub_odom_topic_);
  nh_private_.getParam("/grid/terrainCloudSubTopic", sub_terrain_point_cloud_topic_);
  nh_private_.getParam("/grid/pubGridPointsTopic", pub_grid_points_topic_);
  nh_private_.getParam("/grid/kMapWidth", kMapWidth);
  nh_private_.getParam("/grid/kGridSize", kGridSize);
  nh_private_.getParam("/grid/kDownsampleSize", kDownsampleSize);
  nh_private_.getParam("/grid/kObstacleHeightThre", kObstacleHeightThre);
  nh_private_.getParam("/grid/kFlyingObstacleHeightThre", kFlyingObstacleHeightThre);
  nh_private_.getParam("/rm/kBoundX", kCollisionCheckX);
  nh_private_.getParam("/rm/kBoundY", kCollisionCheckY);

  return true;
}

bool OccupancyGrid::initialize()
{
  // Read in parameters
  if (!readParameters())
    return false;
  // Initialize subscriber
  odom_sub_.subscribe(nh_, sub_odom_topic_, 1);
  terrain_point_cloud_sub_.subscribe(nh_, sub_terrain_point_cloud_topic_, 1);
  // 创建一个新的 Sync 对象，用于同步 odom_sub_ 和 terrain_point_cloud_sub_ 订阅器
  sync_.reset(new Sync(syncPolicy(100), odom_sub_, terrain_point_cloud_sub_));
  // 注册一个回调函数，当 Sync 对象接收到同步的数据时，会调用这个回调函数
  sync_->registerCallback(boost::bind(&OccupancyGrid::terrainCloudAndOdomCallback, this, _1, _2));
  //通过这种方式，OccupancyGrid 类可以确保在处理数据时，里程计和地形点云数据是同步的，从而提高数据处理的准确性和可靠性。


  grid_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_grid_points_topic_, 1);

  map_half_width_grid_num_ = int(kMapWidth / 2 / kGridSize);
  map_width_grid_num_ = map_half_width_grid_num_ * 2 + 1;

  clearGrid();

  ROS_INFO("Successfully launched OccupancyGrid node");

  return true;
}
/*回调函数的参数包括两个常量指针：odom_msg 和 terrain_msg。odom_msg 指向一个 nav_msgs::Odometry 类型的消息，
包含了机器人的里程计信息，如位置、速度和姿态等。terrain_msg 指向一个 sensor_msgs::PointCloud2 类型的消息，包含了地形的点云数据。
在 ROS 中，回调函数通常被设计为接收常量指针作为参数，这是一种约定俗成的做法，有助于提高代码的可读性和可维护性。这意味着它们指向的内容是不可修改的*/
void OccupancyGrid::terrainCloudAndOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                                const sensor_msgs::PointCloud2::ConstPtr& terrain_msg)
{
  terrain_time_ = terrain_msg->header.stamp;
  robot_position_[0] = odom_msg->pose.pose.position.x;
  robot_position_[1] = odom_msg->pose.pose.position.y;
  robot_position_[2] = odom_msg->pose.pose.position.z;
  terrain_cloud_->clear();
  terrain_cloud_ds->clear();
  terrain_cloud_traversable_->clear();
  terrain_cloud_obstacle_->clear();
  // 使用 PCL 库的 fromROSMsg 函数将 ROS 格式的点云消息 terrain_msg 转换为 PCL 格式的点云对象 terrain_cloud_
  pcl::fromROSMsg(*terrain_msg, *terrain_cloud_);
  // 对点云进行下采样处理，将点云的分辨率降低到 kDownsampleSize
  pcl::VoxelGrid<pcl::PointXYZI> point_ds;
  point_ds.setLeafSize(kDownsampleSize, kDownsampleSize, kDownsampleSize);
  point_ds.setInputCloud(terrain_cloud_);
  point_ds.filter(*terrain_cloud_ds);

  pcl::PointXYZI point;
  int terrainCloudSize = terrain_cloud_ds->points.size();
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point.x = terrain_cloud_ds->points[i].x;
    point.y = terrain_cloud_ds->points[i].y;
    point.z = terrain_cloud_ds->points[i].z;
    point.intensity = terrain_cloud_ds->points[i].intensity;
    // crop all ground points
    // 如果点的强度大于障碍物高度阈值且小于飞行障碍物高度阈值，则将该点添加到障碍物点云集合中
    if (point.intensity > kObstacleHeightThre && point.intensity < kFlyingObstacleHeightThre)
    {
      terrain_cloud_obstacle_->push_back(point);
    }
    // 如果点的强度小于等于障碍物高度阈值，则将该点添加到可通行地面点云集合中
    else if (point.intensity <= kObstacleHeightThre)
    {
      terrain_cloud_traversable_->push_back(point);
    }
  }

  clearGrid();
  updateGrid();
  publishGridMap();
}

/**
 * \brief 将网格索引转换为对应的几何点
 * \param p 网格索引，是一个包含两个整数的数组，分别表示 x 和 y 方向的索引
 * \return 对应的几何点，包含 x、y 和 z 坐标
 */
geometry_msgs::Point OccupancyGrid::getPoint(GridIndex p)
{
  int indX = p[0];
  int indY = p[1];
  // 根据网格索引和机器人位置计算点的 x 坐标
  double x = kGridSize * (indX - map_half_width_grid_num_) + robot_position_[0];
  // 根据网格索引和机器人位置计算点的 y 坐标
  double y = kGridSize * (indY - map_half_width_grid_num_) + robot_position_[1];
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  // 点的 z 坐标设置为机器人的 z 坐标
  point.z = robot_position_[2];
  return point;
}

/**
 * \brief 将几何点转换为对应的网格索引
 * \param point 几何点，是一个包含 x、y 和 z 坐标的向量
 * \return 对应的网格索引，是一个包含两个整数的数组，分别表示 x 和 y 方向的索引
 */
GridIndex OccupancyGrid::getIndex(StateVec point)
{
  int indX = int((point.x() - robot_position_[0] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
  int indY = int((point.y() - robot_position_[1] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
  if (point.x() - robot_position_[0] + kGridSize / 2 < 0)
    indX--;
  if (point.y() - robot_position_[1] + kGridSize / 2 < 0)
    indY--;
  if (indX < 0)
    indX = 0;
  if (indY < 0)
    indY = 0;
  if (indX > map_width_grid_num_ - 1)
    indX = map_width_grid_num_ - 1;
  if (indY > map_width_grid_num_ - 1)
    indY = map_width_grid_num_ - 1;
  GridIndex grid_index(indX, indY);
  return grid_index;
}

void OccupancyGrid::clearGrid()
{
  gridState_.clear();
  std::vector<int> y_vector;
  for (int i = 0; i < map_width_grid_num_; i++)
  {
    y_vector.clear();
    for (int j = 0; j < map_width_grid_num_; j++)
    {
      gridStatus grid_state = unknown;
      y_vector.push_back(grid_state);
    }
    gridState_.push_back(y_vector);
  }
}
// update网格地图

void OccupancyGrid::updateGrid()
{
  pcl::PointXYZI point;
  for (int i = 0; i < terrain_cloud_obstacle_->points.size(); i++)
  {
    point = terrain_cloud_obstacle_->points[i];
    int indX = int((point.x - robot_position_[0] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    int indY = int((point.y - robot_position_[1] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    if (point.x - robot_position_[0] + kGridSize / 2 < 0)
      indX--;
    if (point.y - robot_position_[1] + kGridSize / 2 < 0)
      indY--;
    if (indX < 0)
      indX = 0;
    if (indY < 0)
      indY = 0;
    if (indX > map_width_grid_num_ - 1)
      indX = map_width_grid_num_ - 1;
    if (indY > map_width_grid_num_ - 1)
      indY = map_width_grid_num_ - 1;

    if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 && indY < map_width_grid_num_)
    {
      gridStatus grid_state = occupied;
      gridState_[indX][indY] = grid_state;
    }
  }
  for (int i = 0; i < terrain_cloud_traversable_->points.size(); i++)
  {
    point = terrain_cloud_traversable_->points[i];
    int indX = int((point.x - robot_position_[0] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    int indY = int((point.y - robot_position_[1] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    if (point.x - robot_position_[0] + kGridSize / 2 < 0)
      indX--;
    if (point.y - robot_position_[1] + kGridSize / 2 < 0)
      indY--;
    if (indX < 0)
      indX = 0;
    if (indY < 0)
      indY = 0;
    if (indX > map_width_grid_num_ - 1)
      indX = map_width_grid_num_ - 1;
    if (indY > map_width_grid_num_ - 1)
      indY = map_width_grid_num_ - 1;
    if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 && indY < map_width_grid_num_)
    {
      if (gridState_[indX][indY] == 2)
      {
        continue;
      }
      if (updateFreeGridWithSurroundingGrids(indX, indY) == false)
      {
        gridStatus grid_state = free;
        gridState_[indX][indY] = grid_state;
      }
      else
      {
        gridStatus grid_state = near_occupied;
        gridState_[indX][indY] = grid_state;
      }
    }
  }
}
// 发布网格地图

void OccupancyGrid::publishGridMap()
{
  grid_cloud_->clear();
  pcl::PointXYZI p1;
  geometry_msgs::Point p2;
  GridIndex p3;
  for (int i = 0; i < map_width_grid_num_; i++)
  {
    for (int j = 0; j < map_width_grid_num_; j++)
    {
      p3[0] = i;
      p3[1] = j;
      p2 = getPoint(p3);
      p1.x = p2.x;
      p1.y = p2.y;
      p1.z = p2.z;
      p1.intensity = gridState_[i][j];
      grid_cloud_->points.push_back(p1);
    }
  }
  sensor_msgs::PointCloud2 gridCloud2;
  pcl::toROSMsg(*grid_cloud_, gridCloud2);
  gridCloud2.header.stamp = terrain_time_;
  gridCloud2.header.frame_id = world_frame_id_;
  grid_cloud_pub_.publish(gridCloud2);
}
// 检查网格地图中是否存在障碍物

bool OccupancyGrid::updateFreeGridWithSurroundingGrids(int indx, int indy)
{
  int count_x = ceil(0.5 * kCollisionCheckX / kGridSize);
  int count_y = ceil(0.5 * kCollisionCheckY / kGridSize);
  int indX;
  int indY;
  for (int i = -count_x; i <= count_x; i++)
  {
    for (int j = -count_y; j <= count_y; j++)
    {
      indX = indx + i;
      indY = indy + j;
      if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 && indY < map_width_grid_num_)
      {
        if (gridState_[indX][indY] == 2)
        {
          return true;
        }
      }
    }
  }
  return false;
}

// 检查机器人是否与障碍物发生碰撞

bool OccupancyGrid::collisionCheckByTerrainWithVector(StateVec origin_point, StateVec goal_point)
{
  //  ROS_INFO("Start Check Collision");
  GridIndex origin_grid_index = getIndex(origin_point);
  GridIndex goal_grid_index = getIndex(goal_point);
  GridIndex max_grid_index(map_width_grid_num_ - 1, map_width_grid_num_ - 1);
  GridIndex min_grid_index(0, 0);
  GridIndex grid_index;
  std::vector<GridIndex> ray_tracing_grids =
      rayCast(origin_grid_index, goal_grid_index, max_grid_index, min_grid_index);
      
  int length = ray_tracing_grids.size();
  for (int i = 0; i < length; i++)
  {
    grid_index = ray_tracing_grids[i];
    if (gridState_[grid_index[0]][grid_index[1]] == 2 || gridState_[grid_index[0]][grid_index[1]] == 3)
    {
      return true;
    }
  }
  return false;
}
/**
 * \brief 检查从原点到目标点的路径上是否存在障碍物
 * \param origin 原点的几何点，包含 x、y 和 z 坐标
 * \param goal 目标点的几何点，包含 x、y 和 z 坐标
 * \return 如果路径上存在障碍物，返回 true；否则返回 false
 */

bool OccupancyGrid::collisionCheckByTerrain(geometry_msgs::Point origin, geometry_msgs::Point goal)
{
  StateVec origin_point(origin.x, origin.y, origin.z);
  StateVec goal_point(goal.x, goal.y, goal.z);

  return collisionCheckByTerrainWithVector(origin_point, goal_point);
}

bool OccupancyGrid::InRange(const GridIndex sub, const GridIndex max_sub, const GridIndex min_sub)
{
  return sub.x() >= min_sub.x() && sub.x() <= max_sub.x() && sub.y() >= min_sub.y() && sub.y() <= max_sub.y();
}
/*If x is 0, the function returns 0.
If x is less than 0, the function returns -1.
If x is greater than 0, the function returns 1.
*/
int OccupancyGrid::signum(int x)
{
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double OccupancyGrid::intbound(double s, double ds)
{
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0)
  {
    return intbound(-s, -ds);
  }
  else
  {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}
// 计算两个数的模
double OccupancyGrid::mod(double value, double modulus)
{
  return fmod(fmod(value, modulus) + modulus, modulus);
}
// 射线追踪算法 这个函数的作用是从一个网格的起点（origin）向一个目标点（goal）进行射线追踪，并返回追踪过程中经过的网格索引列表。
/*origin：起点的网格索引。
goal：目标点的网格索引。
max_grid：网格的最大索引。
min_grid：网格的最小索引。
这个函数的作用是通过射线追踪算法来找到从起点到目标点的最短路径，并返回路径上经过的网格索引。*/
std::vector<GridIndex> OccupancyGrid::rayCast(GridIndex origin, GridIndex goal, GridIndex max_grid, GridIndex min_grid)
{
  std::vector<GridIndex> grid_pairs;
  if (origin == goal)
  {
    grid_pairs.push_back(origin);
    return grid_pairs;
  }
  //函数计算起点和目标点之间的差异（diff），并计算出最大距离（max_dist）。
  GridIndex diff = goal - origin;
  double max_dist = diff.squaredNorm();
  //函数计算出在 x 和 y 方向上的步长（step_x 和 step_y），以及在 x 和 y 方向上的最大 t 值（t_max_x 和 t_max_y）和 t 值的增量（t_delta_x 和 t_delta_y）。
  int step_x = signum(diff.x());
  int step_y = signum(diff.y());
  double t_max_x = step_x == 0 ? DBL_MAX : intbound(origin.x(), diff.x());
  double t_max_y = step_y == 0 ? DBL_MAX : intbound(origin.y(), diff.y());
  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff.x();
  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff.y();
  double dist = 0;
  GridIndex cur_sub = origin;
//函数进入一个无限循环，在循环中不断更新当前网格索引，并检查是否已经到达目标点或者超出最大距离。如果已经到达目标点或者超出最大距离，则返回结果列表。
  while (true)
  {
    if (InRange(cur_sub, max_grid, min_grid))
    {
      grid_pairs.push_back(cur_sub);
      dist = (cur_sub - origin).squaredNorm();
      if (cur_sub == goal || dist > max_dist)
      {
        return grid_pairs;
      }
      if (t_max_x < t_max_y)
      {
        cur_sub.x() += step_x;
        t_max_x += t_delta_x;
      }
      else
      {
        cur_sub.y() += step_y;
        t_max_y += t_delta_y;
      }
    }
    else
    {
      return grid_pairs;
    }
  }
}
}
#endif
