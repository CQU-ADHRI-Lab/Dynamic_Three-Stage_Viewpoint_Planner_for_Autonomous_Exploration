/**************************************************************************
drrt.h
Header of the Drrt(Dynamic rrt) class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/

#ifndef DRRT_H_
#define DRRT_H_

#include "dsvplanner/drrt_base.h"
#include "dsvplanner/dual_state_frontier.h"
#include "dsvplanner/dual_state_graph.h"
#include "dsvplanner/dra.h"
#include "dsvplanner/grid.h"
#include "kdtree/kdtree.h"
#include "octomap_world/octomap_manager.h"

using namespace Eigen;

/**
 * @namespace dsvplanner_ns
 * @brief 包含动态RRT（Drrt）规划器相关的类和函数
 *
 * 该命名空间包含了实现动态RRT规划器所需的各种类和函数。
 * 主要组件包括Drrt类，它是动态RRT规划器的核心实现，以及其他相关的辅助类和数据结构。
 *
 * @section classes 类
 * - @ref Drrt "Drrt"：动态RRT规划器的主要实现类，负责路径规划和节点管理。
 *
 * @section functions 函数
 * - @ref init "init()"：初始化Drrt规划器的成员变量和相关数据结构。
 * - @ref clear "clear()"：清除Drrt规划器的内部状态和数据。
 * - @ref setParams "setParams(Params params)"：设置规划器的参数。
 * - @ref setRootWithOdom "setRootWithOdom(const nav_msgs::Odometry& pose)"：根据里程计信息设置规划器的根节点。
 * - @ref setBoundary "setBoundary(const geometry_msgs::PolygonStamped& boundary)"：设置规划器的边界。
 * - @ref setTerrainVoxelElev "setTerrainVoxelElev()"：设置地形体素高程。
 * - @ref getThreeLocalFrontierPoint "getThreeLocalFrontierPoint()"：获取三个局部边界点。
 * - @ref getNextNodeToClosestGlobalFrontier "getNextNodeToClosestGlobalFrontier()"：获取到最近全局边界的下一个节点。
 * - @ref plannerInit "plannerInit()"：初始化规划器。
 * - @ref plannerIterate "plannerIterate()"：执行规划器的迭代。
 * - @ref pruneTree "pruneTree(StateVec root)"：修剪树结构。
 * - @ref publishNode "publishNode()"：发布节点信息。
 * - @ref publishPlanningHorizon "publishPlanningHorizon()"：发布规划视野信息。
 * - @ref gotoxy "gotoxy(int x, int y)"：将光标移动到指定位置。
 * - @ref gainFound "gainFound()"：检查是否找到增益。
 * - @ref remainingLocalFrontier "remainingLocalFrontier()"：检查是否存在剩余的局部边界。
 * - @ref generateRrtNodeToLocalFrontier "generateRrtNodeToLocalFrontier(StateVec& newNode)"：生成到局部边界的RRT节点。
 * - @ref inSensorRange "inSensorRange(StateVec& node)"：检查节点是否在传感器范围内。
 * - @ref inPlanningBoundary "inPlanningBoundary(StateVec node)"：检查节点是否在规划边界内。
 * - @ref inGlobalBoundary "inGlobalBoundary(StateVec node)"：检查节点是否在全局边界内。
 * - @ref getNodeCounter "getNodeCounter()"：获取节点计数器的值。
 * - @ref getRemainingNodeCounter "getRemainingNodeCounter()"：获取剩余节点计数器的值。
 * - @ref angleDiff "angleDiff(StateVec direction1, StateVec direction2)"：计算两个方向向量的角度差。
 * - @ref getZvalue "getZvalue(double x_position, double z_position)"：获取指定位置的Z值。
 * - @ref gain "gain(StateVec state)"：计算给定状态的增益。
 *
 * @section data_structures 数据结构
 * - @ref Node "Node"：表示RRT树中的节点。
 * - @ref Params "Params"：包含规划器的参数。
 * - @ref StateVec "StateVec"：表示状态向量，通常是三维空间中的位置和方向。
 *
 * @section dependencies 依赖
 * - @ref volumetric_mapping::OctomapManager "volumetric_mapping::OctomapManager"：用于管理八叉树地图。
 * - @ref DualStateGraph "DualStateGraph"：用于表示双状态图。
 * - @ref DualStateFrontier "DualStateFrontier"：用于表示双状态边界。
 * - @ref OccupancyGrid "OccupancyGrid"：用于表示占用网格。
 * - @ref kdtree::kdtree "kdtree::kdtree"：用于实现KD树数据结构。
 * - @ref Eigen::Vector3d "Eigen::Vector3d"：用于表示三维向量。
 * - @ref pcl::PointXYZ "pcl::PointXYZ"：用于表示三维点。
 * - @ref pcl::PointCloud<pcl::PointXYZ> "pcl::PointCloud<pcl::PointXYZ>"：用于表示点云。
 * - @ref nav_msgs::Odometry "nav_msgs::Odometry"：用于表示里程计信息。
 * - @ref geometry_msgs::PolygonStamped "geometry_msgs::PolygonStamped"：用于表示边界多边形。
 *
 * @note 该命名空间中的所有类和函数都与动态RRT规划器的实现相关。
 */

namespace dsvplanner_ns
{
class Drrt
{
public:
  Drrt(volumetric_mapping::OctomapManager* manager, DualStateGraph* graph,
                          DualStateFrontier* frontier, OccupancyGrid* grid, Dra* dra);
  ~Drrt();

  typedef Vector3d StateVec;
  bool plannerReady_;
  bool boundaryLoaded_;
  bool global_plan_;
  bool global_plan_pre_;
  bool local_plan_;
  bool nextNodeFound_;
  bool remainingFrontier_;
  bool return_home_;
  bool normal_local_iteration_;
  int global_vertex_size_;
  int NextBestNodeIdx_;  // this is for global planner that still can find global
                         // frontier
  int bestNodeId_;       // this is for global plan that cannot find global frontier
  int loopCount_;        // this value is the same with params_.loopCount_ =
                         // params_.kLoopCountThres when there is still local frontier
                         // while 2
                         // or 3 times when there is no local frontier
  pcl::PointXYZ selectedGlobalFrontier_;
  geometry_msgs::Polygon boundary_polygon_;
  StateVec root_;

  void init();
  void clear();
  void setParams(Params params);
  void setRootWithOdom(const nav_msgs::Odometry& pose);
  void setBoundary(const geometry_msgs::PolygonStamped& boundary);
  void setTerrainVoxelElev();
  void getThreeLocalFrontierPoint();
  void getNextNodeToClosestGlobalFrontier();
  void plannerInit();
  void plannerIterate();
  void pruneTree(StateVec root);
  void publishNode();
  void publishPlanningHorizon();
  void gotoxy(int x, int y);
  bool gainFound();
  bool remainingLocalFrontier();
  bool generateRrtNodeToLocalFrontier(StateVec& newNode);
  bool inSensorRange(StateVec& node);
  bool inPlanningBoundary(StateVec node);
  bool inGlobalBoundary(StateVec node);
  int getNodeCounter();
  int getRemainingNodeCounter();
  int getCovarianceIndex(double max_time_step, double time_step, double t);
  double angleDiff(StateVec direction1, StateVec direction2);
  double getZvalue(double x_position, double z_position);
  double gain(StateVec state, double time_of_arrival);

protected:
  kdtree* kdTree_;
  Params params_;
  bool localPlanOnceMore_;
  bool visualize_static_and_dynamic_rays = true;
  int keepTryingNum_;
  int iterationCount_;
  int nodeCounter_;
  int remainingNodeCount_;


  std::vector<int> executedBestNodeList_;
  double bestGain_;
  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  double KFiterations = 20;
  double time_step = 0.5;
  double dynamic_width = 0.4;
  double dynamic_height= 1.8;
  Eigen::Vector3d frontier1_direction_, frontier2_direction_, frontier3_direction_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr localThreeFrontier_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr globalThreeFrontier_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr sampledPoint_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  std::mutex vecMutex;

  Node* bestNode_;
  Node* rootNode_;
  std::vector<Node*> node_array;
  std::vector<double> terrain_voxle_elev_;
  volumetric_mapping::OctomapManager* manager_;
  DualStateGraph* dual_state_graph_;
  DualStateFrontier* dual_state_frontier_;
  OccupancyGrid* grid_;
  Dra* dra_;
};
}

#endif  // DRRT_H
