/*
drrt.cpp
Implementation of Drrt class. Dynamic tree is used to get random viewpoints in
local area. New rrt
is generated based on the pruned tree of last time.

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <cstdlib>
#include <dsvplanner/drrt.h>
#include <dsvplanner/drrtp.h>
#include <misc_utils/misc_utils.h>

/**
 * 构造函数
 * #@param manager 八叉树管理器指针
 * #@param graph 双状态图指针
 * #@param frontier 双状态边界指针
 * #@param grid 占用网格指针
 */


dsvplanner_ns::Drrt::Drrt(volumetric_mapping::OctomapManager* manager, DualStateGraph* graph,
                          DualStateFrontier* frontier, OccupancyGrid* grid, Dra* dra)
{
  // 初始化成员变量
  manager_ = manager;
  grid_ = grid;
  dual_state_graph_ = graph;
  dual_state_frontier_ = frontier;
  dra_ = dra;

  // 打印日志信息
  ROS_INFO("Successfully launched Drrt node");
}


/**
 * 析构函数
 * 释放 Drrt 类实例所占用的资源，包括根节点和 kd 树
 */
dsvplanner_ns::Drrt::~Drrt()
{
  // 释放动态分配的内存
  delete rootNode_;
  // 释放 kd 树
  kd_free(kdTree_);
}


/**
 * 初始化函数
 * 初始化 Drrt 类的成员变量和一些全局变量
 */
void dsvplanner_ns::Drrt::init()
{
  // 创建一个 3 维的 kd 树
  kdTree_ = kd_create(3);
  // 初始化迭代次数为 0
  iterationCount_ = 0;
  // 初始化最佳增益为 0
  bestGain_ = params_.kZeroGain;
  // 初始化最佳节点为空
  bestNode_ = NULL;
  // 初始化根节点为空
  rootNode_ = NULL;
  // 初始化节点计数器为 0
  nodeCounter_ = 0;
  // 初始化规划器未准备好
  plannerReady_ = false;
  // 初始化边界未加载
  boundaryLoaded_ = false;

  // 初始化全局规划标志为 false
  global_plan_ = false;
  // 初始化全局规划预标志为 true
  global_plan_pre_ = true;
  // 初始化局部规划标志为 true
  local_plan_ = true;
  // 初始化下一个节点未找到标志为 false
  nextNodeFound_ = false;
  // 初始化剩余边界标志为 true
  remainingFrontier_ = true;
  // 初始化返回 home 标志为 false
  return_home_ = false;
  // 初始化全局顶点大小为 0
  global_vertex_size_ = 0;
  // 初始化下一个最佳节点索引为 0
  NextBestNodeIdx_ = 0;

  // 初始化地形体素高程数组，初始值为车辆高度
  for (int i = 0; i < params_.kTerrainVoxelWidth * params_.kTerrainVoxelWidth; i++)
  {
    terrain_voxle_elev_.push_back(params_.kVehicleHeight);
  }

  // 设置随机数种子
  srand((unsigned)time(NULL));
}


void dsvplanner_ns::Drrt::setParams(Params params)
{
  params_ = params;
}

/**
 * @brief 根据里程计信息设置根节点位置
 * @param pose 里程计消息，包含机器人的位姿信息
 */
void dsvplanner_ns::Drrt::setRootWithOdom(const nav_msgs::Odometry& pose)
{
  // 设置根节点的 x 坐标为里程计消息中的 x 坐标
  root_[0] = pose.pose.pose.position.x;
  // 设置根节点的 y 坐标为里程计消息中的 y 坐标
  root_[1] = pose.pose.pose.position.y;
  // 设置根节点的 z 坐标为里程计消息中的 z 坐标
  root_[2] = pose.pose.pose.position.z;
}

/**
 * @brief 设置规划器的边界
 * @param boundary 包含边界信息的 PolygonStamped 消息
 */
void dsvplanner_ns::Drrt::setBoundary(const geometry_msgs::PolygonStamped& boundary)
{
  // 将传入的边界消息中的多边形赋值给类成员 boundary_polygon_
  boundary_polygon_ = boundary.polygon;
  // 设置边界已加载标志为 true
  boundaryLoaded_ = true;
}


/**
 * @brief 设置地形体素高程数组
 *
 * 这个函数会检查 `dual_state_frontier_` 对象的 `getTerrainVoxelElev` 方法返回的数组是否为空。
 * 如果不为空，则清除当前的 `terrain_voxle_elev_` 数组，并将 `dual_state_frontier_` 返回的数组赋值给它。
 *
 * @note 这个函数通常在接收到新的地形体素高程数据时被调用,update in realtime。
 */
void dsvplanner_ns::Drrt::setTerrainVoxelElev()
{
  if (dual_state_frontier_->getTerrainVoxelElev().size() > 0)
  {
    terrain_voxle_elev_.clear();
    terrain_voxle_elev_ = dual_state_frontier_->getTerrainVoxelElev();
  }
}

int dsvplanner_ns::Drrt::getNodeCounter()
{
  return nodeCounter_;
}

int dsvplanner_ns::Drrt::getRemainingNodeCounter()
{
  return remainingNodeCount_;
}

bool dsvplanner_ns::Drrt::gainFound()
{// 检查最佳增益是否大于零
  return bestGain_ > params_.kZeroGain;
}
//计算两个二维向量之间的角度差，以度为单位。

double dsvplanner_ns::Drrt::angleDiff(StateVec direction1, StateVec direction2)
{
  double degree;
  degree = acos((direction1[0] * direction2[0] + direction1[1] * direction2[1]) /
                (sqrt(direction1[0] * direction1[0] + direction1[1] * direction1[1]) *
                 sqrt(direction2[0] * direction2[0] + direction2[1] * direction2[1]))) *
           180 / M_PI;
  return degree;
}
//用于根据给定的 x 和 y 位置计算对应的 z 坐标值。z 坐标值是通过查询地形体素高程数组 terrain_voxle_elev_ 得到的，并考虑了车辆的高度。

double dsvplanner_ns::Drrt::getZvalue(double x_position, double y_position)
{
  int indX =
      int((x_position + params_.kTerrainVoxelSize / 2) / params_.kTerrainVoxelSize) + params_.kTerrainVoxelHalfWidth;
  int indY =
      int((y_position + params_.kTerrainVoxelSize / 2) / params_.kTerrainVoxelSize) + params_.kTerrainVoxelHalfWidth;
  if (x_position + params_.kTerrainVoxelSize / 2 < 0)
    indX--;
  if (y_position + params_.kTerrainVoxelSize / 2 < 0)
    indY--;
  if (indX > params_.kTerrainVoxelWidth - 1)
    indX = params_.kTerrainVoxelWidth - 1;
  if (indX < 0)
    indX = 0;
  if (indY > params_.kTerrainVoxelWidth - 1)
    indY = params_.kTerrainVoxelWidth - 1;
  if (indY < 0)
    indY = 0;
  double z_position = terrain_voxle_elev_[params_.kTerrainVoxelWidth * indX + indY] + params_.kVehicleHeight;
  return z_position;
}

bool dsvplanner_ns::Drrt::inSensorRange(StateVec& node)
{
  StateVec root_node(rootNode_->state_[0], rootNode_->state_[1], rootNode_->state_[2]);
  StateVec init_node = node;
  StateVec dir;
  bool insideFieldOfView = false;
  for (int i = 0; i < localThreeFrontier_->points.size(); i++)
  {
    StateVec frontier_point(localThreeFrontier_->points[i].x, localThreeFrontier_->points[i].y,
                            localThreeFrontier_->points[i].z);
    node[0] = init_node[0] + frontier_point[0];
    node[1] = init_node[1] + frontier_point[1];
    double x_position = node[0] - root_node[0];
    double y_position = node[1] - root_node[1];
    node[2] = getZvalue(x_position, y_position);
    if (!inPlanningBoundary(node))
      continue;

    dir = frontier_point - node;
    // Skip if distance to sensor is too large
    double rangeSq = pow(params_.kGainRange, 2.0);
    if (dir.transpose().dot(dir) > rangeSq)
    {
      continue;
    }

    if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * params_.sensorVerticalView / 360)))
    {
      insideFieldOfView = true;
    }
    if (!insideFieldOfView)
    {
      continue;
    }

    if (manager_->getCellStatusPoint(node) == volumetric_mapping::OctomapManager::CellStatus::kFree)
    {
      if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
          this->manager_->getVisibility(node, frontier_point, false))
      {
        return true;
      }
    }
  }
  return false;
}

bool dsvplanner_ns::Drrt::inPlanningBoundary(StateVec node)
{
  if (node.x() < minX_ + 0.5 * params_.boundingBox.x())
  {
    return false;
  }
  else if (node.y() < minY_ + 0.5 * params_.boundingBox.y())
  {
    return false;
  }
  else if (node.z() < minZ_ + 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else if (node.x() > maxX_ - 0.5 * params_.boundingBox.x())
  {
    return false;
  }
  else if (node.y() > maxY_ - 0.5 * params_.boundingBox.y())
  {
    return false;
  }
  else if (node.z() > maxZ_ - 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else
  {
    return true;
  }
}


bool dsvplanner_ns::Drrt::inGlobalBoundary(StateVec node)
{
  if (boundaryLoaded_)
  {
    geometry_msgs::Point node_point;
    node_point.x = node.x();
    node_point.y = node.y();
    node_point.z = node.z();
    if (!misc_utils_ns::PointInPolygon(node_point, boundary_polygon_))
    {
      return false;
    }
  }
  else
  {
    if (node.x() < params_.kMinXGlobalBound + 0.5 * params_.boundingBox.x())
    {
      return false;
    }
    else if (node.y() < params_.kMinYGlobalBound + 0.5 * params_.boundingBox.y())
    {
      return false;
    }
    else if (node.x() > params_.kMaxXGlobalBound - 0.5 * params_.boundingBox.x())
    {
      return false;
    }
    else if (node.y() > params_.kMaxYGlobalBound - 0.5 * params_.boundingBox.y())
    {
      return false;
    }
  }
  if (node.z() > params_.kMaxZGlobalBound - 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else if (node.z() < params_.kMinZGlobalBound + 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else
  {
    return true;
  }
}

/**
 * @brief 生成一个随机的 RRT 节点，使其位于局部边界内，并且在传感器的探测范围内
 * @param newNode 生成的新节点
 * @return 如果成功生成节点，则返回 true，否则返回 false
 * 用于生成一个随机的快速扩展随机树（RRT）节点，使其位于局部边界内，并且在传感器的探测范围内。
 * add time and collosion item
 */
bool dsvplanner_ns::Drrt::generateRrtNodeToLocalFrontier(StateVec& newNode)
{
  //用于存储潜在的节点、节点发现标志、迭代计数器和搜索半径。
  StateVec potentialNode;
  bool nodeFound = false;
  int count = 0;
  double radius = sqrt(SQ(params_.kGainRange) + SQ(params_.kGainRange));
  while (!nodeFound)
  {
    count++;
    if (count >= 300)
    {
      return false;
    }//潜在的节点，并检查它是否在搜索半径内。如果不在，循环将继续。
    potentialNode[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
    potentialNode[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
    potentialNode[2] = 0;
    //计算了点potentialNode到原点的距离的平方（使用了向量的模长公式），并将其与radius的平方进行比较。
    if ((SQ(potentialNode[0]) + SQ(potentialNode[1])) > pow(radius, 2.0))
      continue;
    //检查传感器范围
    if (!inSensorRange(potentialNode))
    {
      continue;
    }
    //检查规划边界和全局边界
    if (!inPlanningBoundary(potentialNode) || !inGlobalBoundary(potentialNode) || dra_->inDynamicRiskArea(potentialNode))
    {
      continue;
    }
    nodeFound = true;
    newNode[0] = potentialNode[0];
    newNode[1] = potentialNode[1];
    newNode[2] = potentialNode[2];
    return true;
  }
  return false;
}

void dsvplanner_ns::Drrt::getNextNodeToClosestGlobalFrontier()
{
  StateVec p1, p2;
  pcl::PointXYZ p3;
  double length1, length2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr globalSelectedFrontier =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  nextNodeFound_ = false;
  for (int i = dual_state_graph_->local_graph_.vertices.size() - 1; i >= 0; i--)  // Search from end to the begining.
                                                                                  // Nodes with large index means they
                                                                                  // are closer to the current position
                                                                                  // of the robot
  {
    p1.x() = dual_state_graph_->local_graph_.vertices[i].location.x;
    p1.y() = dual_state_graph_->local_graph_.vertices[i].location.y;
    p1.z() = dual_state_graph_->local_graph_.vertices[i].location.z;
    for (int j = 0; j < dual_state_frontier_->global_frontier_->size(); j++)
    {
      p3 = dual_state_frontier_->global_frontier_->points[j];
      p2.x() = dual_state_frontier_->global_frontier_->points[j].x;
      p2.y() = dual_state_frontier_->global_frontier_->points[j].y;
      p2.z() = dual_state_frontier_->global_frontier_->points[j].z;
      length1 = sqrt(SQ(p1.x() - p2.x()) + SQ(p1.y() - p2.y()));
      if (length1 > (this->manager_->getSensorMaxRange() + params_.kGainRange) ||
          fabs(p1.z() - p2.z()) > params_.kMaxExtensionAlongZ)  // Not only
                                                                // consider the
      // sensor range,
      // also take the
      // node's view range into consideration
      {
        continue;  // When the node is too far away from the frontier or the
                   // difference between the z
      }            // of this node and frontier is too large, then skip to next frontier.
      // No need to use FOV here.
      if (volumetric_mapping::OctomapManager::CellStatus::kOccupied == manager_->getVisibility(p1, p2, false))
      {
        continue;  // Only when there is no occupied voxels between the node
                   // and
                   // frontier, we consider
      }            // the node can potentially see this frontier
      else
      {
        NextBestNodeIdx_ = i;
        nextNodeFound_ = true;
        globalSelectedFrontier->points.push_back(p3);
        selectedGlobalFrontier_ = p3;
        break;
      }
    }
    if (nextNodeFound_)
      break;
  }
  if (nextNodeFound_)
  {
    for (int i = dual_state_graph_->local_graph_.vertices.size() - 1; i >= 0; i--)
    {
      p1.x() = dual_state_graph_->local_graph_.vertices[i].location.x;
      p1.y() = dual_state_graph_->local_graph_.vertices[i].location.y;
      p1.z() = dual_state_graph_->local_graph_.vertices[i].location.z;
      length2 = sqrt(SQ(p1.x() - p2.x()) + SQ(p1.y() - p2.y()));
      if (length2 > length1 || fabs(p1.z() - p2.z()) > params_.kMaxExtensionAlongZ)
      {
        continue;
      }
      if (volumetric_mapping::OctomapManager::CellStatus::kOccupied ==
          manager_->getLineStatusBoundingBox(p1, p2, params_.boundingBox))
      {
        continue;
      }
      length1 = length2;
      NextBestNodeIdx_ = i;
      nextNodeFound_ = true;
      p3.x = p1.x();
      p3.y = p1.y();
      p3.z = p1.z();
    }
    globalSelectedFrontier->points.push_back(p3);
  }
  sensor_msgs::PointCloud2 globalFrontier;
  pcl::toROSMsg(*globalSelectedFrontier, globalFrontier);
  globalFrontier.header.frame_id = params_.explorationFrame;
  params_.globalSelectedFrontierPub_.publish(globalFrontier);  // publish the next goal node and corresponing frontier
}

void dsvplanner_ns::Drrt::getThreeLocalFrontierPoint()  // Three local frontiers
                                                        // that are most close to
                                                        // the last
                                                        // exploration direciton
{                                                       // will be selected.
  StateVec exploreDirection, frontierDirection;
  double firstDirection = 180, secondDirection = 180, thirdDirection = 180;
  pcl::PointXYZ p1, p2, p3;  // three points to save frontiers.
  exploreDirection = dual_state_graph_->getExploreDirection();
  int localFrontierSize = dual_state_frontier_->local_frontier_->size();
  for (int i = 0; i < localFrontierSize; i++)
  {
    frontierDirection[0] =
        dual_state_frontier_->local_frontier_->points[i].x - root_[0];  // For ground robot, we only consider the
                                                                        // direction along x-y plane
    frontierDirection[1] = dual_state_frontier_->local_frontier_->points[i].y - root_[1];
    double theta = angleDiff(frontierDirection, exploreDirection);
    if (theta < firstDirection)
    {
      thirdDirection = secondDirection;
      secondDirection = firstDirection;
      firstDirection = theta;
      frontier3_direction_ = frontier2_direction_;
      frontier2_direction_ = frontier1_direction_;
      frontier1_direction_ = frontierDirection;
      p3 = p2;
      p2 = p1;
      p1 = dual_state_frontier_->local_frontier_->points[i];
    }
    else if (theta < secondDirection)
    {
      thirdDirection = secondDirection;
      secondDirection = theta;
      frontier3_direction_ = frontier2_direction_;
      frontier2_direction_ = frontierDirection;
      p3 = p2;
      p2 = dual_state_frontier_->local_frontier_->points[i];
    }
    else if (theta < thirdDirection)
    {
      thirdDirection = theta;
      frontier3_direction_ = frontierDirection;
      p3 = dual_state_frontier_->local_frontier_->points[i];
    }
  }

  localThreeFrontier_->clear();
  localThreeFrontier_->points.push_back(p1);
  localThreeFrontier_->points.push_back(p2);
  localThreeFrontier_->points.push_back(p3);
  sensor_msgs::PointCloud2 localThreeFrontier;
  pcl::toROSMsg(*localThreeFrontier_, localThreeFrontier);
  localThreeFrontier.header.frame_id = params_.explorationFrame;
  params_.localSelectedFrontierPub_.publish(localThreeFrontier);
}

bool dsvplanner_ns::Drrt::remainingLocalFrontier()
{
  int localFrontierSize = dual_state_frontier_->local_frontier_->points.size();
  if (localFrontierSize > 0)
    return true;
  return false;
}

/**
 * @brief 执行规划器的迭代操作
 *
 * 这个函数会生成一个新的配置，并将其添加到树中。首先，它会生成一个随机的状态向量 `newState`，
 * 然后检查这个状态是否在规划边界内，并且是否与已存在的节点过于接近。如果满足这些条件，
 * 它会将这个新节点添加到树中，并更新最佳增益和节点计数器。
 *
 * @return 如果成功添加了新节点，则返回 `true`，否则返回 `false`。
 */
void dsvplanner_ns::Drrt::plannerIterate()
{
  // In this function a new configuration is sampled and added to the tree.
  //这些变量用于存储新状态、生成节点的成功标志、搜索半径、候选节点的发现标志和迭代计数器。
  StateVec newState;
  bool generateNodeArroundFrontierSuccess = false;

  double radius = 0.5 * sqrt(SQ(minX_ - maxX_) + SQ(minY_ - maxY_));
  bool candidateFound = false;
  int count = 0;
  //这个循环会一直运行，直到找到一个候选节点。如果在1000次迭代后仍未找到，函数将返回。
  while (!candidateFound)
  {
    count++;
    if (count > 1000)
      return;  // Plan fail if cannot find a required node in 1000 iterations
    //生成节点的条件判断
    if (((double)rand()) / ((double)RAND_MAX) > 0.75 && localThreeFrontier_->size() > 0)
    {
      if (local_plan_ == true)
      {
        generateNodeArroundFrontierSuccess = generateRrtNodeToLocalFrontier(newState);
      }
      if (!generateNodeArroundFrontierSuccess)
      {  // Generate node near local
         // frontier fail
        newState[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
        newState[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
        newState[2] = 0;  // Do not consider z value because ground robot cannot
                          // move along z
        if (SQ(newState[0]) + SQ(newState[1]) > pow(radius, 2.0))
          continue;
        newState[0] += root_[0];
        newState[1] += root_[1];
        newState[2] += root_[2];
        if ((!inPlanningBoundary(newState)) || (!inGlobalBoundary(newState)) ) //(dra_->inDynamicRiskArea(newState)
        {
          continue;
        }
      }
    }
    else
    {
      newState[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
      newState[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
      newState[2] = 0;
      if (SQ(newState[0]) + SQ(newState[1]) > pow(radius, 2.0))
        continue;
      newState[0] += root_[0];
      newState[1] += root_[1];
      newState[2] += root_[2];
      if ((!inPlanningBoundary(newState)) || (!inGlobalBoundary(newState)) ) //|| (dra_->inDynamicRiskArea(newState))
      {
        continue;
      }
    }
    candidateFound = true;
  }

  pcl::PointXYZI sampledPoint;
  sampledPoint.x = newState[0];
  sampledPoint.y = newState[1];
  sampledPoint.z = newState[2];
  sampledPoint_->points.push_back(sampledPoint);
  //使用KD树查找最近邻
  // Find nearest neighbour
  kdres* nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return;
  }
  //KD树（kdTree_）查找距离新状态（newState）最近的节点，并将其作为新节点的父节点（newParent）
  dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
  kd_res_free(nearest);


  //检查新连接是否碰撞：
  // Check for collision of new connection.
  StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  StateVec direction(newState[0] - origin[0], newState[1] - origin[1], 0);
  //如果连接长度小于最小扩展范围（kMinextensionRange），则放弃该连接。如果连接长度大于最大扩展范围（kExtensionRange），则将其缩短至最大扩展范围。
  if (direction.norm() < params_.kMinextensionRange)
  {
    return;
  }
  else if (direction.norm() > params_.kExtensionRange)
  {
    direction = params_.kExtensionRange * direction.normalized();
  }
  StateVec endPoint = origin + direction;
  newState[0] = endPoint[0];
  newState[1] = endPoint[1];
  double x_position = newState[0] - root_[0];
  double y_position = newState[1] - root_[1];
  newState[2] = getZvalue(x_position, y_position);

  if (newState[2] >= 1000)  // the sampled position is above the untraversed area
  {
    return;
  }
  // Check if the new node is too close to any existing nodes after extension
  kdres* nearest_node = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest_node) <= 0)
  {
    kd_res_free(nearest_node);
    return;
  }
  dsvplanner_ns::Node* nearestNode = (dsvplanner_ns::Node*)kd_res_item_data(nearest_node);
  kd_res_free(nearest_node);

  origin[0] = newParent->state_[0];
  origin[1] = newParent->state_[1];
  origin[2] = newParent->state_[2];
  direction[0] = newState[0] - newParent->state_[0];
  direction[1] = newState[1] - newParent->state_[1];
  direction[2] = newState[2] - newParent->state_[2];
  //再次检查新节点的扩展方向和长度，确保它符合最小扩展范围和最大Z方向扩展范围的限制。
  if (direction.norm() < params_.kMinextensionRange || direction[2] > params_.kMaxExtensionAlongZ)
  {
    return;
  }
  // check collision if the new node is in the planning boundary
  //检查新节点是否在规划边界内
  if (!inPlanningBoundary(newState) || (dra_->inDynamicRiskArea(newState)))
  {
    return;
  }
  else
  {//如果连接是自由的（没有碰撞），则创建一个新节点并将其插入到RRT树中。同时，更新最佳增益（bestGain_）和节点计数器（nodeCounter_）。
    if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
            manager_->getLineStatusBoundingBox(origin, newState, params_.boundingBox) &&
        (!grid_->collisionCheckByTerrainWithVector(origin, newState)))
    {  // connection is free
      // Create new node and insert into tree

      dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
     //检查新连接是否存在潜在的动态碰撞：
     // Check for potential dynamic collision of new connection.
     double time_to_reach_node = newParent->time_cost() + newParent->time_to_reach(newNode);
     bool dynamic_collision = dra_->checkCollision(time_to_reach_node,newNode->state_);
    //  if(dynamic_collision){
    //   return;
    //  }
      if(!dynamic_collision){
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ = gain(newNode->state_,newNode->time_cost());
      newNode->collision = false;
      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      geometry_msgs::Pose p1;
      p1.position.x = newState.x();
      p1.position.y = newState.y();
      p1.position.z = newState.z();
      p1.orientation.y = newNode->gain_;
      dual_state_graph_->addNewLocalVertexWithoutDuplicates(p1, dual_state_graph_->local_graph_);
      dual_state_graph_->execute();

      // Display new node
      node_array.push_back(newNode);
      // Update best IG and node if applicable bestGain_
      if (newNode->gain_ > bestGain_)
      {
        bestGain_ = newNode->gain_;
      }
      }
      else{
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ = 0;
      newNode->collision = true;
            // Display new node
      node_array.push_back(newNode);

      }
      nodeCounter_++;
    }
  }
}

// void dsvplanner_ns::Drrt::plannerIterate()
// {
//   // In this function a new configuration is sampled and added to the tree.
//   //这些变量用于存储新状态、生成节点的成功标志、搜索半径、候选节点的发现标志和迭代计数器。
//   StateVec newState;
//   bool generateNodeArroundFrontierSuccess = false;

//   double radius = 0.5 * sqrt(SQ(minX_ - maxX_) + SQ(minY_ - maxY_));
//   bool candidateFound = false;
//   int count = 0;
//   //这个循环会一直运行，直到找到一个候选节点。如果在1000次迭代后仍未找到，函数将返回。
//   while (!candidateFound)
//   {
//     count++;
//     if (count > 1000)
//       return;  // Plan fail if cannot find a required node in 1000 iterations
//     //生成节点的条件判断
//     if (((double)rand()) / ((double)RAND_MAX) > 0.75 && localThreeFrontier_->size() > 0)
//     {
//       if (local_plan_ == true)
//       {
//         generateNodeArroundFrontierSuccess = generateRrtNodeToLocalFrontier(newState);
//       }
//       if (!generateNodeArroundFrontierSuccess)
//       {  // Generate node near local
//          // frontier fail
//         newState[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
//         newState[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
//         newState[2] = 0;  // Do not consider z value because ground robot cannot
//                           // move along z
//         if (SQ(newState[0]) + SQ(newState[1]) > pow(radius, 2.0))
//           continue;
//         newState[0] += root_[0];
//         newState[1] += root_[1];
//         newState[2] += root_[2];
//         if ((!inPlanningBoundary(newState)) || (!inGlobalBoundary(newState)) ) //(dra_->inDynamicRiskArea(newState)
//         {
//           continue;
//         }
//       }
//     }
//     else
//     {
//       newState[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
//       newState[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
//       newState[2] = 0;
//       if (SQ(newState[0]) + SQ(newState[1]) > pow(radius, 2.0))
//         continue;
//       newState[0] += root_[0];
//       newState[1] += root_[1];
//       newState[2] += root_[2];
//       if ((!inPlanningBoundary(newState)) || (!inGlobalBoundary(newState)) ) //|| (dra_->inDynamicRiskArea(newState))
//       {
//         continue;
//       }
//     }
//     candidateFound = true;
//   }

//   pcl::PointXYZI sampledPoint;
//   sampledPoint.x = newState[0];
//   sampledPoint.y = newState[1];
//   sampledPoint.z = newState[2];
//   sampledPoint_->points.push_back(sampledPoint);
//   //使用KD树查找最近邻
//   // Find nearest neighbour
//   kdres* nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
//   if (kd_res_size(nearest) <= 0)
//   {
//     kd_res_free(nearest);
//     return;
//   }
//   //KD树（kdTree_）查找距离新状态（newState）最近的节点，并将其作为新节点的父节点（newParent）
//   dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
//   kd_res_free(nearest);


//   //检查新连接是否碰撞：
//   // Check for collision of new connection.
//   StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
//   StateVec direction(newState[0] - origin[0], newState[1] - origin[1], 0);
//   //如果连接长度小于最小扩展范围（kMinextensionRange），则放弃该连接。如果连接长度大于最大扩展范围（kExtensionRange），则将其缩短至最大扩展范围。
//   if (direction.norm() < params_.kMinextensionRange)
//   {
//     return;
//   }
//   else if (direction.norm() > params_.kExtensionRange)
//   {
//     direction = params_.kExtensionRange * direction.normalized();
//   }
//   StateVec endPoint = origin + direction;
//   newState[0] = endPoint[0];
//   newState[1] = endPoint[1];
//   double x_position = newState[0] - root_[0];
//   double y_position = newState[1] - root_[1];
//   newState[2] = getZvalue(x_position, y_position);

//   if (newState[2] >= 1000)  // the sampled position is above the untraversed area
//   {
//     return;
//   }
//   // Check if the new node is too close to any existing nodes after extension
//   kdres* nearest_node = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
//   if (kd_res_size(nearest_node) <= 0)
//   {
//     kd_res_free(nearest_node);
//     return;
//   }
//   dsvplanner_ns::Node* nearestNode = (dsvplanner_ns::Node*)kd_res_item_data(nearest_node);
//   kd_res_free(nearest_node);

//   origin[0] = newParent->state_[0];
//   origin[1] = newParent->state_[1];
//   origin[2] = newParent->state_[2];
//   direction[0] = newState[0] - newParent->state_[0];
//   direction[1] = newState[1] - newParent->state_[1];
//   direction[2] = newState[2] - newParent->state_[2];
//   //再次检查新节点的扩展方向和长度，确保它符合最小扩展范围和最大Z方向扩展范围的限制。
//   if (direction.norm() < params_.kMinextensionRange || direction[2] > params_.kMaxExtensionAlongZ)
//   {
//     return;
//   }
//   // check collision if the new node is in the planning boundary
//   //检查新节点是否在规划边界内
//   if (!inPlanningBoundary(newState) || (dra_->inDynamicRiskArea(newState)))
//   {
//     return;
//   }
//   else
//   {//如果连接是自由的（没有碰撞），则创建一个新节点并将其插入到RRT树中。同时，更新最佳增益（bestGain_）和节点计数器（nodeCounter_）。
//     if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
//             manager_->getLineStatusBoundingBox(origin, newState, params_.boundingBox) &&
//         (!grid_->collisionCheckByTerrainWithVector(origin, newState)))
//     {  // connection is free
//       // Create new node and insert into tree

//       dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
//       newNode->state_ = newState;
//       newNode->parent_ = newParent;
//       newNode->distance_ = newParent->distance_ + direction.norm();
//       newParent->children_.push_back(newNode);
//       newNode->gain_ = gain(newNode->state_);

//       kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

//       geometry_msgs::Pose p1;
//       p1.position.x = newState.x();
//       p1.position.y = newState.y();
//       p1.position.z = newState.z();
//       p1.orientation.y = newNode->gain_;
//       dual_state_graph_->addNewLocalVertexWithoutDuplicates(p1, dual_state_graph_->local_graph_);
//       dual_state_graph_->execute();

//       // Display new node
//       node_array.push_back(newNode);
//       // Update best IG and node if applicable bestGain_
//       if (newNode->gain_ > bestGain_)
//       {
//         bestGain_ = newNode->gain_;
//       }
//       nodeCounter_++;
//     }
//   }
// }

//create the rrt function
void dsvplanner_ns::Drrt::plannerInit()
{
  // This function is to initialize the tree
  kdTree_ = kd_create(3);

  node_array.clear();
  rootNode_ = new Node;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.kZeroGain;
  rootNode_->parent_ = NULL;

  global_vertex_size_ = 0;
  geometry_msgs::Pose p1;
  if (global_plan_ == false)
  {//在探索阶段，它会初始化根节点的状态，并根据是否存在局部边界来决定是否进行局部规划。
    std::cout << "Exploration Stage & Moving potential field " << std::endl;
    rootNode_->state_ = root_;
    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
    iterationCount_++;

    if (remainingLocalFrontier())
    {
      localPlanOnceMore_ = true;
      loopCount_ = params_.kLoopCountThres;
      normal_local_iteration_ = true;
      keepTryingNum_ = params_.kKeepTryingNum;  // Try 1 or 2 more times even if there
                                                // is no local frontier
      remainingFrontier_ = true;
      getThreeLocalFrontierPoint();
      pruneTree(root_);
      dual_state_graph_->clearLocalGraph();
      dual_state_graph_->local_graph_ = dual_state_graph_->pruned_graph_;
      dual_state_graph_->execute();
    }
    else
    {
      if (!localPlanOnceMore_)
      {
        dual_state_graph_->clearLocalGraph();
        dual_state_graph_->pruned_graph_.vertices.clear();
        remainingFrontier_ = false;
        localPlanOnceMore_ = true;
        normal_local_iteration_ = true;
      }
      else
      {
        remainingFrontier_ = true;
        loopCount_ = params_.kLoopCountThres * 3;
        normal_local_iteration_ = false;
        localThreeFrontier_->clear();
        //        pruneTree(root_);
        dual_state_graph_->clearLocalGraph();
        dual_state_graph_->pruned_graph_.vertices.clear();
        //        dual_state_graph_->local_graph_ =
        //        dual_state_graph_->pruned_graph_;
        dual_state_graph_->execute();
        keepTryingNum_--;
        if (keepTryingNum_ <= 0)
        {
          localPlanOnceMore_ = false;
          keepTryingNum_ = params_.kKeepTryingNum + 1;  // After switching to relocation stage, give
          // another more chance in case that some frontiers
          // are not updated
        }
      }
    }
    //这段代码设置了 RRT 扩展的边界，这些边界是根据根节点的状态和一些参数计算得出的

    maxX_ = rootNode_->state_.x() + params_.kMaxXLocalBound;
    maxY_ = rootNode_->state_.y() + params_.kMaxYLocalBound;
    maxZ_ = rootNode_->state_.z() + params_.kMaxZLocalBound;
    minX_ = rootNode_->state_.x() + params_.kMinXLocalBound;
    minY_ = rootNode_->state_.y() + params_.kMinYLocalBound;
    minZ_ = rootNode_->state_.z() + params_.kMinZLocalBound;


  }
  else
  {
    std::cout << "Relocation Stage & Moving potential field  " << std::endl;
    localPlanOnceMore_ = true;
    StateVec node1;
    double gain1;
    //如果不为空，则将第一个顶点的位置设置为根节点的状态，并将其插入到 k-d 树中。然后，它清除局部图并将其设置为全局图的副本，并设置根节点的增益。
    if (dual_state_graph_->global_graph_.vertices.size() > 0)
    {
      node1[0] = dual_state_graph_->global_graph_.vertices[0].location.x;
      node1[1] = dual_state_graph_->global_graph_.vertices[0].location.y;
      node1[2] = dual_state_graph_->global_graph_.vertices[0].location.z;
      rootNode_->state_ = node1;
      kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);

      dual_state_graph_->clearLocalGraph();
      //change to the global graph
      dual_state_graph_->local_graph_ = dual_state_graph_->global_graph_;
      dual_state_graph_->local_graph_.vertices[0].information_gain = rootNode_->gain_;
      dual_state_graph_->execute();
      //执行 getNextNodeToClosestGlobalFrontier 函数来寻找下一个最接近全局边界的节点。如果找到了这样的节点，它将该节点的增益设置为一个非常大的值，并更新 bestGain_、nodeCounter_ 和 global_vertex_size_，然后发布全局图。
      getNextNodeToClosestGlobalFrontier();
      if (nextNodeFound_)
      {
        dual_state_graph_->local_graph_.vertices[NextBestNodeIdx_].information_gain =
            300000;  // set a large enough value as the best gain
        bestGain_ = 300000;
        nodeCounter_ = dual_state_graph_->global_graph_.vertices.size();
        global_vertex_size_ = nodeCounter_;
        dual_state_graph_->publishGlobalGraph();
      }
      else
      {  // Rebuild the rrt accordingt to current graph and then extend
         // in
         // plannerIterate. This only happens when no
         // global frontiers can be seen. Mostly used at the end of the
         // exploration in case that there are some narrow
         // areas are ignored.
        for (int i = 1; i < dual_state_graph_->global_graph_.vertices.size(); i++)
        {
          p1.position = dual_state_graph_->global_graph_.vertices[i].location;
          node1[0] = p1.position.x;
          node1[1] = p1.position.y;
          node1[2] = p1.position.z;

          kdres* nearest = kd_nearest3(kdTree_, node1.x(), node1.y(), node1.z());
          if (kd_res_size(nearest) <= 0)
          {
            kd_res_free(nearest);
            continue;
          }
          dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
          kd_res_free(nearest);

          StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
          StateVec direction(node1[0] - origin[0], node1[1] - origin[1], node1[2] - origin[2]);
          if (direction.norm() > params_.kExtensionRange)
          {
            direction = params_.kExtensionRange * direction.normalized();
          }
          node1[0] = origin[0] + direction[0];
          node1[1] = origin[1] + direction[1];
          node1[2] = origin[2] + direction[2];
          global_vertex_size_++;
          // Create new node and insert into tree
          dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
          newNode->state_ = node1;
          newNode->parent_ = newParent;
          newNode->distance_ = newParent->distance_ + direction.norm();
          newParent->children_.push_back(newNode);
          newNode->gain_ = gain(newNode->state_,newNode->time_cost());

          kd_insert3(kdTree_, node1.x(), node1.y(), node1.z(), newNode);

          // save new node to node_array
          dual_state_graph_->local_graph_.vertices[i].information_gain = newNode->gain_;
          node_array.push_back(newNode);

          if (newNode->gain_ > bestGain_)
          {
            if (std::find(executedBestNodeList_.begin(), executedBestNodeList_.end(), i) != executedBestNodeList_.end())
            {
              bestGain_ = newNode->gain_;
              bestNodeId_ = i;
            }
          }
          // nodeCounter_++;
        }
        executedBestNodeList_.push_back(bestNodeId_);
        nodeCounter_ = dual_state_graph_->global_graph_.vertices.size();
      }
    }
    else
    {
      rootNode_->state_ = root_;
      kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
      iterationCount_++;
    }
    maxX_ = params_.kMaxXGlobalBound;
    maxY_ = params_.kMaxYGlobalBound;
    maxZ_ = params_.kMaxZGlobalBound;
    minX_ = params_.kMinXGlobalBound;
    minY_ = params_.kMinYGlobalBound;
    minZ_ = params_.kMinZGlobalBound;
  }

  // ROS_INFO("params_.kMinRepulsiveFieldBound");
  // ROS_INFO("X: %f, Y: %f, Z: %f", params_.kMinXRepulsiveFieldBound, params_.kMinYRepulsiveFieldBound, params_.kMinZRepulsiveFieldBound);
    
  // ROS_INFO("Max Repulsive Field Bounds:");
  // ROS_INFO("X: %f, Y: %f, Z: %f", params_.kMaxXRepulsiveFieldBound, params_.kMaxYRepulsiveFieldBound, params_.kMaxZRepulsiveFieldBound);
  publishPlanningHorizon();

}



void dsvplanner_ns::Drrt::publishPlanningHorizon()
{  // Publish visualization of
   // current planning horizon
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.frame_id = params_.explorationFrame;
  p.id = 0;
  p.ns = "boundary";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (minX_ + maxX_);
  p.pose.position.y = 0.5 * (minY_ + maxY_);
  p.pose.position.z = 0.5 * (minZ_ + maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = maxX_ - minX_;
  p.scale.y = maxY_ - minY_;
  p.scale.z = maxZ_ - minZ_;
  p.color.r = 252.0 / 255.0;
  p.color.g = 145.0 / 255.0;
  p.color.b = 37.0 / 255.0;
  p.color.a = 0.3;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.boundaryPub_.publish(p);
}



//修剪一个树状结构
void dsvplanner_ns::Drrt::pruneTree(StateVec root)
{
  dual_state_graph_->pruned_graph_.vertices.clear();
  geometry_msgs::Pose p1;
  p1.position.x = root[0];
  p1.position.y = root[1];
  p1.position.z = root[2];
  p1.orientation.y = params_.kZeroGain;
  dual_state_graph_->addNewLocalVertexWithoutDuplicates(p1, dual_state_graph_->pruned_graph_);

  geometry_msgs::Point root_point;
  root_point.x = root[0];
  root_point.y = root[1];
  root_point.z = root[2];
  dual_state_graph_->pruneGraph(root_point);

  StateVec node;
  for (int i = 1; i < dual_state_graph_->pruned_graph_.vertices.size(); i++)
  {
    node[0] = dual_state_graph_->pruned_graph_.vertices[i].location.x;
    node[1] = dual_state_graph_->pruned_graph_.vertices[i].location.y;
    node[2] = dual_state_graph_->pruned_graph_.vertices[i].location.z;

    kdres* nearest = kd_nearest3(kdTree_, node.x(), node.y(), node.z());
    if (kd_res_size(nearest) <= 0)
    {
      kd_res_free(nearest);
      continue;
    }
    dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
    kd_res_free(nearest);

    // Check for collision
    StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    StateVec direction(node[0] - origin[0], node[1] - origin[1], node[2] - origin[2]);
    if(dra_->inDynamicRiskArea(node))
    {
      continue;
    }
    dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
    newNode->state_ = node;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);
    if (dual_state_graph_->pruned_graph_.vertices[i].information_gain > 0)
      newNode->gain_ = gain(newNode->state_,newNode->time_cost());
    else
    {
      newNode->gain_ = 0;
    }
    kd_insert3(kdTree_, node.x(), node.y(), node.z(), newNode);
    node_array.push_back(newNode);

    if (newNode->gain_ > bestGain_)
    {
      bestGain_ = newNode->gain_;
    }
    dual_state_graph_->pruned_graph_.vertices[i].information_gain = newNode->gain_;
  }
  remainingNodeCount_ = node_array.size();
}
//通过探索周围环境可以获得的增益。增益的计算基于OctomapManager提供的单元格状态信息，包括未知（kUnknown）、占据（kOccupied）和空闲（kFree）。
//here you can change the gain to dynamic gain

double dsvplanner_ns::Drrt::gain(StateVec state, double time_of_arrival)
{
  // This function computes the gain
  double gain = 0.0;
  const double disc = manager_->getResolution();
  StateVec origin(state[0], state[1], state[2]);
  StateVec vec;
  // Compute the index for the covariance matrix的索引 index，它表示在时间 time_of_arrival 处的协方差矩阵的索引
  double max_time_step = KFiterations * time_step;
  int index = getCovarianceIndex(max_time_step, time_step, time_of_arrival);
  bool blocked = false;
  //定义一个搜索范围 rangeSq，它是 params_.kGainRange 的平方 5
  double rangeSq = pow(params_.kGainRange, 2.0);
  //使用三个嵌套的循环遍历以 state 为中心的立方体区域内的所有单元格。循环变量 vec 表示当前单元格的位置
  // Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - params_.kGainRange, minX_); vec[0] < std::min(state[0] + params_.kGainRange, maxX_);
       vec[0] += disc)
  {
    for (vec[1] = std::max(state[1] - params_.kGainRange, minY_);
         vec[1] < std::min(state[1] + params_.kGainRange, maxY_); vec[1] += disc)
    {
      for (vec[2] = std::max(state[2] - params_.kGainRangeZMinus, minZ_);
           vec[2] < std::min(state[2] + params_.kGainRangeZPlus, maxZ_); vec[2] += disc)
      {
        StateVec dir = vec - origin;
        //计算从 state 到该单元格的向量 dir，并检查其距离是否在搜索范围内。如果距离大于 rangeSq，则跳过该单元格
        // Skip if distance is too large
        if (dir.transpose().dot(dir) > rangeSq)
        {
          continue;
        }
        //检查单元格是否在传感器的视场角内。对于 Velodyne 传感器，这通过比较单元格的 Z 坐标与水平方向上的投影距离来实现。如果单元格不在视场角内，则跳过该单元格。
        bool insideAFieldOfView = false;
        // Check that voxel center is inside the field of view. This check is
        // for velodyne.
        if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * params_.sensorVerticalView / 360)))
        {
          insideAFieldOfView = true;
        }
        if (!insideAFieldOfView)
        {
          continue;
        }

        // Check cell status and add to the gain considering the corresponding
        // factor.
        double probability;
        //根据 OctomapManager 提供的单元格状态信息，更新增益值。如果单元格状态为未知，且从 state 到该单元格的视线未被占据，
        //则增加 params_.kGainUnknown 的增益。如果单元格状态为占据，且视线未被占据，则增加 params_.kGainOccupied 的增益。如果单元格状态为空闲，且视线未被占据，则增加 params_.kGainFree 的增益
        volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(vec, &probability);
        if(not blocked && index != -1){
          blocked = dra_->willViewBeBlocked(vec, index, visualize_static_and_dynamic_rays);
        }
        if(blocked)
        {
            gain += params_.kGainCollision;
        }
        else{
        if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown)
        {
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
              this->manager_->getVisibility(origin, vec, false))
          {
            gain += params_.kGainUnknown;
          }
        }
        else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied)
        {
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
              this->manager_->getVisibility(origin, vec, false))
          {
            gain += params_.kGainOccupied;
          }
        }
        else
        {
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
              this->manager_->getVisibility(origin, vec, false))
          {
            gain += params_.kGainFree;
          }
        }
        }
      }
    }
  }

  // Scale with volume
  if (gain < params_.kMinEffectiveGain)
    gain = 0;
    //将增益值乘以单元格的体积（即 disc 的立方），以考虑搜索区域的大小。
  gain *= pow(disc, 3.0);

  return gain;
}



int dsvplanner_ns::Drrt::getCovarianceIndex(double max_time_step, double time_step, double t){
  
  int num_steps = static_cast<int>(max_time_step / time_step); // Calculate the number of time steps

  if(t > max_time_step)
  {
    //No prediction available
    return -1;
  }

  // Initialize the minimum difference and index
  double min_difference = std::abs(t - time_step);
  int index = 0;

  // Loop through each time step and find the closest one
  for (int i = 1; i < num_steps; ++i) {
    double time = i * time_step; // Calculate the time for the current step
    double difference = std::abs(t - time);
    if (difference < min_difference) {
      min_difference = difference;
      index = i;
    }
  }
  return index;
}


// double dsvplanner_ns::Drrt::gain(StateVec state)
// {
//   // This function computes the gain
//   double gain = 0.0;
//   const double disc = manager_->getResolution();
//   StateVec origin(state[0], state[1], state[2]);
//   StateVec vec;
//   //定义一个搜索范围 rangeSq，它是 params_.kGainRange 的平方 5
//   double rangeSq = pow(params_.kGainRange, 2.0);
//   //使用三个嵌套的循环遍历以 state 为中心的立方体区域内的所有单元格。循环变量 vec 表示当前单元格的位置
//   // Iterate over all nodes within the allowed distance
//   for (vec[0] = std::max(state[0] - params_.kGainRange, minX_); vec[0] < std::min(state[0] + params_.kGainRange, maxX_);
//        vec[0] += disc)
//   {
//     for (vec[1] = std::max(state[1] - params_.kGainRange, minY_);
//          vec[1] < std::min(state[1] + params_.kGainRange, maxY_); vec[1] += disc)
//     {
//       for (vec[2] = std::max(state[2] - params_.kGainRangeZMinus, minZ_);
//            vec[2] < std::min(state[2] + params_.kGainRangeZPlus, maxZ_); vec[2] += disc)
//       {
//         StateVec dir = vec - origin;
//         //计算从 state 到该单元格的向量 dir，并检查其距离是否在搜索范围内。如果距离大于 rangeSq，则跳过该单元格
//         // Skip if distance is too large
//         if (dir.transpose().dot(dir) > rangeSq)
//         {
//           continue;
//         }
//         //检查单元格是否在传感器的视场角内。对于 Velodyne 传感器，这通过比较单元格的 Z 坐标与水平方向上的投影距离来实现。如果单元格不在视场角内，则跳过该单元格。
//         bool insideAFieldOfView = false;
//         // Check that voxel center is inside the field of view. This check is
//         // for velodyne.
//         if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * params_.sensorVerticalView / 360)))
//         {
//           insideAFieldOfView = true;
//         }
//         if (!insideAFieldOfView)
//         {
//           continue;
//         }

//         // Check cell status and add to the gain considering the corresponding
//         // factor.
//         double probability;
//         //根据 OctomapManager 提供的单元格状态信息，更新增益值。如果单元格状态为未知，且从 state 到该单元格的视线未被占据，
//         //则增加 params_.kGainUnknown 的增益。如果单元格状态为占据，且视线未被占据，则增加 params_.kGainOccupied 的增益。如果单元格状态为空闲，且视线未被占据，则增加 params_.kGainFree 的增益
//         volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(vec, &probability);
//         if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown)
//         {
//           if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
//               this->manager_->getVisibility(origin, vec, false))
//           {
//             gain += params_.kGainUnknown;
//           }
//         }
//         else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied)
//         {
//           if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
//               this->manager_->getVisibility(origin, vec, false))
//           {
//             gain += params_.kGainOccupied;
//           }
//         }
//         else
//         {
//           if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
//               this->manager_->getVisibility(origin, vec, false))
//           {
//             gain += params_.kGainFree;
//           }
//         }
//       }
//     }
//   }

//   // Scale with volume
//   if (gain < params_.kMinEffectiveGain)
//     gain = 0;
//     //将增益值乘以单元格的体积（即 disc 的立方），以考虑搜索区域的大小。
//   gain *= pow(disc, 3.0);

//   return gain;
// }

void dsvplanner_ns::Drrt::clear()
{
  delete rootNode_;
  rootNode_ = NULL;

  nodeCounter_ = 0;
  bestGain_ = params_.kZeroGain;
  bestNode_ = NULL;
  if (nextNodeFound_)
  {
    dual_state_graph_->clearLocalGraph();
  }
  nextNodeFound_ = false;
  remainingFrontier_ = false;
  remainingNodeCount_ = 0;

  sampledPoint_->points.clear();

  kd_free(kdTree_);
}

void dsvplanner_ns::Drrt::publishNode()
{
  sensor_msgs::PointCloud2 random_sampled_points_pc;
  pcl::toROSMsg(*sampledPoint_, random_sampled_points_pc);
  random_sampled_points_pc.header.frame_id = params_.explorationFrame;
  params_.randomSampledPointsPub_.publish(random_sampled_points_pc);

  visualization_msgs::Marker node;
  visualization_msgs::Marker branch;
  node.header.stamp = ros::Time::now();
  node.header.frame_id = params_.explorationFrame;
  node.ns = "drrt_node";
  node.type = visualization_msgs::Marker::POINTS;
  node.action = visualization_msgs::Marker::ADD;
  node.scale.x = params_.kRemainingNodeScaleSize;
  node.color.r = 167.0 / 255.0;
  node.color.g = 167.0 / 255.0;
  node.color.b = 0.0;
  node.color.a = 1.0;
  node.frame_locked = false;

  branch.ns = "drrt_branches";
  branch.header.stamp = ros::Time::now();
  branch.header.frame_id = params_.explorationFrame;
  branch.type = visualization_msgs::Marker::LINE_LIST;
  branch.action = visualization_msgs::Marker::ADD;
  branch.scale.x = params_.kRemainingBranchScaleSize;
  branch.color.r = 167.0 / 255.0;
  branch.color.g = 167.0 / 255.0;
  branch.color.b = 0.0;
  branch.color.a = 1.0;
  branch.frame_locked = false;

  geometry_msgs::Point node_position;
  geometry_msgs::Point parent_position;
  if (remainingNodeCount_ > 0 && remainingNodeCount_ <= node_array.size())
  {
    for (int i = 0; i < remainingNodeCount_; i++)
    {
      node_position.x = node_array[i]->state_[0];
      node_position.y = node_array[i]->state_[1];
      node_position.z = node_array[i]->state_[2];
      node.points.push_back(node_position);

      if (node_array[i]->parent_)
      {
        parent_position.x = node_array[i]->parent_->state_[0];
        parent_position.y = node_array[i]->parent_->state_[1];
        parent_position.z = node_array[i]->parent_->state_[2];

        branch.points.push_back(parent_position);
        branch.points.push_back(node_position);
      }
    }
    params_.remainingTreePathPub_.publish(node);
    params_.remainingTreePathPub_.publish(branch);
    node.points.clear();
    branch.points.clear();
    node.scale.x = params_.kNewNodeScaleSize;
    node.color.r = 167.0 / 255.0;
    node.color.g = 0.0 / 255.0;
    node.color.b = 167.0 / 255.0;
    node.color.a = 1.0;
    branch.scale.x = params_.kNewBranchScaleSize;
    branch.color.r = 167.0 / 255.0;
    branch.color.g = 0.0 / 255.0;
    branch.color.b = 167.0 / 255.0;
    branch.color.a = 1.0;
    for (int i = remainingNodeCount_; i < node_array.size(); i++)
    {
      node_position.x = node_array[i]->state_[0];
      node_position.y = node_array[i]->state_[1];
      node_position.z = node_array[i]->state_[2];
      node.points.push_back(node_position);

      if (node_array[i]->parent_)
      {
        parent_position.x = node_array[i]->parent_->state_[0];
        parent_position.y = node_array[i]->parent_->state_[1];
        parent_position.z = node_array[i]->parent_->state_[2];

        branch.points.push_back(parent_position);
        branch.points.push_back(node_position);
      }
    }
    params_.newTreePathPub_.publish(node);
    params_.newTreePathPub_.publish(branch);
  }
  else
  {
    for (int i = 0; i < node_array.size(); i++)
    {
      node_position.x = node_array[i]->state_[0];
      node_position.y = node_array[i]->state_[1];
      node_position.z = node_array[i]->state_[2];
      node.points.push_back(node_position);

      if (node_array[i]->parent_)
      {
        parent_position.x = node_array[i]->parent_->state_[0];
        parent_position.y = node_array[i]->parent_->state_[1];
        parent_position.z = node_array[i]->parent_->state_[2];

        branch.points.push_back(parent_position);
        branch.points.push_back(node_position);
      }
    }
    params_.newTreePathPub_.publish(node);
    params_.newTreePathPub_.publish(branch);

    // When there is no remaining node, publish an empty one
    node.points.clear();
    branch.points.clear();
    params_.remainingTreePathPub_.publish(node);
    params_.remainingTreePathPub_.publish(branch);
  }
}

void dsvplanner_ns::Drrt::gotoxy(int x, int y)
{
  printf("%c[%d;%df", 0x1B, y, x);
}

#endif
