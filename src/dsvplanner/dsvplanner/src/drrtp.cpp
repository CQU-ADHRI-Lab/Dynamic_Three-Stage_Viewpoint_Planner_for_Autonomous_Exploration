/*
drrtp.cpp
Implementation of drrt_planner class

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>

#include <visualization_msgs/Marker.h>

#include <dsvplanner/drrtp.h>

using namespace Eigen;

/**
 * @brief 构造函数，初始化 drrtPlanner 对象
 * @param nh ROS 节点句柄
 * @param nh_private 私有 ROS 节点句柄
 * new 是一个关键字，用于动态分配内存。它的作用是在程序运行时，从堆（heap）上分配一块内存，并返回该内存的地址。
 */
dsvplanner_ns::drrtPlanner::drrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  // 创建 OctomapManager 对象，用于管理 Octomap
  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);
  // 创建 OccupancyGrid 对象，用于存储占用网格
  grid_ = new OccupancyGrid(nh_, nh_private_);
  // 创建 DualStateGraph 对象，用于存储双状态图
  dual_state_graph_ = new DualStateGraph(nh_, nh_private_, manager_, grid_);
  // 创建 DualStateFrontier 对象，用于存储双状态边界
  dual_state_frontier_ = new DualStateFrontier(nh_, nh_private_, manager_, grid_);
  // 创建 Dra 对象，用于可视化动态障碍物
  dra_ = new Dra(nh_, nh_private_);
    // 创建 Drrt 对象，用于执行动态 RRT 规划
  drrt_ = new Drrt(manager_, dual_state_graph_, dual_state_frontier_, grid_, dra_);
  

  // 初始化规划器
  init();
  // 设置规划器参数
  drrt_->setParams(params_);
  // 初始化规划器
  drrt_->init();

  // 输出日志信息，表示 DSVP 节点已成功启动
  ROS_INFO("Successfully launched DSVP node");
}

dsvplanner_ns::drrtPlanner::~drrtPlanner()
{
  if (manager_)
  {
    delete manager_;
  }
  if (dual_state_graph_)
  {
    delete dual_state_graph_;
  }
  if (dual_state_frontier_)
  {
    delete dual_state_frontier_;
  }
  if (grid_)
  {
    delete grid_;
  }
  if (drrt_)
  {
    delete drrt_;
  }
  if (dra_)
  {
    delete dra_;
  }
}

void dsvplanner_ns::drrtPlanner::updateDynamicObstaclesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

    dra_->updateDynamicPositions(*msg);
}

void dsvplanner_ns::drrtPlanner::odomCallback(const nav_msgs::Odometry& pose)
{
  
  //调用 drrt_->setRootWithOdom(pose) 函数，根据接收到的里程计消息更新规划器的根节点位置。
  drrt_->setRootWithOdom(pose);
  robot_pos.x = drrt_->root_[0];
  robot_pos.y = drrt_->root_[1];
  robot_pos.z = drrt_->root_[2];
  // Planner is now ready to plan.
  drrt_->plannerReady_ = true;
    {
    min_repulsive_field_X_ = drrt_->root_[0] + params_.kMinXRepulsiveFieldBound;
    min_repulsive_field_Y_ = drrt_->root_[1] + params_.kMinYRepulsiveFieldBound;
    min_repulsive_field_Z_ = drrt_->root_[2] + params_.kMinZRepulsiveFieldBound;
    max_repulsive_field_X_ = drrt_->root_[0] + params_.kMaxXRepulsiveFieldBound;
    max_repulsive_field_Y_ = drrt_->root_[1] + params_.kMaxYRepulsiveFieldBound;
    max_repulsive_field_Z_ = drrt_->root_[2] + params_.kMaxZRepulsiveFieldBound;
  }
  repulsive_force.x = 0, repulsive_force.y = 0, repulsive_force.z = 0;
  publishRepulsiveField();
  repulsive_force = Repulsive_field(robot_pos);
  if (repulsive_force.x != 0.0 && repulsive_force.y != 0.0 ) {
      avoidance_ = true;
      dual_state_graph_->get_repulsive_force(repulsive_force);
      // 如果 repulsive_force 的任一分量不为零，则执行相关逻辑
  }
  else{
    avoidance_ = false;
  }
  dual_state_graph_->get_avoidance(avoidance_);

}




void dsvplanner_ns::drrtPlanner::getRobotAndRepulsiveForce(geometry_msgs::Point& out_repulsive_force, bool avoidance)
{
    // 返回机器人位置和斥力
    out_repulsive_force = repulsive_force;
    avoidance = avoidance_;
}



void dsvplanner_ns::drrtPlanner::boundaryCallback(const geometry_msgs::PolygonStamped& boundary)
{
  drrt_->setBoundary(boundary);
  dual_state_frontier_->setBoundary(boundary);
}
//处理来自客户端的规划请求，并生成相应的路径规划响应。
bool dsvplanner_ns::drrtPlanner::plannerServiceCallback(dsvplanner::dsvplanner_srv::Request& req,
                                                        dsvplanner::dsvplanner_srv::Response& res)
{
  //记录了规划开始的时间，使用 std::chrono::steady_clock::now() 获取当前时间。
  plan_start_ = std::chrono::steady_clock::now();
  // drrt_->gotoxy(0, 10);  // Go to the specific line on the screen
  // Check if the planner is ready.
  if (!drrt_->plannerReady_)
  {
    std::cout << "No odometry. Planner is not ready!" << std::endl;
    return true;
  }
  //如果是，则打印一条错误消息并返回 true，表示没有八叉树地图
  if (manager_ == NULL)
  {
    std::cout << "No octomap. Planner is not ready!" << std::endl;
    return true;
  }
  //如果地图大小的范数小于等于 0.0，则打印一条错误消息并返回 true，表示地图为空
  if (manager_->getMapSize().norm() <= 0.0)
  {
    std::cout << "Octomap is empty. Planner is not set up!" << std::endl;
    return true;
  }
  //调用 drrt_ 对象的 setTerrainVoxelElev 方法，设置地形体素高程
  // set terrain points and terrain voxel elevation
  drrt_->setTerrainVoxelElev();


  // Clear old tree and the last global frontier.
  cleanLastSelectedGlobalFrontier();
  drrt_->clear();
  // Reinitialize.
  drrt_->plannerInit();
    // Update planner state of next iteration
  geometry_msgs::Point robot_position;
  robot_position.x = drrt_->root_[0];
  robot_position.y = drrt_->root_[1];
  robot_position.z = drrt_->root_[2];

  // Iterate the tree construction method.
  int loopCount = 0;
   // 进入循环，当 ROS 运行正常、存在剩余边界、节点计数器未达到截止迭代次数，并且不满足局部迭代且节点数达到阈值且找到增益的条件时
  while (ros::ok() && drrt_->remainingFrontier_ && drrt_->getNodeCounter() < params_.kCuttoffIterations &&
         !(drrt_->normal_local_iteration_ && (drrt_->getNodeCounter() >= params_.kVertexSize && drrt_->gainFound())))
  {
    // 如果循环次数超过规划器循环计数的 (节点计数器 + 1) 倍，则跳出循环
    if (loopCount > drrt_->loopCount_ * (drrt_->getNodeCounter() + 1))
    {
      break;
    }
    // 执行规划器的迭代操作,create the rrt
    drrt_->plannerIterate();
    loopCount++;
  }

  // Publish rrt
  drrt_->publishNode();
  std::cout << "     New node number is " << drrt_->getNodeCounter() << "\n"
            << "     Current local RRT size is " << dual_state_graph_->getLocalVertexSize() << "\n"
            << "     Current global graph size is " << dual_state_graph_->getGlobalVertexSize() << std::endl;
  RRT_generate_over_ = std::chrono::steady_clock::now();
  time_span = RRT_generate_over_ - plan_start_;
  double rrtGenerateTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  // Reset planner state
  drrt_->global_plan_pre_ = drrt_->global_plan_;
  drrt_->global_plan_ = false;
  drrt_->local_plan_ = false;

  // update planner state of last iteration
  dual_state_frontier_->setPlannerStatus(drrt_->global_plan_pre_);

  //用于处理特定情况下的返回路径规划。
  if (!drrt_->nextNodeFound_ && drrt_->global_plan_pre_ && drrt_->gainFound() <= 0 )  // && drrt_->gainFound() <= 0
  {
    drrt_->return_home_ = true;
    geometry_msgs::Point home_position;
    home_position.x = 0;
    home_position.y = 0;
    home_position.z = 0;
    res.goal.push_back(home_position);
    res.mode.data = 2;  // mode 2 means returning home

    dual_state_frontier_->cleanAllUselessFrontiers();
    return true;
  }
  else if (!drrt_->nextNodeFound_ && !drrt_->global_plan_pre_ && dual_state_graph_->getGain(robot_position) <= 0)
  {
    drrt_->global_plan_ = true;
    std::cout << "     No Remaining local frontiers  "
              << "\n"
              << "     Switch to relocation stage "
              << "\n"
              << "     Total plan lasted " << 0 << std::endl;
    return true;
  }
  else
  {
    drrt_->local_plan_ = true;
  }
  gain_computation_over_ = std::chrono::steady_clock::now();
  time_span = gain_computation_over_ - RRT_generate_over_;
  double getGainTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;

  // Extract next goal.
  geometry_msgs::Point next_goal_position;
  // if(avoidance_)
  // {
  //   std::cout << "     avoidance_ " << avoidance_ << std::endl;

  // }
  if (drrt_->nextNodeFound_)
  {
    dual_state_graph_->best_vertex_id_ = drrt_->NextBestNodeIdx_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestGlobalVertexPosition();
  }
  else if (drrt_->global_plan_pre_ == true && drrt_->gainFound())
  {
    dual_state_graph_->best_vertex_id_ = drrt_->bestNodeId_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  }
  else
  {
    dual_state_graph_->updateGlobalGraph();
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  }
  dual_state_graph_->setCurrentPlannerStatus(drrt_->global_plan_pre_);
  res.goal.push_back(next_goal_position);
  res.mode.data = 1;  // mode 1 means exploration

  geometry_msgs::PointStamped next_goal_point;
  next_goal_point.header.frame_id = "map";
  next_goal_point.point = next_goal_position;
  params_.nextGoalPub_.publish(next_goal_point);

  plan_over_ = std::chrono::steady_clock::now();
  time_span = plan_over_ - plan_start_;
  double plantime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  std::cout << "     RRT generation lasted  " << rrtGenerateTime << "\n"
            << "     Computiong gain lasted " << getGainTime << "\n"
            << "     Total plan lasted " << plantime << std::endl;
  return true;
}

void dsvplanner_ns::drrtPlanner::publishRepulsiveField()
{ 
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.frame_id = params_.explorationFrame;
  p.id = 0;
  p.ns = "RepulsiveField";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (min_repulsive_field_X_ + max_repulsive_field_X_);
  p.pose.position.y = 0.5 * (min_repulsive_field_Y_ + max_repulsive_field_Y_);
  p.pose.position.z = 0.5 * (min_repulsive_field_Z_ + max_repulsive_field_Z_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = max_repulsive_field_X_ - min_repulsive_field_X_;
  p.scale.y = max_repulsive_field_Y_ - min_repulsive_field_Y_;
  p.scale.z = max_repulsive_field_Z_ - min_repulsive_field_Z_;
  p.color.r = 200.0 / 255.0; // 红色
  p.color.g = 160.0 / 255.0; // 绿色
  p.color.b = 255.0 / 255.0; // 蓝色
  p.color.a = 0.3;            // 不透明
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.repulsiveFieldPub_.publish(p);
}

bool dsvplanner_ns::drrtPlanner::inRepulsiveFieldBoundary(const geometry_msgs::Point& position, double major_length, double minor_length) {
    // 检查椭圆是否与排斥场边界相交
    if (position.x + major_length / 2 >= min_repulsive_field_X_ &&
        position.x - major_length / 2 <= max_repulsive_field_X_ &&
        position.y + minor_length / 2 >= min_repulsive_field_Y_ &&
        position.y - minor_length / 2 <= max_repulsive_field_Y_ &&
        position.z + 0.05 >= min_repulsive_field_Z_ && // 假设椭圆在z方向的厚度为0.1
        position.z - 0.05 <= max_repulsive_field_Z_) {
        return true; // 如果椭圆有一部分在边界内，返回 true
    }
    return false; // 否则返回 false
}

geometry_msgs::Point dsvplanner_ns::drrtPlanner::Repulsive_field(const geometry_msgs::Point& robot_pose) {

        visualization_msgs::MarkerArray dynamic_obscacles = dra_->getEllipseMarkerArray();
            // 初始化合力
        geometry_msgs::Point resultant_force;
        resultant_force.x = 0.0;
        resultant_force.y = 0.0;
        resultant_force.z = 0.0;

        for (const auto& marker : dynamic_obscacles.markers) {
          const geometry_msgs::Point& position = marker.pose.position;
          double major_length = marker.scale.x; // 椭圆的长轴
          double minor_length = marker.scale.y; // 椭圆的短轴

        // 检查椭圆是否在排斥场边界内
        if (inRepulsiveFieldBoundary(position, major_length, minor_length)) {
            float dis_obst = sqrt(pow(position.x - robot_pose.x, 2) + pow(position.y - robot_pose.y, 2));

            if (dis_obst <= DIS_OBSTACLE) {
                float temp = (1 / dis_obst - 1 / DIS_OBSTACLE);
                
                // 计算斥力强度
                float repulsive_strength = 0.5 * ETA_REPLUSIVE * temp * temp;

                // 计算斥力的方向（负梯度）
                float norm_x = (robot_pose.x - position.x) / dis_obst; // 归一化方向
                float norm_y = (robot_pose.y - position.y) / dis_obst; // 归一化方向

                // 斥力合力的计算
                resultant_force.x += repulsive_strength * norm_x;
                resultant_force.y += repulsive_strength * norm_y;
            }
        }
    }

       visualizeRepulsiveForce(resultant_force, robot_pose);
       return resultant_force;
    }


void dsvplanner_ns::drrtPlanner::visualizeRepulsiveForce(const geometry_msgs::Point& resultant_force, const geometry_msgs::Point& robot_pose) {
    visualization_msgs::Marker marker;
    // 设置标记的类型为箭头
    marker.header.frame_id = "map"; // 使用适当的坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "repulsive_force";
    marker.id = 0; // 每个标记的唯一ID
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // 设置箭头的起点为机器人位置
    marker.points.resize(2);
    marker.points[0] = robot_pose;

    // 计算箭头的终点
    geometry_msgs::Point end_point;
    end_point.x = robot_pose.x + resultant_force.x; // 箭头终点位置
    end_point.y = robot_pose.y + resultant_force.y;
    end_point.z = robot_pose.z; // 可以保持z坐标不变

    marker.points[1] = end_point;

    // 只有在终点有效时才发布
    if (end_point.x != robot_pose.x || end_point.y != robot_pose.y) {
        // 设置标记的颜色和大小
        marker.scale.x = 1; // 箭头的粗度
        marker.scale.y = 2; // 箭头的头部大小
        marker.scale.z = 0.0; // Z轴无意义

        marker.color.r = 1.0; // 红色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // 透明度

        // 发布标记
        params_.force_pub_.publish(marker);
    }
}


bool dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback(dsvplanner::clean_frontier_srv::Request& req,
                                                              dsvplanner::clean_frontier_srv::Response& res)
{
  if (drrt_->nextNodeFound_)
  {
    dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
    dual_state_frontier_->gloabalFrontierUpdate();
  }
  else
  {
    dual_state_graph_->clearLocalGraph();
  }
  res.success = true;

  return true;
}

void dsvplanner_ns::drrtPlanner::cleanLastSelectedGlobalFrontier()
{
  // only when last plan is global plan, this function will be executed to clear
  // last selected global
  // frontier.
  if (drrt_->nextNodeFound_)
  {
    dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
    dual_state_frontier_->gloabalFrontierUpdate();
  }
}

bool dsvplanner_ns::drrtPlanner::setParams()
{
  nh_private_.getParam("/rm/kSensorPitch", params_.sensorPitch);
  nh_private_.getParam("/rm/kSensorHorizontal", params_.sensorHorizontalView);
  nh_private_.getParam("/rm/kSensorVertical", params_.sensorVerticalView);
  nh_private_.getParam("/rm/kVehicleHeight", params_.kVehicleHeight);
  nh_private_.getParam("/rm/kBoundX", params_.boundingBox[0]);
  nh_private_.getParam("/rm/kBoundY", params_.boundingBox[1]);
  nh_private_.getParam("/rm/kBoundZ", params_.boundingBox[2]);
  nh_private_.getParam("/drrt/gain/kFree", params_.kGainFree);
  nh_private_.getParam("/drrt/gain/kOccupied", params_.kGainOccupied);
  nh_private_.getParam("/drrt/gain/kUnknown", params_.kGainUnknown);
  nh_private_.getParam("/drrt/gain/kCollision", params_.kGainCollision);
  nh_private_.getParam("/drrt/gain/kMinEffectiveGain", params_.kMinEffectiveGain);
  nh_private_.getParam("/drrt/gain/kRange", params_.kGainRange);
  nh_private_.getParam("/drrt/gain/kRangeZMinus", params_.kGainRangeZMinus);
  nh_private_.getParam("/drrt/gain/kRangeZPlus", params_.kGainRangeZPlus);
  nh_private_.getParam("/drrt/gain/kZero", params_.kZeroGain);
  nh_private_.getParam("/drrt/tree/kExtensionRange", params_.kExtensionRange);
  nh_private_.getParam("/drrt/tree/kMinExtensionRange", params_.kMinextensionRange);
  nh_private_.getParam("/drrt/tree/kMaxExtensionAlongZ", params_.kMaxExtensionAlongZ);
  nh_private_.getParam("/rrt/tree/kExactRoot", params_.kExactRoot);
  nh_private_.getParam("/drrt/tree/kCuttoffIterations", params_.kCuttoffIterations);
  nh_private_.getParam("/drrt/tree/kGlobalExtraIterations", params_.kGlobalExtraIterations);
  nh_private_.getParam("/drrt/tree/kRemainingNodeScaleSize", params_.kRemainingNodeScaleSize);
  nh_private_.getParam("/drrt/tree/kRemainingBranchScaleSize", params_.kRemainingBranchScaleSize);
  nh_private_.getParam("/drrt/tree/kNewNodeScaleSize", params_.kNewNodeScaleSize);
  nh_private_.getParam("/drrt/tree/kNewBranchScaleSize", params_.kNewBranchScaleSize);
  nh_private_.getParam("/drrt/tfFrame", params_.explorationFrame);
  nh_private_.getParam("/drrt/vertexSize", params_.kVertexSize);
  nh_private_.getParam("/drrt/keepTryingNum", params_.kKeepTryingNum);
  nh_private_.getParam("/drrt/kLoopCountThres", params_.kLoopCountThres);
  nh_private_.getParam("/lb/kMinXLocal", params_.kMinXLocalBound);
  nh_private_.getParam("/lb/kMinYLocal", params_.kMinYLocalBound);
  nh_private_.getParam("/lb/kMinZLocal", params_.kMinZLocalBound);
  nh_private_.getParam("/lb/kMaxXLocal", params_.kMaxXLocalBound);
  nh_private_.getParam("/lb/kMaxYLocal", params_.kMaxYLocalBound);
  nh_private_.getParam("/lb/kMaxZLocal", params_.kMaxZLocalBound);
  nh_private_.getParam("/gb/kMinXGlobal", params_.kMinXGlobalBound);
  nh_private_.getParam("/gb/kMinYGlobal", params_.kMinYGlobalBound);
  nh_private_.getParam("/gb/kMinZGlobal", params_.kMinZGlobalBound);
  nh_private_.getParam("/gb/kMaxXGlobal", params_.kMaxXGlobalBound);
  nh_private_.getParam("/gb/kMaxYGlobal", params_.kMaxYGlobalBound);
  nh_private_.getParam("/gb/kMaxZGlobal", params_.kMaxZGlobalBound);
  nh_private_.getParam("/rf/kMinX", params_.kMinXRepulsiveFieldBound);
  nh_private_.getParam("/rf/kMinY", params_.kMinYRepulsiveFieldBound);
  nh_private_.getParam("/rf/kMinZ", params_.kMinZRepulsiveFieldBound);
  nh_private_.getParam("/rf/kMaxX", params_.kMaxXRepulsiveFieldBound);
  nh_private_.getParam("/rf/kMaxY", params_.kMaxYRepulsiveFieldBound);
  nh_private_.getParam("/rf/kMaxZ", params_.kMaxZRepulsiveFieldBound); 
  nh_private_.getParam("/elevation/kTerrainVoxelSize", params_.kTerrainVoxelSize);
  nh_private_.getParam("/elevation/kTerrainVoxelWidth", params_.kTerrainVoxelWidth);
  nh_private_.getParam("/elevation/kTerrainVoxelHalfWidth", params_.kTerrainVoxelHalfWidth);
  nh_private_.getParam("/planner/odomSubTopic", odomSubTopic);
  nh_private_.getParam("/planner/boundarySubTopic", boundarySubTopic);
  nh_private_.getParam("/planner/newTreePathPubTopic", newTreePathPubTopic);
  nh_private_.getParam("/planner/remainingTreePathPubTopic", remainingTreePathPubTopic);
  nh_private_.getParam("/planner/boundaryPubTopic", boundaryPubTopic);
  nh_private_.getParam("/planner/repulsiveFieldPubTopic", repulsiveFieldPubTopic);
  nh_private_.getParam("/planner/globalSelectedFrontierPubTopic", globalSelectedFrontierPubTopic);
  nh_private_.getParam("/planner/localSelectedFrontierPubTopic", localSelectedFrontierPubTopic);
  nh_private_.getParam("/planner/plantimePubTopic", plantimePubTopic);
  nh_private_.getParam("/planner/nextGoalPubTopic", nextGoalPubTopic);
  nh_private_.getParam("/planner/randomSampledPointsPubTopic", randomSampledPointsPubTopic);
  nh_private_.getParam("/planner/shutDownTopic", shutDownTopic);
  nh_private_.getParam("/planner/forcePubTopic",forcePubTopic);
  nh_private_.getParam("/planner/plannerServiceName", plannerServiceName);
  nh_private_.getParam("/planner/cleanFrontierServiceName", cleanFrontierServiceName);

  return true;
}

bool dsvplanner_ns::drrtPlanner::init()
{
  if (!setParams())
  {
    ROS_ERROR("Set parameters fail. Cannot start planning!");
  }

  odomSub_ = nh_.subscribe(odomSubTopic, 10, &dsvplanner_ns::drrtPlanner::odomCallback, this);
  boundarySub_ = nh_.subscribe(boundarySubTopic, 10, &dsvplanner_ns::drrtPlanner::boundaryCallback, this);
  dynamic_sub_ = nh_.subscribe("/gazebo/model_states", 1, &dsvplanner_ns::drrtPlanner::updateDynamicObstaclesCallback, this);
  params_.newTreePathPub_ = nh_.advertise<visualization_msgs::Marker>(newTreePathPubTopic, 1000);
  params_.remainingTreePathPub_ = nh_.advertise<visualization_msgs::Marker>(remainingTreePathPubTopic, 1000);
  params_.boundaryPub_ = nh_.advertise<visualization_msgs::Marker>(boundaryPubTopic, 1000);
  params_.repulsiveFieldPub_ = nh_.advertise<visualization_msgs::Marker>(repulsiveFieldPubTopic, 1000);
  params_.globalSelectedFrontierPub_ = nh_.advertise<sensor_msgs::PointCloud2>(globalSelectedFrontierPubTopic, 1000);
  params_.localSelectedFrontierPub_ = nh_.advertise<sensor_msgs::PointCloud2>(localSelectedFrontierPubTopic, 1000);
  params_.randomSampledPointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(randomSampledPointsPubTopic, 1000);
  params_.plantimePub_ = nh_.advertise<std_msgs::Float32>(plantimePubTopic, 1000);
  params_.nextGoalPub_ = nh_.advertise<geometry_msgs::PointStamped>(nextGoalPubTopic, 1000);
  params_.shutdownSignalPub = nh_.advertise<std_msgs::Bool>(shutDownTopic, 1000);
  params_.force_pub_ = nh_.advertise<visualization_msgs::Marker>(forcePubTopic, 10);

  plannerService_ = nh_.advertiseService(plannerServiceName, &dsvplanner_ns::drrtPlanner::plannerServiceCallback, this);
  cleanFrontierService_ =
      nh_.advertiseService(cleanFrontierServiceName, &dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback, this);

  return true;
}
