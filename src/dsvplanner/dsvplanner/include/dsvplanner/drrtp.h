/**************************************************************************
drrtp.h
Header of the drrtp (dynamic_rrtplanner) class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/

#ifndef DRRTP_H_
#define DRRTP_H_

#include "dsvplanner/clean_frontier_srv.h"
#include "dsvplanner/drrt.h"
#include "dsvplanner/drrt_base.h"
#include "dsvplanner/dsvplanner_srv.h"
#include "dsvplanner/dual_state_frontier.h"
#include "dsvplanner/dual_state_graph.h"
#include "dsvplanner/grid.h"
#include "octomap_world/octomap_manager.h"
#include <dsvplanner/dra.h>

namespace dsvplanner_ns
{
class drrtPlanner
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber odomSub_;
  ros::Subscriber boundarySub_;
  ros::Subscriber dynamic_sub_;

  ros::ServiceServer plannerService_;
  ros::ServiceServer cleanFrontierService_;

  Params params_;
  volumetric_mapping::OctomapManager* manager_;
  DualStateGraph* dual_state_graph_;
  DualStateFrontier* dual_state_frontier_;
  Drrt* drrt_;
  OccupancyGrid* grid_;
  Dra* dra_;
  visualization_msgs::MarkerArray dynamicRiskArea;

  bool init();
  bool setParams();
  // bool setPublisherPointer();
  void odomCallback(const nav_msgs::Odometry& pose);
  void updateDynamicObstaclesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg); 
  void boundaryCallback(const geometry_msgs::PolygonStamped& boundary);
  bool plannerServiceCallback(dsvplanner::dsvplanner_srv::Request& req, dsvplanner::dsvplanner_srv::Response& res);
  bool cleanFrontierServiceCallback(dsvplanner::clean_frontier_srv::Request& req,
                                    dsvplanner::clean_frontier_srv::Response& res);
  void cleanLastSelectedGlobalFrontier();
  geometry_msgs::Point Repulsive_field(const geometry_msgs::Point& robot_pose);
  void visualizeRepulsiveForce(const geometry_msgs::Point& resultant_force, const geometry_msgs::Point& robot_pose);
  void publishRepulsiveField();
  bool inRepulsiveFieldBoundary(const geometry_msgs::Point& position, double major_length, double minor_length);
  void getRobotAndRepulsiveForce(geometry_msgs::Point& out_repulsive_force, bool avoidance);

  drrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~drrtPlanner();

private:
  std::string odomSubTopic;
  std::string boundarySubTopic;
  std::string newTreePathPubTopic;
  std::string remainingTreePathPubTopic;
  std::string boundaryPubTopic;
  std::string repulsiveFieldPubTopic;
  std::string forcePubTopic;
  std::string globalSelectedFrontierPubTopic;
  std::string localSelectedFrontierPubTopic;
  std::string plantimePubTopic;
  std::string nextGoalPubTopic;
  std::string randomSampledPointsPubTopic;
  std::string shutDownTopic;
  std::string plannerServiceName;
  std::string cleanFrontierServiceName;
  int ETA_REPLUSIVE = 3;
  int DIS_OBSTACLE = 6;
  double min_repulsive_field_X_;
  double min_repulsive_field_Y_;
  double min_repulsive_field_Z_;
  double max_repulsive_field_X_;
  double max_repulsive_field_Y_;
  double max_repulsive_field_Z_;
  bool avoidance_ = false;
  geometry_msgs::Point repulsive_force;
  geometry_msgs::Point robot_pos;

  std::chrono::steady_clock::time_point plan_start_;
  std::chrono::steady_clock::time_point RRT_generate_over_;
  std::chrono::steady_clock::time_point gain_computation_over_;
  std::chrono::steady_clock::time_point plan_over_;
  std::chrono::steady_clock::duration time_span;
};
}

#endif  // DRRTP_H
