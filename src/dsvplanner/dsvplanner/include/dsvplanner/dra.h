/**************************************************************************
dra.h
Header of the dra (dynamic_risk_area) class

Jianfeng Mao(202312131132@stu.cqu.edu.cn)
10/28/2024

**************************************************************************/

#ifndef DRA_H_
#define DRA_H_


#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "dsvplanner/dual_state_frontier.h"
#include "dsvplanner/grid.h"
#include "graph_planner/GraphPlannerStatus.h"
#include "graph_utils/TopologicalGraph.h"
#include "octomap_world/octomap_manager.h"
#include <gazebo_msgs/ModelStates.h> 

namespace dsvplanner_ns
{
class Dra
{
public:
 
  void updateDynamicPositions(const gazebo_msgs::ModelStates& model_states);
  std::vector<Eigen::Vector2d> CreateDynamicRiskAreacalculateContours(const auto& covariances);
  void dealwithPredictedData();
  void CreateDynamicRiskArea(ros::Publisher& marker_pub);
  void Visualize_Dynamic_Pose(ros::Publisher& marker_pub, const int& object_id, const geometry_msgs::Pose& pose);
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories();
  void visualizePrediction(ros::Publisher& line_marker_pub, 
                         ros::Publisher& ellipse_marker_pub, 
                         const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                         const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses
                        ) ;
  void calculateAcceleration(double vx_prev, double vy_prev, double vx_curr, double vy_curr, double dt, double& ax, double& ay);
  void visualizeGhostPedestrian(std::vector<std::tuple<double, double, double>> positions); 
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices);
  bool inDynamicRiskArea(StateVec node);
  bool checkCollision(double time, Eigen::Vector3d point);
  int  getCovarianceIndex(double max_time_step, double time_step, double t);
  bool isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
            std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse);
  bool isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z);
  bool willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts);
  visualization_msgs::MarkerArray getEllipseMarkerArray();

  // General Functions
  
  // Constructor
  Dra(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Destructor
  ~Dra();

  

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher dynamic_obstacle_pub;
    // ros::Subscriber dynamic_sub;
    ros::Publisher dynamic_risk_area_pub;
    ros::Publisher pred_marker_pub;
    ros::Publisher ghost_marker_pub;
    bool dynamic_mode_ = false;
    double dynamic_long = 0.55;
    double dynamic_width = 0.4;
    double dynamic_height= 1.8;
    double drone_height = 0.4;
    double vx_prev = 0, vy_prev = 0;
    std::map<std::string, std::tuple<geometry_msgs::Pose, geometry_msgs::Twist, geometry_msgs::Twist>> dynamic_objects;
    std::map<std::string, std::tuple<geometry_msgs::Pose, geometry_msgs::Twist, geometry_msgs::Twist>> previous_dynamic_objects; // 存储上一时刻数据
    std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data;
    int KFiterations = 20;
    double time_step = 0.5;   //0.5
    int vpre_counter = 0;
    std::mutex vecMutex;
    visualization_msgs::Marker ellipse_marker;
    visualization_msgs::MarkerArray ellipse_marker_array;
    std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
    std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};
};
}

#endif  // DRA_H