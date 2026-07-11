/**************************************************************************
drrt_base.h
Header of the based class for drrt

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/
#ifndef DRRT_BASE_H_
#define DRRT_BASE_H_

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

using namespace Eigen;
namespace dsvplanner_ns
{
struct Params
{
  ros::Publisher newTreePathPub_;
  ros::Publisher remainingTreePathPub_;
  ros::Publisher boundaryPub_;
  ros::Publisher repulsiveFieldPub_;
  ros::Publisher force_pub_;
  ros::Publisher globalSelectedFrontierPub_;
  ros::Publisher localSelectedFrontierPub_;
  ros::Publisher nextGoalPub_;
  ros::Publisher plantimePub_;
  ros::Publisher randomSampledPointsPub_;
  ros::Publisher shutdownSignalPub;

  double sensorPitch;
  double sensorHorizontalView;
  double sensorVerticalView;
  double kVehicleHeight;
  Eigen::Vector3d boundingBox;
  Eigen::Vector3d localBoundary;
  Eigen::Vector3d globalBoundary;

  double kGainFree;
  double kGainOccupied;
  double kGainUnknown;
  double kGainCollision;
  double kGainRange;
  double kGainRangeZMinus;
  double kGainRangeZPlus;
  double kZeroGain;

  double kExtensionRange;
  double kMinextensionRange;
  double kMaxExtensionAlongZ;
  bool kExactRoot;
  int kMinEffectiveGain;
  int kGlobalExtraIterations;
  int kCuttoffIterations;
  int kVertexSize;
  int kKeepTryingNum;
  int kLoopCountThres;

  double kMinXLocalBound;
  double kMinYLocalBound;
  double kMinZLocalBound;
  double kMaxXLocalBound;
  double kMaxYLocalBound;
  double kMaxZLocalBound;
  double kMinXGlobalBound;
  double kMinYGlobalBound;
  double kMinZGlobalBound;
  double kMaxXGlobalBound;
  double kMaxYGlobalBound;
  double kMaxZGlobalBound;

  double kMinXRepulsiveFieldBound;
  double kMinYRepulsiveFieldBound;
  double kMinZRepulsiveFieldBound;
  double kMaxXRepulsiveFieldBound;
  double kMaxYRepulsiveFieldBound;
  double kMaxZRepulsiveFieldBound;

  double kTerrainVoxelSize;
  int kTerrainVoxelWidth;
  int kTerrainVoxelHalfWidth;

  double kRemainingNodeScaleSize;
  double kRemainingBranchScaleSize;
  double kNewNodeScaleSize;
  double kNewBranchScaleSize;


  std::string explorationFrame;
};

class Node
{
public:
  Node(){};
  ~Node(){};
  Vector3d state_;
  Node* parent_;
  std::vector<Node*> children_;
  double gain_;
  double distance_;
  double average_linear_velocity = 0.8;
  bool collision ;
  double time_cost()
  {
    if (this->parent_)
      return this->time_to_reach(this->parent_) + this->parent_->time_cost();
    else
      return 0;
  }
  double time_to_reach(Node* other) {
    double current_x = this->state_[0];
    double current_y = this->state_[1];
    double current_z = this->state_[2];

    double target_x = other->state_[0];
    double target_y = other->state_[1];
    double target_z = other->state_[2];
    // Calculate Euclidean distance in x-y plane
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    double euclidean_distance = (p3 - q3).norm();

    // Calculate time to move linearly to target position
    double linear_time = euclidean_distance / average_linear_velocity;

    return linear_time;
  }
};
}

#endif  // DRRT_BASE_H_
