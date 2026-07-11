/*
drrtp_node.cpp
node to launch drrtplanner

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <dsvplanner/drrtp.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // 初始化 drrtPlanner 规划器对象，传入节点句柄和私有节点句柄
  dsvplanner_ns::drrtPlanner planner(nh, nh_private);
  // 进入 ROS 循环，等待回调函数
  ros::spin();
  return 0;
}
