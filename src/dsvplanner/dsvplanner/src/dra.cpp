/*
dra.cpp
Implementation of Dra class. Provide dynamic risk areas with dynamically moving obstacles.

Created by Jianfeng Mao (202312131132@stu.cqu.edu.cn) 
10/28/2024
*/

#include <eigen3/Eigen/Dense>
#include <dsvplanner/dra.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_set>
#include <gazebo_msgs/ModelStates.h> 
#include <gazebo_msgs/ModelState.h>
#include <dsvplanner/kalman.h>

// Constructor

dsvplanner_ns::Dra::Dra(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)  : nh_(nh), nh_private_(nh_private)
 {
    // 初始化 ROS 节点
    // dynamic_sub = nh_.subscribe("/gazebo/model_states", 1, &dsvplanner_ns::Dra::updateDynamicPositions, this);
    dynamic_obstacle_pub = nh_.advertise<visualization_msgs::Marker>("/dynamic_obstacles_markers", 1000);
    dynamic_risk_area_pub = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_risk_area", 1000);
    pred_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("predicted_trajectory", 1000);
    ghost_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/ghost_pedestrian", 1000);

}

dsvplanner_ns::Dra::~Dra() {
    ROS_INFO("Dra destructor called, cleaning up resources.");
    // 析构函数
}

void dsvplanner_ns::Dra::updateDynamicPositions(const gazebo_msgs::ModelStates& model_states) {
    int dynamic_id = 0;
    for (size_t i = 0; i < model_states.name.size(); ++i) {
        if (model_states.name[i].find("person_walking") != std::string::npos || 
            model_states.name[i].find("small_robot") != std::string::npos) {
            dynamic_mode_ = true;
            std::string model_name = model_states.name[i];
            geometry_msgs::Pose person_pose = model_states.pose[i];
            geometry_msgs::Twist person_twist = model_states.twist[i];
            // 存储当前动态对象，包括位置、速度和初始加速度（假设为 0）
            geometry_msgs::Twist initial_acc; // 初始化加速度
            initial_acc.linear.x = 0.0;
            initial_acc.linear.y = 0.0;
            initial_acc.linear.z = 0.0;
            dynamic_objects[model_name] = std::make_tuple(person_pose, person_twist, initial_acc);
            Visualize_Dynamic_Pose(dynamic_obstacle_pub, dynamic_id, person_pose);
            }
        dynamic_id++;
        vpre_counter++;
    }
    if (vpre_counter >= 1) {
        // 进行加速度计算
        for (const auto& entry : dynamic_objects) {
            const std::string& model_name = entry.first;
            const geometry_msgs::Twist& current_twist = std::get<1>(entry.second);
            const geometry_msgs::Twist& previous_twist = std::get<1>(previous_dynamic_objects[model_name]);

            // 计算加速度
            geometry_msgs::Twist person_acc;
            person_acc.linear.x = (current_twist.linear.x - previous_twist.linear.x) / 0.03; // 计算 x 方向加速度
            person_acc.linear.y = (current_twist.linear.y - previous_twist.linear.y) / 0.03; // 计算 y 方向加速度
            // std::cout<<"person_acc.linear.x: "<<person_acc.linear.x<<"     "<<"person_acc.linear.y: "<<person_acc.linear.y<<std::endl;
            if( std::abs(person_acc.linear.x) >= 1.6 || std::abs(person_acc.linear.y) >= 1.6){
            person_acc.linear.x = 0;
            person_acc.linear.y = 0;
            }
            person_acc.linear.z = 0.0; // 假设 z 方向加速度为 0

            // 更新动态对象，包含加速度
            dynamic_objects[model_name] = std::make_tuple(std::get<0>(entry.second), current_twist, person_acc);
        }

        // 处理预测数据
        dealwithPredictedData();    
        vpre_counter = 0; // 重置计数器
    }
        // 保存当前状态为上一时刻状态
    previous_dynamic_objects = dynamic_objects; // 更新为当前状态
    // dealwithPredictedData();    
}


std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> dsvplanner_ns::Dra::KFpredictTrajectories()
{
 std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

    // For each pedestrian, we need to build a new Kalman filter
    int n = 6; // Number of states (x, y, vx, vy, ax, ay)
    int m = 2; // Number of states we measure (x, y)

    double dt = time_step;

    // Matrices for the Kalman filter
    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // Constant Acceleration Model
    A << 1, 0, dt, 0, 0.5 * dt * dt, 0,
         0, 1, 0, dt, 0, 0.5 * dt * dt,
         0, 0, 1, 0, dt, 0,
         0, 0, 0, 1, 0, dt,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    C << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0;

    // Reasonable covariance matrices
    Q << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    R << 0.01, 0,
         0, 0.01;

    P << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    Eigen::VectorXd y(m);
    Eigen::VectorXd x0(n);
    Eigen::MatrixXd P0(n, n);
    double t = 0;
    int safety_iteration = 1; // Bounding covariance (1 second)

    for (const auto& dynamic_obstacle : dynamic_objects) {
        const std::string& key = dynamic_obstacle.first;
        const geometry_msgs::Pose& pose =  std::get<0>(dynamic_obstacle.second);; // 第一个元素
        const geometry_msgs::Twist& twist = std::get<1>(dynamic_obstacle.second); // 第二个元素
        const geometry_msgs::Twist& acc = std::get<2>(dynamic_obstacle.second); // 第三个元素

        // Create a new Kalman filter for each dynamic obstacle
        KalmanFilter kf(dt, A, C, Q, R, P);
        std::vector<double> xcoords{};
        std::vector<double> ycoords{};
        std::vector<double> zcoords{};
        std::vector<Eigen::MatrixXd> covariance_matrices(KFiterations);

        // Initialize with the last measurement
        double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
        double vx = twist.linear.x, vy = twist.linear.y;
        double ax = acc.linear.x, ay = acc.linear.y;
     
        x0 << xcoord, ycoord, vx, vy, ax, ay;

        // Initial covariance
        P0 << 1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1;

        kf.init(t, x0, P0);

        // Perform one round of the Kalman filter
        y << xcoord, ycoord;
        kf.predict();
        kf.update(y);

        // Create the trajectory for the current dynamic obstacle
        Eigen::MatrixXd safety_margin(n, n);

        for (int i = 0; i < KFiterations; i++)
        {
            kf.predict();
            xcoords.push_back(kf.state()(0));
            ycoords.push_back(kf.state()(1));
            zcoords.push_back(zcoord);
            if (i == safety_iteration)
            {
                // Because of controllability we can only extract one of these
                // ellipses. See paper for explanation.
                safety_margin = kf.covariance();
            }
        }

        std::fill(covariance_matrices.begin(), covariance_matrices.end(), safety_margin);
        kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices));
        // Update previous velocities for the next iteration
    }
    return kf_data;
}


void dsvplanner_ns::Dra::calculateAcceleration(double vx_prev, double vy_prev, double vx_curr, double vy_curr, double dt, double& ax, double& ay) {
    if (dt > 0) { // 确保时间间隔大于零
        ax = (vx_curr - vx_prev) / dt; // 计算 x 方向的加速度
        ay = (vy_curr - vy_prev) / dt; // 计算 y 方向的加速度
    } else {
        ax = 0.0; // 如果 dt <= 0，设置加速度为 0
        ay = 0.0;
    }
}

std::vector<std::tuple<double, double, Eigen::MatrixXd>> dsvplanner_ns::Dra::createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices)
{
  
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> ellipses{};

  for(auto const& cov_matrix : cov_matrices)
  {
    //Extract the part concerning x,y
    Eigen::MatrixXd cov_matrix_xy = cov_matrix.block<2,2>(0,0);

    // Compute the eigenvalues and eigenvectors of the covariance matrix
    //使用Eigen库的SelfAdjointEigenSolver类计算协方差矩阵的特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov_matrix_xy);
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

    // Compute the length of the major and minor axes of the ellipse
    //计算椭圆的长轴和短轴长度
    double major_length = std::sqrt(std::max(eigenvalues(0), eigenvalues(1)));
    double minor_length = std::sqrt(std::min(eigenvalues(0), eigenvalues(1)));
    // 将长轴长度、短轴长度和特征向量组成一个元组，并添加到椭圆列表中

    ellipses.push_back(std::make_tuple(major_length, minor_length, eigenvectors));
  }
  return ellipses;
}

void dsvplanner_ns::Dra::dealwithPredictedData()
{
      auto new_vec = KFpredictTrajectories();
  { 
    // Race condition
    std::lock_guard<std::mutex> lock(vecMutex);
    predicted_data = new_vec;
  }

  trajectories.clear();
  all_ellipses.clear();

  // Separate the covariance ellipses and the trajectories.
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  visualizePrediction(pred_marker_pub, dynamic_risk_area_pub, trajectories, all_ellipses);
}


//show three colors in the same time
// void dsvplanner_ns::Dra::visualizePrediction(ros::Publisher& line_marker_pub, 
//                          ros::Publisher& ellipse_marker_pub, 
//                          const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
//                          const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses
//                         ) 
// {
//     visualization_msgs::MarkerArray line_marker_array;
//     visualization_msgs::MarkerArray ellipse_marker_array;
//     int line_id = 0; // Line Marker ID counter
//     int ellipse_id = 0; // Ellipse Marker ID counter
//     int N_persons = all_ellipses.size();

//     for (int i = 0; i < N_persons; i++){

//       // Visualize trajectory
//       visualization_msgs::Marker line_marker;
//       line_marker.header.frame_id = "map";
//       line_marker.header.stamp = ros::Time::now();
//       line_marker.ns = "predicted_position";
//       line_marker.action = visualization_msgs::Marker::ADD;
//       line_marker.type = visualization_msgs::Marker::LINE_STRIP;
//       line_marker.id = line_id++;
//       line_marker.scale.x = 0.1;  
//       line_marker.color.r = 0.5;  
//       line_marker.color.g = 0.4;  
//       line_marker.color.b = 0.4;  
//       line_marker.color.a = 1.0; 
//       line_marker.pose.orientation.x = 0.0;
//       line_marker.pose.orientation.y = 0.0;
//       line_marker.pose.orientation.z = 0.0;
//       line_marker.pose.orientation.w = 1.0;

//       // Extract trajectory and ellipses for a certain dynamic obstacle (person)
//       const std::vector<double>& xCoords = std::get<0>(trajectories[i]);
//       const std::vector<double>& yCoords = std::get<1>(trajectories[i]);
//       const std::vector<double>& zCoords = std::get<2>(trajectories[i]);
      
//       const std::vector<std::tuple<double, double, Eigen::MatrixXd>>& ellipses = all_ellipses[i]; //List of tuples

//       int N_trajectory_steps = xCoords.size();
      
//       for (int j = 0; j < N_trajectory_steps; ++j) {
         
//           // Visualize trajectory by filling a line with points
//           geometry_msgs::Point point;
//           point.x = xCoords[j];
//           point.y = yCoords[j];
//           point.z = zCoords[j] + (dynamic_height/2);
//           line_marker.points.push_back(point);

//           //Extract the values to build the ellipse
//           double major_length = std::get<0>(ellipses[j]);
//           double minor_length = std::get<1>(ellipses[j]);
//           Eigen::MatrixXd eigenvectors = std::get<2>(ellipses[j]);

// // Draw three ellipses with equal spacing
//         for (int k = 0; k < 3; ++k) {
//             double scale_factor = 1.0 - (k * 0.2); // Adjust 0.2 to change spacing
//             visualization_msgs::Marker ellipse_marker;
//             ellipse_marker.header.frame_id = "map";
//             ellipse_marker.header.stamp = ros::Time::now();
//             ellipse_marker.type = visualization_msgs::Marker::SPHERE;
//             ellipse_marker.pose.position.x = xCoords[j] ;
//             ellipse_marker.pose.position.y = yCoords[j] ;
//             if (k == 0) {
//                 ellipse_marker.pose.position.z = zCoords[j] + 0.2; // 红色底层
//             } else if (k == 1) {
//                 ellipse_marker.pose.position.z = zCoords[j] + 0.3; // 绿色中间层
//             } else if (k == 2) {
//                 ellipse_marker.pose.position.z = zCoords[j] + 0.4; // 蓝色顶层
//             }
//             ellipse_marker.id = ellipse_id++;
//             // Set the color for each ellipse
//             ellipse_marker.color.a = 1.0; // Alpha channel for opacity
//             if (k == 0) {
//                 ellipse_marker.color.r = 0.0; // green
//                 ellipse_marker.color.g = 1.0;
//                 ellipse_marker.color.b = 0.0;
//             } else if (k == 1) {
//                 ellipse_marker.color.r = 0.0; // blue
//                 ellipse_marker.color.g = 0.0;
//                 ellipse_marker.color.b = 1.0;
//             } else if (k == 2) {
//                 ellipse_marker.color.r = 1.0; // red
//                 ellipse_marker.color.g = 0.0;
//                 ellipse_marker.color.b = 0.0;
//             }
//             // Calculate scale for each ellipse
//             ellipse_marker.scale.x = 2.0 * major_length * scale_factor;
//             ellipse_marker.scale.y = 2.0 * minor_length * scale_factor;
//             ellipse_marker.scale.z = 0.1;

//             // Compute orientation quaternion of covariance ellipse
//             Eigen::Quaterniond q(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d(eigenvectors(0), eigenvectors(1), 0.0)));
//             q.x() = 0;
//             q.y() = 0;
//             q.normalize();
//             ellipse_marker.pose.orientation.x = q.x();
//             ellipse_marker.pose.orientation.y = q.y();
//             ellipse_marker.pose.orientation.z = q.z();
//             ellipse_marker.pose.orientation.w = q.w();

//             ellipse_marker_array.markers.push_back(ellipse_marker);
//         }
          
//       }
//       line_marker_array.markers.push_back(line_marker);    
      
//       }      
//     // Publish the marker arrays (all dynamic obstacles)
//     line_marker_pub.publish(line_marker_array);
//     ellipse_marker_pub.publish(ellipse_marker_array);
// }


void dsvplanner_ns::Dra::visualizePrediction(ros::Publisher& line_marker_pub, 
                         ros::Publisher& ellipse_marker_pub, 
                         const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                         const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses
                        ) 
{
    visualization_msgs::MarkerArray line_marker_array;
    ellipse_marker_array.markers.clear(); // 清空所有标记

    int line_id = 0; // Line Marker ID counter
    int ellipse_id = 0; // Ellipse Marker ID counter
    int N_persons = all_ellipses.size();

    for (int i = 0; i < N_persons; i++){

      // Visualize trajectory
      visualization_msgs::Marker line_marker;
      line_marker.header.frame_id = "map";
      line_marker.header.stamp = ros::Time::now();
      line_marker.ns = "predicted_position";
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.type = visualization_msgs::Marker::LINE_STRIP;
      line_marker.id = line_id++;
      line_marker.scale.x = 0.1;  
      line_marker.color.r = 0.5;  
      line_marker.color.g = 0.4;  
      line_marker.color.b = 0.4;  
      line_marker.color.a = 1.0; 
      line_marker.pose.orientation.x = 0.0;
      line_marker.pose.orientation.y = 0.0;
      line_marker.pose.orientation.z = 0.0;
      line_marker.pose.orientation.w = 1.0;

      // Extract trajectory and ellipses for a certain dynamic obstacle (person)
      const std::vector<double>& xCoords = std::get<0>(trajectories[i]);
      const std::vector<double>& yCoords = std::get<1>(trajectories[i]);
      const std::vector<double>& zCoords = std::get<2>(trajectories[i]);
      
      const std::vector<std::tuple<double, double, Eigen::MatrixXd>>& ellipses = all_ellipses[i]; //List of tuples

      int N_trajectory_steps = xCoords.size();
      
      for (int j = 0; j < N_trajectory_steps; ++j) {
         
          // Visualize trajectory by filling a line with points
          geometry_msgs::Point point;
          point.x = xCoords[j];
          point.y = yCoords[j];
          point.z = zCoords[j] + (dynamic_height/2);
          line_marker.points.push_back(point);

          //Extract the values to build the ellipse
          double major_length = std::get<0>(ellipses[j]);
          double minor_length = std::get<1>(ellipses[j]);
          Eigen::MatrixXd eigenvectors = std::get<2>(ellipses[j]);

// Draw three ellipses with equal spacing

            ellipse_marker.header.frame_id = "map";
            ellipse_marker.header.stamp = ros::Time::now();
            ellipse_marker.type = visualization_msgs::Marker::SPHERE;
            ellipse_marker.pose.position.x = xCoords[j] ;
            ellipse_marker.pose.position.y = yCoords[j] ;
            ellipse_marker.id = ellipse_id++;
            // Set the color for each ellipse
            ellipse_marker.color.a = 1.0; // Alpha channel for opacity
            ellipse_marker.color.r = 0.0; // green
            ellipse_marker.color.g = 1.0;
            ellipse_marker.color.b = 0.0;
            // Calculate scale for each ellipse
            ellipse_marker.scale.x = 1 * major_length  ;
            ellipse_marker.scale.y = 1 * minor_length  ;
            ellipse_marker.scale.z = 0.1;
            // Compute orientation quaternion of covariance ellipse
            Eigen::Quaterniond q(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d(eigenvectors(0), eigenvectors(1), 0.0)));
            q.x() = 0;
            q.y() = 0;
            q.normalize();
            ellipse_marker.pose.orientation.x = q.x();
            ellipse_marker.pose.orientation.y = q.y();
            ellipse_marker.pose.orientation.z = q.z();
            ellipse_marker.pose.orientation.w = q.w();

            ellipse_marker_array.markers.push_back(ellipse_marker);
            
        
          
      }
      line_marker_array.markers.push_back(line_marker);    
      
      }      
    // Publish the marker arrays (all dynamic obstacles)
    line_marker_pub.publish(line_marker_array);
    ellipse_marker_pub.publish(ellipse_marker_array);
}


visualization_msgs::MarkerArray dsvplanner_ns::Dra::getEllipseMarkerArray() {
    return ellipse_marker_array; // 返回成员变量
}


void dsvplanner_ns::Dra::Visualize_Dynamic_Pose(ros::Publisher& marker_pub, const int& object_id, const geometry_msgs::Pose& pose) {
   
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // 根据需要设置帧 ID
        marker.header.stamp = ros::Time::now();
        marker.ns = "dynamic_objects";
        marker.id = object_id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        //ROS_INFO("Dynamic object pose: x=%.2f, y=%.2f, z=%.2f", pose.position.x, pose.position.y, pose.position.z);
        marker.pose.position.z = pose.position.z + dynamic_height/2; // 圆柱体底部高度
        marker.scale.x = dynamic_long; // 圆柱体直径
        marker.scale.y = dynamic_long;
        marker.scale.z = dynamic_height; // 圆柱体高度
        marker.color.r = 1.0; // 红色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // 不透明

        marker_pub.publish(marker);
       // ROS_INFO("Dynamic obstacle marker published.");
}


bool dsvplanner_ns::Dra::inDynamicRiskArea(StateVec node)
{
  //  ROS_INFO("Start Check Collision");
  for(const auto& ellipse_marker : ellipse_marker_array.markers){
  // 提取椭圆的中心和半径
  geometry_msgs::Point center = ellipse_marker.pose.position;
  double radius_x = ellipse_marker.scale.x / 2.0; // 假设 scale.x 是椭圆的长轴半径
  double radius_y = ellipse_marker.scale.y / 2.0; // 假设 scale.y 是椭圆的短轴半径

  // 计算节点到椭圆中心的距离
  double distance_x = std::abs(node[0] - center.x);
  double distance_y = std::abs(node[1] - center.y);

  // 使用椭圆的标准方程来检查节点是否在椭圆内
  // 如果节点到椭圆中心的距离的平方小于等于椭圆半径的平方，则节点在椭圆内
  if ((distance_x * distance_x) / (radius_x * radius_x) + (distance_y * distance_y) / (radius_y * radius_y) <= 1.0)
  {
    return true; // 节点在椭圆内
  }
  }
    return false; // 节点不在椭圆内
    
  }

bool dsvplanner_ns::Dra::checkCollision(double time, Eigen::Vector3d point){

    double max_time_step = KFiterations * time_step;
    int N_persons = trajectories.size();
    // Index the correct circle
    int covariance_index = getCovarianceIndex(max_time_step, time_step, time);

    if(covariance_index == -1)
    { 
      // Outside our prediction
      return false;
    }
    for(int i = 0; i < N_persons; i++)
    { 
      
      auto personEllipses = all_ellipses[i];
      auto personTrajectory = trajectories[i];
      
      //Extract the correct mean and ellipse
      std::tuple<double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
      
      double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
      double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
      double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];

      double person_current_x = std::get<0>(personTrajectory)[0];
      double person_current_y = std::get<1>(personTrajectory)[0];

      //if(sqrt(pow(person_current_x-point[0], 2.0) + pow(person_current_y-point[1], 2.0)) < params_.KFiterations * params_.time_step * params_.human_linear_velocity)
      //{
        // Check if person is close enough
        const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);
        
        if(isCircleInsideEllipse(point.head(3), center, covarianceEllipse)) {
          return true;
        }
      //}
   }
     return false; 
}


/*
* Gets the correct covariance ellipse from a prediction given time t.
*/
int dsvplanner_ns::Dra::getCovarianceIndex(double max_time_step, double time_step, double t){
  
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


/*
* Check intersection between a circle around a goal and the predicted covariance ellipse (currently implemented for a circle).
*/
bool dsvplanner_ns::Dra::isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
            std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse) 
{ 
  /*
  // Radius around goal
  double radius = 0.2;

  // Extract major and minor axis lengths and eigenvectors
  double major_axis_length = std::get<0>(covariance_ellipse);
  double minor_axis_length = std::get<1>(covariance_ellipse);
  Eigen::MatrixXd eigenvectors = std::get<2>(covariance_ellipse);
  
  // Calculate relative coordinates of the point with respect to the ellipse center (XY)
  Eigen::Vector2d relative_point = point.head(2) - center.head(2);

  // Rotate the relative coordinates by the angle of the major axis
  Eigen::Vector2d rotated_relative_point = eigenvectors.transpose() * relative_point;

  // Normalize the rotated relative coordinates
  double normalized_x = rotated_relative_point(0) / major_axis_length;
  double normalized_y = rotated_relative_point(1) / minor_axis_length;

  // Check if the normalized coordinates are inside the unit circle
  double distance_from_origin = normalized_x * normalized_x + normalized_y * normalized_y;
  
  // Calculate the normalized radius of the circle
  double normalized_radius = radius / std::max(major_axis_length, minor_axis_length);

  // Check if the circle intersects with the ellipse
  bool does_intersect_XY = distance_from_origin <= 1.0 + normalized_radius;
  // Check if the sphere intersects with the cylinder
  bool does_intersect_XYZ = does_intersect_XY and point(2) < center(2) + params_.human_height + (2*params_.drone_height);
  return does_intersect_XYZ;
  */
  double circle_radius = std::get<0>(covariance_ellipse);
  // Calculate the distance between the centers of the two circles
  Eigen::Vector2d point_XY = point.head(2);
  double distance = (point_XY - center.head(2)).norm();

  // Check if the two circles intersect
  bool does_intersect_XY = distance <= 0.2 + circle_radius;
  bool does_intersect_XYZ = does_intersect_XY and point(2) < center(2) + dynamic_height + (2*drone_height);

  return does_intersect_XYZ;

}
/**
 * Visualize the ghost pedestrian that will block our view in the future.
*/

void dsvplanner_ns::Dra::visualizeGhostPedestrian( std::vector<std::tuple<double, double, double>> positions) 
{
    
    visualization_msgs::MarkerArray ghosts;
    int id = 0;
    for(auto const& pos : positions)
    {
      // Create visualization marker
      visualization_msgs::Marker ghost;
      ghost.header.frame_id = "map";  
      ghost.header.stamp = ros::Time::now();
      ghost.ns = "dynamic_objects";
      ghost.id = id++;
      ghost.type = visualization_msgs::Marker::CUBE;
      ghost.action = visualization_msgs::Marker::ADD;
      ghost.pose.position.x = std::get<0>(pos);
      ghost.pose.position.y = std::get<1>(pos);
      ghost.pose.position.z = std::get<2>(pos)  + (dynamic_height/2);
      ghost.scale.x = dynamic_width;  
      ghost.scale.y = dynamic_width;  
      ghost.scale.z = dynamic_height; 

      //GREY
      ghost.color.r = 0.769;  
      ghost.color.g = 0.769;  
      ghost.color.b = 0.769;
      ghost.color.a = 1.0;

      ghost.pose.orientation.x = 0.0;
      ghost.pose.orientation.y = 0.0;
      ghost.pose.orientation.z = 0.0;
      ghost.pose.orientation.w = 1.0;

      ghosts.markers.push_back(ghost);
    }
    
    // Publish the marker
    ghost_marker_pub.publish(ghosts);
}


bool dsvplanner_ns::Dra::willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts)
{
  // Race condition
  std::lock_guard<std::mutex> lock(vecMutex);

  if(visualize_ghosts)
  {
    std::vector<std::tuple<double, double, double>> positions;
    for(auto const& person : predicted_data)
    { 
      double x = std::get<0>(person.first)[index];
      double y = std::get<1>(person.first)[index];
      double z = std::get<2>(person.first)[index];
      std::tuple<double, double, double> position = std::make_tuple(x,y,z);
      positions.push_back(position);
    }
    visualizeGhostPedestrian(positions);
  }
  
  for(auto const& person : predicted_data)
  { 
    double x = std::get<0>(person.first)[index];
    double y = std::get<1>(person.first)[index];
    double z = std::get<2>(person.first)[index];
    if(isCollisionWithBoundingBox(point, x, y, z))
    {
      return true;
    }
    
  }
  return false;
}


bool dsvplanner_ns::Dra::isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z)
{
  return point[0] >= (x - dynamic_width/2) and point[0] <= (x + dynamic_width/2) and
         point[1] >= (y - dynamic_width/2) and point[1] <= (y + dynamic_width/2) and
         point[2] >= z and point[2] <= (z + dynamic_height);
}

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "dynamic_obstacle_detector");
//     dsvplanner_ns::Dra  detector;
//     ros::spin();
//     return 0;
// }
