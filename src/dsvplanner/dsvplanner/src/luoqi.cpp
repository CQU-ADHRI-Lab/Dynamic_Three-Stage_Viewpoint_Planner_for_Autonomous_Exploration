void groundextract(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle , hight,dxy;
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){
                if (rangeMat.at<float>(i,j) == FLT_MAX) continue;
                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN; 
                hight = fullCloud->points[lowerInd].z;
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;
                // dxy = sqrt(diffX*diffX + diffY*diffY);
                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                // 根据传感器高度设置
                // if (abs(angle)<= 5 && diffZ<0.15 && hight<-1.7 && hight > -1.9 && dxy>0.5)
                if (abs(angle)<= 5 && diffZ<0.15 && hight<-1.7 && hight > -1.9){
                    // if(pointDistance(fullCloud->points[lowerInd])<30){
                    //     groundPoint->push_back(fullCloud->points[lowerInd]);
                    // }
                    groundPoint->push_back(fullCloud->points[lowerInd]);
                }
            }
        }
        if(groundPoint->size()<500){
            ROS_INFO("not enouph");
            return;
        } 
        // 先降采样点云
        downSizeFilter.setInputCloud(groundPoint);
        downSizeFilter.filter(*groundPointDS);
        // 将每一帧的地面点云进行拟合剔除离地面过远的点
        // todo 没写地面点少于100时后续操作要跳过
        // 设置平面参数
        float pa = 1;
        float pb = 1;
        float pc = 1;
        float pd = 1;
        int groundNums=groundPointDS->size();
        Eigen::MatrixXf matA0;
        Eigen::MatrixXf matB0;
        matA0.resize(groundNums,3);
        matB0.resize(groundNums,1);
        Eigen::Vector3f matX0;// 系数abc
        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();
        for (int j = 0; j < groundNums; j++) {
            matA0(j, 0) = groundPointDS->points[j].x;
            matA0(j, 1) = groundPointDS->points[j].y;
            matA0(j, 2) = groundPointDS->points[j].z;
        }
        // 假设平面方程为ax+by+cz+1=0，这里就是求方程的系数abc，d=1
        matX0 = matA0.colPivHouseholderQr().solve(matB0);//先QR分解再求解Ax=B
        pa = matX0(0, 0);
        pb = matX0(1, 0);
        pc = matX0(2, 0);
        pd = 1;
        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps; pb /= ps; pc /= ps; pd /= ps;
        // #pragma omp parallel for num_threads(numberOfCores)
        for (size_t i = 0; i < groundNums; i++) {
            // 平面过于离散 原来是大于0.2，为了使平面质量更好我设置为了0.2
            if (fabs(pa * groundPointDS->points[i].x +pb * groundPointDS->points[i].y +pc * groundPointDS->points[i].z + pd) > 0.2) {
                groundPointDS->points[i].x = 0;
                groundPointDS->points[i].y = 0;
                groundPointDS->points[i].z = 0;
            }
            else if (groundPointDS->points[i].intensity>3) {
                // 突出车道线
                groundPointDS->points[i].intensity=100;
            }
        }
        cloudInfo.ground = publishCloud(PubgroundCloud, groundPointDS, cloudHeader.stamp, lidarFrame);
    }

void RemoveDynamicObject(){

        if(groundPose.size()<2) return; 

        float groundlength=0;
        float dx = groundPose.front().x-groundPose.back().x;
        float dy = groundPose.front().x-groundPose.back().x;
        float dz = groundPose.front().x-groundPose.back().x;
        groundlength = sqrt(dx*dx+dy*dy+dz*dz);
        // if not enough ground length
        if(groundForDynamic.size()<70&&groundlength<70) return;
        pcl::PointCloud<PointType>::Ptr allGroundPoint (new pcl::PointCloud<PointType>()); 
        pcl::PointCloud<PointType>::Ptr middlePointCloudTransed(new pcl::PointCloud<PointType>()); 
        pcl::PointCloud<PointType>::Ptr cloudWithOutDynam(new pcl::PointCloud<PointType>()); 
        pcl::PointCloud<PointType>::Ptr pointCloudRemoveDynamic(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr WithOutDynamicForOpt(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr WithOutDynamicForOptDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType2D>::Ptr ground2D(new pcl::PointCloud<PointType2D>());
        

        for(size_t i; i<groundForDynamic.size();i++){
            *allGroundPoint += *transformPointCloud(groundForDynamic[i],&groundPose[i]);
        }
        cout<<"allGroundPoint size is:"<<allGroundPoint->size()<<endl;
        int halfLen = floor(groundForDynamic.size()/2);
        *middlePointCloudTransed = *transformPointCloud(dynamicOri[halfLen],&groundPose[halfLen]);
        *ground2D = *point3Dto2D(allGroundPoint);
        kdtreeGround->setInputCloud(ground2D);
        // 这个要一行一行的判断才能做到判断是否是和地面接触的
        for(size_t i=0; i<middlePointCloudTransed->size();i++){
            // 高于车1米的点都不考虑
            if (middlePointCloudTransed->points[i].z>(groundPose[halfLen].z+2)){
                WithOutDynamicForOpt->points.push_back(dynamicOri[halfLen]->points[i]);
                cloudWithOutDynam->points.push_back(middlePointCloudTransed->points[i]);
            }
            PointType2D point;
            vector<int> index;
            vector<float> distance;
            point.x = middlePointCloudTransed->points[i].x;
            point.y = middlePointCloudTransed->points[i].y;
            // 找临近的10个点，与自身相减作和要接近0
            kdtreeGround->nearestKSearch(point,10,index,distance);
            // 如果投影上去和地面点有重合，则认为是动态物体，如果没有则认为不是动态物体。
            if(distance[0]<0.3) pointCloudRemoveDynamic->push_back(middlePointCloudTransed->points[i]);
            else {
                cloudWithOutDynam->points.push_back(middlePointCloudTransed->points[i]);
                WithOutDynamicForOpt->points.push_back(dynamicOri[halfLen]->points[i]);
            }
        } 
        // dynamicOri[halfLen] = cloudWithOutDynam;
        downSizeFilterICP.setInputCloud(WithOutDynamicForOpt);
        downSizeFilterICP.filter(*WithOutDynamicForOptDS);
        if(groundPose[halfLen].intensity){
            cout<<"one frame saved"<<endl;
            withOutDynamicCloud.push_back(WithOutDynamicForOptDS);
        }
        if(withOutDynamicCloud.size()==1){
            dynamicStartindex = groundPose[halfLen].intensity;
        }
        publishCloud(pubDynamicCloud,pointCloudRemoveDynamic,timeLaserInfoStamp,odometryFrame);
        publishCloud(pubWithoutDynamicCloud,cloudWithOutDynam,timeLaserInfoStamp,odometryFrame);
        // 清理
        groundForDynamic.pop_front();
        dynamicOri.pop_front();
        groundPose.pop_front();
    }


     // Draw three ellipses with equal spacing
        for (int k = 0; k < 3; ++k) {
            visualization_msgs::Marker ellipse_marker;
            ellipse_marker.header.frame_id = "map";
            ellipse_marker.header.stamp = ros::Time::now();
            ellipse_marker.type = visualization_msgs::Marker::SPHERE;
            ellipse_marker.pose.position.x = xCoords[j];
            ellipse_marker.pose.position.y = yCoords[j];
            ellipse_marker.pose.position.z = zCoords[j] + 0.2;
            if (k == 3)
            {
                ellipse_marker.id = ellipse_id++;
            }

            // Set the color for each ellipse
            ellipse_marker.color.a = 1.0; // Alpha channel for opacity
            if (k == 0) {
                ellipse_marker.color.r = 1.0; // 红色
                ellipse_marker.color.g = 0.0;
                ellipse_marker.color.b = 0.0;
            } else if (k == 1) {
                ellipse_marker.color.r = 0.0; // 绿色
                ellipse_marker.color.g = 1.0;
                ellipse_marker.color.b = 0.0;
            } else if (k == 2) {
                ellipse_marker.color.r = 0.0; // 蓝色
                ellipse_marker.color.g = 0.0;
                ellipse_marker.color.b = 1.0;
            }
            // Calculate scale for each ellipse
            double scale_factor = 1.0 - (k * 0.2); // Adjust 0.2 to change spacing
            ellipse_marker.scale.x = 2.0 * major_length * scale_factor;
            ellipse_marker.scale.y = 2.0 * minor_length * scale_factor;
            ellipse_marker.scale.z = 0.1;

            // Compute orientation quaternion of covariance ellipse
            Eigen::Quaterniond q(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d(eigenvectors(0), eigenvectors(1), 0.0)));
            ellipse_marker.pose.orientation.x = q.x();
            ellipse_marker.pose.orientation.y = q.y();
            ellipse_marker.pose.orientation.z = q.z();
            ellipse_marker.pose.orientation.w = q.w();

            ellipse_marker_array.markers.push_back(ellipse_marker);
        }