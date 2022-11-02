#include <pycoral_ros/add_detection.h>
#include "pycoral_ros/DetectCompressedImage.h"

void AddDetection::init(ros::NodeHandle& nh) {

  nh.param<std::string>("camera_topic", camera_topic, "camera_topic");
  nh.param<std::string>("info_topic", info_topic, "info_topic");
  nh.param<std::string>("cloud_topic", cloud_topic, "cloud_topic");
  nh.param<std::string>("cloud_out_topic", cloud_out_topic, "cloud_out_topic");
  nh.param<std::string>("image_out_topic", image_out_topic, "image_out_topic");
  nh.param<std::string>("camera_frame", camera_frame, "camera_frame");
  nh.param<std::string>("cloud_frame", cloud_frame, "cloud_frame");
  nh.param<int>("num_labels", num_labels, 7);
  nh.param<int>("drone_idx", drone_idx, 0);

  for (int i=0; i<num_labels; i++){
    int max_objects_temp, min_objects_temp, min_points_temp;
    float tolerance_temp, min_intensity_temp;

    nh.param<int>("max_objects_" + std::to_string(i), max_objects_temp, 9999);
    nh.param<int>("min_objects_" + std::to_string(i), min_objects_temp, 0);
    nh.param<int>("min_points_" + std::to_string(i), min_points_temp, 3);
    nh.param<float>("cluster_tolerance_" + std::to_string(i), tolerance_temp, 1);
    nh.param<float>("min_intensity_" + std::to_string(i), min_intensity_temp, 0.3);
    
    max_objects.push_back(max_objects_temp);
    min_objects.push_back(min_objects_temp);
    min_points.push_back(min_points_temp);
    tolerances.push_back(tolerance_temp);
    min_intensities.push_back(min_intensity_temp);
 
  }

  std::cout<<"PARAMETERS : "<< std::endl;
  std::cout<<"camera_topic : "<< camera_topic << std::endl;
  std::cout<<"info_topic : "<< info_topic << std::endl;
  std::cout<<"cloud_topic : "<< cloud_topic << std::endl;
  std::cout<<"cloud_out_topic : "<< cloud_out_topic << std::endl;
  std::cout<<"image_out_topic : "<< image_out_topic << std::endl;
  std::cout<<"camera_frame : "<< camera_frame << std::endl;
  std::cout<<"cloud_frame : "<< cloud_frame << std::endl;
  std::cout<<"num_labels : "<< num_labels << std::endl;

  AddDetection::set_camera_model();

  cam_info_subscriber = nh.subscribe(info_topic, 1, &AddDetection::camera_info_callback, this);
  odom_subscriber = nh.subscribe("/odom", 1, &AddDetection::odom_callback, this);
  mission_subscriber = nh.subscribe("/scout/GoalAction", 1, &AddDetection::mission_callback, this);


  cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_out_topic, 1);
  image_pub = nh.advertise<sensor_msgs::Image>(image_out_topic, 1 );

  detection_client = nh.serviceClient<pycoral_ros::DetectCompressedImage>("/detect_compressed_image");

  detection_pub = nh.advertise<vision_msgs::Detection3DArray>("/detection_3d", 1 );
  detection_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/detection_3d_pub", 1 );

  for (int i =0; i<num_labels; i++){
    dets_pub.push_back(nh.advertise<sensor_msgs::PointCloud2>("/det_cloud_" + std::to_string(i), 1 ));
    dets_filtered_pub.push_back(nh.advertise<sensor_msgs::PointCloud2>("/det_cloud_filtered_" + std::to_string(i), 1 ));
    dets_loc_pub.push_back(nh.advertise<sensor_msgs::PointCloud2>("/det_loc_" + std::to_string(i), 1 ));
    
    dets_cloud.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>));
    dets_cloud_filtered.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>));
    dets_cloud_clustered.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>));
    // dets_pub.at(i) = nh.advertise<sensor_msgs::PointCloud2>("/det_cloud_" + std::to_string(i), 1 );
    // dets_cloud.at(i) = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }

  det_pub_timer = nh.createTimer(ros::Duration(1.0), &AddDetection::det_pub_callback, this);

  // CAMERA LIDAR CALIBRATION PARAMETERS
  if (drone_idx == 0){ //ADD DRONE
    T_baselink2camera.transform.translation.x = -0.00706296; // TRANSLATION
    T_baselink2camera.transform.translation.y = -0.11697807;
    T_baselink2camera.transform.translation.z = -0.24420192;
    T_baselink2camera_q.setRPY(0.52218648, -1.52973857,  0.97920602); // ROTATION
    T_baselink2camera_q.normalize();
    T_baselink2camera.transform.rotation.x = T_baselink2camera_q.getX();
    T_baselink2camera.transform.rotation.y = T_baselink2camera_q.getY();
    T_baselink2camera.transform.rotation.z = T_baselink2camera_q.getZ();
    T_baselink2camera.transform.rotation.w = T_baselink2camera_q.getW();
  }
  else{ // DRONEBOT DRONE
    T_baselink2camera.transform.translation.x = 0.07236185; // TRANSLATION
    T_baselink2camera.transform.translation.y = -0.1511707;
    T_baselink2camera.transform.translation.z = -0.2269421;
    T_baselink2camera_q.setRPY(-2.32750747, -1.50213443186,  -2.453036553857); // ROTATION
    T_baselink2camera_q.normalize();
    T_baselink2camera.transform.rotation.x = T_baselink2camera_q.getX();
    T_baselink2camera.transform.rotation.y = T_baselink2camera_q.getY();
    T_baselink2camera.transform.rotation.z = T_baselink2camera_q.getZ();
    T_baselink2camera.transform.rotation.w = T_baselink2camera_q.getW();
  }



  // Make the inverse transform
  T_camera2baselink = tf2::inverse(T_baselink2camera);

  // std::cout<<T_baselink2camera.transform.translation.x<<" "<<T_baselink2camera.transform.translation.y<<" "<<T_baselink2camera.transform.translation.z<<std::endl;
  // std::cout<<T_baselink2camera.transform.rotation.x<<" "<<T_baselink2camera.transform.rotation.y<<" "<<T_baselink2camera.transform.rotation.z<<std::endl;

  // T_camera2baselink.transform = T_baselink2camera.


  image_sub_.reset(new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh, camera_topic, 1));
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, cloud_topic, 1));
  
  sync_image_cloud_.reset(new message_filters::Synchronizer<SyncPolicyImageCloud>(
    SyncPolicyImageCloud(100), *image_sub_, *cloud_sub_
  ));
  
  sync_image_cloud_->registerCallback(boost::bind(&AddDetection::image_cloud_callback, this, _1, _2));

  // while (!tf_received){
  //   ROS_INFO("Waiting for transform between %s to %s...", cloud_frame.c_str(), camera_frame.c_str());

  //   try{
  //     T_baselink2camera = tf_buffer_.lookupTransform(camera_frame, cloud_frame, ros::Time::now());
  //     T_camera2baselink = tf_buffer_.lookupTransform(cloud_frame, camera_frame, ros::Time::now());
  //     tf_received = true;

  //   }
  //   catch (tf2::TransformException ex){
  //     ROS_INFO("%s",ex.what());
  //   }
  //   ros::Duration(0.5).sleep();
  // }
  // ROS_INFO("Tranformation is received!...");

  // std::cout<<T_baselink2camera.transform.translation.x<<" "<<T_baselink2camera.transform.translation.y<<" "<<T_baselink2camera.transform.translation.z<<std::endl;
  // std::cout<<T_baselink2camera.transform.rotation.x<<" "<<T_baselink2camera.transform.rotation.y<<" "<<T_baselink2camera.transform.rotation.z<<std::endl;
}

void AddDetection::visualize_detection(
  vision_msgs::Detection3DArray dets
){
  visualization_msgs::MarkerArray ms;
  
  for (unsigned i=0; i<dets.detections.size(); i++){

    visualization_msgs::Marker m;
    m.header.frame_id = "base_link";
    m.pose.position = dets.detections[i].results[0].pose.pose.position;
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.scale.x = 0.4;
    m.scale.y = 0.4;
    m.scale.z = 0.4;

    m.color.a = 1;
    m.color.r = (rand() % 255)/255.0;
    m.color.g = (rand() % 255)/255.0;
    m.color.b = (rand() % 255)/255.0;

    m.type = 2;
    m.id = i;
    m.action = 3;

    m.text = std::to_string(dets.detections[i].results[0].id);
    ms.markers.push_back(m);
  }

  detection_vis_pub.publish(ms);
}

void AddDetection::mission_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    det_flag = msg->data[10];
}

void AddDetection::image_cloud_callback(
  const sensor_msgs::CompressedImageConstPtr& image_msg, 
  const sensor_msgs::PointCloud2ConstPtr& cloud_msg
)
  {
    // Solve all of perception here...
    if (!det_flag){
      ROS_INFO("Detection is disabled for this moment...");
    }
    else if (!info_available){
      ROS_INFO("Camera info is not available, waiting...");
    }
    else if (!odom_received){
      ROS_INFO("Waiting for odometry topic....");
    }
    else {
      cv_bridge::CvImagePtr img_ptr;
      try
      {
        img_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        // cv::cvtColor(img_ptr->image, img_ptr->image, CV_BGR2RGB);
        // cv::GaussianBlur(img_ptr->image, img_ptr->image, cv::Size(13,13), 0);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      sensor_msgs::PointCloud2 cloud_msg_out;
      tf2::doTransform(*cloud_msg, cloud_msg_out, T_baselink2camera);
      // tf2::doTransform()

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(cloud_msg_out, *cloud);


      // Filtering the cloud points
      std::vector<Eigen::Vector2d> cam_points;
      std::vector<Eigen::Vector3d> cloud_points;

      for (unsigned int i = 0; i < cloud->points.size(); i++){
        cv::Point3d cloud_point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        cv::Point2d cam_point = cam_model.project3dToPixel(cloud_point);

        if (
          cam_point.x >= 0 &&
          cam_point.x <= img_ptr->image.cols &&
          cam_point.y >= 0 &&
          cam_point.y <= img_ptr->image.rows &&
          cloud->points[i].z > 0
        ){
          cam_points.push_back(Eigen::Vector2d(cam_point.x, cam_point.y));
          cloud_points.push_back(Eigen::Vector3d(cloud_point.x, cloud_point.y, cloud_point.z));
        }
      }

      // transform_to_odom = tf_buffer_.lookupTransform(camera_frame, cloud_frame, ros::Time::now());

      // Calling object detection service
      pycoral_ros::DetectCompressedImage img_srv;
      img_srv.request.image = *image_msg;

      if (detection_client.call(img_srv)){
        vision_msgs::Detection3DArray det3ds;

        det3ds.header.frame_id = cloud_msg->header.frame_id;
        Eigen::Matrix3d matRot = Eigen::Quaterniond(
            T_camera2baselink.transform.rotation.w, T_camera2baselink.transform.rotation.x, T_camera2baselink.transform.rotation.y, T_camera2baselink.transform.rotation.z
          ).toRotationMatrix();
        Eigen::Vector3d matTrans(T_camera2baselink.transform.translation.x, T_camera2baselink.transform.translation.y, T_camera2baselink.transform.translation.z);

        // Eigen::Isometry3d T_inverse = tf2::transformToEigen(T_camera2odom);

        for (unsigned int i = 0; i < img_srv.response.detections.detections.size(); i++){
          vision_msgs::Detection2D det2d = img_srv.response.detections.detections[i];
          
          Eigen::Vector2d detection_center(det2d.bbox.center.x, det2d.bbox.center.y);
          Eigen::Vector3d location_3d_cam(999.9, 999.9, 999.9);

          for (unsigned int j = 0; j < cam_points.size(); j++){
            if((cam_points[j]-detection_center).norm() < 25){
              if(cloud_points[j].norm() < location_3d_cam.norm()){
                location_3d_cam = cloud_points[j];
              }
            }
          }

          // If detection distance from camera is bigger than 20 meters, then invalid
          if (location_3d_cam.norm() > 20){
            continue;
          }

          vision_msgs::Detection3D det3d;
          vision_msgs::ObjectHypothesisWithPose hypotesis;
          Eigen::Vector3d location_3d_baselink, location_3d_odom;

          // location_3d_baselink = matRot*location_3d_cam + matTrans;
          tf2::doTransform(location_3d_cam, location_3d_baselink, T_camera2baselink);
          tf2::doTransform(location_3d_baselink, location_3d_odom, T_baselink2odom);

          hypotesis.id = det2d.results[0].id;
          hypotesis.score = det2d.results[0].score;

          hypotesis.pose.pose.position.x = location_3d_baselink[0];
          hypotesis.pose.pose.position.y = location_3d_baselink[1];
          hypotesis.pose.pose.position.z = location_3d_baselink[2];
          
          det3d.results.push_back(hypotesis);
          det3d.bbox.center.position.x = location_3d_baselink[0];
          det3d.bbox.center.position.y = location_3d_baselink[1];
          det3d.bbox.center.position.z = location_3d_baselink[2];

          det3ds.detections.push_back(det3d);

          //TRANSFORM FROM BASELINK TO ODOM FRAME
          AddDetection::append_detection(hypotesis.id, hypotesis.score, location_3d_odom);

          // if (int(hypotesis.id)==1){

          //   AddDetection::append_detection(2, hypotesis.score, location_3d_odom + Eigen::Vector3d(0.2, 0.2, 0));
          // }
        }

        detection_pub.publish(det3ds);
        // AddDetection::visualize_detection(det3ds);

      }
      else{
        ROS_ERROR("Failed to call service detect_image");
        // return 1;
      }

    }
}

void AddDetection::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg){
  if (!info_available){
    cam_model.fromCameraInfo(cam_info_msg);
    info_available = true;
    std::cout<<"Camera model is ready!!!"<<std::endl;
  }
}

void AddDetection::set_camera_model(){
  info_available = true;
  // FROM CAMERA LIDAR CALIBRATION
  sensor_msgs::CameraInfo cam_model_msg;
  cam_model_msg.height = 720;
  cam_model_msg.width = 1280;
  cam_model_msg.distortion_model = "plumb_bob";
  cam_model_msg.D = {0.128931, -0.235657, 0.000888, 0.000933, 0.0};
  cam_model_msg.K = {900.12645, 0.0, 648.28651, 0.0, 902.19837, 359.34368, 0.0, 0.0, 1.0};
  cam_model_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  cam_model_msg.P = {904.94904, 0.0, 650.68234, 0.0, 0.0, 918.67725, 359.80495, 0.0, 0.0, 0.0, 1.0, 0.0};
  cam_model_msg.binning_x = 0;
  cam_model_msg.binning_y = 0;
  cam_model_msg.roi.do_rectify = false;
  cam_model_msg.roi.x_offset = 0;
  cam_model_msg.roi.y_offset = 0;
  cam_model_msg.roi.height = 0;
  cam_model_msg.roi.width = 0;

  cam_model.fromCameraInfo(cam_model_msg);
  std::cout<<"Camera model is ready!!!"<<std::endl;
}

void AddDetection::odom_callback(const nav_msgs::Odometry& odom){
  if (!odom_received){
    odom_received = true;
  }
  geometry_msgs::TransformStamped T_odom2baselink; 
  T_odom2baselink.transform.translation.x = odom.pose.pose.position.x;
  T_odom2baselink.transform.translation.y = odom.pose.pose.position.y;
  T_odom2baselink.transform.translation.z = odom.pose.pose.position.z;
  T_odom2baselink.transform.rotation.x = odom.pose.pose.orientation.x;
  T_odom2baselink.transform.rotation.y = odom.pose.pose.orientation.y;
  T_odom2baselink.transform.rotation.z = odom.pose.pose.orientation.z;
  T_odom2baselink.transform.rotation.w = odom.pose.pose.orientation.w;

  // T_baselink2odom = tf2::inverse(T_odom2baselink);
  T_baselink2odom = T_odom2baselink;
}

void AddDetection::append_detection(
    int id, float score, Eigen::Vector3d pos
){
    pcl::PointXYZI cur_det;
    cur_det.x = pos[0];
    cur_det.y = pos[1];
    cur_det.z = pos[2];
    cur_det.intensity = score;

    if (id >=0 && id<num_labels){
        dets_cloud.at(id)->push_back(cur_det);
        if (id==1){
          dets_cloud.at(2)->push_back(cur_det);
        }
    }

}

void AddDetection::det_pub_callback(const ros::TimerEvent&){
    // RESET THE MARKER ARRAY
    visualization_msgs::MarkerArray ds;
    visualization_msgs::Marker d;
    d.header.frame_id = "odom";
    d.type = 9;
    d.id = 0;
    d.action = 3;
    ds.markers.push_back(d);

    detection_vis_pub.publish(ds);    

    // VARIABLES FOR MARKER
    visualization_msgs::MarkerArray ms;
    int vis_count = 0;

    for (int i =0; i<num_labels; i++){
      AddDetection::cluster_detections(
        dets_cloud.at(i),
        dets_cloud_filtered.at(i),
        dets_cloud_clustered.at(i),
        tolerances.at(i),
        min_points.at(i),
        max_objects.at(i),
        min_intensities.at(i)
      );

      // if (i==1){
      //   for (unsigned int j = 0; j < dets_cloud_filtered.at(1)->points.size(); j++){
      //       pcl::PointXYZI new_point;
      //       new_point.x = dets_cloud_filtered.at(1)->points[j].x;
      //       new_point.y = dets_cloud_filtered.at(1)->points[j].y;
      //       new_point.z = dets_cloud_filtered.at(1)->points[j].z;
      //       new_point.intensity = dets_cloud_filtered.at(1)->points[j].intensity;
      //       dets_cloud.at(2)->push_back(new_point);
      //   }
      // }

      // if (i==2){
      //   for (unsigned int j = 0; j < dets_cloud_clustered.at(1)->points.size(); j++){
      //       pcl::PointXYZI new_point;
      //       new_point.x = dets_cloud_clustered.at(1)->points[j].x + 0.2;
      //       new_point.y = dets_cloud_clustered.at(1)->points[j].y + 0.2;
      //       new_point.z = dets_cloud_clustered.at(1)->points[j].z + 0.2;
      //       new_point.intensity = dets_cloud_filtered.at(1)->points[j].intensity;
      //       dets_cloud_clustered.at(2)->push_back(new_point);
      //   }
      // }

      // PUBLISHING POINT CLOUD FOR DETECTION
      dets_cloud.at(i)->header.frame_id = "odom";
      dets_cloud_filtered.at(i)->header.frame_id = "odom";
      dets_cloud_clustered.at(i)->header.frame_id = "odom";
      dets_pub.at(i).publish(dets_cloud.at(i));
      dets_filtered_pub.at(i).publish(dets_cloud_filtered.at(i));
      dets_loc_pub.at(i).publish(dets_cloud_clustered.at(i));

      // CONSTRUCTING MARKER ARRAY
      for (unsigned int k=0; k<dets_cloud_clustered.at(i)->points.size(); k++){

        visualization_msgs::Marker m;
        m.header.frame_id = "odom";
        m.pose.position.x = (*dets_cloud_clustered.at(i))[k].x;
        m.pose.position.y = (*dets_cloud_clustered.at(i))[k].y;
        m.pose.position.z = (*dets_cloud_clustered.at(i))[k].z;
        // m.pose.position.x =
        m.pose.orientation.w = 1;
        m.pose.orientation.x = 0;
        m.pose.orientation.y = 0;
        m.pose.orientation.z = 0;
        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.8;

        m.color.a = 1;
        m.color.r = 1;
        m.color.g = 1;
        m.color.b = 1;

        m.type = 9;
        m.id = vis_count;
        m.action = 0;

        m.text = labels.at(i);
        ms.markers.push_back(m);
        vis_count++;
      }
    }
    detection_vis_pub.publish(ms);
}

void AddDetection::cluster_detections(
    pcl::PointCloud<pcl::PointXYZI>::Ptr det_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters_final,
    float tolerance,
    int min_cluster_size,
    int max_objects,
    float min_intensity
){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_filtered_final->clear();

    for (unsigned int i = 0; i < det_cloud->points.size(); i++){
      if (det_cloud->points[i].intensity >= min_intensity){
        cloud_filtered->push_back(det_cloud->points[i]);
      }
    }

    if (cloud_filtered->size() <= 0){
      // std::cout<<"size "<<cloud_filtered->size()<<std::endl;
      return;
    }
    

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters_all (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters_final (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_clusters_final->clear();

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it!=cluster_indices.end(); it++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
        float x_mean=0.0, y_mean=0.0, z_mean=0.0, intensity_mean=0.0, size;

        for (const auto& idx : it->indices){
            pcl::PointXYZI new_point;
            new_point.x = (*cloud_filtered)[idx].x;
            new_point.y = (*cloud_filtered)[idx].y;
            new_point.z = (*cloud_filtered)[idx].z;
            new_point.intensity = (*cloud_filtered)[idx].intensity;
            cloud_temp->push_back(new_point);
            cloud_filtered_final->push_back(new_point);

            x_mean += new_point.x;
            y_mean += new_point.y;
            z_mean += new_point.z;
            intensity_mean += new_point.intensity;

        }
        cloud_temp->width = cloud_temp->size();
        cloud_temp->height = 1;
        cloud_temp->is_dense = false;

        // cloud_filtered_final->points += cloud_temp->points;

        // CALCULATE THE MIDDLE POINT OF THE CLUSTERS
        pcl::PointXYZI cluster_mean;
        cluster_mean.x = x_mean / cloud_temp->size();
        cluster_mean.y = y_mean / cloud_temp->size();
        cluster_mean.z = z_mean / cloud_temp->size();
        cluster_mean.intensity = cloud_temp->size();

        cloud_clusters_all->push_back(cluster_mean);
    }
    std::sort(cloud_clusters_all->points.begin(), cloud_clusters_all->points.end(), AddDetection::cluster_sort_fn);

    max_objects = std::min(max_objects, int(cloud_clusters_all->size()));
    for (int i=0;i<max_objects;i++){
      cloud_clusters_final->push_back(cloud_clusters_all->points.at(i));
    }

    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_camera_detection");
  ros::NodeHandle nh("~");
  AddDetection lidar_camera_detection;
  lidar_camera_detection.init(nh);
  
  ros::spin();

  return 0;
}