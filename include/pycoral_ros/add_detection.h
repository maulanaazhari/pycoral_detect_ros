#ifndef _ADD_DETECTION_H_
#define _ADD_DETECTION_H_

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <array>
#include <Eigen/Eigen>
#include <string>

// #include <opencv.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>



#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class AddDetection{
    private:
    // PARAMETERS DEFINITION
    std::string camera_topic, cloud_topic, info_topic, camera_frame, cloud_frame, cloud_out_topic, image_out_topic;
    int num_labels;
    bool info_available = false;
    bool tf_received = false;
    bool odom_received = false;
    int det_flag = false;
    int wait_det_flag = true;
    int drone_idx = 0;


    image_geometry::PinholeCameraModel cam_model;
    cv::Matx34d projection_matrix;

    tf::TransformListener tf_listener;
    geometry_msgs::TransformStamped T_baselink2camera, T_camera2baselink, T_baselink2odom;

    tf2::Quaternion T_baselink2camera_q, T_camera2baselink_q;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher cloud_pub, image_pub, detection_pub, detection_vis_pub;
    // ros::Publisher det0_pub, det1_pub, det2_pub, det3_pub, det4_pub, det5_pub, det6_pub; 
    std::vector<ros::Publisher> dets_pub, dets_loc_pub, dets_filtered_pub;
    ros::Subscriber cam_info_subscriber, odom_subscriber, mission_subscriber;

    ros::ServiceClient detection_client;

    public:
    
    AddDetection(): tf_listener_(tf_buffer_){

    }
    ~AddDetection(){

    }

    // class Detection{

    // }

    // int NUM_DETECTIONS = 7;

    void init(ros::NodeHandle& nh);

    void image_cloud_callback(
        const sensor_msgs::CompressedImageConstPtr& image_msg, 
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg
    );

    void camera_info_callback(
        const sensor_msgs::CameraInfoConstPtr& cam_info_msg
    );

    void odom_callback(
        const nav_msgs::Odometry& odom
    );

    void mission_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    void visualize_detection(vision_msgs::Detection3DArray);

    void append_detection(int id, float score, Eigen::Vector3d pos);

    void set_camera_model();

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> SyncPolicyImageCloud;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageCloud>> SynchronizerImageCloud;

    SynchronizerImageCloud sync_image_cloud_;

    cv::Mat rgb_image;
    cv::Mat blurred_image;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> dets_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> dets_cloud_filtered;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> dets_cloud_clustered;

    std::vector<std::string> labels {"soldier", "soldier", "field bed", "command center", "communication equipment", "weapon storage", "barbed wire"};
    // labels.push_back("person");
    // labels.push_back("person");
    // labels.push_back("stretcher");
    // labels.push_back("computer");
    // labels.push_back("comm equipment");
    // labels.push_back("weapon storage");
    // labels.push_back("barbed wire");

    // CLUSTERING PARAMS
    std::vector<int> max_objects;
    std::vector<int> min_objects;
    std::vector<int> min_points;
    std::vector<float> tolerances;
    std::vector<float> min_intensities;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr detections_0; //PERSON
    // pcl::PointCloud<pcl::PointXYZI> detections_1; //STRETCHER
    // pcl::PointCloud<pcl::PointXYZI> detections_2; //COMPUTER
    // pcl::PointCloud<pcl::PointXYZI> detections_3; //DESK
    // pcl::PointCloud<pcl::PointXYZI> detections_4; //COMMUNICATION EQUIPMENT
    // pcl::PointCloud<pcl::PointXYZI> detections_5; //WEAPON STORAGE
    // pcl::PointCloud<pcl::PointXYZI> detections_6; //BARBED WIRE

    ros::Timer det_pub_timer, cluster_timer;
    
    void det_pub_callback(const ros::TimerEvent&);
    void cluster_timer_callback(const ros::TimerEvent&);
    
    void cluster_detections(
        pcl::PointCloud<pcl::PointXYZI>::Ptr det_cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters_all,
        float tolerance = 1,
        int min_cluster_size = 2,
        int max_objects = 3,
        float min_intensity = 0.3
    );
    typedef std::vector< std::tuple<int, int, int> > my_tuple;

    static bool cluster_sort_fn(const pcl::PointXYZI &a, const pcl::PointXYZI &b){
        return a.intensity > b.intensity;
    }
};

namespace tf2{
    geometry_msgs::TransformStamped inverse(geometry_msgs::TransformStamped tf){
        tf::Transform tf_;
        tf::transformMsgToTF(tf.transform, tf_);
        geometry_msgs::Transform tf_msg_;
        tf::transformTFToMsg(tf_.inverse(), tf_msg_);
        
        geometry_msgs::TransformStamped tf_out;
        tf_out.transform = tf_msg_;
        return tf_out;
    }
}

#endif