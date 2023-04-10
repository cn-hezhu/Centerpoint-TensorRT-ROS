#pragma once
// headers in STL
#include <memory>
#include <vector>

// headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// headers in PCL
#include <pcl/io/pcd_io.h>

// headers in local files
#include "../centerpoint/centerpoint.h"

#define PP_SENSOR_HEIGHT  1.73f

class MMdetPPROS
{
private:
    const std::string DB_CONF_;
    std::string pfe_file_,backbone_file_;
    std::unique_ptr<CenterPoint> center_point_ptr_;
    float* anchor_data_;

    // initializer list
    ros::NodeHandle private_nh_;
    bool has_subscribed_baselink_;
    const int NUM_POINT_FEATURE_;
    const int OUTPUT_NUM_BOX_FEATURE_;
    const float TRAINED_SENSOR_HEIGHT_;
    const float NORMALIZING_INTENSITY_VALUE_;
    const std::string BASELINK_FRAME_;
    // end initializer list

    // rosparam
    bool baselink_support_;
    bool reproduce_result_mode_;
    // end rosparam
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_objects_;

    float offset_z_from_trained_data_;

    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    void pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array, const float offset_z = 0);
    void pubDetectedObject(const std::vector<Box>& detections, const std_msgs::Header& in_header);

public:
    MMdetPPROS();
    void createROSPubSub();
};
