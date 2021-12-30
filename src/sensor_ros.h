#ifndef PURE_LOCALIZATION_ROS_MSG_H
#define PURE_LOCALIZATION_ROS_MSG_H

#include <ros/ros.h>
#include "cartographer/mapping/id.h"
#include "cartographer/io/submap_painter.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/transform/transform.h"
#include "visualization_msgs/MarkerArray.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "tf2_ros/transform_broadcaster.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "db_carto.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
struct LandmarkObservation {
    std::string id;
    cartographer::transform::Rigid3d landmark_to_tracking_transform;
    double translation_weight;
    double rotation_weight;
};
struct LandmarkData {
    cartographer::common::Time time;
    std::vector<LandmarkObservation> landmark_observations;
};
class sensor_ros
{
public:
    explicit sensor_ros(cartographer_interface &lo,float resolution);
    ~sensor_ros();
    // 获取可视化数据
    std::unique_ptr<nav_msgs::OccupancyGrid>  DrawAndPublish();
    visualization_msgs::MarkerArray GetTrajectoryNodeList();
    // 获取TF发布数据
    void publishTF(void);
    // 传感器数据处理函数
    void laser_callback(sensor_msgs::MultiEchoLaserScanConstPtr msg);
    void laser_onewave_callback(sensor_msgs::LaserScanConstPtr msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);

private:
    std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
            const cartographer::io::PaintSubmapSlicesResult& painted_slices,
            double resolution, const std::string& frame_id,
            const ros::Time& time);
    void set_submaps();

    std::unique_ptr<cartographer::sensor::OdometryData> ToOdomData(
            const nav_msgs::Odometry::ConstPtr& msg);

    std::unique_ptr<cartographer::sensor::ImuData> ToImuData(
            const sensor_msgs::Imu::ConstPtr& msg);



    cartographer::common::Time FromRos(const ::ros::Time& time);

    ros::Time ToRos(::cartographer::common::Time time);

    static Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
        return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
    }
    static Eigen::Quaterniond QuaternionToEigen(const geometry_msgs::Quaternion & Quaternion) {
        return Eigen::Quaterniond (Quaternion.w,Quaternion.x, Quaternion.y,Quaternion.z);
    }

    // LandmarkData ToLandmarkData(const LandmarkList& landmark_list) {
    //     LandmarkData landmark_data;
    //     landmark_data.time = FromRos(landmark_list.header.stamp);
    //     for (const LandmarkEntry& entry : landmark_list.landmarks) {
    //         landmark_data.landmark_observations.push_back(
    //                 {entry.id, ToRigid3d(entry.tracking_from_landmark_transform),
    //                  entry.translation_weight, entry.rotation_weight});
    //     }
    //     return landmark_data;
    // }


    std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
    std::map<cartographer::mapping::SubmapId, cartographer::io::SubmapSlice> m_submap_slices;
    std::string m_last_frame_id;
    ros::Time m_last_timestamp;
    float m_resolution;
    //int m_num_subdivisions_per_laser_scan = 0;
    std::unique_ptr<cartographer::mapping::MapBuilderInterface>& m_map_builder;
    //std::map<std::string, cartographer::common::Time> m_sensor_to_previous_subdivision_time;
    cartographer_interface &m_localization;

    tf::TransformListener m_tf;
    tf::TransformBroadcaster m_tf_broadcaster;
};


#endif //PURE_LOCALIZATION_ROS_MSG_H