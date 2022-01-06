#ifndef PURE_LOCALIZATION_LOCALIZATION_H
#define PURE_LOCALIZATION_LOCALIZATION_H
#include <string>
#include <unordered_map>
#include <glog/logging.h>

#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "absl/strings/str_cat.h"

// 轨迹参数结构体
struct TrajectoryOptions {
    ::cartographer::mapping::proto::TrajectoryBuilderOptions
            trajectory_builder_options;
    std::string tracking_frame;
    std::string published_frame;
    std::string odom_frame;
    bool provide_odom_frame;
    bool use_odometry;
    bool use_nav_sat;
    bool use_landmarks;
    bool publish_frame_projected_to_2d;
    int num_laser_scans;
    int num_multi_echo_laser_scans;
    int num_subdivisions_per_laser_scan;
    int num_point_clouds;
    double rangefinder_sampling_ratio;
    double odometry_sampling_ratio;
    double fixed_frame_pose_sampling_ratio;
    double imu_sampling_ratio;
    double landmarks_sampling_ratio;
};

// 节点参数结构体
struct NodeOptions {
    ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
    std::string map_frame;
    double lookup_transform_timeout_sec;
    double submap_publish_period_sec;
    double pose_publish_period_sec;
    double trajectory_publish_period_sec;
};




/**
 * @brief cartographer 算法接口 
 * */
class cartographer_interface {
public:
    // 机器人当前的位姿
    cartographer::transform::Rigid3d robot_pose;

    // 构造和析构函数
    // explicit关键字只对有一个参数的类构造函数有效, 如果类构造函数参数大于或等于两个时, 是不会产生隐式转换的, 所以explicit关键字也就无效了
    explicit cartographer_interface(const std::string &map,
                                    const std::string &configuration_directory,
                                    const std::string &configuration_basename);

    ~cartographer_interface();
    // 处理2D激光雷达数据
    void Handle2DLaserScanMessage(const std::string &sensor_id,
                                  cartographer::common::Time time,
                                  const std::string &frame_id,
                                  const cartographer::sensor::TimedPointCloud &ranges);

    // @TODO 处理3D激光雷达数据
    void Handle3DLaserScanMessage() {}

    // 处理里程计信息
    void HandleOdometryMessage(const std::string &sensor_id,
                               cartographer::common::Time time,
                               cartographer::transform::Rigid3d pose);

    // 处理IMU数据
    void HandleImuMessage(const std::string &sensor_id,
                          std::unique_ptr<cartographer::sensor::ImuData> &data);

    std::unique_ptr<cartographer::mapping::MapBuilderInterface> &get_map_builder() { return m_map_builder; }
private:
    void add2DLaserScanMessage(const std::string &sensor_id,
                               cartographer::common::Time time,
                               const std::string &frame_id,
                               const cartographer::sensor::TimedPointCloud &ranges);

    bool read_map(const std::string &map_path);// read map from file
    // 加载配置参数
    static std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
            const std::string &configuration_directory,
            const std::string &configuration_basename);

    static NodeOptions CreateNodeOptions(
            ::cartographer::common::LuaParameterDictionary *lua_parameter_dictionary);

    static TrajectoryOptions CreateTrajectoryOptions(
            ::cartographer::common::LuaParameterDictionary *lua_parameter_dictionary);

    void OnLocalSlamResult2(
            int trajectory_id, cartographer::common::Time time,
            const cartographer::transform::Rigid3d &local_pose,
            const cartographer::sensor::RangeData &range_data_in_local);

    // 激光雷达和车体中心的转换关系
    Eigen::Vector3d m_trans;
    Eigen::Quaterniond m_rot;
    std::map<std::string, cartographer::common::Time> m_sensor_to_previous_subdivision_time;
    cartographer::io::PaintSubmapSlicesResult *m_painted_slices{};
    int m_trajectory_id;
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> m_sensor_ids;
    cartographer::mapping::TrajectoryBuilderInterface *m_trajectory_builder;
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> m_map_builder;
    NodeOptions m_node_options;
    TrajectoryOptions m_trajectory_options;
};


#endif //PURE_LOCALIZATION_LOCALIZATION_H
