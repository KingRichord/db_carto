#include "db_carto.h"
#include <utility>
#include "json.hpp"
/**
 * @brief  构造函数，设置传感器数据，并添加一条轨迹，加载当前地图 
 * @param map_path                  需要加载的地图路径
 * @param configuration_directory   配置参数目录
 * @param configuration_basename    配置参数文件
 */
cartographer_interface::cartographer_interface(const std::string& map_path,
                           const std::string& configuration_directory,
                           const std::string& configuration_basename)
{
    // @TODO 激光雷达和机器人中心的距离
    m_trans << 0.16,0,0.2;
    m_rot = Eigen::Quaterniond(1, 0, 0, 0);
    // 从配置文件中加载配置参数
    std::tie(m_node_options, m_trajectory_options) = LoadOptions(configuration_directory,configuration_basename);
    // 调用 cartographer 接口，构造 MapBuilder 类
    m_map_builder = cartographer::common::make_unique<cartographer::mapping::MapBuilder>(m_node_options.map_builder_options);
    // 纯定位模式下，加载现有的地图数据
    if(m_trajectory_options.trajectory_builder_options.pure_localization())
    {
        read_map(map_path);
    }
    //@TODO need to change when change the sensors
    // 目前在本项目中使用一个2d雷达 + IMU
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    // 用什么传感器打开那个传感器
    // 1,配置文件需要更改；2，配置传感器的总配置；3，IMU传感器的回调函数
    m_sensor_ids.insert(SensorId{SensorType::RANGE, "echoes"});// laser 2d
    //m_sensor_ids.insert(SensorId{SensorType::IMU, "imu"});// IMU
    m_sensor_ids.insert(SensorId{SensorType::ODOMETRY, "odom"});
    // 调用接口构造一条轨迹，返回轨迹id
    // m_sensor_ids   传感器类型及id
    // trajectory_builder_options 轨迹的配置参数
    // 最后一个参数为lambda表达式，为定位结果回调调用函数
    m_trajectory_id = m_map_builder->AddTrajectoryBuilder(m_sensor_ids, m_trajectory_options.trajectory_builder_options,
                                                          [this](const int id,
                                                                 const cartographer::common::Time time,
                                                                 const cartographer::transform::Rigid3d local_pose,
                                                                 cartographer::sensor::RangeData range_data_in_local,
                                                                 const std::unique_ptr<const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult> res) {
                                                                 OnLocalSlamResult2(id, time, local_pose,
                                                                                    range_data_in_local);
                                                          });
    // 获取轨迹轨迹生成类指针
    m_trajectory_builder = m_map_builder->GetTrajectoryBuilder(m_trajectory_id);
}

cartographer_interface::~cartographer_interface()
{
    delete m_painted_slices;
    m_map_builder.release();

}

/**
 * @brief  回调函数
 * @param id                   轨迹id
 * @param time                 时间
 * @param local_pose           局部地图上的位置
 * @param range_data_in_local  局部地图到全局地图的转换关系
 */
void cartographer_interface::OnLocalSlamResult2(
            const int id, const cartographer::common::Time time,
            const cartographer::transform::Rigid3d& local_pose,
            const cartographer::sensor::RangeData& range_data_in_local)
{
    cartographer::transform::Rigid3d local2global = m_map_builder->pose_graph()->GetLocalToGlobalTransform(m_trajectory_id);
    cartographer::transform::Rigid3d pose3d = local2global * local_pose;
    robot_pose = pose3d;
    // 将机器人当前位置实时写入文件
    LOG(INFO)<<"pose: x"<<pose3d.translation().x()<<" y "<<pose3d.translation().y()<<" angle "<<pose3d.rotation().z();

    std::ofstream ofs("/home/moi/.amcl_pose");  //第一次写入的位置
    nlohmann::json position;
    position["x"] = pose3d.translation().x();
    position["y"] = pose3d.translation().y();
    position["yaw"] = pose3d.translation().z();
    position["cov0"] = 0.0;
    position["cov1"] = 0.0;
    position["cov2"] = 0.0;
    ofs << position << std::endl;
    ofs.close();
    //写入实时位置
    std::ofstream ofs2("/home/moi/.amcl_posec"); //第二次写入的位置
    ofs2 << position << std::endl;
    ofs2.close();
}


/**
 * @brief 从 pbstream 文件中读取地图
 * @param map_path 地图路径
 * @return 是否成功读取地图
 */
bool cartographer_interface::read_map(const std::string& map_path)
{
    const std::string suffix = ".pbstream";
    CHECK_EQ(map_path.substr(
            std::max<int>(map_path.size() - suffix.size(), 0)),
             suffix)
        << "The file containing the state to be loaded must be a "
           ".pbstream file.";
    cartographer::io::ProtoStreamReader stream(map_path);
    m_map_builder->LoadState(&stream,true);
    return true;
}

/**
 * @brief 2D激光雷达数据处理
 * @param sensor_id   传感器id
 * @param time        采集时间
 * @param ranges      激光雷达数据
 */
void cartographer_interface::Handle2DLaserScanMessage(const std::string& sensor_id,
                                                      const cartographer::common::Time time,
                                                      const std::string& frame_id,
                                                      const cartographer::sensor::TimedPointCloud& ranges)
{
    // 检查点云时间
    CHECK_LE(ranges.back().time, 0);
    // 将一帧激光数据分为10次输入
    for (int i = 0; i != m_trajectory_options.num_subdivisions_per_laser_scan; ++i)
    {
        // 起始id
        const size_t start_index =
                ranges.size() * i / m_trajectory_options.num_subdivisions_per_laser_scan;
        // 结束id
        const size_t end_index =
                ranges.size() * (i + 1) / m_trajectory_options.num_subdivisions_per_laser_scan;
        // 构造  subdivision
        cartographer::sensor::TimedPointCloud subdivision(
                ranges.begin() + start_index, ranges.begin() + end_index);
        if (start_index == end_index)
        {
            continue;
        }
        // subdivision 的时间
        const float time_to_subdivision_end = subdivision.back().time;
        // `subdivision_time` is the end of the measurement so sensor::Collator will
        // send all other sensor data first.
        // 重新分配时间
        const cartographer::common::Time subdivision_time =
                time + cartographer::common::FromSeconds(time_to_subdivision_end);
        auto it = m_sensor_to_previous_subdivision_time.find(sensor_id);
        if (it != m_sensor_to_previous_subdivision_time.end() &&
            it->second >= subdivision_time)
        {
            LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                         << sensor_id << " because previous subdivision time "
                         << it->second << " is not before current subdivision time "
                         << subdivision_time;
            continue;
        }
        m_sensor_to_previous_subdivision_time[sensor_id] = subdivision_time;
        for (auto& point : subdivision)
        {
            point.time -= time_to_subdivision_end;
        }
        // 调用接口对数据进行处理
        CHECK_EQ(subdivision.back().time, 0);
        add2DLaserScanMessage(sensor_id, subdivision_time, frame_id, subdivision);
    }
}

/**
 * @brief 2D激光雷达数据处理
 * @param sensor_id   传感器id
 * @param time        采集时间
 * @param ranges      激光雷达数据
 */
void cartographer_interface::add2DLaserScanMessage(const std::string &sensor_id, cartographer::common::Time time,
                                                   const std::string &frame_id,
                                                   const cartographer::sensor::TimedPointCloud &ranges)
{
    cartographer::transform::Rigid3d sensor_to_tracking(m_trans, m_rot);
    m_trajectory_builder->AddSensorData(sensor_id,
                                        cartographer::sensor::TimedPointCloudData{
                                                time,
                                                sensor_to_tracking.translation().cast<float>(),
                                                cartographer::sensor::TransformTimedPointCloud(ranges,sensor_to_tracking.cast<float>())});
}

/**
 * @brief 里程计数据处理
 * @param sensor_id  传感器id
 * @param time       采集时间
 * @param pose       里程计数据
 */
void cartographer_interface::HandleOdometryMessage(const std::string& sensor_id,
                                         cartographer::common::Time time,
                                         cartographer::transform::Rigid3d pose)
{
    m_trajectory_builder->AddSensorData(
            sensor_id,
            cartographer::sensor::OdometryData{time, std::move(pose)});
}

/**
 * @brief IMU数据处理
 * @param sensor_id 传感器id 
 * @param data      IMU数据
 */
void cartographer_interface::HandleImuMessage(
        const std::string& sensor_id,
        std::unique_ptr<cartographer::sensor::ImuData> &data)
{
    m_trajectory_builder->AddSensorData(
            sensor_id,
            *data);
}

/**
 * @brief  从配置文件中加载参数
 * @param configuration_directory  配置文件目录
 * @param configuration_basename 配置文件名称
 * @return
 */
std::tuple<NodeOptions, TrajectoryOptions>
cartographer_interface::LoadOptions(const std::string &configuration_directory,
                                    const std::string &configuration_basename)
{
    // 获取文件名
    auto file_resolver = cartographer::common::make_unique<
            cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string code =
            file_resolver->GetFileContentOrDie(configuration_basename);
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
            code, std::move(file_resolver));
    // 调用函数获取配置参数
    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                           CreateTrajectoryOptions(&lua_parameter_dictionary));
}


/**
 * @brief  加载节点参数 
 * @param lua_parameter_dictionary 节点参数目录
 * @return 
 */
NodeOptions cartographer_interface::CreateNodeOptions(
        ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary)
{
    NodeOptions options;
    // 地图参数
    options.map_builder_options =
            ::cartographer::mapping::CreateMapBuilderOptions(
                    lua_parameter_dictionary->GetDictionary("map_builder").get());
    // 其他的地图参数
    // 地图坐标
    options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    // tf变换数据发布频率
    options.lookup_transform_timeout_sec =
            lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
    // 子地图发布频率
    options.submap_publish_period_sec =
            lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
    // 位姿发布频率        
    options.pose_publish_period_sec =
            lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
    // 轨迹发布频率        
    options.trajectory_publish_period_sec =
            lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
    return options;
}

/**
 * @brief  加载轨迹参数
 * @param lua_parameter_dictionary
 * @return
 */
TrajectoryOptions cartographer_interface::CreateTrajectoryOptions(
        ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
    TrajectoryOptions options;
    // 轨迹参数
    options.trajectory_builder_options =
            ::cartographer::mapping::CreateTrajectoryBuilderOptions(
                    lua_parameter_dictionary->GetDictionary("trajectory_builder").get());
    // 其他参数
    // 跟踪坐标系
    options.tracking_frame =
            lua_parameter_dictionary->GetString("tracking_frame");
    // 发布的坐标系
    options.published_frame =
            lua_parameter_dictionary->GetString("published_frame");
    // 里程计坐标系
    options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
    // 是否发布里程计坐标系
    options.provide_odom_frame =
            lua_parameter_dictionary->GetBool("provide_odom_frame");
    // 是否采用里程计信息
    options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
    // 是否采用 navset
    options.use_nav_sat = lua_parameter_dictionary->GetBool("use_nav_sat");
    // 是否采用路标点
    options.use_landmarks = lua_parameter_dictionary->GetBool("use_landmarks");
    // 是否将坐标投影到2D
    options.publish_frame_projected_to_2d =
            lua_parameter_dictionary->GetBool("publish_frame_projected_to_2d");
    // 使用激光雷达数目
    options.num_laser_scans =
            lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
    // 使用激光雷达数目       
    options.num_multi_echo_laser_scans =
            lua_parameter_dictionary->GetNonNegativeInt("num_multi_echo_laser_scans");
    // 一帧激光雷达分成几次进行处理        
    options.num_subdivisions_per_laser_scan =
            lua_parameter_dictionary->GetNonNegativeInt(
                    "num_subdivisions_per_laser_scan");
    // 3D点云的数量                
    options.num_point_clouds =
            lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
    // 激光信息采样频率       
    options.rangefinder_sampling_ratio =
            lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
    // 里程计信息采样频率        
    options.odometry_sampling_ratio =
            lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
    // GPS信息采样频率        
    options.fixed_frame_pose_sampling_ratio =
            lua_parameter_dictionary->GetDouble("fixed_frame_pose_sampling_ratio");
    // IMU信息采样频率
    options.imu_sampling_ratio =
            lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
    // 路标点信息采样频率
    options.landmarks_sampling_ratio =
            lua_parameter_dictionary->GetDouble("landmarks_sampling_ratio");
    return options;
}