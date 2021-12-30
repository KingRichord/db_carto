#include "db_carto.h"
#include <ros/ros.h>
#include "sensor_ros.h"

#include <std_msgs/UInt16.h>
#include <dbrobot_msg/mapping_state.h>
#include <glog/logging.h>
using TrajectoryState =::cartographer::mapping::PoseGraphInterface::TrajectoryState;
class SlamManager {
public:
    SlamManager();
    ~SlamManager();
    void start_slam();
    void end_slam();
    void SlamManagerCallback(const std_msgs::UInt16ConstPtr &msg);
    bool mapping_stateCallBack(dbrobot_msg::mapping_state::Request &rep, dbrobot_msg::mapping_state::Response &res) const;
    std::string TrajectoryStateToString(const TrajectoryState trajectory_state);
private:
    std::shared_ptr<cartographer_interface> localize{nullptr};
    std::shared_ptr<sensor_ros> msg{nullptr};
    ros::NodeHandle m_node;
    // 业务层
    ros::Subscriber robot_manager_sub_;
    ros::Publisher p_mappping_state_;
    ros::ServiceServer mappping_state_;
    // 算法层
    ros::Publisher occupancy_grid_publisher;
    ros::Subscriber m_laser_subscriber;
    ros::Subscriber m_odometry_subscriber;
    ros::Subscriber m_landmarker_subscriber;
    ros::Publisher trajectory_node_list_publisher;

    bool mapping_;
    bool is_simulation_;
};

SlamManager::SlamManager()
{
    LOG(INFO)<<"SLAM管理系统构造完成";
    robot_manager_sub_ = m_node.subscribe("/rm_to_slam", 1, &SlamManager::SlamManagerCallback, this);
    mappping_state_ = m_node.advertiseService<dbrobot_msg::mapping_state::Request, dbrobot_msg::mapping_state::Response>(
            "mapping_state", boost::bind(&SlamManager::mapping_stateCallBack, this, _1, _2));
    p_mappping_state_ = m_node.advertise<std_msgs::UInt16>("/s_mapping_state", 1);
    mapping_ = false;
    ros::spin();
}
bool SlamManager::mapping_stateCallBack(dbrobot_msg::mapping_state::Request &rep,
                                        dbrobot_msg::mapping_state::Response &res) const {
    res.res = mapping_;
    return true;
}

SlamManager::~SlamManager() {
}
// 返回轨迹的状态
std::string SlamManager::TrajectoryStateToString(const TrajectoryState trajectory_state) {
    switch (trajectory_state) {
        case TrajectoryState::ACTIVE:
            return "ACTIVE";
        case TrajectoryState::FINISHED:
            return "FINISHED";
        case TrajectoryState::FROZEN:
            return "FROZEN";
        case TrajectoryState::DELETED:
            return "DELETED";
    }
    return "";
}

void SlamManager::start_slam() {
        LOG(INFO)<<"开始创建地图";
        mapping_ = true;
        std_msgs::UInt16 info;
        info.data = 1;
        p_mappping_state_.publish(info);
        LOG(INFO)<<"创建cartographer核心";
        localize =  std::make_shared<cartographer_interface>("/home/lyc/Downloads/b2-2016-04-05-14-44-52.bag.pbstream",
                                              "/home/moi/3_ws/src/slam/db_carto/configuration_files",
                                              "revo_lds.lua");
        LOG(INFO)<<"创建订阅消息接口";
        msg = std::make_shared<sensor_ros>(*localize, 0.03); //修改地图分辨率
        LOG(INFO)<<"开始订阅消息";
        trajectory_node_list_publisher =
                m_node.advertise<visualization_msgs::MarkerArray>(
                        "Trajectory", 1);
        // 占用栅格地图
        occupancy_grid_publisher = m_node.advertise<nav_msgs::OccupancyGrid>("map", 1,true);
        // 订阅传感器数据
        // 激光雷达数据 + IMU 数据
        // 调用回调函数 ros_msg::laser_callback  ros_msg::imu_callback
        m_laser_subscriber = m_node.subscribe("scan_mapping", 1,
                                                                   &sensor_ros::laser_onewave_callback, &*msg);
        //ros::Subscriber m_imu_subscriber = node_handle.subscribe("imu/raw_data",10,&sensor_ros::imu_callback,&msg);
        m_odometry_subscriber = m_node.subscribe("odom", 1, &sensor_ros::odometry_callback, &*msg);
        //m_landmarker_subscriber =m_node.subscribe("marke")
        ros::Rate loop_rate(20);// 指定循环频率
        int count = 0;
        while (ros::ok()) {
            count++;
            // 发布地图和轨迹数据
            if (count % 40 == 0) //20HZ，五秒发布一次
            {
                LOG(INFO) << "发布地图更新!";
                occupancy_grid_publisher.publish(*msg->DrawAndPublish()); // publish map
            }
            if (count % 2 == 0) {
                std_msgs::UInt16 info;info.data = 1;
                p_mappping_state_.publish(info);
                msg->publishTF();
            }
            trajectory_node_list_publisher.publish(msg->GetTrajectoryNodeList());
            ros::spinOnce();
            loop_rate.sleep();
        }
}

void SlamManager::end_slam() {
    if (msg == nullptr &&localize == nullptr) {
        LOG(WARNING)<<"地图创建已经结束";
        std_msgs::UInt16 info;info.data = 0;
        p_mappping_state_.publish(info);
        return;
    }
    else {
        LOG(WARNING)<<"收到停止创建地图命令";
        int trajectory_num = localize->get_map_builder()->num_trajectory_builders();
        for (int i = 0; i < trajectory_num; ++i) {
            std::cout << "delete trajectory_id: " << i << std::endl;
            LOG(WARNING)<<"正在保存当前轨迹";
            localize->get_map_builder()->FinishTrajectory(i);
            LOG(WARNING)<<"正在优化地图";
            localize->get_map_builder()->pose_graph()->RunFinalOptimization();

            localize->get_map_builder()->pose_graph()->DeleteTrajectory(i);
        }
        const auto trajectory_states = localize->get_map_builder()->pose_graph()->GetTrajectoryStates();
        for (auto stats: trajectory_states) {
            LOG(INFO) << "轨迹的id：" << stats.first << "轨迹的状态：" << TrajectoryStateToString(stats.second);
        }
        LOG(WARNING)<<"关闭所有消息输入";
        m_laser_subscriber.shutdown();
        m_odometry_subscriber.shutdown();
        occupancy_grid_publisher.shutdown();
        trajectory_node_list_publisher.shutdown();
        msg.reset();
        localize.reset();
        mapping_ = false;
        LOG(WARNING)<<"建图结束";
        while (ros::ok()) {
            std_msgs::UInt16 info;
            info.data = 0;
            p_mappping_state_.publish(info);
            ros::Duration(0.05).sleep();
            ros::spinOnce();
        }
    }
}

void SlamManager::SlamManagerCallback(const std_msgs::UInt16ConstPtr &msg) {
    if (msg->data == 0) {
        start_slam();
    } else {
        end_slam();
    }
}
// demo 示例程序
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logbufsecs = 0;  //日志实时输出
    FLAGS_max_log_size=10; //最大日志文件大小 10M
    FLAGS_logtostderr = false; // 设置日志消息是否转到标准输出而不是日志文件
    FLAGS_stop_logging_if_full_disk = true; // 设置是否在磁盘已满时避免日志记录到磁盘
    FLAGS_colorlogtostderr = true; // 设置记录到标准输出的颜色消息（如果终端支持）
    FLAGS_alsologtostderr = true; // 设置日志消息除了日志文件之外是否需要标准输出
    if (access ( "/home/moi/logdir/carto/",0) == -1)
    {
        std::string fp = "mkdir -p ";
        fp = fp + "/home/moi/logdir/carto/";
        system(fp.c_str());
    }
    google::SetLogDestination(google::INFO, "/home/moi/logdir/carto/carto_");
    std::cout << "GLOG ON" << std::endl;

    ros::init(argc, argv, "my_carto");
    SlamManager slam_manager;
}