#include "db_carto.h"
#include <ros/ros.h>
#include "sensor_ros.h"
#include <std_msgs/UInt16.h>
#include <dbrobot_msg/mapping_state.h>
class SlamManager {
public:
    SlamManager();
    ~SlamManager();
    void start_slam();
    void end_slam();
    void SlamManagerCallback(const std_msgs::UInt16ConstPtr &msg);
    bool mapping_stateCallBack(dbrobot_msg::mapping_state::Request &rep, dbrobot_msg::mapping_state::Response &res) const;

private:
    std::shared_ptr<cartographer_interface> localize{nullptr};
    std::shared_ptr<sensor_ros> msg{nullptr};
    ros::NodeHandle m_node;
    ros::Subscriber robot_manager_sub_;
    ros::Publisher p_mappping_state_;
    ros::ServiceServer mappping_state_;

    bool mapping_;
    bool is_simulation_;
};

SlamManager::SlamManager()
{
    //核心启动模组
    LOG(INFO)<<"SlamManager构造完成";
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
    msg = nullptr;
    localize = nullptr;
}

void SlamManager::start_slam() {

    if (localize == nullptr&&msg == nullptr) {
        mapping_ = true;
        std_msgs::UInt16 info;
        info.data = 1;
        p_mappping_state_.publish(info);

        localize =  std::make_shared<cartographer_interface>("/home/lyc/Downloads/b2-2016-04-05-14-44-52.bag.pbstream",
                                              "/home/moi/3_ws/src/slam/db_carto/configuration_files",
                                              "revo_lds.lua");
        msg = std::make_shared<sensor_ros>(*localize, 0.03); //修改地图分辨率
        ros::Publisher trajectory_node_list_publisher =
                m_node.advertise<visualization_msgs::MarkerArray>(
                        "Trajectory", 1);
        // 占用栅格地图
        ros::Publisher occupancy_grid_publisher = m_node.advertise<nav_msgs::OccupancyGrid>("map", 1,
                                                                                                   true /* latched */);
        // 订阅传感器数据
        // 激光雷达数据 + IMU 数据
        // 调用回调函数 ros_msg::laser_callback  ros_msg::imu_callback
        ros::Subscriber m_laser_subscriber = m_node.subscribe("scan_mapping", 10,
                                                                   &sensor_ros::laser_onewave_callback, &*msg);
        //ros::Subscriber m_imu_subscriber = node_handle.subscribe("imu/raw_data",10,&sensor_ros::imu_callback,&msg);
        ros::Subscriber m_odometry_subscriber = m_node.subscribe("odom", 10, &sensor_ros::odometry_callback, &*msg);
        ros::Rate loop_rate(20);// 指定循环频率
        int count = 0;
        while (ros::ok()) {
            count++;
            // 发布地图和轨迹数据
            if (count % 100 == 0) //20HZ，五秒发布一次
            {
                LOG(INFO) << "publish new map!";
                occupancy_grid_publisher.publish(*msg->DrawAndPublish()); // publish map
            }
            if (count % 2 == 0) {
                msg->publishTF();
            }

            // publish traje
            trajectory_node_list_publisher.publish(msg->GetTrajectoryNodeList());
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

void SlamManager::end_slam() {
    if (msg == nullptr &&localize == nullptr) {
        ROS_INFO("kn null cannot stop slam");
        std_msgs::UInt16 info;
        info.data = 0;
        p_mappping_state_.publish(info);
        return;
    }
    else {
        LOG(WARNING)<<"释放智能指针";
        int trajectory_id =localize->get_map_builder()->num_trajectory_builders();
        localize->get_map_builder()->FinishTrajectory(trajectory_id);
        localize->get_map_builder()->pose_graph()->RunFinalOptimization();
        localize = nullptr;
        msg = nullptr;
        mapping_ = false;
        LOG(WARNING)<<"释放智能指针完成";
        while (ros::ok()) {
            if (msg == nullptr &&localize == nullptr)  {
                std_msgs::UInt16 info;
                info.data = 0;
                p_mappping_state_.publish(info);
            }
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
    std::cout << "Glog ON" << std::endl;

    ros::init(argc, argv, "carto");
    SlamManager slam_manager;

}