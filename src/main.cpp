#include "db_carto.h"
#include <ros/ros.h>
#include "sensor_ros.h"
// 日志相关
#include <glog/logging.h>
#include <fstream>
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

    // 构造 cartographer_interface 类，传入配置文件目录和文件名
    cartographer_interface localize("/home/lyc/Downloads/b2-2016-04-05-14-44-52.bag.pbstream",
                          "/home/moi/3_ws/src/slam/db_carto/configuration_files",
                          "revo_lds.lua");
    // ROS 初始化 
    ros::init(argc, argv, "localization");
    ros::start();
    ros::NodeHandle node_handle;
    //TF发布
    // tf::TransformListener tf_;
    // tf::TransformBroadcaster *tfB_;
    // ROS 数据接口类初始化
    sensor_ros msg(localize, 0.03); // 修改地图分辨率
    // 发布两条数据 
    // 轨迹数据
    ros::Publisher trajectory_node_list_publisher =
            node_handle.advertise<::visualization_msgs::MarkerArray>(
                    "Trajectory", 1);
    // 占用栅格地图
    ros::Publisher occupancy_grid_publisher = node_handle.advertise<::nav_msgs::OccupancyGrid>("map", 1,true);
    // 订阅传感器数据
    // 激光雷达数据 + IMU 数据
    // 调用回调函数 ros_msg::laser_callback  ros_msg::imu_callback
    ros::Subscriber m_laser_subscriber = node_handle.subscribe("scan_mapping",10,&sensor_ros::laser_onewave_callback,&msg);
    //ros::Subscriber m_imu_subscriber = node_handle.subscribe("imu/raw_data",10,&sensor_ros::imu_callback,&msg);
    ros::Subscriber m_odometry_subscriber = node_handle.subscribe("odom",10,&sensor_ros::odometry_callback,&msg);
    ros::Rate loop_rate(20);// 指定循环频率
    int count =0;
    while(ros::ok())
    {
        count ++;
        // 发布地图和轨迹数据
        if(count %40 ==0) //20HZ，五秒发布一次
        {
            LOG(INFO)<<"publish new map!";
            occupancy_grid_publisher.publish(*msg.DrawAndPublish()); // publish map
        }
        if(count %2 ==0)
        {
            msg.publishTF();
        }
        // publish traje
        trajectory_node_list_publisher.publish(msg.GetTrajectoryNodeList());
        //ros::spin();
        ros::spinOnce();
        loop_rate.sleep();
    }
}