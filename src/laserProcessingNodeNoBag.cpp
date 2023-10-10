// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local lib
#include "lidar.h"
#include "laserProcessingClass.h"

#include <glog/logging.h>

LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

double total_time = 0;
int total_frame = 0;
void laser_processing()
{

    // while (1)
    {
        if (!pointCloudBuf.empty())
        {
            // LOG(INFO) << "processing nobag lidar cloud " << total_frame;

            // read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            ros::Time pointcloud_time = ros::Time::now();
            pointcloud_in = pointCloudBuf.front();
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time += time_temp;
            // ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            *pointcloud_filtered += *pointcloud_edge;
            *pointcloud_filtered += *pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "velodyne";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "velodyne";
            pubEdgePoints.publish(edgePointsMsg);

            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "velodyne";
            pubSurfPoints.publish(surfPointsMsg);
        }
        // sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file;
    lidar_data_file.open(lidar_data_path,
                         std::ifstream::in | std::ifstream::binary); // 二进制模式打开文件
    // 文件不存在，返回空数组
    if (!lidar_data_file)
    {
        std::cout << "Read End..." << std::endl;
        std::vector<float> nan_data;
        return nan_data;
        // exit(-1);
    }
    // 定位到文件末尾获取数据的元素数量
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    // 从文件中读取LiDAR数据到缓存区
    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                         num_elements * sizeof(float));
    return lidar_data_buffer;
}
pcl::PointXYZI vec2point(const Eigen::Vector3d &vec)
{
    pcl::PointXYZI pi;
    pi.x = vec[0];
    pi.y = vec[1];
    pi.z = vec[2];
    return pi;
}
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi)
{
    return Eigen::Vector3d(pi.x, pi.y, pi.z);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "NObag");
    ros::NodeHandle nh;

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/gj/catkin_ws_iscloam/src/iscloam/log";

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period = 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    std::string lidar_path = "/mnt/f/datasets/KITTI raw data/training/05/2011_09_30_drive_0018_sync/2011_09_30/2011_09_30_drive_0018_sync/velodyne_points/data/";

    nh.getParam("/scan_period", scan_period);
    nh.getParam("/vertical_angle", vertical_angle);
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/file_path", lidar_path);

    std::cout << lidar_path << std::endl;

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100);

    ros::Rate loop(500);
    size_t cloudInd = 0;
    while (ros::ok())
    {

        // 1.读取雷达数据到lidar_data
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        std::stringstream lidar_data_path;
        // lidar_data_path << lidar_path << std::setfill('0') << std::setw(10)
        //                 << cloudInd << ".bin";
        // std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());

        lidar_data_path << lidar_path << std::setfill('0') << std::setw(8)
                        << cloudInd << ".pcd";
        pcl::io::loadPCDFile(lidar_data_path.str(), *current_cloud);

        LOG(INFO) << lidar_data_path.str();
        cloudInd++;

        // for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        // {
        //     pcl::PointXYZI point;
        //     point.x = lidar_data[i];
        //     point.y = lidar_data[i + 1];
        //     point.z = lidar_data[i + 2];
        //     point.intensity = lidar_data[i + 3];
        //     Eigen::Vector3d pv = point2vec(point);
        //     point = vec2point(pv);
        //     current_cloud->push_back(point);
        // }

        // 空点云就退出循环
        if (current_cloud->size() == 0)
        {
            std::cerr << "Lidar has no data" << std::endl;
            break;
        }

        pointCloudBuf.push(current_cloud);

        laser_processing();

        loop.sleep();
    }

    return 0;
}
