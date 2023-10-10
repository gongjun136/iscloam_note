// Author of ISCLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "iscOptimizationClass.h"

ISCOptimizationClass::ISCOptimizationClass()
{
    
}
// #include "include/STDesc.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "include/ScanContext/scanContext.h"
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>

/**
 * @brief 从pose_file中解析时间戳和位姿信息
 *
 * @param pose_file 文本文件，数据顺序：时间戳、平移xyz、四元数wxyz
 * @param poses_vec
 * @param times_vec
 */
void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec)
{
    // 1.清空时间戳容器、位姿容器
    times_vec.clear();
    poses_vec.clear();

    // 2.将pose_file所有时间戳给时间戳容器，所有位姿给位姿容器
    std::ifstream fin(pose_file);

    if (!fin.is_open())
    {
        LOG(WARNING) << "Error: Unable to open the file " << pose_file;

        return;
    }
    std::string line; // 用于存储每一行内容
    Eigen::Matrix<double, 1, 7> temp_matrix;
    while (getline(fin, line))
    {
        std::istringstream sin(line);
        std::vector<std::string> Waypoints;
        std::string info;
        int number = 0;
        while (getline(sin, info, ' '))
        {
            if (number == 0)
            {
                double time;
                std::stringstream data;
                data << info;
                data >> time;
                times_vec.push_back(time);
                number++;
            }
            else
            {
                double p;
                std::stringstream data;
                data << info;
                data >> p;
                temp_matrix[number - 1] = p;
                if (number == 7)
                {
                    Eigen::Vector3d translation(temp_matrix[0], temp_matrix[1],
                                                temp_matrix[2]);
                    Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4],
                                         temp_matrix[5]);
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
                    single_pose.first = translation;
                    single_pose.second = q.toRotationMatrix();
                    poses_vec.push_back(single_pose);
                }
                number++;
            }
        }
    }
}
// Read KITTI data
/**
 * @brief 读取KITTI的bin数据集
 *
 * @param lidar_data_path
 * @return std::vector<float>
 */
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

void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting)
{

    // pre-preocess
    nh.param<double>("ds_size", config_setting.ds_size_, 0.5);
    nh.param<int>("maximum_corner_num", config_setting.maximum_corner_num_, 100);

    // key points
    nh.param<double>("plane_merge_normal_thre",
                     config_setting.plane_merge_normal_thre_, 0.1);
    nh.param<double>("plane_detection_thre", config_setting.plane_detection_thre_,
                     0.01);
    nh.param<double>("voxel_size", config_setting.voxel_size_, 2.0);
    nh.param<int>("voxel_init_num", config_setting.voxel_init_num_, 10);
    nh.param<double>("proj_image_resolution",
                     config_setting.proj_image_resolution_, 0.5);
    nh.param<double>("proj_dis_min", config_setting.proj_dis_min_, 0);
    nh.param<double>("proj_dis_max", config_setting.proj_dis_max_, 2);
    nh.param<double>("corner_thre", config_setting.corner_thre_, 10);

    // std descriptor
    nh.param<int>("descriptor_near_num", config_setting.descriptor_near_num_, 10);
    nh.param<double>("descriptor_min_len", config_setting.descriptor_min_len_, 2);
    nh.param<double>("descriptor_max_len", config_setting.descriptor_max_len_,
                     50);
    nh.param<double>("non_max_suppression_radius",
                     config_setting.non_max_suppression_radius_, 2.0);
    nh.param<double>("std_side_resolution", config_setting.std_side_resolution_,
                     0.2);

    // candidate search
    nh.param<int>("skip_near_num", config_setting.skip_near_num_, 50);
    nh.param<int>("candidate_num", config_setting.candidate_num_, 50);
    nh.param<int>("sub_frame_num", config_setting.sub_frame_num_, 10);
    nh.param<double>("rough_dis_threshold", config_setting.rough_dis_threshold_,
                     0.01);
    nh.param<double>("vertex_diff_threshold",
                     config_setting.vertex_diff_threshold_, 0.5);
    nh.param<double>("icp_threshold", config_setting.icp_threshold_, 0.5);
    nh.param<double>("normal_threshold", config_setting.normal_threshold_, 0.2);
    nh.param<double>("dis_threshold", config_setting.dis_threshold_, 0.5);

    nh.param<double>("SC_DIST_THRES", config_setting.SC_DIST_THRES, 0.4);

    LOG(INFO) << "Sucessfully load parameters:";
    LOG(INFO) << "----------------Main Parameters-------------------";
    // LOG(INFO) << "downsample size:" << config_setting.ds_size_;
    // LOG(INFO) << "maximum_corner_num:" << config_setting.maximum_corner_num_;

    // LOG(INFO) << "plane_detection_thre:" << config_setting.plane_detection_thre_;
    // LOG(INFO) << "plane_merge_normal_thre:" << config_setting.plane_merge_normal_thre_;
    // LOG(INFO) << "voxel size:" << config_setting.voxel_size_;
    // LOG(INFO) << "voxel_init_num:" << config_setting.voxel_init_num_;
    // LOG(INFO) << "proj_image_resolution:" << config_setting.proj_image_resolution_;
    // LOG(INFO) << "proj_dis_min:" << config_setting.proj_dis_min_;
    // LOG(INFO) << "proj_dis_max:" << config_setting.proj_dis_max_;
    // LOG(INFO) << "corner_thre:" << config_setting.corner_thre_;

    // LOG(INFO) << "descriptor_near_num:" << config_setting.descriptor_near_num_;
    // LOG(INFO) << "descriptor_min_len:" << config_setting.descriptor_min_len_;
    // LOG(INFO) << "descriptor_max_len:" << config_setting.descriptor_max_len_;
    // LOG(INFO) << "non_max_suppression_radius:" << config_setting.non_max_suppression_radius_;
    // LOG(INFO) << "std_side_resolution:" << config_setting.std_side_resolution_;

    // LOG(INFO) << "skip_near_num:" << config_setting.skip_near_num_;
    // LOG(INFO) << "candidate_num:" << config_setting.candidate_num_;
    LOG(INFO) << "sub_frame_num:" << config_setting.sub_frame_num_;
    // LOG(INFO) << "vertex_diff_threshold:" << config_setting.vertex_diff_threshold_;
    // LOG(INFO) << "rough_dis_threshold:" << config_setting.rough_dis_threshold_;
    // LOG(INFO) << "normal_threshold:" << config_setting.normal_threshold_;
    // LOG(INFO) << "dis_threshold:" << config_setting.dis_threshold_;
    // LOG(INFO) << "loop detection threshold: " << config_setting.icp_threshold_;
    LOG(INFO)<<"SC_DIST_THRES: "<<config_setting.SC_DIST_THRES;
    LOG(INFO) << "----------------Main Parameters-------------------";
    // LOG(INFO) << "voxel size:" << config_setting.voxel_size_;
    // LOG(INFO) << "loop detection threshold: " << config_setting.icp_threshold_;
    // LOG(INFO) << "sub-frame number: " << config_setting.sub_frame_num_;
    // LOG(INFO) << "candidate number: " << config_setting.candidate_num_;
    // LOG(INFO) << "maximum corners size: " << config_setting.maximum_corner_num_;
}

/**
 * 计算两个时间点之间的差异，并返回该差异的毫秒值。
 *
 * @param t_end 结束的时间点。
 * @param t_begin 开始的时间点。
 * @return 两个时间点之间的差异，以毫秒为单位。
 */
double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin)
{
    // 计算 t_end 和 t_begin 之间的差异，并将其转换为 double 类型的秒数。
    // 然后将秒数乘以 1000，以获取毫秒值。
    return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                     t_begin)
               .count() *
           1000;
}

inline pcl::PointCloud<pcl::PointXYZI>::Ptr translatePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, const Eigen::Vector3f &t)
{
    int cloudSize = cloudIn->size();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>());
    cloudOut->resize(cloudSize);

    auto trans = t;

    // #pragma omp parallel for num_threads(8)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = pointFrom.x + trans[0];
        cloudOut->points[i].y = pointFrom.y + trans[1];
        cloudOut->points[i].z = pointFrom.z + trans[2];
    }
    return cloudOut;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_lixel");
    ros::NodeHandle nh;
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/gj/catkin_ws_STD/src/STD/logs/";
    FLAGS_alsologtostderr = 1;

    // 从ROS参数服务器获取数据集路径和配置文件路径
    std::string lidar_path = "";
    std::string pose_path = "";
    std::string config_path = "";
    nh.param<std::string>("lidar_path", lidar_path, ""); // LiDAR数据集
    nh.param<std::string>("pose_path", pose_path, "");   // 位姿文件

    ConfigSetting config_setting;
    read_parameters(nh, config_setting);
    // 初始化ROS发布器，用于发布点云、位姿和其他可视化信息
    ros::Publisher pubOdomAftMapped =
        nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    // ros::Publisher pubRegisterCloud =
    //     nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    ros::Publisher pubCureentCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
    ros::Publisher pubCurrentCorner =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
    ros::Publisher pubMatchedCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    ros::Publisher pubMatchedCorner =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
    ros::Publisher pubSTD =
        nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
    tf2_ros::TransformBroadcaster br;

    ros::Rate loop(500);
    ros::Rate slow_loop(10);

    // 加载位姿和时间戳
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec; // 位姿容器
    std::vector<double> times_vec;                                      // 时间戳容器
    load_pose_with_time(pose_path, poses_vec, times_vec);
    LOG(INFO) << "Sucessfully load pose with number: " << poses_vec.size();
    // 构造一个描述符管理器用于SLAM的回环检测
    // 闭环检测
    SCManager scManager(config_setting);

    size_t cloudInd = 0;
    size_t keyCloudInd = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());

    std::vector<double> descriptor_time;
    std::vector<double> querying_time;
    std::vector<double> update_time;
    int triggle_loop_num = 0;
    while (ros::ok())
    {
        // 1.读取雷达数据到lidar_data

        // 使用给定的格式将lidar_path和cloudInd组合到一起
        // std::setfill('0') 和 std::setw(10) 确保cloudInd是一个前导零的、宽度为10的数字
        // 例如, 如果cloudInd是123, 它会被格式化为0000000123
        std::stringstream lidar_data_path;
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        lidar_data_path << lidar_path << std::setfill('0') << std::setw(8)
                        << cloudInd << ".pcd";
        pcl::io::loadPCDFile(lidar_data_path.str(), *current_cloud);
        // std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());

        // 空点云就退出循环
        if (current_cloud->size() == 0)
        {
            LOG(INFO) << "pcd data " << cloudInd << " is empty";
            break;
        }
        // // 2.将点云转到世界系，存放到PCL点云current_cloud
        Eigen::Vector3d translation = poses_vec[cloudInd].first;
        Eigen::Matrix3d rotation = poses_vec[cloudInd].second;

        // Convert to float for PCL compatibility
        Eigen::Vector3f translation_f = translation.cast<float>();
        Eigen::Matrix3f rotation_f = rotation.cast<float>();

        // Create a 4x4 transformation matrix
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        // Set rotation and translation components
        transform.linear() = rotation_f;
        transform.translation() = translation_f;

        // Perform the point cloud transformation in-place
        pcl::transformPointCloud(*current_cloud, *current_cloud, transform);

        // 下采样
        // 创建均匀采样对象，并设置输入点云
        pcl::UniformSampling<pcl::PointXYZI> uniform_sampling;
        uniform_sampling.setInputCloud(current_cloud);
        // 设置采样半径，即3D网格的大小
        uniform_sampling.setRadiusSearch(0.1f); // 例如，0.01米

        // 进行均匀采样处理，并将结果存储在原始点云对象中
        uniform_sampling.filter(*current_cloud);

        for (auto pv : current_cloud->points)
        {
            temp_cloud->points.push_back(pv);
        }

        // check if keyframe
        if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0)
        {
            LOG(INFO) << "--------------";
            LOG(INFO) << lidar_data_path.str();
            Eigen::Quaterniond qua(rotation);
            LOG(INFO) << "key frame pos: " << translation[0] << " " << translation[1] << " " << translation[2]
                      << " att: " << qua.x() << " " << qua.y() << " " << qua.z() << " " << qua.w();
            // LOG(INFO) << "Key Frame id:" << keyCloudInd
            //           << ", cloud size: " << temp_cloud->size() ;
            // step1. 描述符提取
            auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
            Eigen::Vector3f inverseTranslation = -1 * translation_f;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZI>());
            cloud_input = translatePointCloud(temp_cloud, inverseTranslation);
            // 创建并保存Scan Context及其关键表示
            scManager.makeAndSaveScancontextAndKeys(*cloud_input);
            auto t_descriptor_end = std::chrono::high_resolution_clock::now();
            descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));

            // step2. 使用描述符搜索回环
            auto t_query_begin = std::chrono::high_resolution_clock::now();
            // 使用扫描上下文检测潜在的闭环
            auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
            if (detectResult.first != -1)
            {
                LOG(INFO) << "Loop found! between: " << keyCloudInd << "--"
                          << detectResult.first << ", score:" << detectResult.second;
            }
            auto t_query_end = std::chrono::high_resolution_clock::now();
            querying_time.push_back(time_inc(t_query_end, t_query_begin));

            LOG(INFO) << "[Time] descriptor extraction: "
                      << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                      << "query: " << time_inc(t_query_end, t_query_begin) << "ms, ";

            // publish
            // 创建一个用于发布的点云消息
            sensor_msgs::PointCloud2 pub_cloud;
            // 发布temp_cloud
            pcl::toROSMsg(*temp_cloud, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pubCureentCloud.publish(pub_cloud);

            // // 如果搜索结果的第一个元素大于0，表示找到了一个可能的回环
            // if (search_result.first > 0)
            // {
            //     // 增加检测到的回环数量
            //     triggle_loop_num++;
            //     // 将检测到的回环的关键帧转换为ROS消息格式
            //     pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
            //                   pub_cloud);
            //     // 设置消息的坐标系,发布点云
            //     pub_cloud.header.frame_id = "camera_init";
            //     pubMatchedCloud.publish(pub_cloud);

            //     // 短暂休眠，确保数据发布成功
            //     slow_loop.sleep();

            //     // 转换和发布检测到的回环的关键点的点云
            //     pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
            //                   pub_cloud);
            //     pub_cloud.header.frame_id = "camera_init";
            //     pubMatchedCorner.publish(pub_cloud);
            //     // 发布与回环匹配的描述符对
            //     publish_std_pairs(loop_std_pair, pubSTD);
            //     // 再次短暂休眠
            //     slow_loop.sleep();
            //     // getchar();
            // }
            temp_cloud->clear();
            keyCloudInd++;
            loop.sleep();
        }
        // 发布当前帧的位姿
        nav_msgs::Odometry odom;
        odom.header.frame_id = "camera_init";
        odom.pose.pose.position.x = translation[0];
        odom.pose.pose.position.y = translation[1];
        odom.pose.pose.position.z = translation[2];
        Eigen::Quaterniond q(rotation);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        pubOdomAftMapped.publish(odom);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "camera_init";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = odom.pose.pose.position.x;
        transformStamped.transform.translation.y = odom.pose.pose.position.y;
        transformStamped.transform.translation.z = odom.pose.pose.position.z;
        transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;

        br.sendTransform(transformStamped);

        loop.sleep();
        cloudInd++;
    }
    double mean_descriptor_time =
        std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
        descriptor_time.size();
    double mean_query_time =
        std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
        querying_time.size();
    // double mean_update_time =
    //     std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
    //     update_time.size();
    LOG(INFO) << "Total key frame number:" << keyCloudInd
              << ", loop number:" << triggle_loop_num;
    LOG(INFO) << "Mean time for descriptor extraction: " << mean_descriptor_time
              << "ms, query: " << mean_query_time << "ms, total: "
              << mean_descriptor_time + mean_query_time << "ms";
    google::ShutdownGoogleLogging();
    return 0;
}
void ISCOptimizationClass::init(void){

    pointcloud_arr.clear();

    // A prior factor consists of a mean value and a noise model (covariance matrix)
    priorModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    
    // odometry measurement noise model (covariance matrix)
    odomModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<  0.10, 0.10, 0.10, 0.10, 0.10, 0.10).finished());

    //loop noise model
    loopModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.10, 0.10, 0.10, 0.10, 0.10, 0.10).finished());
    //attention, here is x y z r p y order
    stop_check_loop_count=0;
    
    downSizeFilter.setLeafSize(0.8, 0.8, 0.8);
}

bool ISCOptimizationClass::addPoseToGraph(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud_surf_in, std::vector<int>& matched_frame_id, Eigen::Isometry3d& odom_in){

    //pointcloud_arr.push_back(pointcloud_in);
    pcl::VoxelGrid<pcl::PointXYZI> downSizeEdgeFilter;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeSurfFilter;
    downSizeSurfFilter.setLeafSize(0.8, 0.8, 0.8);
    downSizeEdgeFilter.setLeafSize(0.4, 0.4, 0.4);
    downSizeEdgeFilter.setInputCloud(pointcloud_edge_in);
    downSizeSurfFilter.setInputCloud(pointcloud_surf_in);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_edge_in(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_surf_in(new pcl::PointCloud<pcl::PointXYZI>()); 
    downSizeEdgeFilter.filter(*filtered_edge_in);
    downSizeSurfFilter.filter(*filtered_surf_in);
    pointcloud_edge_arr.push_back(filtered_edge_in);
    pointcloud_surf_arr.push_back(filtered_surf_in);
    
    //ROS_INFO("input pc size %d %d",(int)filtered_edge_in->points.size(),(int)filtered_surf_in->points.size());
    gtsam::Pose3 pose3_current = eigenToPose3(odom_in);

    if(pointcloud_edge_arr.size()<=1){
        //if first time 
        pose_optimized_arr.push_back(pose3_current);
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', 1), pose_optimized_arr.front(), priorModel));   
        initials.insert(gtsam::Symbol('x', 1), pose_optimized_arr.back());
        last_pose3 = pose3_current;
        return false;
    }

    odom_original_arr.push_back(last_pose3.between(pose3_current));
    pose_optimized_arr.push_back(pose_optimized_arr.back() * odom_original_arr.back());
    last_pose3 = pose3_current;
    initials.insert(gtsam::Symbol('x', pointcloud_edge_arr.size()), pose_optimized_arr.back());
    graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', pointcloud_edge_arr.size()-1), gtsam::Symbol('x', pointcloud_edge_arr.size()), odom_original_arr.back(), odomModel));

    if(stop_check_loop_count>0){
        stop_check_loop_count--;
        return false;
    }
    if(matched_frame_id.size()!=0){
        //if loop closure detected
        for(int i=0;i<(int)matched_frame_id.size();i++){
            //get initial guess
            gtsam::Pose3 transform_pose3 =  pose_optimized_arr[matched_frame_id[i]].between(pose_optimized_arr.back());
            //ROS_WARN("pose %f,%f,%f, [%f,%f,%f,%f]",transform_pose3.translation().x(),transform_pose3.translation().y(),transform_pose3.translation().z(),transform_pose3.rotation().toQuaternion().w(),transform_pose3.rotation().toQuaternion().x(),transform_pose3.rotation().toQuaternion().y(),transform_pose3.rotation().toQuaternion().z());
            Eigen::Isometry3d transform = pose3ToEigen(pose_optimized_arr[matched_frame_id[i]].between(pose_optimized_arr.back()));
            //ROS_WARN("flag 1 %d,%d",matched_frame_id[i],pointcloud_edge_arr.size());
            if(geometryConsistencyVerification(pointcloud_edge_arr.size()-1, matched_frame_id[i], transform)){
                gtsam::Pose3 loop_temp = eigenToPose3(transform);
                graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', matched_frame_id[i]+1), gtsam::Symbol('x', pointcloud_edge_arr.size()), loop_temp, loopModel));
                gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initials).optimize();
                if(updateStates(result, matched_frame_id[i], pointcloud_edge_arr.size()-1)){
                    stop_check_loop_count=STOP_LOOP_CHECK_COUNTER;
                    ROS_WARN("global optimization finished%d,%d with tranform %f,%f,%f, [%f,%f,%f,%f]",pointcloud_edge_arr.size()-1, matched_frame_id[i],loop_temp.translation().x(),loop_temp.translation().y(),loop_temp.translation().z(),loop_temp.rotation().toQuaternion().w(),loop_temp.rotation().toQuaternion().x(),loop_temp.rotation().toQuaternion().y(),loop_temp.rotation().toQuaternion().z());
                    
                    return true;
                }else{
                    stop_check_loop_count=2;
                }

            }else{
                stop_check_loop_count=2;
            }
        }
    }
    return false;
}

Eigen::Isometry3d ISCOptimizationClass::pose3ToEigen(const gtsam::Pose3& pose3){
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    gtsam::Quaternion q_temp = pose3.rotation().toQuaternion();
    pose_eigen.rotate(Eigen::Quaterniond(q_temp.w(),q_temp.x(),q_temp.y(),q_temp.z()));
    pose_eigen.pretranslate(Eigen::Vector3d(pose3.translation().x(),pose3.translation().y(),pose3.translation().z()));
    return pose_eigen;

}

gtsam::Pose3 ISCOptimizationClass::eigenToPose3(const Eigen::Isometry3d& pose_eigen){
    Eigen::Quaterniond q(pose_eigen.rotation());
    return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()), gtsam::Point3(pose_eigen.translation().x(), pose_eigen.translation().y(), pose_eigen.translation().z()));
}

bool ISCOptimizationClass::updateStates(gtsam::Values& result, int matched_id, int current_id){
    //verify states first
    double sum_residual_q = 0.0;
    double sum_residual_t = 0.0;
    int total_counter=0;
    for(int i =current_id-STOP_LOOP_CHECK_COUNTER-10;i<current_id;i++){
        if(i<0) continue;
        total_counter++;
        gtsam::Pose3 pose_temp1= result.at<gtsam::Pose3>(gtsam::Symbol('x',current_id+1));
        gtsam::Pose3 pose_temp2= result.at<gtsam::Pose3>(gtsam::Symbol('x',i+1));
        gtsam::Pose3 tranform1 = pose_temp2.between(pose_temp1);
        gtsam::Pose3 tranform2 = pose_optimized_arr[i].between(pose_optimized_arr[current_id]);
        gtsam::Pose3 tranform = tranform1.between(tranform2);
        sum_residual_t += std::abs(tranform.translation().x())+std::abs(tranform.translation().y())+std::abs(tranform.translation().z());
        sum_residual_q += std::abs(tranform.rotation().toQuaternion().w()-1)+std::abs(tranform.rotation().toQuaternion().x())+std::abs(tranform.rotation().toQuaternion().y())+std::abs(tranform.rotation().toQuaternion().z());
    }
    for(int i =matched_id-STOP_LOOP_CHECK_COUNTER-10;i<matched_id;i++){
        if(i<0) continue;
        total_counter++;
        gtsam::Pose3 pose_temp1= result.at<gtsam::Pose3>(gtsam::Symbol('x',matched_id+1));
        gtsam::Pose3 pose_temp2= result.at<gtsam::Pose3>(gtsam::Symbol('x',i+1));
        gtsam::Pose3 tranform1 = pose_temp2.between(pose_temp1);
        gtsam::Pose3 tranform2 = pose_optimized_arr[i].between(pose_optimized_arr[matched_id]);
        gtsam::Pose3 tranform = tranform1.between(tranform2);
        sum_residual_t += std::abs(tranform.translation().x())+std::abs(tranform.translation().y())+std::abs(tranform.translation().z());
        sum_residual_q += std::abs(tranform.rotation().toQuaternion().w()-1)+std::abs(tranform.rotation().toQuaternion().x())+std::abs(tranform.rotation().toQuaternion().y())+std::abs(tranform.rotation().toQuaternion().z());
    }
    sum_residual_q = sum_residual_q / total_counter;
    sum_residual_t = sum_residual_t / total_counter;
    //ROS_INFO("optimization discard due to frame unaligned, residual_q:%f, residual_t:%f",sum_residual_q,sum_residual_t);
    
    if(sum_residual_q>0.02 || sum_residual_t>0.5){
        ROS_INFO("optimization discard due to frame unaligned, residual_q:%f, residual_t:%f",sum_residual_q,sum_residual_t);
        //graph.pop_back();
        graph.remove(graph.size()-1);
        return false;
    }
    //update states
    initials.clear();
    for(int i =0;i<(int)result.size();i++){
        pose_optimized_arr[i]= result.at<gtsam::Pose3>(gtsam::Symbol('x',i+1));
        initials.insert(gtsam::Symbol('x', i+1), pose_optimized_arr[i]);
    }
    return true;


}
bool ISCOptimizationClass::geometryConsistencyVerification(int current_id, int matched_id, Eigen::Isometry3d& transform){
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_surf_temp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_edge_temp(new pcl::PointCloud<pcl::PointXYZI>());  
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_surf(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_edge(new pcl::PointCloud<pcl::PointXYZI>()); 

    for(int i = -10; i <=10; i=i+5){
        if(matched_id+i>= current_id || matched_id+i<0)
            continue;
        Eigen::Isometry3d transform_pose = pose3ToEigen(pose_optimized_arr[matched_id+i]);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_temp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*pointcloud_surf_arr[matched_id+i], *transformed_temp, transform_pose.cast<float>());
        *map_surf_temp+=*transformed_temp;
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_temp2(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*pointcloud_edge_arr[matched_id+i], *transformed_temp2, transform_pose.cast<float>());
        *map_edge_temp+=*transformed_temp2;
    }

    Eigen::Isometry3d transform_pose = pose3ToEigen(pose_optimized_arr[matched_id]);
    pcl::transformPointCloud(*map_edge_temp, *map_edge, transform_pose.cast<float>().inverse());
    pcl::transformPointCloud(*map_surf_temp, *map_surf, transform_pose.cast<float>().inverse());
    //ROS_INFO("tag31");
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_surf_temp(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_edge_temp(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_surf(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_edge(new pcl::PointCloud<pcl::PointXYZI>()); 
    for(int i = 0; i <=0; i=i+3){
        if(current_id-i<0)
            continue;
        Eigen::Isometry3d transform_pose = pose3ToEigen(pose_optimized_arr[current_id+i]);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_temp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*pointcloud_surf_arr[current_id-i], *transformed_temp, transform_pose.cast<float>());
        *current_scan_surf_temp+=*transformed_temp;
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_temp2(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*pointcloud_edge_arr[current_id-i], *transformed_temp2, transform_pose.cast<float>());
        *current_scan_edge_temp+=*transformed_temp2;
    }

    transform_pose = pose3ToEigen(pose_optimized_arr[current_id]);
    pcl::transformPointCloud(*current_scan_edge_temp, *current_scan_edge, transform_pose.cast<float>().inverse());
    pcl::transformPointCloud(*current_scan_surf_temp, *current_scan_surf, transform_pose.cast<float>().inverse());

    //this is for visualization only
    loop_candidate_pc->clear();
    loop_map_pc->clear();
    transform_pose = transform;
    transform_pose.translation() = Eigen::Vector3d(0,0,10);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::transformPointCloud(*current_scan_surf, *loop_candidate_pc, transform_pose.cast<float>());
    pcl::transformPointCloud(*current_scan_edge, *cloud_temp, transform_pose.cast<float>());
    *loop_candidate_pc+=*cloud_temp;
    *loop_map_pc += *map_surf;
    *loop_map_pc += *map_edge;
    double match_score = estimateOdom(map_edge,map_surf,current_scan_edge,current_scan_surf,transform);
    //ROS_WARN("matched score %f",match_score);

    if(match_score < LOOPCLOSURE_THRESHOLD){
        return true;
    }
    else{
        //ROS_INFO("loop rejected due to geometry verification current_id%d matched_id %d, score: %f",current_id,matched_id,match_score);
        return false;
    }
 return false;
}

Eigen::Isometry3d ISCOptimizationClass::getLastPose(void){
    return pose3ToEigen(pose_optimized_arr.back());
}

Eigen::Isometry3d ISCOptimizationClass::getPose(int frame_num){

    return pose3ToEigen(pose_optimized_arr[frame_num]);
}

double ISCOptimizationClass::estimateOdom(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_source_edge, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_source_surf, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_target_edge, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_target_surf, Eigen::Isometry3d& transform){
    Eigen::Quaterniond init_q(transform.rotation());
    Eigen::Vector3d init_t(0,0,0);
    double parameters[7] = {init_q.x(), init_q.y(), init_q.z(), init_q.w(),init_t.x(), init_t.y(), init_t.z()};
    Eigen::Map<Eigen::Quaterniond> q_temp = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_temp = Eigen::Map<Eigen::Vector3d>(parameters + 4);
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCorner = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurf = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeCorner->setInputCloud(pc_source_edge);
    kdtreeSurf->setInputCloud(pc_source_surf);
    double total_cost = 300;
    for (int opti_counter = 0; opti_counter < 10; opti_counter++)
    {
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
 
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() = t_temp;
        transform.linear() = q_temp.toRotationMatrix();
        //add edge cost factor
        int corner_num=0;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tranformed_edge(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*pc_target_edge, *tranformed_edge, transform);
        for (int i = 0; i < (int) tranformed_edge->points.size(); i++)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeCorner->nearestKSearch(tranformed_edge->points[i], 5, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[4] < 2.0)
            {
                std::vector<Eigen::Vector3d> nearCorners;
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d tmp(pc_source_edge->points[pointSearchInd[j]].x,
                                        pc_source_edge->points[pointSearchInd[j]].y,
                                        pc_source_edge->points[pointSearchInd[j]].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                center = center / 5.0;

                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }
                
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                Eigen::Vector3d curr_point(pc_target_edge->points[i].x, pc_target_edge->points[i].y, pc_target_edge->points[i].z);
                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                { 
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;

                    ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                    corner_num++;   
                }                           
            }
        }
        if(corner_num<20){
            ROS_INFO("not enough corresponding points");
            return 300.0;
        }
        //add surf cost factor
        int surf_num=0; 
        pcl::PointCloud<pcl::PointXYZI>::Ptr tranformed_surf(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*pc_target_surf, *tranformed_surf, transform);    
        // find correspondence for plane features
        for (int i = 0; i <(int) tranformed_surf->points.size(); ++i)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeSurf->nearestKSearch(tranformed_surf->points[i], 5, pointSearchInd, pointSearchSqDis);
            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < 2.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = pc_source_surf->points[pointSearchInd[j]].x;
                    matA0(j, 1) = pc_source_surf->points[pointSearchInd[j]].y;
                    matA0(j, 2) = pc_source_surf->points[pointSearchInd[j]].z;
                }
                
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();
                bool planeValid = true;
                
                for (int j = 0; j < 5; j++)
                {
                    
                    if (fabs(norm(0) * pc_source_surf->points[pointSearchInd[j]].x +
                             norm(1) * pc_source_surf->points[pointSearchInd[j]].y +
                             norm(2) * pc_source_surf->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                Eigen::Vector3d curr_point(pc_target_surf->points[i].x, pc_target_surf->points[i].y, pc_target_surf->points[i].z);
                if (planeValid)
                {
                    ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                    surf_num++;
                }

            }
        }

        if(surf_num<20){
            ROS_INFO("not enough corresponding points");
            return 300.0;
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if(summary.final_cost<total_cost)
            total_cost = summary.final_cost;
    }
    //transform = Eigen::Isometry3d::Identity();
    transform.linear() = q_temp.toRotationMatrix();
    transform.translation() = t_temp;
    return total_cost;

}
