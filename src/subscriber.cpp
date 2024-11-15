#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <open3d/Open3D.h>
#include <vector>
#include <eigen3/Eigen/Dense>


size_t repeat_count = 0;
auto stacked_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
ros::Time prev_timestamp;

void save_pcd(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    auto source_cloud = std::make_shared<open3d::geometry::PointCloud>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *pcl_cloud);

    for (const auto& point : pcl_cloud->points) {
        source_cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    std::cout << "Point data size(source_cloud): " << source_cloud->points_.size() << std::endl;

    // Save the point cloud data
    std::string file_path = "/home/kenji/ws_livox/src/fast_lio_subscriber/data/cloud_registerd/fast-lio_data-";
    //std::string file_name = "/home/kenji/ws_livox/src/fast_lio_subscriber/data/Laser_map/fast-lio_data-" + std::to_string(repeat_count) + ".pcd";
    std::string file_name = file_path + std::to_string(repeat_count) + ".pcd";
    //repeat_count++;
    open3d::io::WritePointCloud(file_name, *source_cloud);

    std::cout << "Saved point cloud data!" << std::endl;

    return;
}

void stack_point_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud, bool is_save) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *pcl_cloud);

    for (const auto& point : pcl_cloud->points) {
        stacked_point_cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    std::cout << "Point data size(stacked_point_cloud): " << stacked_point_cloud->points_.size() << std::endl;

    if (is_save) {
        ros::Duration timestamp_duration = cloud->header.stamp - prev_timestamp;
        std::cout << "Timestamp duration: " << timestamp_duration.toNSec() << std::endl;
        prev_timestamp = cloud->header.stamp;

        std::string file_path = "/home/kenji/ws_livox/src/fast_lio_subscriber/data/cloud_registerd/fast-lio_data-";
        std::string file_name = file_path + std::to_string(repeat_count) + "_point-num_" + std::to_string(stacked_point_cloud->points_.size()) + ".pcd";
        open3d::io::WritePointCloud(file_name, *stacked_point_cloud);
        std::cout << "Saved point cloud data!" << std::endl;

        stacked_point_cloud->points_.clear();
    }

    return;
}

void callback(const sensor_msgs::PointCloud2ConstPtr& fast_lio_cloud) {
    repeat_count++;

    if (!fast_lio_cloud) {
        ROS_WARN("Received an empty point cloud");
        return;
    }

    ROS_INFO("Received point cloud with %d points", fast_lio_cloud->width * fast_lio_cloud->height);

    stack_point_cloud(fast_lio_cloud, false);

    // Limited the number of saving pcd files
    //if (repeat_count % 30 == 0) {
    if (repeat_count % 3 == 0) {
        stack_point_cloud(fast_lio_cloud, true);
        //save_pcd(fast_lio_cloud);
    }

    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fast_lio_listener");
    ros::NodeHandle nh;

    // The second argument is the queue size
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_effected", 1, callback); // cloud_effected can't be used.
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/Laser_map", 1, callback);  // Laser_map can't be used.
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_body", 1, callback);    // OK, But almost this setting is as same as  /cloud_registered
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 1, callback);   // OK

    ros::spin();

    return 0;
}