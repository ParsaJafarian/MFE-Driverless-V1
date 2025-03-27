#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// dependencies that require ament_cmake to imports
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "Eigen/Dense"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/angles.h"

#include "pcl_conversions/pcl_conversions.h"    // Interface for ROS and PCL

namespace lidar_cone_detector {
    class GroundPlaneRemovalNode : public rclcpp::Node 
    {
        public:
            GroundPlaneRemovalNode(const rclcpp::NodeOptions &options);

            // public params of the node
            bool run_visualization;

        private:
            // sharedptr used for thread saafety and automatic memory management
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2> point_cloud_sub;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2> point_cloud_ground_pub;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2> point_cloud_cones_pub;

            void remove_ground_plane_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
            void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd, const Eigen::Vector3f &vehicle_position);

            // params of the node
            // TODO: parameterize (generalize) the algorithm variables to ROS parameters

            pcl::PointCloud<pcl::PointXYZ>::Ptr acc_cloud;  // Accumulative cloud the algorithm operates on

            pcl::VoxelGrid<pcl::PointXYZ> sor;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_outlier;
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::ExtractIndices<pcl::PointXYZ> extract;

    }

    std::vector<Eigen::Vector3f> load_bin_pointcloud(const std::string &bin_file);
}