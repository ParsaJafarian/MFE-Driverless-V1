#include "lidar_cone_detector/ground_plane_removal.hpp"

namespace lidar_cone_detector {
    
    class GroundPlaneRemovalNode :: GroundPlaneRemovalNode(const rclcpp::NodeOptions &options)
    : Node("ground_plane_removal_node", options)
    {
        public:
            GroundPlaneRemovalNode() : Node("ground_plane_removal_node")
            {
                this->declare_parameter("run_visualization", false);
                this->get_parameter("run_visualization", this->run_visualization);

                // Subscribes to the general point cloud and publishes ground data and the rest in two separate streams
                point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "lidar/pcl/raw", rclcpp::SensorDataQoS(),
                    std::bind(&GroundPlaneRemovalNode::remove_ground_plane_callback, this, std::placeholders::_1)
                );
                point_cloud_ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "lidar/pcl/ground", rclcpp::SensorDataQoS()
                );
                point_cloud_cones_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "lidar/pcl/cones", rclcpp::SensorDataQoS()
                );

                // params defined in header file
            }

        private:
            rclcpp::TimerBase::SharedPtr timer_;    // Used to measure performance metrics

            // copy of the main method from 02/2025 demo ransac
            void remove_ground_plane_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*msg, *pcd);

                // TODO: filter remove all points beyond a certain distance from the origin

                // Voxel downsampling
                pcl::VoxelGrid<pcl::PointXYZ> sor;
                sor.setInputCloud(pcd);
                sor.setLeafSize(0.1f, 0.1f, 0.1f); // sets box size in which will only contain 1 point
                // can change how many points are in each box with setMinimumPointsNumberPerVoxel
                pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pcd(new pcl::PointCloud<pcl::PointXYZ>);
                sor.filter(*downsampled_pcd);

                // Remove statistical outliers
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_outlier;
                sor_outlier.setInputCloud(downsampled_pcd);
                sor_outlier.setMeanK(50);            // number of neighbours to analyze
                sor_outlier.setStddevMulThresh(1.0); // threshold of std devs away from mean that will be marked as outlier
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcd(new pcl::PointCloud<pcl::PointXYZ>);
                sor_outlier.filter(*filtered_pcd);

                // Plane segmentation (RANSAC)
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setAxis(Eigen::Vector3f(0, 0, 1)); // Forces a horizontal plane
                seg.setMaxIterations(1000);
                seg.setDistanceThreshold(0.05);
                seg.setInputCloud(filtered_pcd);

                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                
                seg.segment(*inliers, *coefficients);

                // Logging coefficient data
                if (coefficients->values.size() >= 4) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "RANSAC plane coefficients: [a=%f, b=%f, c=%f, d=%f]",
                        coefficients->values[0],
                        coefficients->values[1],
                        coefficients->values[2],
                        coefficients->values[3]
                    );  // macro for logger
                } else {
                    RCLCPP_WARN(this->get_logger(), "Not enough coefficients returned by the segmenter.");
                }

                // Extract inliers and outliers
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                extract.setInputCloud(filtered_pcd);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*outlier_cloud);
                extract.setNegative(true);
                extract.filter(*inlier_cloud);

                // Output processed data and publish to topics
                sensor_msgs::msg::PointCloud2 cones_output;
                sensor_msgs::msg::PointCloud2 ground_output;

                pcl::toROSMsg(*inlier_cloud, cones_output);
                pcl::toROSMsg(*outlier_cloud, ground_output);

                cones_output.header = msg->header;
                ground_output.header = msg->header;

                point_cloud_cones_pub->publish(cones_output);
                point_cloud_ground_pub->publish(ground_output);

                // Perform visualization depending on if set in ROS params
                // WARNING: this does not work yet
                if (this->run_visualization)
                {
                    // Visualize the point cloud using PCL visualization
                    // Eigen::Vector3f vehicle_position(data["data"][i]["odom"]["x"].asFloat(),
                    // data["data"][i]["odom"]["y"].asFloat(),
                    // data["data"][i]["odom"]["z"].asFloat());
                    // this->visualize(inlier_cloud, vehicle_position);
                }
            }

            /*
                Performs visualization of the RANSAC algorithm using RViz2. 

                Note: this function should only execute when ROS2 Parameter is setup properly.
                @param pcd pcl::PointerCloud<pcl::PointXYZ> The point cloud
                @param vehicle_position Eigen::Vector3f The 3D coordinate of the vehicle from SLAM
            */
            void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd, const Eigen::Vector3f &vehicle_position)
            {
                pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

                // Add point cloud to the viewer
                viewer.addPointCloud<pcl::PointXYZ>(pcd, "cloud");

                // Set up the viewer
                viewer.setBackgroundColor(0.0, 0.0, 0.0);
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

                // Add coordinate frame at the vehicle position
                viewer.addCoordinateSystem(1.0, vehicle_position.x(), vehicle_position.y(), vehicle_position.z());

                // Main loop for visualization
                while (!viewer.wasStopped())
                {
                    viewer.spinOnce(100);
                }
            }
    };

    /*
        Loads a LiDAR point cloud from a binary file using the KITTI format.

        (Deprecation warning) Currently not in use.

        @param bin_file pointer to the binary file
        @return point cloud as a std::vector of Eigen::Vector3f
    */
    std::vector<Eigen::Vector3f> load_bin_pointcloud(const std::string &bin_file)
    {
        std::ifstream file(bin_file, std::ios::binary);
        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << bin_file << std::endl;
            return {};
        }

        std::vector<Eigen::Vector3f> points;
        while (file)
        {
            Eigen::Vector4f point;
            file.read(reinterpret_cast<char *>(&point), sizeof(Eigen::Vector4f));
            if (file)
            {
                points.push_back(point.head<3>());
            }
        }
        return points;
    }
}
