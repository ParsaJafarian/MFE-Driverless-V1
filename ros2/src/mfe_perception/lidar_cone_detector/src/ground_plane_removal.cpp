#include "lidar_cone_detector/ground_plane_removal.hpp"

namespace lidar_cone_detector {
    
    class GroundPlaneRemovalNode :: GroundPlaneRemovalNode(const rclcpp::NodeOptions &options)
    : Node("ground_plane_removal_node", options)
    {
        public:
            GroundPlaneRemovalNode() : Node("ground_plane_removal_node")
            {
                // TODO: Rename topic names to better fit
                point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "pcl_points", rclcpp::SensorDataQoS(),
                    std::bind(&GroundPlaneRemovalNode::remove_ground_plane, this)
                );
                point_cloud_ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "lidar/ground_pcl", rclcpp::SensorDataQoS()
                );
                point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "lidar/pcl", rclcpp::SensorDataQoS()
                );

                // define appropriate inner params to use
            }
z
        private:
            rclcpp::TimerBase::SharedPtr timer_;

            // copy of the main method from 02/2025 demo ransac
            void remove_ground_plane(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {
                // Dataset path
                std::string path_to_dataset = "./cfs_vargarda8/";

                // Load JSON metadata
                std::ifstream json_file(path_to_dataset + "metadata.json");
                Json::Value data;
                json_file >> data;

                // Load point cloud from .bin file
                int i = 0;
                std::string bin_file = path_to_dataset + data["data"][i]["pointcloud"]["file"].asString();
                std::cout << "Loading point cloud from: " << bin_file << std::endl;
                auto points = load_bin_pointcloud(bin_file);

                // Convert to PCL PointCloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto &point : points)
                {
                    pcd->points.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
                }
                std::cout << "Loaded point cloud with " << pcd->points.size() << " points." << std::endl;

                // Save the loaded point cloud
                pcl::PCDWriter writer;
                writer.writeBinary("start.pcd", *pcd);

                // TODO: filter remove all points beyond a certain distance from the origin

                // Voxel downsampling
                pcl::VoxelGrid<pcl::PointXYZ> sor;
                sor.setInputCloud(pcd);
                sor.setLeafSize(0.1f, 0.1f, 0.1f); // sets box size in which will only contain 1 point
                // can change how many points are in each box with setMinimumPointsNumberPerVoxel
                pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pcd(new pcl::PointCloud<pcl::PointXYZ>);
                sor.filter(*downsampled_pcd);
                writer.writeBinary("downsample.pcd", *downsampled_pcd);

                // Remove statistical outliers
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_outlier;
                sor_outlier.setInputCloud(downsampled_pcd);
                sor_outlier.setMeanK(50);            // number of neighbours to analyze
                sor_outlier.setStddevMulThresh(1.0); // threshold of std devs away from mean that will be marked as outlier
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcd(new pcl::PointCloud<pcl::PointXYZ>);
                sor_outlier.filter(*filtered_pcd);
                writer.writeBinary("filtered.pcd", *filtered_pcd);

                // Plane segmentation (RANSAC)
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);

                seg.setAxis(Eigen::Vector3f(0, 0, 1)); // Forces a horizontal plane
                // seg.setEpsAngle(pcl::deg2rad(10.0));   // Allow slight angle variation

                seg.setMaxIterations(1000);
                seg.setDistanceThreshold(0.05);
                seg.setInputCloud(filtered_pcd);
                seg.segment(*inliers, *coefficients);

                std::cout << "Plane equation: " << coefficients->values[0] << "x + "
                        << coefficients->values[1] << "y + "
                        << coefficients->values[2] << "z + "
                        << coefficients->values[3] << " = 0" << std::endl;

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

                writer.writeBinary("remove_ground.pcd", *inlier_cloud);
                writer.writeBinary("ground.pcd", *outlier_cloud);

                // Visualize the point cloud using PCL visualization
                Eigen::Vector3f vehicle_position(data["data"][i]["odom"]["x"].asFloat(),
                                                data["data"][i]["odom"]["y"].asFloat(),
                                                data["data"][i]["odom"]["z"].asFloat());

                this->visualize(inlier_cloud, vehicle_position);
            }

            // execute visualization usin RViz2
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
    }

    // Function to load the LiDAR point cloud from a binary file (KITTI format)
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