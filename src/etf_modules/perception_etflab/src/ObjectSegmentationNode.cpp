#include "ObjectSegmentationNode.h"

perception_etflab::ObjectSegmentationNode::ObjectSegmentationNode(const std::string &node_name, const std::string &config_file_path) : 
    Node(node_name),
    Robot(config_file_path),
    Clusters(config_file_path),
	AABB(),
	ConvexHulls(),
	Obstacles(config_file_path)
{
	if (config_file_path.find("real") != std::string::npos)
		real_robot = true;
	else if (config_file_path.find("sim") != std::string::npos)
		real_robot = false;
	else
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuration file path must start with either 'real' or 'sim'. ");
		return;
	}

	this->declare_parameter<std::string>("objects_cloud", "objects_cloud");		
	this->get_parameter("objects_cloud", objects_cloud);
		
	if (real_robot)
	{
		this->declare_parameter<std::string>("input_cloud", "pointcloud_combined");
		this->get_parameter("input_cloud", input_cloud);
		pcl_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud, 
			rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
			std::bind(&ObjectSegmentationNode::realPointCloudCallback, this, std::placeholders::_1));
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using real robot. Starting up %s with input topic %s and output topic %s", 
			node_name.c_str(), input_cloud.c_str(), objects_cloud.c_str());
	}
	else
	{
        timer = this->create_wall_timer(std::chrono::microseconds(size_t(Obstacles::getPeriod() * 1e6)), 
			std::bind(&ObjectSegmentationNode::simPointCloudCallback, this));
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using simulated robot. Starting up %s without input topic and output topic %s", 
			node_name.c_str(), objects_cloud.c_str());
	}

	object_pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(objects_cloud, 10);
    
    Robot::joints_state_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
		("/xarm6_traj_controller/state", 10, std::bind(&Robot::jointsStateCallback, this, std::placeholders::_1));
    Robot::marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/free_cells_vis_array", 10);
	
    AABB::publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/bounding_boxes", 10);
    AABB::marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array", 10);

    ConvexHulls::points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/convex_hulls", 10);
    ConvexHulls::polygons_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/convex_hulls_polygons", 10);
	ConvexHulls::marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array", 10);
    
}

void perception_etflab::ObjectSegmentationNode::realPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	auto time_start { std::chrono::steady_clock::now() };
	pcl::PCLPointCloud2::Ptr input_pcl_cloud(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2());
	
	pcl::moveFromROSMsg(*msg, *cloud_xyzrgb);
	pcl::toPCLPointCloud2(*cloud_xyzrgb, *input_pcl_cloud);	
	
  	// Create the filtering object: Downsample the dataset using a leaf size of 1 [cm]
  	pcl::VoxelGrid<pcl::PCLPointCloud2> donwnsampler;
  	donwnsampler.setInputCloud(input_pcl_cloud);
  	donwnsampler.setLeafSize(0.01f, 0.01f, 0.01f);
  	donwnsampler.filter(*output_cloud);
  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Downsampled the dataset using a leaf size of 1 [cm].");
  	
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_xyzrgb1(new pcl::PointCloud<pcl::PointXYZRGB>), 
										   output_cloud_xyzrgb2(new pcl::PointCloud<pcl::PointXYZRGB>),
  										   output_cloud_xyzrgb3(new pcl::PointCloud<pcl::PointXYZRGB>);
  										   
  	pcl::fromPCLPointCloud2(*output_cloud, *output_cloud_xyzrgb1);
  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After downsampling, point cloud size is %ld.", output_cloud_xyzrgb1->size());
  	
	// Green color filtering
  	pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;
  	pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr table_green_cond
		(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, 60));
 	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
 	color_cond->addComparison(table_green_cond);
	
 	// Build the filter
 	color_filter.setInputCloud(output_cloud_xyzrgb1);
 	color_filter.setCondition(color_cond);
 	color_filter.filter(*output_cloud_xyzrgb1);
  	
  	pcl::PassThrough<pcl::PointXYZRGB> passThroughZAxis;
  	passThroughZAxis.setFilterFieldName("z");
    passThroughZAxis.setFilterLimits(-0.05, 1.5);
  	passThroughZAxis.setInputCloud(output_cloud_xyzrgb1);
  	passThroughZAxis.filter(*output_cloud_xyzrgb2);
  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After green filtering, point cloud size is %ld.", output_cloud_xyzrgb2->size());
  	
  	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  	// Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  	// Optional
  	seg.setOptimizeCoefficients(true);
  	// Mandatory
  	seg.setModelType(pcl::SACMODEL_PLANE);
  	seg.setMethodType(pcl::SAC_RANSAC);
  	seg.setMaxIterations(1000);
  	seg.setDistanceThreshold(0.02);
  	seg.setInputCloud(output_cloud_xyzrgb2);
  	
  	// Extract the inliers
   	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
   	extract.setInputCloud(output_cloud_xyzrgb2);
   	extract.setIndices(inliers);
   	extract.setNegative(true);
   	extract.filter(*output_cloud_xyzrgb3);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_clusters {};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_subclusters {};

    removeOutliers(output_cloud_xyzrgb3);
	// Robot::removeFromScene2(output_cloud_xyzrgb3);
    Clusters::computeClusters(output_cloud_xyzrgb3, pcl_clusters);

	Robot::removeFromScene(pcl_clusters);	// Better for real robot!
	// Robot::removeFromScene3(pcl_clusters);  // If using, uncomment "xarm_client_node" and "xarm_client" in the 'Robot' constructor

  	publishObjectsPointCloud(pcl_clusters);

    Clusters::computeSubclusters(pcl_clusters, pcl_subclusters);

    AABB::make(pcl_subclusters);
    AABB::publish();
    AABB::visualize();
 
   	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time elapsed: %ld [ms] ", 
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count());
   	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------------------------------");
}

void perception_etflab::ObjectSegmentationNode::simPointCloudCallback()
{
	auto time_start = std::chrono::steady_clock::now();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_subclusters;

	// Robot::visualizeCapsules();
    Robot::visualizeSkeleton();

	Obstacles::move(pcl_clusters);

  	publishObjectsPointCloud(pcl_clusters);

    AABB::make(pcl_clusters);
    AABB::publish();
    AABB::visualize();

    // ConvexHulls::make(pcl_clusters);
    // ConvexHulls::publish();
    // ConvexHulls::visualize();
 
   	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time elapsed: %ld [ms] ", 
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count());
   	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------------------------------");
}

void perception_etflab::ObjectSegmentationNode::publishObjectsPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator cluster = clusters.begin(); cluster < clusters.end(); cluster++)
    {
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = (*cluster)->begin(); point < (*cluster)->end(); point++)
            pcl->emplace_back(*point);
    }
    pcl->width = pcl->points.size();
    pcl->height = 1;
    pcl->is_dense = true;

	sensor_msgs::msg::PointCloud2 output_cloud_ros;	
	pcl::toROSMsg(*pcl, output_cloud_ros);
    output_cloud_ros.header.frame_id = "world";
	output_cloud_ros.header.stamp = now();
	object_pcl_publisher->publish(output_cloud_ros);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing output point cloud of size %ld...", pcl->size());
}

void perception_etflab::ObjectSegmentationNode::removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
    if (pcl->empty())
        return;

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pcl_point = pcl->end()-1; pcl_point >= pcl->begin(); pcl_point--)
	{
        Eigen::Vector3f P(pcl_point->x, pcl_point->y, pcl_point->z);
        if ((P.z() < 0 || P.head(2).norm() > Robot::getTableRadius()) ||				// All points under or outside the table
			(P.x() > -0.2 && P.x() < 0 && std::abs(P.y()) < 0.05 && P.z() < 0.07)) 	// The robot cable from base through the table
            	pcl->erase(pcl_point);
	}

  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After removing outliers, point cloud size is %ld.", pcl->size());
}
