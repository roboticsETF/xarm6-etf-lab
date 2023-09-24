#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

namespace perception_etflab
{
    class Clusters
    {
    public:
        Clusters(const std::string config_file_path);

        inline void setMaxDimSubcluster(const Eigen::Vector3f &max_dim_subcluster_) { max_dim_subcluster = max_dim_subcluster_; }
        inline void setConcatenationTolerance(float concatenation_tolerance_) { concatenation_tolerance = concatenation_tolerance_; }

        void computeClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
		void computeSubclusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters,
			                    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &subclusters);
		    
    private:
        void divideCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &subclusters, 
			float min_point, float max_point, float max_dim, std::string &axis);

        Eigen::Vector3f max_dim_subcluster;
        float concatenation_tolerance;
    };
}