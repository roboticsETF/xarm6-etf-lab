#include "environments/Clusters.h"

perception_etflab::Clusters::Clusters(const std::string &config_file_path)
{
    std::string project_abs_path(__FILE__);
	for (size_t i = 0; i < 4; i++)
		project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    YAML::Node perception_node { node["perception"] };
	for (size_t i = 0; i < 3; i++)
        max_dim_subcluster(i) = perception_node["max_dim_subcluster"][i].as<float>();

    concatenation_tolerance = perception_node["concatenation_tolerance"].as<float>();
}

void perception_etflab::Clusters::computeClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, 
                                                  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    // Set up KD-Tree for searching and perform Euclidean clustering
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(pcl);
    std::vector<pcl::PointIndices> cluster_indices {};
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec {};
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(pcl);
    ec.extract(cluster_indices);

    // Create separate point cloud for each cluster
    for (pcl::PointIndices cluster_index : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t idx : cluster_index.indices)
        {
            cluster->points.emplace_back(pcl->points[idx]);
            // pcl::PointXYZRGB point = pcl->points[idx];
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "(%f, %f, %f)", point.x, point.y, point.z);
        }        
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.emplace_back(cluster);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point cloud is segmented into %ld clusters.", clusters.size());
}

void perception_etflab::Clusters::computeSubclusters(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &subclusters)
{
    Eigen::Vector4f min_point {}, max_point {};
    std::vector<std::string> axes = {"x", "y", "z"};

    for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster : clusters)
    {
        pcl::getMinMax3D(*cluster, min_point, max_point);
        std::vector<float> dim {max_point.x() - min_point.x(), 
                                max_point.y() - min_point.y(), 
                                max_point.z() - min_point.z()};
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bouding-box dim: (%f, %f, %f)", dim[0], dim[1], dim[2]);
        std::vector<size_t> idx(dim.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&dim](size_t a, size_t b) { return dim[a] > dim[b]; });

        // The cluster is firstly divided in axis which has the longest dimension
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> subclusters1 {}; 
        divideCluster(cluster, subclusters1, min_point(idx[0]), max_point(idx[0]), max_dim_subcluster(idx[0]), axes[idx[0]]);

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> subclusters2 {};
        for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcluster1 : subclusters1)
            divideCluster(subcluster1, subclusters2, min_point(idx[1]), max_point(idx[1]), max_dim_subcluster(idx[1]), axes[idx[1]]);
        
        for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcluster2 : subclusters2)
            divideCluster(subcluster2, subclusters, min_point(idx[2]), max_point(idx[2]), max_dim_subcluster(idx[2]), axes[idx[2]]);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Clusters are divided into totally %ld subclusters.", subclusters.size());
}

void perception_etflab::Clusters::divideCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, 
                                                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &subclusters, 
                                                float min_point, float max_point, float max_dim, const std::string &axis)
{
    size_t num_pieces = std::ceil((max_point - min_point) / max_dim);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to divide cluster into %ld pieces according to %s axis.", num_pieces, axis.c_str());
    if (num_pieces > 1)
    {
        float delta { (max_point - min_point) / num_pieces };
        Eigen::Vector4f min_point_temp {}, max_point_temp {};
        Eigen::Vector4f min_point_result {}, max_point_result {};
        std::vector<std::string> axes = {"x", "y", "z"};
        size_t idx_prev { 0 }, added_clusters { 0 };

        for (size_t i = 0; i < num_pieces; i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PassThrough<pcl::PointXYZRGB> passThroughAxis;
            passThroughAxis.setFilterFieldName(axis);
            passThroughAxis.setFilterLimits(min_point + i*delta, min_point + (i+1)*delta);  	
            passThroughAxis.setInputCloud(cluster);
            passThroughAxis.filter(*subcluster);
            
            if (subcluster->empty())
            {
                idx_prev = i + 1;
                continue;
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Resulting subcluster %ld is between %f and %f w.r.t. %s axis.", 
            //     i, min_point + i*delta, min_point + (i+1)*delta, axis.c_str());
            // for (auto pt : subcluster->points)
            //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point: (%f, %f, %f)", pt.x, pt.y, pt.z);  

            pcl::getMinMax3D(*subcluster, min_point_temp, max_point_temp);
            if (i == idx_prev)
            {
                min_point_result = min_point_temp;
                max_point_result = max_point_temp;
                subclusters.emplace_back(subcluster);
                added_clusters++;
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adding as a new subcluster!");
            }
            else
            {
                idx_prev = i;
                bool concatenate { true };
                for (size_t j = 0; j < 3; j++)
                {
                    if (axes[j] != axis &&
                        std::abs(min_point_temp(j) - min_point_result(j)) +
                        std::abs(max_point_temp(j) - max_point_result(j)) > concatenation_tolerance)
                        {
                            concatenate = false;
                            break;
                        }
                }

                if (concatenate)
                {
                    for (pcl::PointXYZRGB point : subcluster->points)
                        subclusters.back()->emplace_back(point);
                    min_point_result = min_point_result.cwiseMin(min_point_temp);
                    max_point_result = max_point_result.cwiseMax(max_point_temp);
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Concatenated with the previous subcluster!");
                }
                else
                {
                    min_point_result = min_point_temp;
                    max_point_result = max_point_temp;
                    subclusters.emplace_back(subcluster);
                    added_clusters++;
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adding as a new subcluster!");
                } 
            }
        }
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Divided into %ld subclusters.", added_clusters);
    }
    else
        subclusters.emplace_back(cluster);
}
