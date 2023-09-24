#include "environment/Obstacles.h"

perception_etflab::Obstacles::Obstacles()
{
    step = 0.02;
	delta_y = 0;
	sign = -1;
}

void perception_etflab::Obstacles::move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator cluster = clusters.begin(); cluster < clusters.end(); cluster++)
    {
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = (*cluster)->begin(); point < (*cluster)->end(); point++)
            point->y += delta_y;
    }

    delta_y += sign * step;
    if (delta_y < -1 || delta_y > 0)
        sign *= -1;
}

void perception_etflab::Obstacles::move(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = pcl->begin(); point < pcl->end(); point++)
        point->y += delta_y;
    
    delta_y += sign * step;
    if (delta_y < -1 || delta_y > 0)
        sign *= -1;
}