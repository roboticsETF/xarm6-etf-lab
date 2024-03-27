#include "Robot.h"

perception_etflab::Robot::Robot(const std::string &config_file_path)
{
    std::string project_abs_path(__FILE__);
	for (size_t i = 0; i < 3; i++)
		project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    YAML::Node robot_node { node["robot"] };
    num_DOFs = robot_node["num_DOFs"].as<size_t>();

    std::vector<float> capsules_radius {};
    for (size_t i = 0; i < num_DOFs; i++)
        capsules_radius.emplace_back(robot_node["capsules_radius"][i].as<float>());

    robot = std::make_shared<robots::xArm6>(project_abs_path + robot_node["urdf"].as<std::string>(),
                                            robot_node["gripper_length"].as<float>(),
                                            robot_node["table_included"].as<bool>());
    robot->setCapsulesRadius(capsules_radius);
                                            
    Eigen::VectorXf q_start(num_DOFs);
    for (size_t i = 0; i < num_DOFs; i++)
        q_start(i) = robot_node["q_start"][i].as<float>();
    joints_state = std::make_shared<base::RealVectorSpaceState>(q_start);
    robot->setConfiguration(joints_state);
    skeleton = robot->computeSkeleton(joints_state);

    table_radius = robot_node["table_radius"].as<float>();
    for (size_t i = 0; i < num_DOFs; i++)
        tolerance_factors.emplace_back(robot_node["tolerance_factors"][i].as<float>());

    // Uncomment if you are using 'removeFromScene3' function
    // xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    // xarm_client.init(xarm_client_node, "xarm");
}

void perception_etflab::Robot::jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
	Eigen::VectorXf q(num_DOFs);
    for (size_t i = 0; i < num_DOFs; i++)
        q(i) = msg->actual.positions[i];

    joints_state = std::make_shared<base::RealVectorSpaceState>(q);

    // if (num_DOFs == 6)
	//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

// Remove all PCL points occupied by the robot's capsules.
// If a single point from cluster (i-th component of 'clusters') is occupied, such cluster is completely removed.
void perception_etflab::Robot::removeFromScene(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    if (clusters.empty())
        return;

	// Compute robot skeleton only if the robot changed its configuration
	if ((joints_state->getCoord() - robot->getConfiguration()->getCoord()).norm() > 1e-3)
	{
		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot is moving. Computing new skeleton for (%f, %f, %f, %f, %f, %f).", 
		// 	joints_state->getCoord(0), joints_state->getCoord(1), joints_state->getCoord(2),
		// 	joints_state->getCoord(3), joints_state->getCoord(4), joints_state->getCoord(5));
		skeleton = robot->computeSkeleton(joints_state);
	}
    
    bool remove_cluster { false };
    size_t cnt { clusters.size() };
    for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator cluster = clusters.end()-1; cluster >= clusters.begin(); cluster--)
    {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Considering cluster %ld", --cnt);
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pcl_point = (*cluster)->begin(); pcl_point < (*cluster)->end(); pcl_point++)
	    {
		    Eigen::Vector3f point(pcl_point->x, pcl_point->y, pcl_point->z);
            for (int k = robot->getNumLinks()-1; k >= 0; k--)
            {
                float d_c = std::get<0>(base::RealVectorSpace::distanceLineSegToPoint(skeleton->col(k), skeleton->col(k+1), point));
                if (d_c < robot->getCapsuleRadius(k) * tolerance_factors[k])
                {
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Skeleton points for k = %d: (%f, %f, %f) to (%f, %f, %f) ", k,
                    //     skeleton->col(k).x(), skeleton->col(k).y(), skeleton->col(k).z(), 
                    //     skeleton->col(k+1).x(), skeleton->col(k+1).y(), skeleton->col(k+1).z());
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point: (%f, %f, %f)", point.x(), point.y(), point.z());
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance from %d-th segment is %f.", k, d_c);
                    remove_cluster = true;
                    break;
                }
            }

            if (remove_cluster)
            {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Removing cluster!");
                clusters.erase(cluster);
                remove_cluster = false;
                break;
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After removing robot from the scene, there are %ld clusters.", clusters.size());
}

// Remove all PCL points occupied by the robot's capsules.
void perception_etflab::Robot::removeFromScene2(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
    if (pcl->empty())
        return;

	// Compute robot skeleton only if the robot changed its configuration
	if ((joints_state->getCoord() - robot->getConfiguration()->getCoord()).norm() > 1e-3)
	{
		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot is moving. Computing new skeleton for (%f, %f, %f, %f, %f, %f).", 
		// 	joints_state->getCoord(0), joints_state->getCoord(1), joints_state->getCoord(2),
		// 	joints_state->getCoord(3), joints_state->getCoord(4), joints_state->getCoord(5));
		skeleton = robot->computeSkeleton(joints_state);
	}
    
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pcl_point = pcl->end()-1; pcl_point >= pcl->begin(); pcl_point--)
    {
        Eigen::Vector3f point(pcl_point->x, pcl_point->y, pcl_point->z);
        for (int k = robot->getNumLinks()-1; k >= 0; k--)
        {
            float d_c = std::get<0>(base::RealVectorSpace::distanceLineSegToPoint(skeleton->col(k), skeleton->col(k+1), point));
            if (d_c < robot->getCapsuleRadius(k) * tolerance_factors[k])
            {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Skeleton points for k = %d: (%f, %f, %f) to (%f, %f, %f) ", k,
                //     skeleton->col(k).x(), skeleton->col(k).y(), skeleton->col(k).z(), 
                //     skeleton->col(k+1).x(), skeleton->col(k+1).y(), skeleton->col(k+1).z());
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point: (%f, %f, %f)", point.x(), point.y(), point.z());
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance from %d-th segment is %f.", k, d_c);
                pcl->erase(pcl_point);
                break;
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After removing robot from the scene, point cloud size is %ld.", pcl->size());
}

// This method requires using xarm_client.
// Only points around robot gripper and cable are filtered, assuming that color filter was used previously to filter other points occupied by the robot.
// If a single point from cluster (i-th component of 'clusters') is occupied, such cluster is completely removed.
void perception_etflab::Robot::removeFromScene3(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    if (clusters.empty())
        return;

    std::vector<float> pose {};
    xarm_client.get_position(pose);
    Eigen::Vector3f TCP(pose[0]/1000, pose[1]/1000, pose[2]/1000);
    Eigen::Matrix3f R {};
    R = Eigen::AngleAxisf(pose[5], Eigen::Vector3f::UnitZ()) 
      * Eigen::AngleAxisf(pose[4], Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(pose[3], Eigen::Vector3f::UnitX());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector pose: (%f, %f, %f)", TCP(0), TCP(1), TCP(2));        // XYZ in [m]
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector RPY:  (%f, %f, %f)", pose[3], pose[4], pose[5]);     // RPY angles in [rad]
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));

    // (AB, r) represents a capsule that encloses the last robot link including the attached gripper
    Eigen::Vector3f A { TCP - R.col(2) * 0.07 };
    Eigen::Vector3f B { A - R.col(2) * 0.13 };
    float r { 0.1 };
    size_t cnt { clusters.size() };

    for (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator cluster = clusters.end()-1; cluster >= clusters.begin(); cluster--)
    {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Considering cluster %ld", --cnt);
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pcl_point = (*cluster)->begin(); pcl_point < (*cluster)->end(); pcl_point++)
	    {
            // Filter points occupying the last link
            Eigen::Vector3f point(pcl_point->x, pcl_point->y, pcl_point->z);
            if (std::get<0>(base::RealVectorSpace::distanceLineSegToPoint(A, B, point)) < r * tolerance_factors.back())
            {
		        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point: (%f, %f, %f). Removing cluster! ", point.x(), point.y(), point.z());
                clusters.erase(cluster);
				break;
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After removing robot from the scene, there are %ld clusters.", clusters.size());
}

void perception_etflab::Robot::visualizeCapsules()
{
    std::shared_ptr<Eigen::MatrixXf> skeleton = robot->computeSkeleton(joints_state);
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "robot_capsules";
        marker.header.frame_id = "world";
        // marker.header.stamp = now();
    Eigen::Vector3f A {}, B {}, C {}, AB {};
    float theta {};

    for (long int i = 0; i < skeleton->cols() - 1; i++) 
    {
        A = skeleton->col(i);
        B = skeleton->col(i+1);
        AB = B - A;
        C = Eigen::Vector3f::UnitZ().cross(AB);
        C.normalize();
        theta = acos(Eigen::Vector3f::UnitZ().dot(AB) / AB.norm());
        // LOG(INFO) << "Skeleton: " << A.transpose() << " --- " << B.transpose();

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.id = i;
        marker.pose.position.x = (A(0) + B(0)) / 2;
        marker.pose.position.y = (A(1) + B(1)) / 2;
        marker.pose.position.z = (A(2) + B(2)) / 2;
        marker.pose.orientation.x = C.x() * sin(theta/2);
        marker.pose.orientation.y = C.y() * sin(theta/2);
        marker.pose.orientation.z = C.z() * sin(theta/2);
        marker.pose.orientation.w = cos(theta/2);
        marker.scale.x = 2 * robot->getCapsuleRadius(i);
        marker.scale.y = 2 * robot->getCapsuleRadius(i);
        marker.scale.z = AB.norm();
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.2;
        marker_array_msg.markers.emplace_back(marker);

        for (size_t j = 0; j < 2; j++)
        {
            C = skeleton->col(i+j);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.id = 6 * (j+1) + i;
            marker.pose.position.x = C(0);
            marker.pose.position.y = C(1);
            marker.pose.position.z = C(2);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 0.0;
            marker.scale.x = 2 * robot->getCapsuleRadius(i);
            marker.scale.y = 2 * robot->getCapsuleRadius(i);
            marker.scale.z = 2 * robot->getCapsuleRadius(i);
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.2;
            marker_array_msg.markers.emplace_back(marker);
        }
    }

    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing robot capsules...");
}

void perception_etflab::Robot::visualizeSkeleton()
{
    std::shared_ptr<Eigen::MatrixXf> skeleton = robot->computeSkeleton(joints_state);
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker;
    Eigen::Vector3f P {};
    
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "robot_skeleton";
    marker.id = 0;
    marker.header.frame_id = "world";
    // marker.header.stamp = now();
    geometry_msgs::msg::Point point;
    for (long int i = 0; i < skeleton->cols(); i++) 
    {
        P = skeleton->col(i);
        point.x = P(0); 
		point.y = P(1); 
		point.z = P(2);
        marker.points.emplace_back(point);
    }
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_array_msg.markers.emplace_back(marker);

    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.id = 1;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker_array_msg.markers.emplace_back(marker);

    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing robot skeleton...");
}
