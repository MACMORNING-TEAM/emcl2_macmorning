// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later
// CAUTION: Some lines came from amcl (LGPL).

#ifndef EMCL2__EMCL2_NODE_H_
#define EMCL2__EMCL2_NODE_H_

#include "emcl2/ExpResetMcl2.h"
#include "emcl2/LikelihoodFieldMap.h"
#include "emcl2/OdomModel.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <memory>
#include <string>

namespace emcl2
{
class EMcl2Node : public rclcpp::Node
{
      public:
	EMcl2Node();
	~EMcl2Node();

	void loop(void);
	int getOdomFreq(void);

      private:
	std::shared_ptr<ExpResetMcl2> pf_;

	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr alpha_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_cloud_pub_;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub1_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub2_;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_;


	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
	  initial_pose_sub_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

	// ros::ServiceServer global_loc_srv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
	rclcpp::Time scan_time_stamp_;

	std::string footprint_frame_id_;
	std::string global_frame_id_;
	std::string odom_frame_id_;
	
	std::string scan1_frame_id_;
	std::string scan2_frame_id_;
	std::string scan1_topic_;
	std::string scan2_topic_;

	std::string base_frame_id_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	std::shared_ptr<tf2_ros::Buffer> tf_;

	tf2::Transform latest_tf_;

	rclcpp::Clock ros_clock_;

	int odom_freq_;
	bool init_pf_;
	bool init_request_;
	bool initialpose_receive_;
	bool simple_reset_request_;
	bool scan1_receive_;
	bool scan2_receive_;
	bool map_receive_;
	double init_x_, init_y_, init_t_;
	double transform_tolerance_;


	void publishPose(
	  double x, double y, double t, double x_dev, double y_dev, double t_dev, double xy_cov,
	  double yt_cov, double tx_cov);
	void publishOdomFrame(double x, double y, double t);
	void publishParticles(void);
	bool getOdomPose(double & x, double & y, double & yaw);	 // same name is found in amcl
	bool getLidarPose(double & x, double & y, double & yaw, bool & inv, std::string scan_frame_id);
	void receiveMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

	void publishCombinedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &combined_cloud);

	void declareParameter();

	void initCommunication(void);
	void initTF();
	void initPF(void);
	std::shared_ptr<LikelihoodFieldMap> initMap(void);
	std::shared_ptr<OdomModel> initOdometry(void);

	nav_msgs::msg::OccupancyGrid map_;

	void cbScan1(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
	void cbScan2(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

	// bool cbSimpleReset(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
	bool cbSimpleReset(
	  const std_srvs::srv::Empty::Request::ConstSharedPtr,
	  std_srvs::srv::Empty::Response::SharedPtr);
	void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
				   msg);  // same name is found in amcl
};

}  // namespace emcl2

#endif	// EMCL2__EMCL2_NODE_H_
