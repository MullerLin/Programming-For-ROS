#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subcriber_;
	// ros::Subscriber Point_sub;
	std::string name_topic;
	std::int32_t size_queue_topic;

	// void pointcloudCallback(const sensor_msgs::PointCloud2& msg);
	void scanCallback(const sensor_msgs::LaserScan& msg);

	ros::ServiceClient Emergency_stop;
	std_srvs::SetBool Stop_SMB_Service;

};

} /* namespace */
