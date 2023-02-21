#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

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
	ros::Publisher vis_pub;
	ros::Subscriber subcriber_;
	ros::Subscriber Point_sub;
	std::string name_topic;
	std::int32_t size_queue_topic;
	std::float_t linear_gain;
	std::float_t angular_gain;
	visualization_msgs::Marker marker;
	void pointcloudCallback(const sensor_msgs::PointCloud2& msg);
	void scanCallback(const sensor_msgs::LaserScan& msg);
	void Marker_publish(const float pos[]);
};

} /* namespace */
