#include "smb_emrgency_stop/SmbEmrgencyStop.hpp"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace smb_highlevel_controller {


SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	 //if (!(nodeHandle_.getParam("param_topic_name",name_topic))& nodeHandle_.getParam("param_topic_queue_size",size_queue_topic))
	 if (!(nodeHandle_.getParam("param_topic_subcribe/name",name_topic))& nodeHandle_.getParam("param_topic_subcribe/queue_size",size_queue_topic))
	 {
	   ROS_ERROR("Parameters Reading Error!");
	   ros::requestShutdown();
	 }
	 subcriber_ = nodeHandle_.subscribe("/scan", 10, &SmbHighlevelController::scanCallback, this);
	 //subcriber_ = nodeHandle_.subscribe(name_topic, size_queue_topic, &SmbHighlevelController::scanCallback, this);
	 // Point_sub = nodeHandle_.subscribe("/rslidar_points", 1, &SmbHighlevelController::pointcloudCallback, this);
	 Emergency_stop = nodeHandle_.serviceClient<std_srvs::SetBool>("/smb_service_controller/SMB_Switch", this);
	 ROS_INFO("Successfully Launched Node.");
	 Stop_SMB_Service.request.data = false;
}

SmbHighlevelController::~SmbHighlevelController()
{
}


void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
	// ROS_INFO_STREAM("TEST");

	std::vector<float> ranges = msg.ranges;
	float min_range = ranges[0];
	for (float m:ranges)
	{
		if (min_range > m)
			min_range = m;
	}
	ROS_INFO_STREAM("MINIMUN RANGE : " + std::to_string(min_range));
	Stop_SMB_Service.request.data = false;
	if (min_range <= 1.6){
		Emergency_stop.call(Stop_SMB_Service);
		if (Stop_SMB_Service.response.success)
		{
			ROS_INFO_STREAM(Stop_SMB_Service.response.message);
			ROS_INFO("COLLISION WARNING!!SMB STOPS SUCCESSFULLY!!");
		}
		else
		{
			ROS_INFO_STREAM(Stop_SMB_Service.response.message);
			ROS_INFO("COLLISION WARNINGd!!SMB FAILS IN STOP!!");
		}
	}
}


//void SmbHighlevelController::pointcloudCallback(const sensor_msgs::PointCloud2& msg)
//{
//	// ROS_INFO_STREAM("The number of point cloud: " + std::to_string(msg.height * msg.width) );
//}

} /* namespace */
