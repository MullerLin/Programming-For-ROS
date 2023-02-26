#include "smb_service_controller/SmbServiceController.hpp"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>


namespace smb_highlevel_controller {


SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle):
  nodeHandle_(nodeHandle)
{
	//if (!(nodeHandle_.getParam("param_topic_name",name_topic))& nodeHandle_.getParam("param_topic_queue_size",size_queue_topic))
	 if (!(nodeHandle_.getParam("param_topic_subcribe/name",name_topic))
			& nodeHandle_.getParam("param_topic_subcribe/queue_size", size_queue_topic)
			& nodeHandle_.getParam("Gain/linear",linear_gain)
			& nodeHandle_.getParam("Gain/angular",angular_gain)	 )

	 {
	   ROS_ERROR("Parameters Reading Error!");
	   ros::requestShutdown();
	 }
	 is_on = false;
	 subcriber_ = nodeHandle_.subscribe(name_topic, size_queue_topic, &SmbHighlevelController::scanCallback, this);
	 Point_sub = nodeHandle_.subscribe("/rslidar_points", 1, &SmbHighlevelController::pointcloudCallback, this);
	 vis_pub = nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	 Vel_pub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	 smb_switch = nodeHandle_.advertiseService("SMB_Switch", &SmbHighlevelController::SwitchCallback, this);
	 ROS_INFO("Successfully Launched Node.");
}

SmbHighlevelController::~SmbHighlevelController()
{
}


void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{

	//ROS_INFO_STREAM("TEST");

	std::vector<float> ranges = msg.ranges;
	std::vector<int> detect;
	std::vector<float> detect_ranges;
	std::vector<std::vector<float>> detect_positions;
	for(int i = 0; i <= ranges.size(); i++)
	{
		if( (ranges[i]<msg.range_max) & (ranges[i]>msg.range_min))
		{
			detect.push_back(i);
			detect_ranges.push_back(ranges[i]);
			float x; float y;
		    x = (ranges[i] + msg.range_min) * cos(msg.angle_min + i * msg.angle_increment);
		    y = (ranges[i] + msg.range_min) * sin(msg.angle_min + i * msg.angle_increment);
//		    ROS_INFO_STREAM( "x_pos: " + std::to_string(x) + " y_pos: " + std::to_string(y));
		    std::vector<float> pos_vector;
		    pos_vector.push_back(x);
		    pos_vector.push_back(y);
		    detect_positions.push_back(pos_vector);
//		    float pos[2]={x,y};
//		    Marker_publish(pos);
		}
	}
	float pos[2]={0,0};
	for(std::vector<float> v:detect_positions)
	{
		pos[0] = pos[0] + v[0];
		pos[1] = pos[1] + v[1];
//		ROS_INFO_STREAM( "x_pos: " + std::to_string(v[0]) + " y_pos: " + std::to_string(v[1]));
	}
//	ROS_INFO_STREAM( "x_pos_SUM: " + std::to_string(pos[0]) + " y_pos_SUM: " + std::to_string(pos[1]));
	pos[0] = pos[0] / detect_positions.size();
	pos[1] = pos[1] / detect_positions.size();
	Marker_publish(pos);
	ROS_INFO_STREAM( "Nearest Target_x: " + std::to_string(pos[0]) + " Nearest Target_y: " + std::to_string(pos[1]));
	Motion_Controller(pos);
//	ROS_INFO_STREAM("SIZE : " + std::to_string(detect_positions.size()));
}

void SmbHighlevelController::pointcloudCallback(const sensor_msgs::PointCloud2& msg)
{
	//ROS_INFO_STREAM("The number of point cloud: " + std::to_string(msg.height * msg.width) );
}

void SmbHighlevelController::Marker_publish(const float position[])
{
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position[0];
	marker.pose.position.y = position[1];
	// ROS_INFO_STREAM( "x_dposition: " + std::to_string(marker.pose.position.x ) + " y_dposition: " + std::to_string(marker.pose.position.y));
	marker.pose.position.z = 0.434;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub.publish( marker );
}

void SmbHighlevelController::Motion_Controller(const float position[])
{
	geometry_msgs::Twist smb_vel_msg;

	float dis_x = position[0];
	float dis_y = position[1];
	//ROS_INFO_STREAM( "x_dposition: " + std::to_string(dis_x) + " y_dposition: " + std::to_string(dis_y));
	float distance = dis_x *dis_x + dis_y*dis_y;
	if (distance <= 0.4)
	{
		smb_vel_msg.linear.x = 0;
		smb_vel_msg.angular.z = 0;
		ROS_INFO_STREAM("--ARRIVED!!--");
	}
	else
	{
		float angle_error =  atan(dis_y/dis_x);
		float dis_eror = sqrt(dis_x *dis_x + dis_y*dis_y);
		//ROS_INFO("SMB ERROR at [%0.2f m , %0.2f rad]", dis_eror, angle_error);

		if (abs(angle_error) >= 0.018)
		{
			smb_vel_msg.linear.x = 0;
			smb_vel_msg.angular.z = angular_gain * angle_error;
		}
		else
		{
			smb_vel_msg.angular.z = 0;
			smb_vel_msg.linear.x = linear_gain * dis_eror;
		}
		if(is_on == true)
		{
			Vel_pub.publish(smb_vel_msg);
			ROS_INFO_STREAM("--On The Way!!--");
			ROS_INFO("SMB velocity at [%0.2f m/s , %0.2f rad/s]",smb_vel_msg.linear.x, smb_vel_msg.angular.z);
		}
		else
		{
			smb_vel_msg.angular.z = 0;
			smb_vel_msg.linear.x = 0;
			Vel_pub.publish(smb_vel_msg);
			ROS_INFO_STREAM("-!-SMB stop-!-");
		}

	}
}

} /* namespace */
bool smb_highlevel_controller::SmbHighlevelController::SwitchCallback(std_srvs::SetBool::Request &request,
																	  std_srvs::SetBool::Response &response)
{

	bool input = request.data;
	if (input == true)
	{
		is_on = true;
		response.message = "SMB move!";
		ROS_INFO("!! SMB MOVE !!");
	}
	else
	{
		is_on = false;
		response.message = "SMB stop!";
		ROS_INFO("!! SMB STOP !!");
	}

return true;
}
