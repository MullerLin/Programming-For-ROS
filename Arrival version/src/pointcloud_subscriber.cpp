//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
//
//// 接收到订阅的消息后，会进入消息回调函数
//void pointCallback(const sensor_msgs::PointCloud2& msg)
//{
//    // 将接收到的消息打印出来
//    ROS_INFO_STREAM("The number of point cloud: " + std::to_string(msg.height * msg.width) );
//}
//
//int main(int argc, char **argv)
//{
//
//    // 初始化ROS节点
//    ros::init(argc, argv, "pointcloud_subscriber");
//
//    // 创建节点句柄
//    ros::NodeHandle n;
//
//    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
//    ros::Subscriber posintcloud_sub = n.subscribe("rslidar_points", 1, pointCallback);
//
//    // 循环等待回调函数
//    ros::spin();
//
//    return 0;
//}
