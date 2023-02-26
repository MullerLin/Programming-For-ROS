#include <ros/ros.h>
#include "smb_service_controller/SmbServiceController.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "smb_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  smb_highlevel_controller::SmbHighlevelController smbHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}
