#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <std_srvs/Trigger.h>
#include <itia_futils/itia_futils.h>
#include <boost/graph/graph_concepts.hpp>
#include <itia_rutils/itia_rutils.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int64.h>

Eigen::Affine3d Tft;
Eigen::Affine3d Tub;

bool configureTool(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ros::NodeHandle nh;
  ros::ServiceClient virtual_sensor_tool=nh.serviceClient<std_srvs::Empty>("/virtual_sensor_hi/virtual_sensor/tool_configured");
  ros::ServiceClient gui_tool=nh.serviceClient<std_srvs::Empty>("/planner_hi/cart_teleop_planner/tool_configured");
  
  
  ROS_INFO("Waiting 0.1 seconds for check existence of '/virtual_sensor_hi/virtual_sensor/tool_configured'");
  std_srvs::Empty srv;
  if (virtual_sensor_tool.waitForExistence(ros::Duration(0.1)))
  {
    ROS_INFO("resetting virtual force sensor tool");
    virtual_sensor_tool.call(srv);
  }
  if (gui_tool.waitForExistence(ros::Duration(0.1)))
  {
    ROS_INFO("resetting cartesian teleop tool");
    gui_tool.call(srv);
  }
  
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "fourbythree_tekniker_itia_wrapper");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner async(4);
  async.start();
  ros::Rate lp(100);
  
  ros::ServiceServer change_tool = nh.advertiseService("/tool_configured",configureTool);
  ros::ServiceServer change_tool2 = nh.advertiseService("/parameter_changed",configureTool);
  ros::Publisher ovr_pub=nh.advertise<std_msgs::Int64>("/speed_ovr",10);
  
  std_msgs::Int64 ovr_msg;
  double ovr=100;
  ovr_msg.data=ovr;
  
  
  
  while (ros::ok())
  {
    if (nh.getParam("/env_param/OVERRIDE",ovr))
    {
      ovr_msg.data=ovr;
      ovr_pub.publish(ovr_msg);
    }
    lp.sleep();
  }
  
  return 0;
}
