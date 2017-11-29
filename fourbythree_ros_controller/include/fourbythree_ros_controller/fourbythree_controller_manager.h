#ifndef __FOURBYTHREE_CONTROLLER_MANAGER__
#define __FOURBYTHREE_CONTROLLER_MANAGER__
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
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
# include <diagnostic_msgs/DiagnosticArray.h>
namespace fourbythree
{
  
class ControllerManager
{
private:
  ros::NodeHandle m_nh;
  
  std::map<std::string,std::string> m_active_controller;
  ros::ServiceClient m_load_nodelet;
  ros::ServiceClient m_unload_nodelet;
  ros::ServiceClient m_list_nodelet;
  ros::Publisher m_diagnostics_pub;
  
  std::string m_active_controller_name;
public:
  
  ControllerManager( const std::string& starting_controller );
  ~ControllerManager();
  bool callback(controller_manager_msgs::SwitchController::Request& req, controller_manager_msgs::SwitchController::Response& res);
  bool activeListController(controller_manager_msgs::ListControllers::Request& req, controller_manager_msgs::ListControllers::Response& res);
  bool listController(controller_manager_msgs::ListControllers::Request& req, controller_manager_msgs::ListControllers::Response& res);
  

};
}


#endif