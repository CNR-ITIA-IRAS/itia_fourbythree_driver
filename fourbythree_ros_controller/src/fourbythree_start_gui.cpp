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

int main(int argc, char **argv){
  ros::init(argc, argv, "fourbythree_hardware_interface_gui");
  ros::NodeHandle nh;

  
	ros::ServiceClient load_nodelet = nh.serviceClient<nodelet::NodeletLoad>("/fourbythree_nodelet/load_nodelet");
	ros::ServiceClient unload_nodelet = nh.serviceClient<nodelet::NodeletUnload>("/fourbythree_nodelet/unload_nodelet");
	ros::ServiceClient list_nodelet = nh.serviceClient<nodelet::NodeletList>("/fourbythree_nodelet/list");
	ros::ServiceClient set_zeros = nh.serviceClient<std_srvs::Trigger>("com/set_zero");
  ros::ServiceClient enable_joints = nh.serviceClient<std_srvs::Trigger>("com/enable_joints");
  ros::ServiceClient disable_joints = nh.serviceClient<std_srvs::Trigger>("com/disable_joints");
  
  ROS_INFO("%sFourByThree Starting driver node%s",BOLDGREEN,RESET);
  
  ROS_INFO("Waiting for FourByThree Nodelet Manager");
  load_nodelet.waitForExistence();
  ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  
  nodelet::NodeletLoad load_srv;
  nodelet::NodeletUnload unload_srv;
  nodelet::NodeletList list;
  
  controller_manager_msgs::LoadController load_ctrl_srv;
  controller_manager_msgs::SwitchController switch_ctrl_srv;
  controller_manager_msgs::UnloadController unload_ctrl_srv;
  controller_manager_msgs::ListControllers list_ctrl_srv;
  switch_ctrl_srv.request.start_controllers.resize(1);
  switch_ctrl_srv.request.stop_controllers.resize(1);
  switch_ctrl_srv.request.strictness=1;
  
  ROS_INFO("%sUNLOADING ALL EXISTING NODELETS%s",BOLDGREEN,RESET);
  list_nodelet.call(list);
  for (unsigned int idx = 0;idx< list.response.nodelets.size();idx++)
  {
    unload_srv.request.name = list.response.nodelets.at(idx);
    unload_nodelet.call(unload_srv);
  }
  ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  
  
  std::vector<std::string> load_hi;
  std::vector<std::string> unload_hi;
  std::vector<std::string> load_controllers;
  
  if (!nh.getParam("starting_hi",load_hi))
  {
    ROS_ERROR("NO HARDWARE INTERFACES TO LOAD");
    return 0;
  }
  if (!nh.getParam("starting_controllers",load_controllers))
  {
    ROS_ERROR("NO CONTROLLERS TO LOAD");
    return 0;
  }
  
  for (unsigned int iHi=0;iHi<load_hi.size();iHi++)
  {
    
    ROS_INFO("%sLoading '%s' hardware interface%s",BOLDGREEN,load_hi.at(iHi).c_str(),RESET);
    load_srv.request.name = load_hi.at(iHi);
    load_srv.request.type = "itia/control/JointStatesNodeletHwInterface";
    if (!load_nodelet.call(load_srv))
    {
      ROS_ERROR("Failed loading hardware interface");
      return -1;
    }
    
    ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  }    
  
  for (unsigned int iCtrl=0; iCtrl<std::min(load_controllers.size(),load_hi.size()); iCtrl++)
  {
    ROS_INFO("%sLoading '%s' controller in '%s' hardware interface%s",BOLDGREEN,load_controllers.at(iCtrl).c_str(), load_hi.at(iCtrl).c_str(),RESET);
    ros::ServiceClient load_ctrl = nh.serviceClient<controller_manager_msgs::LoadController>("/"+load_hi.at(iCtrl)+"/controller_manager/load_controller");
    ROS_INFO("Waiting for controller interface");
    load_ctrl.waitForExistence();
    ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
    load_ctrl_srv.request.name=load_controllers.at(iCtrl);
    if (!load_ctrl.call(load_ctrl_srv))
    {
      ROS_ERROR("Failed during controller loading ");
      return -1;
    }
  }
  
  bool load_ndlcom;
  if (!nh.getParam("load_ndlcom",load_ndlcom))
  {
    ROS_WARN("load_ndlcom flag NOT DEFINED, set false");
    load_ndlcom=false;
  }
  if (load_ndlcom)
  {
    ROS_INFO("%sLoading NDLCOM driver%s",BOLDGREEN,RESET);
    load_srv.request.name = "/driver";
    load_srv.request.type = "ndlcom_ros/NdlcomDriverNodelet";
    if (!load_nodelet.call(load_srv))
    {
      ROS_ERROR("Failed loading NDLCOM driver, check the connection and press 'ld' to start the driver again");
      ROS_ERROR("If the problem persists, switch off and on again the robot power supply");
      return 0;
    }
    else 
    {
      ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
    }
  }
  
  std::vector<ros::ServiceClient> switch_ctrl_(std::min(load_controllers.size(),load_hi.size()));
  
  for (unsigned int iCtrl=0; iCtrl<std::min(load_controllers.size(),load_hi.size()); iCtrl++)
  {
    ROS_INFO("%sstarting '%s' controller in '%s' hardware interface%s",BOLDGREEN,load_controllers.at(iCtrl).c_str(), load_hi.at(iCtrl).c_str(),RESET);
    ros::ServiceClient switch_ctrl = nh.serviceClient<controller_manager_msgs::SwitchController>("/"+load_hi.at(iCtrl)+"/controller_manager/switch_controller");
    switch_ctrl_.at(iCtrl)=switch_ctrl;
    switch_ctrl_srv.request.start_controllers.at(0)=load_controllers.at(iCtrl);
    switch_ctrl_srv.request.stop_controllers.at(0)="";
    if (!switch_ctrl_.at(iCtrl).call(switch_ctrl_srv))
    {
      ROS_ERROR("Failed starting controller");
      
      return -1;
    }
  }
  
  
  bool enable_motor;
  if (!nh.getParam("enable_motor",enable_motor))
  {
    ROS_WARN("enable_motor flag NOT DEFINED, set false");
    enable_motor=false;
  }
  if (!enable_motor)
  {
    double configuration_time;
    if (!nh.getParam("configuration_time",configuration_time))
    {
      ROS_WARN("configuration_time NOT DEFINED, set equal to 4");
      configuration_time=4;
    }
    ROS_INFO("Waiting %3.2f seconds",configuration_time);
    ros::WallDuration(configuration_time).sleep();
    
    nodelet::NodeletList list;
    list_nodelet.call(list);
    for (unsigned int idx = 0;idx< list.response.nodelets.size();idx++)
    {
      nodelet::NodeletUnload srv;
      srv.request.name = list.response.nodelets.at(idx);
      unload_nodelet.call(srv);
    }
    
    
    return 0;
  }
  
  if (load_ndlcom)
  {
    std_srvs::Trigger srv;
    if (!enable_joints.call(srv))
    {
      ROS_ERROR("Fail during joints enabling");
      return 0;
    }
    double configuration_time;
    if (!nh.getParam("configuration_time",configuration_time))
    {
      ROS_WARN("configuration_time NOT DEFINED, set equal to 4");
      configuration_time=4;
    }
    
    ros::WallDuration(1.0).sleep();
    ROS_INFO("Waiting %3.2f seconds before resetting the spring values, do not touch the robot",configuration_time);
    
    ros::WallTime t0=ros::WallTime::now();
    set_zeros.call(srv);
    ros::WallDuration(configuration_time).sleep();
    
    ROS_INFO("Waiting %3.2f [ms] for complete restart",(ros::WallTime::now()-t0).toSec()*1000);

    ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  }
  
  
  
  ROS_INFO("Unload hardware interfaces after robot configuration");
  if (!nh.getParam("after_configuration_unload",unload_hi))
  {
    ROS_WARN("NO HARDWARE INTERFACES TO UNLOAD AFTER CONFIGURATION");
    unload_hi.clear();
  }
  for (unsigned int iHi=0;iHi<unload_hi.size();iHi++)
  {
    
    ROS_INFO("%sUnloading '%s' hardware interface%s",BOLDGREEN,unload_hi.at(iHi).c_str(),RESET);
    unload_srv.request.name = unload_hi.at(iHi);
    if (!unload_nodelet.call(unload_srv))
    {
      ROS_ERROR("Failed loading hardware interface");
      return -1;
    }
    
    ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  }    
  
  ROS_INFO("Load hardware interfaces after robot configuration");
  if (!nh.getParam("after_configuration_hi",load_hi))
  {
    ROS_WARN("NO HARDWARE INTERFACES TO LOAD AFTER CONFIGURATION");
    load_hi.clear();
  }
  if (!nh.getParam("after_configuration_controllers",load_controllers))
  {
    ROS_WARN("NO CONTROLLERS TO LOAD AFTER CONFIGURATION");
    load_controllers.clear();
  }
  
  for (unsigned int iHi=0;iHi<load_hi.size();iHi++)
  {
    
    ROS_INFO("%sLoading '%s' hardware interface%s",BOLDGREEN,load_hi.at(iHi).c_str(),RESET);
    load_srv.request.name = load_hi.at(iHi);
    load_srv.request.type = "itia/control/JointStatesNodeletHwInterface";
    if (!load_nodelet.call(load_srv))
    {
      ROS_ERROR("Failed loading hardware interface");
      return -1;
    }
    
    ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  }    
  
  for (unsigned int iCtrl=0; iCtrl<std::min(load_controllers.size(),load_hi.size()); iCtrl++)
  {
    ROS_INFO("%sLoading '%s' controller in '%s' hardware interface%s",BOLDGREEN,load_controllers.at(iCtrl).c_str(), load_hi.at(iCtrl).c_str(),RESET);
    ros::ServiceClient load_ctrl = nh.serviceClient<controller_manager_msgs::LoadController>("/"+load_hi.at(iCtrl)+"/controller_manager/load_controller");
    ros::ServiceClient switch_ctrl = nh.serviceClient<controller_manager_msgs::SwitchController>("/"+load_hi.at(iCtrl)+"/controller_manager/switch_controller");
    ROS_INFO("Waiting for controller interface");
    load_ctrl.waitForExistence();
    ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
    load_ctrl_srv.request.name=load_controllers.at(iCtrl);
    if (!load_ctrl.call(load_ctrl_srv))
    {
      ROS_ERROR("Failed controller");
      return -1;
    }
    switch_ctrl_srv.request.start_controllers.at(0)=load_controllers.at(iCtrl);
    switch_ctrl_srv.request.stop_controllers.at(0)="";
    if (!switch_ctrl.call(switch_ctrl_srv))
    {
      ROS_ERROR("Failed starting controller");
      return -1;
    }
  }
  
  bool stop_immediately;
  if (!nh.getParam("stop_immediately",stop_immediately))
  {
    ROS_WARN("stop_immediately flag NOT DEFINED, set true");
    stop_immediately=true;
  }
  ROS_INFO("press '%sx%s' to SWITCH OFF THE ROBOT",BOLDGREEN,RESET);
  
  
	std::string gl;
	bool print_menu = true;
	while (ros::ok())
	{
		
		if (print_menu)
		{
			printf("\n\n%sFourbythree user interface\nManage the nodelets life cycle\n%s", BOLDWHITE, RESET);
			printf("\n%sDRIVER%s:\n", GREEN, RESET);
			printf("\t%s'rd'%s to reload deflection\n", BOLDGREEN, RESET);
			printf("\t%s'rl'%s to reload link cascade controller\n", BOLDGREEN, RESET);
      printf("\t%s'rm'%s to reload motor cascade controller\n", BOLDGREEN, RESET);
      
			printf("\n%sUTILS%s:\n", GREEN, RESET);
			printf("\t%s'l'%s  to print the list of the running nodelets\n", BOLDGREEN, RESET);
			printf("\t%s'x'%s  to exit unloading everything\n", BOLDGREEN, RESET);
			printf("\t%sother%s  to reprint commands\n", GREEN, RESET);
			
			print_menu = false;
		}
		if (stop_immediately)
      gl="x";
    else 
      std::getline(std::cin, gl);

    bool reload=false;
    std::string hi;
    std::string controller;
    if (!gl.compare("rl"))
		{
      reload=true;
      hi="high_controller_hi";
      controller="cascade_control";
		}
		else if (!gl.compare("rd"))
    {
      reload=true;
      hi="low_controller_hi";
      controller="deflection_control";
    }
		else if (!gl.compare("rm"))
    {
      reload=true;
      hi="motor_controller_hi";
      controller="motor_cascade_control";
    }
    else if (!gl.compare("l"))
		{
			nodelet::NodeletList list;
			printf("%sRunnning nodelets:%s\n", YELLOW, RESET);
			list_nodelet.call(list);
			if (list.response.nodelets.size() == 0)
				printf(" %snone%s\n", BOLDYELLOW, RESET);
			for (unsigned int idx = 0;idx< list.response.nodelets.size();idx++)
			{
				nodelet::NodeletUnload srv;
				printf(" - %s%s%s\n", BOLDYELLOW, list.response.nodelets.at(idx).c_str(), RESET);
			}
		}
		else if (!gl.compare("x"))
		{
			nodelet::NodeletList list;
			list_nodelet.call(list);
			for (unsigned int idx = 0;idx< list.response.nodelets.size();idx++)
			{
				nodelet::NodeletUnload srv;
				srv.request.name = list.response.nodelets.at(idx);
				unload_nodelet.call(srv);
			}
			break;
		}
		else
			print_menu = true;
    
    if (reload)
    {
      ros::ServiceClient unload_ctrl = nh.serviceClient<controller_manager_msgs::UnloadController>("/"+hi+"/controller_manager/unload_controller");
      ros::ServiceClient load_ctrl = nh.serviceClient<controller_manager_msgs::LoadController>("/"+hi+"/controller_manager/load_controller");
      ros::ServiceClient switch_ctrl = nh.serviceClient<controller_manager_msgs::SwitchController>("/"+hi+"/controller_manager/switch_controller");
      
      switch_ctrl_srv.request.stop_controllers.resize(1);
      switch_ctrl_srv.request.start_controllers.resize(0);
      switch_ctrl_srv.request.strictness=1;
      switch_ctrl_srv.request.stop_controllers.at(0)=controller;
      switch_ctrl.call(switch_ctrl_srv);
      
      unload_ctrl_srv.request.name=controller;
      unload_ctrl.call(unload_ctrl_srv);
      
      load_ctrl_srv.request.name=controller;
      load_ctrl.call(load_ctrl_srv);
      
      switch_ctrl_srv.request.stop_controllers.resize(0);
      switch_ctrl_srv.request.start_controllers.resize(1);
      switch_ctrl_srv.request.strictness=1;
      switch_ctrl_srv.request.start_controllers.at(0)=controller;
      switch_ctrl.call(switch_ctrl_srv);
      reload=false;
    }
	}
	return 0;
}