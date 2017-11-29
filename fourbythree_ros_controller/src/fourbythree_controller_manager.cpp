
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

#include <ros/ros.h>
#include <fourbythree_ros_controller/fourbythree_controller_manager.h>
#include <fourbythree_ros_controller/fourbythree_driver_manager.h>




int main(int argc, char **argv){
  ros::init(argc, argv, "fourbythree_hardware_interface_gui");
  ros::NodeHandle nh;

  
	ros::ServiceClient load_nodelet = nh.serviceClient<nodelet::NodeletLoad>("/fourbythree_nodelet/load_nodelet");
  nodelet::NodeletLoad load_srv;
  
  ROS_INFO("%sFourByThree Starting driver node%s",BOLDGREEN,RESET);
  ROS_INFO("Waiting for FourByThree Nodelet Manager...");
  load_nodelet.waitForExistence();
  ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  
  
  
  fourbythree::DriverManager dm;
  std_srvs::Empty::Request empty_req;
  std_srvs::Empty::Response empty_res;
  
  if (!dm.loadCallback(empty_req,empty_res))
    return 0;
  
  bool robot_status=false;
  if (!nh.getParam("/ndlcom/robot_status",robot_status))
    robot_status=true;
  if (!robot_status)
  {
    ROS_ERROR("ROBOT STATUS NOT OK, SHUTTING DOWN");
    dm.unloadCallback(empty_req,empty_res);
    return 0; 
  }
  
  bool load_force_monitor;
  if (!nh.getParam("load_force_monitor",load_force_monitor))
  {
    ROS_WARN("load_force_monitor flag NOT DEFINED, set true");
    load_force_monitor=true;
  }  
  
  if (load_force_monitor)
  {
    load_srv.request.name = "/force_monitor";
    load_srv.request.type = "itia/ForceMonitor";
    if (!load_nodelet.call(load_srv))
    {
      ROS_ERROR("Failed loading force monitor");
      return 0;
    }
    else 
    {
      ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
    }
  }
  
  ROS_INFO("Switch to controller 'motor'");
  fourbythree::ControllerManager cm("motor");
  
  if (!dm.enableCallback(empty_req,empty_res))
    return 0;
  
  if (!nh.getParam("/ndlcom/robot_status",robot_status))
    robot_status=true;
  if (!robot_status)
  {
    ROS_ERROR("ROBOT STATUS NOT OK, SHUTTING DOWN");
    dm.unloadCallback(empty_req,empty_res);
    return 0; 
  }
  
  std::string starting_controller="motor";
  if (!nh.getParam("/starting_controller",starting_controller))
  {
    ROS_WARN("'/starting_controller' parameter is not defined, using 'motor'");
  }
  
  ROS_INFO("Switch to controller '%s'", starting_controller.c_str());
  
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers.resize(1);
  srv.request.start_controllers.at(0)=starting_controller;
  srv.request.strictness=1;
  if (!cm.callback(srv.request,srv.response))
  {
    ROS_ERROR("Cannot switch to controller '%s'", starting_controller.c_str());
    dm.unloadCallback(empty_req,empty_res);
    return 0; 
  }
  
  ros::ServiceServer controller_manager_service=nh.advertiseService("/fourbythree/controller_manager/switch_controller",&fourbythree::ControllerManager::callback,&cm);
  ros::ServiceServer list_controller_service=nh.advertiseService("/fourbythree/controller_manager/list_controllers",&fourbythree::ControllerManager::listController,&cm);
  ros::ServiceServer list_active_controller_service=nh.advertiseService("/fourbythree/controller_manager/active_list_controllers",&fourbythree::ControllerManager::activeListController,&cm);
  
  ros::AsyncSpinner as(4);
  
  as.start();
  
  
  bool print_menu=true;
  while (ros::ok())
  {
    
    if (!nh.getParam("/ndlcom/robot_status",robot_status))
      robot_status=true;
    if (!robot_status)
    {
      ROS_ERROR("ROBOT STATUS NOT OK, SHUTTING DOWN");
      dm.unloadCallback(empty_req,empty_res);
      return 0; 
    }
    std::string gl;
    
    if (print_menu)
    {
      printf("\n\n%sFourbythree user interface\nManage the nodelets life cycle\n%s", BOLDWHITE, RESET);
      printf("\t%s'x'%s  to exit unloading everything\n", BOLDGREEN, RESET);
      printf("\t%s'u'%s  to unloading everything without closing\n", BOLDGREEN, RESET);
      printf("\t%s'l'%s  to loading everything without closing\n", BOLDGREEN, RESET);
      printf("\t%s's'%s  to store motor offsets\n", BOLDGREEN, RESET);
      printf("\t%sother%s  to reprint commands\n", GREEN, RESET);
      
      print_menu = false;
    }
    std::getline(std::cin, gl);
    
    if (!gl.compare("x"))
    {
      if (!dm.unloadCallback(empty_req,empty_res))
        return 0;
      break;
    }
    else if (!gl.compare("u"))
    {
      if (!dm.unloadCallback(empty_req,empty_res))
        return 0;
    }
    else if (!gl.compare("l"))
    {
      if (!dm.loadCallback(empty_req,empty_res))
        return 0;
      bool robot_status=false;
      if (!nh.getParam("/ndlcom/robot_status",robot_status))
        robot_status=true;
      if (!robot_status)
      {
        ROS_ERROR("ROBOT STATUS NOT OK, SHUTTING DOWN");
        dm.unloadCallback(empty_req,empty_res);
        return 0; 
      }
      fourbythree::ControllerManager cm("motor");
      
      if (!dm.enableCallback(empty_req,empty_res))
        return 0;
      
      bool load_force_monitor;
      if (!nh.getParam("load_force_monitor",load_force_monitor))
      {
        ROS_WARN("load_force_monitor flag NOT DEFINED, set true");
        load_force_monitor=true;
      }  
      
      if (load_force_monitor)
      {
        load_srv.request.name = "/force_monitor";
        load_srv.request.type = "itia/ForceMonitor";
        if (!load_nodelet.call(load_srv))
        {
          ROS_ERROR("Failed loading force monitor");
          return 0;
        }
        else 
        {
          ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
        }
      }
      
      std::string starting_controller="motor";
      if (!nh.getParam("/starting_controller",starting_controller))
      {
        ROS_WARN("'/starting_controller' parameter is not defined, using 'motor'");
      }
      
      ROS_INFO("Switch to controller '%s'", starting_controller.c_str());
      
      controller_manager_msgs::SwitchController srv;
      srv.request.start_controllers.resize(1);
      srv.request.start_controllers.at(0)=starting_controller;
      srv.request.strictness=1;
      if (!cm.callback(srv.request,srv.response))
      {
        ROS_ERROR("FAILED CONTROLLERS LOADING, SHUTTING DOWN");
        dm.unloadCallback(empty_req,empty_res);
        return 0; 
      }
    }
    else if (!gl.compare("s"))
    {
      if (nh.hasParam("/ndlcom/motor_offset"))
      {
        ROS_INFO("Storing motor offsets");
        system("rosparam dump ~/.ros/motor_offset.yaml /ndlcom/motor_offset");
      }
      else
        ROS_WARN("/ndlcom/motor_offset is not defined");
    }
    else
      print_menu = true;
    

  }
  
	return 0;
}
