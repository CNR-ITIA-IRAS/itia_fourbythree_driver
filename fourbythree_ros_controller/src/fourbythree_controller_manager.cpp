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
