
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

#include <fourbythree_ros_controller/fourbythree_controller_manager.h>
# include <diagnostic_msgs/DiagnosticArray.h>

namespace fourbythree 
{

ControllerManager::ControllerManager ( const std::string& starting_controller )
{

  m_load_nodelet = m_nh.serviceClient<nodelet::NodeletLoad>("/fourbythree_nodelet/load_nodelet");
  m_unload_nodelet = m_nh.serviceClient<nodelet::NodeletUnload>("/fourbythree_nodelet/unload_nodelet");
  m_list_nodelet = m_nh.serviceClient<nodelet::NodeletList>("/fourbythree_nodelet/list");
  ros::NodeHandle nh;
  m_diagnostics_pub=nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  
  
  if (!m_nh.getParam("controllers/"+starting_controller,m_active_controller))
  {
    ROS_ERROR("SPECIFIED CONTROLLER DOES NOT EXIST, SET EMPTY");
    m_active_controller.clear();
  }
  m_active_controller.clear();
  controller_manager_msgs::SwitchController::Request req;
  controller_manager_msgs::SwitchController::Response res;
  
  req.start_controllers.resize(1);
  req.start_controllers.at(0)=starting_controller;
  req.strictness=1;
  callback(req,res);
}

bool ControllerManager::callback ( controller_manager_msgs::SwitchController::Request& req, controller_manager_msgs::SwitchController::Response& res )
{
 
  diagnostic_msgs::DiagnosticArray diag_msg;
  diag_msg.header.stamp=ros::Time::now();
  diag_msg.status.resize(1);
  diag_msg.status.at(0).name="CONTROLLER MANAGER";
  diag_msg.status.at(0).hardware_id="FourByThreeRobot";
  
  
  res.ok=false;
  if (req.start_controllers.size()==0)
  {
    diag_msg.status.at(0).message="NO STARTING CONTROLLERS SPECIFIED";
    diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
    m_diagnostics_pub.publish(diag_msg);
    
    ROS_ERROR("NO STARTING CONTROLLERS SPECIFIED");
    return false;
  }
  
  diag_msg.status.at(0).values.resize(1);
  diag_msg.status.at(0).values.at(0).key="New configuration";
  diag_msg.status.at(0).values.at(0).value=req.start_controllers.at(0).c_str();
  
  m_active_controller_name=req.start_controllers.at(0);
  if (req.start_controllers.size()>1)
  {
    diag_msg.status.at(0).message="MORE THEN ONE STARTING CONTROLLER SPECIFIED, USING THE FIRST";
    diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::WARN;
    m_diagnostics_pub.publish(diag_msg);
    ROS_ERROR("MORE THEN ONE STARTING CONTROLLER SPECIFIED, USING THE FIRST");
  }
  ROS_INFO("Starting controller %s", req.start_controllers.at(0).c_str());
  
  
  std::map<std::string,std::string> next_controller;
  if (!m_nh.getParam("controllers/"+req.start_controllers.at(0),next_controller ))
  {
    diag_msg.status.at(0).message="SPECIFIED CONTROLLER DOES NOT EXIST";
    diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
    m_diagnostics_pub.publish(diag_msg);
    ROS_ERROR("SPECIFIED CONTROLLER DOES NOT EXIST");
    return false;
  }
  
  
  for (auto const &entry: next_controller)
  {
    diagnostic_msgs::KeyValue key;
    key.key=entry.first;
    key.value=entry.second;
    diag_msg.status.at(0).values.push_back(key);
    
  }
  
  
  controller_manager_msgs::LoadController load_ctrl_srv;
  controller_manager_msgs::SwitchController switch_ctrl_srv;
  controller_manager_msgs::UnloadController unload_ctrl_srv;
  if (req.strictness==0)
    switch_ctrl_srv.request.strictness=1;
  else
    switch_ctrl_srv.request.strictness=req.strictness;
  
  nodelet::NodeletUnload unload_srv;
  nodelet::NodeletLoad load_srv;
  load_srv.request.type = "itia/control/JointStatesNodeletHwInterface";
  
  for (auto const &entry: m_active_controller)
  {
    if (!next_controller.count(entry.first) || req.strictness==0)
    {
      ROS_INFO("Unloading hardware interface: %s",entry.first.c_str());
      unload_srv.request.name = entry.first;
      if(!m_unload_nodelet.call(unload_srv))
      {
        diag_msg.status.at(0).message="Failed unloading hardware interface";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        m_diagnostics_pub.publish(diag_msg);
        ROS_ERROR("Failed unloading hardware interface");
        return false;
      }
    }
    else
      ROS_INFO("hardware interface %s will remain active",entry.first.c_str());

  }
  
  for (auto const &entry: next_controller)
  {
    if (!m_active_controller.count(entry.first) || req.strictness==0)
    {
      ROS_INFO("loading '%s' hardware_interface",entry.first.c_str());
      load_srv.request.name = entry.first;
      if (!m_load_nodelet.call(load_srv))
      {
        diag_msg.status.at(0).message="Failed loading hardware interface";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        m_diagnostics_pub.publish(diag_msg);
        ROS_ERROR("Failed loading hardware interface");
        return false;
      }
    }
    ros::ServiceClient load_ctrl = m_nh.serviceClient<controller_manager_msgs::LoadController>("/"+entry.first+"/controller_manager/load_controller");
    ros::ServiceClient unload_ctrl = m_nh.serviceClient<controller_manager_msgs::UnloadController>("/"+entry.first+"/controller_manager/unload_controller");
    ros::ServiceClient switch_ctrl = m_nh.serviceClient<controller_manager_msgs::SwitchController>("/"+entry.first+"/controller_manager/switch_controller");
    ROS_INFO("waiting for '%s' server",load_ctrl.getService().c_str());
    load_ctrl.waitForExistence();
    
    ROS_INFO("Starting controller '%s'",entry.second.c_str());
    if (!m_active_controller.count(entry.first))
    {
      load_ctrl_srv.request.name=entry.second;
      if (!load_ctrl.call(load_ctrl_srv))
      {
        diag_msg.status.at(0).message="Failed loading controller";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        m_diagnostics_pub.publish(diag_msg);
        
        ROS_ERROR("Failed loading controller");
        return false;
      }
      ROS_INFO("CONTROLLER LOADED!");
      switch_ctrl_srv.request.start_controllers.resize(1);
      switch_ctrl_srv.request.stop_controllers.resize(0);
      switch_ctrl_srv.request.start_controllers.at(0)=entry.second;
      if (!switch_ctrl.call(switch_ctrl_srv))
      {
        ROS_ERROR("Failed starting  controller");
        return false;
      }
      ROS_INFO("Controller loaded");
    }
    else
    {
      std::string active_controller_name=m_active_controller.at(entry.first);
      if (active_controller_name.compare(entry.second)) //IF IT IS DIFFERENT
      {
        load_ctrl_srv.request.name=entry.second;
        if (!load_ctrl.call(load_ctrl_srv))
        {
          diag_msg.status.at(0).message="Failed loading controller";
          diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
          m_diagnostics_pub.publish(diag_msg);
          ROS_ERROR("Failed loading controller");
          return false;
        }
        switch_ctrl_srv.request.start_controllers.resize(1);
        switch_ctrl_srv.request.stop_controllers.resize(1);
        switch_ctrl_srv.request.start_controllers.at(0)=entry.second;
        switch_ctrl_srv.request.stop_controllers.at(0)=m_active_controller.at(entry.first);
        if (!switch_ctrl.call(switch_ctrl_srv))
        {
          ROS_ERROR("Failed starting  controller");
          return false;
        }
        unload_ctrl_srv.request.name=active_controller_name;
        if (!unload_ctrl.call(unload_ctrl_srv))
        {
          diag_msg.status.at(0).message="Failed unloading previous controller";
          diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
          m_diagnostics_pub.publish(diag_msg);
          ROS_ERROR("Failed unloading previous controller");
          return false;
        }
      }
    }
    
  }
  m_active_controller=next_controller;
  res.ok=true;
  std::string follow_joint_trajectory_name="/planner_hi/fir_planner/follow_joint_trajectory";
  
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(follow_joint_trajectory_name,true);
  ROS_INFO("WAIT FOR ACTION SERVER");
  if (ac.waitForServer(ros::Duration(1)))
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    
    ROS_DEBUG("sending empty goal to resyncronize action server and client (needed in Indigo)");
    ac.sendGoalAndWait(goal);
  }
  else
    ROS_WARN("no action server ready, MoveIt! does not work with this configuration");
  
  diag_msg.status.at(0).message="Switching Controller Configuration";
  diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::OK;
  m_diagnostics_pub.publish(diag_msg);
  
  return true;
}


bool ControllerManager::activeListController(controller_manager_msgs::ListControllers::Request& req, controller_manager_msgs::ListControllers::Response& res)
{
  res.controller.resize(1);
  res.controller.at(0).name=m_active_controller_name;
  res.controller.at(0).state="running";
  res.controller.at(0).type="FourByThree Controller Configuration";
//   res.controller.at(0).claimed_resources.resize(1); //INDIGO HAS NOT THIS FIELD
  //   res.controller.at(0).claimed_resources.at(0).hardware_interface="FourByThree Robot";  //INDIGO HAS NOT THIS FIELD
  
  return true;
}

bool ControllerManager::listController(controller_manager_msgs::ListControllers::Request& req, controller_manager_msgs::ListControllers::Response& res)
{
  XmlRpc::XmlRpcValue controllers;
  if (!m_nh.getParam("controllers",controllers))
    return false;
  
  if ( controllers.getType() != XmlRpc::XmlRpcValue::TypeStruct )
  {
    ROS_ERROR ( "controllers list is not a struct");
    return false;
  }
  
  for (auto  it=controllers.begin();it!=controllers.end();++it )
  {
    controller_manager_msgs::ControllerState cs;
    cs.name=(std::string)(it->first);
    if (!m_active_controller_name.compare((std::string)(it->first)))
      cs.state="running";
    else
      cs.state="loaded";
    cs.type="FourByThree Controller Configuration";
    //     cs.claimed_resources.resize(1);  //INDIGO HAS NOT THIS FIELD
    //     cs.claimed_resources.at(0).hardware_interface="FourByThree Robot"; //INDIGO HAS NOT THIS FIELD
    res.controller.push_back(cs);
  }
  return true;
  
}

ControllerManager::~ControllerManager()
{
//   for (auto const &entry: m_active_controller)
//   {
//     nodelet::NodeletUnload unload_srv;
//     ROS_INFO("Unloading hardware interface: %s",entry.first.c_str());
//     unload_srv.request.name = entry.first;
//     if(!m_unload_nodelet.call(unload_srv))
//     {
//       ROS_ERROR("Failed unloading hardware interface");
//     }
//   }
  
}


}
