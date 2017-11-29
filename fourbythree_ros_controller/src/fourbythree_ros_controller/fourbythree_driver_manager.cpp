
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

#include <fourbythree_ros_controller/fourbythree_driver_manager.h>


namespace fourbythree
{

DriverManager::DriverManager()
{
 
  m_diagnostics_pub=m_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
}
DriverManager::~DriverManager()
{
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;
  unloadCallback(req,res);
}

bool DriverManager::loadCallback  (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ros::ServiceClient load_nodelet = m_nh.serviceClient<nodelet::NodeletLoad>("/fourbythree_nodelet/load_nodelet");
  ros::ServiceClient unload_nodelet = m_nh.serviceClient<nodelet::NodeletUnload>("/fourbythree_nodelet/unload_nodelet");
  ros::ServiceClient list_nodelet = m_nh.serviceClient<nodelet::NodeletList>("/fourbythree_nodelet/list");
  ros::ServiceClient set_zeros = m_nh.serviceClient<std_srvs::Trigger>("com/set_zero");
  ros::ServiceClient enable_joints = m_nh.serviceClient<std_srvs::Trigger>("com/enable_joints");
  ros::ServiceClient disable_joints = m_nh.serviceClient<std_srvs::Trigger>("com/disable_joints");
  
  

  
  nodelet::NodeletLoad load_srv;
  nodelet::NodeletUnload unload_srv;
  nodelet::NodeletList list;
  
  ROS_INFO("%sUNLOADING ALL EXISTING NODELETS%s",BOLDGREEN,RESET);
  list_nodelet.call(list);
  for (unsigned int idx = 0;idx< list.response.nodelets.size();idx++)
  {
    unload_srv.request.name = list.response.nodelets.at(idx);
    unload_nodelet.call(unload_srv);
  }
  ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  
  bool load_ndlcom;
  if (!m_nh.getParam("load_ndlcom",load_ndlcom))
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
      return false;
    }
    else 
    {
      ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
    }
  }

  
  return true;
}

bool DriverManager::enableCallback  (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ros::ServiceClient load_nodelet = m_nh.serviceClient<nodelet::NodeletLoad>("/fourbythree_nodelet/load_nodelet");
  ros::ServiceClient unload_nodelet = m_nh.serviceClient<nodelet::NodeletUnload>("/fourbythree_nodelet/unload_nodelet");
  ros::ServiceClient list_nodelet = m_nh.serviceClient<nodelet::NodeletList>("/fourbythree_nodelet/list");
  ros::ServiceClient set_zeros = m_nh.serviceClient<std_srvs::Trigger>("com/set_zero");
  ros::ServiceClient enable_joints = m_nh.serviceClient<std_srvs::Trigger>("com/enable_joints");
  ros::ServiceClient disable_joints = m_nh.serviceClient<std_srvs::Trigger>("com/disable_joints");
  

  system("rosparam load ~/.ros/motor_offset.yaml /ndlcom/motor_offset");
  bool virtual_sensor_initialization_procedure=true;
  if (m_nh.hasParam("/ndlcom/motor_offset"))
  {
    virtual_sensor_initialization_procedure=false;
    ROS_INFO("Using stored motor offset");
  }
  
  bool enable_motor;
  if (!m_nh.getParam("enable_motor",enable_motor))
  {
    ROS_WARN("enable_motor flag NOT DEFINED, set false");
    enable_motor=false;
  }
  if (!enable_motor)
  {
    double configuration_time;
    if (!m_nh.getParam("configuration_time",configuration_time))
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
    
    return false;
  }
  
  
  bool load_ndlcom;
  if (!m_nh.getParam("load_ndlcom",load_ndlcom))
  {
    ROS_WARN("load_ndlcom flag NOT DEFINED, set false");
    load_ndlcom=false;
  }
  
  if (load_ndlcom)
 {
    std_srvs::Trigger srv;
    if (!enable_joints.call(srv))
    {
      ROS_ERROR("Fail during joints enabling");
      return false;
    }
    double configuration_time;
    if (!m_nh.getParam("configuration_time",configuration_time))
    {
      ROS_WARN("configuration_time NOT DEFINED, set equal to 4");
      configuration_time=4;
    }
    
    if (virtual_sensor_initialization_procedure)
    {
      if (m_nh.getParam("virtual_sensor_initialization_procedure",virtual_sensor_initialization_procedure))
        if (virtual_sensor_initialization_procedure)
        {
          ROS_INFO("Waiting %3.2f seconds during virtual_sensor_initialization_procedure, do not touch the robot",configuration_time);
          set_zeros.call(srv);
          ros::WallDuration(configuration_time).sleep();
          ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
        }
    }
  }
  
  return true;
}

bool DriverManager::unloadCallback (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  
  ros::ServiceClient load_nodelet = m_nh.serviceClient<nodelet::NodeletLoad>("/fourbythree_nodelet/load_nodelet");
  ros::ServiceClient unload_nodelet = m_nh.serviceClient<nodelet::NodeletUnload>("/fourbythree_nodelet/unload_nodelet");
  ros::ServiceClient list_nodelet = m_nh.serviceClient<nodelet::NodeletList>("/fourbythree_nodelet/list");
  
  nodelet::NodeletLoad load_srv;
  nodelet::NodeletUnload unload_srv;
  nodelet::NodeletList list;
  
  ROS_INFO("%sUNLOADING ALL EXISTING NODELETS%s",BOLDGREEN,RESET);
  list_nodelet.call(list);
  for (unsigned int idx = 0;idx< list.response.nodelets.size();idx++)
  {
    unload_srv.request.name = list.response.nodelets.at(idx);
    unload_nodelet.call(unload_srv);
  }
  ROS_INFO("%sDONE%s",BOLDBLUE,RESET);
  
  return true;
}

}