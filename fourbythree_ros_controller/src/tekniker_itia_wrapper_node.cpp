
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
