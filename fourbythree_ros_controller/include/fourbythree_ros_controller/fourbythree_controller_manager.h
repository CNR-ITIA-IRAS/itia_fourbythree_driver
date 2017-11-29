
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