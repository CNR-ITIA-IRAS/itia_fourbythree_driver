
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

#include <ndlcom_driver/ndlcom_nodelet_fake.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ndlcom_ros::NdlcomDriverNodeletFake, nodelet::Nodelet) 


namespace ndlcom_ros{

void NdlcomDriverNodeletFake::onInit()
{
  ROS_INFO("NDLCOM FAKE CONTROLLER");
  m_main_thread = std::thread(&ndlcom_ros::NdlcomDriverNodeletFake::main_thread, this);

};

NdlcomDriverNodeletFake::~NdlcomDriverNodeletFake()
{
  ROS_INFO("shutting down...");
  m_stop_mtx.lock();
  m_stop = true;
  m_stop_mtx.unlock();
  if (m_main_thread.joinable())
    m_main_thread.join();
  ROS_INFO("shutted down...");
  m_stop = false;
};

void ndlcom_ros::NdlcomDriverNodeletFake::main_thread()
{
  ros::NodeHandle m_nh = getNodeHandle();

  ros::ServiceServer err_srv_server     = m_nh.advertiseService("com/error_reset",    &ndlcom_ros::NdlcomDriverNodeletFake::triggerCallback,this);
  ros::ServiceServer enable_srv_server  = m_nh.advertiseService("com/enable_joints",  &ndlcom_ros::NdlcomDriverNodeletFake::triggerCallback,this);
  ros::ServiceServer disable_srv_server = m_nh.advertiseService("com/disable_joints", &ndlcom_ros::NdlcomDriverNodeletFake::triggerCallback,this);
  ros::ServiceServer reset_srv_server   = m_nh.advertiseService("com/set_zero",       &ndlcom_ros::NdlcomDriverNodeletFake::triggerCallback,this);
  
  
  m_js_pub          = m_nh.advertise<sensor_msgs::JointState>("fb/joint_states",1);
  m_missing_data    = m_nh.advertise<std_msgs::Bool>("missing_data",1);
  m_js_red_pub      = m_nh.advertise<sensor_msgs::JointState>("fb/joint_states_reduced_motor",1);
  m_js_redlink_pub  = m_nh.advertise<sensor_msgs::JointState>("rigid/joint_states",1);
  m_ji_pub          = m_nh.advertise<itia_msgs::JointInfo>("fb/temperatures",1);

  m_jt_sub          = m_nh.subscribe("sp/joint_states",1,&ndlcom_ros::NdlcomDriverNodeletFake::callbackJointTarget,this);

  m_ji_msg.reset(new itia_msgs::JointInfo);
  m_js_msg.reset(new sensor_msgs::JointState);
  m_js_red_msg.reset(new sensor_msgs::JointState);
  m_js_redlink_msg.reset(new sensor_msgs::JointState);

  
  std::string par_name="ndlcom/nJoints";
  if (!m_nh.getParam(par_name,m_nJoints))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }
  ROS_DEBUG("[NdlcomDriver::NdlcomDriver] Loading %d joints..",m_nJoints);
  
  m_js_msg->name.resize(4*m_nJoints);
  m_js_msg->position.resize(4*m_nJoints);
  m_js_msg->velocity.resize(4*m_nJoints);
  m_js_msg->effort.resize(4*m_nJoints);

  m_js_red_msg->name.resize(m_nJoints);
  m_js_red_msg->position.resize(m_nJoints);
  m_js_red_msg->velocity.resize(m_nJoints);
  m_js_red_msg->effort.resize(m_nJoints);

  m_js_redlink_msg->name.resize(m_nJoints);
  m_js_redlink_msg->position.resize(m_nJoints);
  m_js_redlink_msg->velocity.resize(m_nJoints);
  m_js_redlink_msg->effort.resize(m_nJoints);

  m_ji_msg->name.resize(m_nJoints);
  m_ji_msg->data.resize(m_nJoints);
  for (int iAx=0;iAx<m_nJoints;iAx++)
  {
    std::string name_string;
    par_name="ndlcom/joint"+std::to_string(iAx+1)+"/name";
    if (!m_nh.getParam(par_name,name_string))
    {
      ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
      throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
    }
    m_ji_msg->name.at(iAx) =name_string;
    m_js_red_msg->name.at(iAx)   = name_string;
    m_js_redlink_msg->name.at(iAx)   = name_string;
    m_js_msg->name.at(4*iAx)   = name_string+"_motor_pos";
    m_js_msg->name.at(4*iAx+1)   = name_string+"_motor_abs_pos";
    m_js_msg->name.at(4*iAx+2) = name_string+"_spring";
    m_js_msg->name.at(4*iAx+3) = name_string+"_link";
    
    m_js_red_msg->position.at(iAx)   = 0;
    m_js_redlink_msg->position.at(iAx)   = 0;
    m_js_msg->position.at(4*iAx)   = 0;
    m_js_msg->position.at(4*iAx+1) = 0;
    m_js_msg->position.at(4*iAx+2) = 0;
    m_js_msg->position.at(4*iAx+3) = 0;
    
    m_js_red_msg->velocity.at(iAx)   = 0;
    m_js_redlink_msg->velocity.at(iAx)   = 0;
    m_js_msg->velocity.at(4*iAx)   = 0;
    m_js_msg->velocity.at(4*iAx+1) = 0;
    m_js_msg->velocity.at(4*iAx+2) = 0;
    m_js_msg->velocity.at(4*iAx+3) = 0;

    m_js_red_msg->effort.at(iAx)   = 0;
    m_js_redlink_msg->effort.at(iAx)   = 0;
    m_js_msg->effort.at(4*iAx)   = 0;
    m_js_msg->effort.at(4*iAx+1) = 0;
    m_js_msg->effort.at(4*iAx+2) = 0;
    m_js_msg->effort.at(4*iAx+3) = 0;
  }
  
  ros::Rate lp(1000);
  bool stop = false;
  while (true)
  {
    m_stop_mtx.lock();
    stop = m_stop;
    m_stop_mtx.unlock();
    if (stop)
      break;
    m_js_msg->header.stamp = ros::Time::now();
    m_js_red_msg->header.stamp = ros::Time::now();
    m_js_redlink_msg->header.stamp = ros::Time::now();
    
    m_js_pub.publish(m_js_msg);
    m_js_red_pub.publish(m_js_red_msg);
    m_js_redlink_pub.publish(m_js_redlink_msg);
    lp.sleep();
  }

}


bool ndlcom_ros::NdlcomDriverNodeletFake::triggerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = true;
  return true;
}

void ndlcom_ros::NdlcomDriverNodeletFake::callbackJointTarget(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int idx=0;idx<m_nJoints;idx++)
  {
    m_js_red_msg->position.at(idx)   = msg->position.at(idx);
    m_js_redlink_msg->position.at(idx)   = msg->position.at(idx);
    m_js_msg->position.at(4*idx)   = msg->position.at(idx);
    m_js_msg->position.at(4*idx+1) = msg->position.at(idx);
    m_js_msg->position.at(4*idx+2) = 0;
    m_js_msg->position.at(4*idx+3) = msg->position.at(idx);
    
    m_js_red_msg->velocity.at(idx)   = msg->velocity.at(idx);
    m_js_redlink_msg->velocity.at(idx)   = msg->velocity.at(idx);
    m_js_msg->velocity.at(4*idx)   = msg->velocity.at(idx);
    m_js_msg->velocity.at(4*idx+1) = msg->velocity.at(idx);
    m_js_msg->velocity.at(4*idx+2) = 0;
    m_js_msg->velocity.at(4*idx+3) = msg->velocity.at(idx);

    m_js_red_msg->effort.at(idx)   = msg->effort.at(idx);
    m_js_redlink_msg->effort.at(idx)   = msg->effort.at(idx);
    m_js_msg->effort.at(4*idx)   = msg->effort.at(idx);
    m_js_msg->effort.at(4*idx+1) = msg->effort.at(idx);
    m_js_msg->effort.at(4*idx+2) = 0;
    m_js_msg->effort.at(4*idx+3) = msg->effort.at(idx);

  }
}

}