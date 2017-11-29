
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
#include <force_monitor/force_monitor.h>
#include <pluginlib/class_list_macros.h>
# include <diagnostic_msgs/DiagnosticArray.h>

PLUGINLIB_EXPORT_CLASS(itia::ForceMonitor, nodelet::Nodelet) 


namespace itia
{

void ForceMonitor::onInit()
{
  
  std::vector<std::string> args = getMyArgv();
  m_thread  = std::thread(&itia::ForceMonitor::main, this);
  
  m_stop=false;
};

ForceMonitor::~ForceMonitor()
{
  ROS_DEBUG("[FORCE MONITOR] Shutting down..");
  m_stop=true;
  if (m_thread.joinable())
    m_thread.join(); 
  ROS_DEBUG("[FORCE MONITOR] shutted down");
  
};

void ForceMonitor::torqueLimitCb(const sensor_msgs::JointStateConstPtr& msg)
{
  if (msg->effort.size()==m_torque_threshold.size())
  {
    for (int idx=0;idx<m_torque_threshold.size();idx++)
    {
      if (msg->effort.at(idx)<m_max_torque_threshold.at(idx))
        m_torque_threshold.at(idx)=msg->effort.at(idx);
      else
      {
        m_torque_threshold.at(idx)=m_max_torque_threshold.at(idx);
        ROS_WARN("[FORCE MONITOR] %d-th torque limit is too high, set default (%f)",idx,m_max_torque_threshold.at(idx));
      }
    }
  }
  else
  {
    ROS_WARN("[FORCE MONITOR] /max_joint_torque dimension (%zu) are different with respect to /env_param/joint_torque_limit (%zu)", msg->effort.size(), m_torque_threshold.size());
  }
}

void ForceMonitor::forceLimitCb(const geometry_msgs::WrenchConstPtr& msg)
{
  int idx=0;
  if (msg->force.x<m_max_force_threshold.at(idx))
    m_force_threshold.at(idx)=msg->force.x;
  else
  {
    m_force_threshold.at(idx)=m_max_force_threshold.at(idx);
    ROS_WARN("[FORCE MONITOR] %d-th force limit is too high, set default (%f)",idx,m_max_force_threshold.at(idx));
  }

  idx=1;
  if (msg->force.y<m_max_force_threshold.at(idx))
    m_force_threshold.at(idx)=msg->force.y;
  else
  {
    m_force_threshold.at(idx)=m_max_force_threshold.at(idx);
    ROS_WARN("[FORCE MONITOR] %d-th force limit is too high, set default (%f)",idx,m_max_force_threshold.at(idx));
  }
  
  idx=2;
  if (msg->force.z<m_max_force_threshold.at(idx))
    m_force_threshold.at(idx)=msg->force.z;
  else
  {
    m_force_threshold.at(idx)=m_max_force_threshold.at(idx);
    ROS_WARN("[FORCE MONITOR] %d-th force limit is too high, set default (%f)",idx,m_max_force_threshold.at(idx));
  }
  
  idx=3;
  if (msg->torque.x<m_max_force_threshold.at(idx))
    m_force_threshold.at(idx)=msg->torque.x;
  else
  {
    m_force_threshold.at(idx)=m_max_force_threshold.at(idx);
    ROS_WARN("[FORCE MONITOR] %d-th force limit is too high, set default (%f)",idx,m_max_force_threshold.at(idx));
  }
  
  idx=4;
  if (msg->torque.y<m_max_force_threshold.at(idx))
    m_force_threshold.at(idx)=msg->torque.y;
  else
  {
    m_force_threshold.at(idx)=m_max_force_threshold.at(idx);
    ROS_WARN("[FORCE MONITOR] %d-th force limit is too high, set default (%f)",idx,m_max_force_threshold.at(idx));
  }
  
  idx=5;
  if (msg->torque.z<m_max_force_threshold.at(idx))
    m_force_threshold.at(idx)=msg->torque.z;
  else
  {
    m_force_threshold.at(idx)=m_max_force_threshold.at(idx);
    ROS_WARN("[FORCE MONITOR] %d-th force limit is too high, set default (%f)",idx,m_max_force_threshold.at(idx));
  }
}


void ForceMonitor::main()
{
  ROS_INFO("[FORCE MONITOR] Starting force monitor");
  itia::rutils::MsgReceiver<sensor_msgs::JointState> extra_torque_rec;
  ros::Subscriber extra_torque_sub = getNodeHandle().subscribe<sensor_msgs::JointState>("/external_torque",1,&itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback,&extra_torque_rec);

  itia::rutils::MsgReceiver<geometry_msgs::WrenchStamped> extra_wrench_rec;

  std::string wrench_name;
   if (!getNodeHandle().getParam("/force_monitor_wrench_name",wrench_name))
  {
    wrench_name="/external_wrench";
    ROS_INFO("FORCE MONITOR wrench_name=/external_wrench");
    return;
  }

  ros::Subscriber extra_wrench_sub = getNodeHandle().subscribe<geometry_msgs::WrenchStamped>(wrench_name,1,&itia::rutils::MsgReceiver<geometry_msgs::WrenchStamped>::callback,&extra_wrench_rec);
  
  ros::Subscriber force_limit_sub = getNodeHandle().subscribe<geometry_msgs::Wrench>("/max_contact_wrench",1,&itia::ForceMonitor::forceLimitCb,this);
  ros::Subscriber torque_limit_sub = getNodeHandle().subscribe<sensor_msgs::JointState>("/max_joint_torque",1,&itia::ForceMonitor::torqueLimitCb,this);
  
  ros::Publisher safe_ovr_pub= getNodeHandle().advertise<std_msgs::Int64>("/safe_ovr_2",1);
  ros::Publisher diagnostics_pub= getNodeHandle().advertise<diagnostic_msgs::DiagnosticArray>("/diagnostic_msgs",1);
  
  double starting_percentage=0.5;
//   if (!getNodeHandle().getParam("/env_param/joint_torque_starting_percentage",starting_percentage))
//   {
//     ROS_ERROR("[FORCE MONITOR] joint_torque_starting_percentage");
//     return;
//   }
//   
  
  if (!getNodeHandle().getParam("/env_param/joint_torque_limit",m_max_torque_threshold))
  {
    ROS_ERROR("[FORCE MONITOR] no joint  max force limit");
    return;
  }
  m_max_force_threshold.resize(6);
  double tmp;
  
  if (!getNodeHandle().getParam("/env_param/tcp_force_limit/x",tmp))
  {
    ROS_ERROR("[FORCE MONITOR] no tcp max force limit");
    return;
  }
  m_max_force_threshold.at(0)=tmp;
  
  if (!getNodeHandle().getParam("/env_param/tcp_force_limit/y",tmp))
  {
    ROS_ERROR("[FORCE MONITOR] no tcp max force limit");
    return;
  }
  m_max_force_threshold.at(1)=tmp;
  
  if (!getNodeHandle().getParam("/env_param/tcp_force_limit/z",tmp))
  {
    ROS_ERROR("[FORCE MONITOR] no tcp max force limit");
    return;
  }
  m_max_force_threshold.at(2)=tmp;
  
  if (!getNodeHandle().getParam("/env_param/tcp_force_limit/a",tmp))
  {
    ROS_ERROR("[FORCE MONITOR] no tcp max force limit");
    return;
  }
  m_max_force_threshold.at(3)=tmp;
  
  if (!getNodeHandle().getParam("/env_param/tcp_force_limit/b",tmp))
  {
    ROS_ERROR("[FORCE MONITOR] no tcp max force limit");
    return;
  }
  m_max_force_threshold.at(4)=tmp;
  
  if (!getNodeHandle().getParam("/env_param/tcp_force_limit/c",tmp))
  {
    ROS_ERROR("[FORCE MONITOR] no tcp max force limit");
    return;
  }
  m_max_force_threshold.at(5)=tmp;
  
  m_force_threshold=m_max_force_threshold;
  m_torque_threshold=m_torque_threshold;
  
  ros::Rate lp(500);
  
  std::vector<double> force(6);
  std::vector<double> torque(m_max_torque_threshold.size());
  
  std::fill(force.begin(),force.end(),0.0);
  std::fill(torque.begin(),torque.end(),0.0);
  std_msgs::Int64Ptr msg;
  
  bool contact=false;
  while(ros::ok() & !m_stop)
  {
    
    if (extra_torque_rec.isANewDataAvailable())
      torque=extra_torque_rec.getData().effort;
    if (extra_wrench_rec.isANewDataAvailable())
    {
      force.at(0)=extra_wrench_rec.getData().wrench.force.x;
      force.at(1)=extra_wrench_rec.getData().wrench.force.y;
      force.at(2)=extra_wrench_rec.getData().wrench.force.z;
      force.at(3)=extra_wrench_rec.getData().wrench.torque.x;
      force.at(4)=extra_wrench_rec.getData().wrench.torque.y;
      force.at(5)=extra_wrench_rec.getData().wrench.torque.z;
    }
    
    
    msg.reset(new std_msgs::Int64());
    double ovr=100;
    for (int idx=0;idx<m_torque_threshold.size();idx++)
    {
      double ovr_ax=100;
      if (std::abs(torque.at(idx))>m_torque_threshold.at(idx))
        ovr_ax=0;
      else if (std::abs(torque.at(idx))>starting_percentage*m_torque_threshold.at(idx))
        ovr_ax=100*std::pow((std::abs(torque.at(idx))/m_torque_threshold.at(idx)-1)/(1-starting_percentage),2.0);
      ovr=std::min(ovr,ovr_ax);
    }
    for (int idx=0;idx<m_force_threshold.size();idx++)
    {
      double ovr_ax=100;
      if (std::abs(force.at(idx))>m_force_threshold.at(idx))
        ovr_ax=0;
      else if (std::abs(force.at(idx))>starting_percentage*m_force_threshold.at(idx))
        ovr_ax=100*std::pow((std::abs(force.at(idx))/m_force_threshold.at(idx)-1)/(1-starting_percentage),2.0);
      ovr=std::min(ovr,ovr_ax);
    }
    
    if (ovr==0)
    {
      ROS_WARN_THROTTLE(0.1,"Stopping robot due to possible contact");

      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="FORCE MONITOR";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_msg.status.at(0).message="Stopping robot due to possible contact";
      diag_msg.status.at(0).values.resize(1);
      diag_msg.status.at(0).values.at(0).key="SAFE OVR";
      diag_msg.status.at(0).values.at(0).value=std::to_string(ovr);
      diagnostics_pub.publish(diag_msg);
      contact=true;
    }
    else if (ovr<50)
    {
      ROS_INFO_THROTTLE(1,"Reducing speed due to possible contact");
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="FORCE MONITOR";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::WARN;
      diag_msg.status.at(0).message="Reducing speed due to possible contact";
      diag_msg.status.at(0).values.resize(1);
      diag_msg.status.at(0).values.at(0).key="SAFE OVR";
      diag_msg.status.at(0).values.at(0).value=std::to_string(ovr);
      diagnostics_pub.publish(diag_msg);
      contact=true;
    }
    else if (contact)
    {
      contact=false;
      ROS_INFO_THROTTLE(1,"no conctacts");
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="FORCE MONITOR";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::OK;
      diag_msg.status.at(0).message="no conctact";
      diag_msg.status.at(0).values.resize(1);
      diag_msg.status.at(0).values.at(0).key="SAFE OVR";
      diag_msg.status.at(0).values.at(0).value=std::to_string(ovr);
      diagnostics_pub.publish(diag_msg);
    }
    
    msg->data=ovr;
    safe_ovr_pub.publish(msg);
    lp.sleep();
  }
  
  
};

  
}
