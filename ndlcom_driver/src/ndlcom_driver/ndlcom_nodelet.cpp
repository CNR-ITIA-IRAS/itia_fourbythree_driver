
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

#include <ndlcom_driver/ndlcom_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ndlcom_ros::NdlcomDriverNodelet, nodelet::Nodelet) 

std::ofstream cnrlogfile;

namespace ndlcom_ros{

void NdlcomDriverNodelet::onInit()
{
  
  cnrlogfile.open("ndlcom_errors.txt", std::ofstream::out );
  if( !cnrlogfile.is_open() )
  {
    ROS_ERROR("Error in opening the NDLCOM log file");
  }
  m_read_thread  = std::thread(&ndlcom_ros::NdlcomDriver::read, &m_ndlcom_driver);
  m_write_thread = std::thread(&ndlcom_ros::NdlcomDriver::write, &m_ndlcom_driver);
  m_ros_thread = std::thread(&ndlcom_ros::NdlcomDriver::spin, &m_ndlcom_driver);
  m_publish_thread = std::thread(&ndlcom_ros::NdlcomDriver::publish, &m_ndlcom_driver);
  
  ros::Duration timeout(2.0);
  ros::WallTime t0=ros::WallTime::now();
  
  if (!m_ndlcom_driver.waitForConfigure(timeout))
  {
    ros::Duration(0.2).sleep();
    ROS_ERROR("error configuring robot");
    m_ndlcom_driver.stopThread();
    if (m_read_thread.joinable())
      m_read_thread.join();
    ROS_INFO("read thread stop");
    if (m_write_thread.joinable())
      m_write_thread.join();
    ROS_INFO("write thread stop");
    if (m_ros_thread.joinable())
      m_ros_thread.join();
    ROS_INFO("spin thread stop");
    if (m_publish_thread.joinable())
      m_publish_thread.join();
    ROS_INFO("publish thread stop");
    
    ROS_INFO("shutted down...");
    getNodeHandle().setParam("/ndlcom/robot_status",false);
    return;
  }
  else  
    ROS_INFO("configured in %f seconds",(ros::WallTime::now()-t0).toSec());
  getNodeHandle().setParam("/ndlcom/robot_status",true);
};

NdlcomDriverNodelet::~NdlcomDriverNodelet()
{
  ros::Duration timeout(2.0);
  ROS_INFO("shutting down...");
  m_ndlcom_driver.waitForDisable(timeout);
  m_ndlcom_driver.stopThread();
  if (m_read_thread.joinable())
    m_read_thread.join();
  if (m_write_thread.joinable())
    m_write_thread.join();
  if (m_ros_thread.joinable())
    m_ros_thread.join();
  if (m_publish_thread.joinable())
    m_publish_thread.join();
  ROS_INFO("shutted down...");
  if( cnrlogfile.is_open() )
    cnrlogfile.close();
  getNodeHandle().setParam("/ndlcom/robot_status",false);
  
};


}