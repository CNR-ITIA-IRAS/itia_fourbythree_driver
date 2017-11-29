
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

#include <fourbythree_test/feedforward_action.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::identification::FeedforwardActionNodelet, nodelet::Nodelet) 



namespace itia{
namespace identification{

void FeedforwardActionNodelet::onInit()
{
  m_stop = false;
  ROS_INFO("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] thread starting", getName().c_str());
  m_main_thread  = std::thread(&itia::identification::FeedforwardActionNodelet::mainThread, this);  
}

FeedforwardActionNodelet::~FeedforwardActionNodelet()
{
  stopThread();
  if (m_main_thread.joinable())
    m_main_thread.join();
  ROS_INFO("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] thread STOP", getName().c_str());
}

void FeedforwardActionNodelet::mainThread()
{
  ros::NodeHandle nh = getPrivateNodeHandle();
  double freq_rate = 1000.0;
  ros::WallRate loop_rate(freq_rate);

  ROS_INFO("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] loading parameters", getName().c_str());
  if (!loadParameters())
    return;
  ROS_INFO("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] parameters loaded", getName().c_str());
  ROS_INFO("Test duration: %s%f%s", GREEN, m_test_duration, RESET);
  ROS_INFO("Transient: %s%f%s", GREEN, m_start_stop_transient, RESET);
  ROS_INFO("Number of sinusoidal waves: %s%d%s", GREEN,m_nsin, RESET);
  ROS_INFO("Number of joints: %s%d%s", GREEN,m_nAx,RESET);
  
  
	ros::Publisher target_js_pub = nh.advertise<sensor_msgs::JointState>(m_topic_name, 1);
  
  sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
  nodelet::NodeletLoad load_srv;
  
  
  msg->position.resize(m_nAx);
  msg->velocity.resize(m_nAx);
  msg->effort.resize(m_nAx);
  msg->name.resize(m_nAx);
  std::fill(msg->position.begin(), msg->position.end(), 0);
  std::fill(msg->velocity.begin(), msg->velocity.end(), 0);
  std::fill(msg->effort.begin(), msg->effort.end(), 0);
  msg->header.stamp=ros::Time::now();
  target_js_pub.publish(msg);
  loop_rate.sleep();
  ros::WallTime t0=ros::WallTime::now();
  while (!m_stop)
  {
    loop_rate.sleep();
    double t = (ros::WallTime::now()-t0).toSec();
    if (t>m_test_duration)
    {
      ROS_INFO("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] TEST FINISHED", getName().c_str());
      std::fill(msg->position.begin(), msg->position.end(), 0);
      std::fill(msg->velocity.begin(), msg->velocity.end(), 0);
      std::fill(msg->effort.begin(), msg->effort.end(), 0);
      msg->header.stamp=ros::Time::now();
      target_js_pub.publish(msg);
      ros::WallTime t0=ros::WallTime::now();
      loop_rate.sleep();
      return;
    } 
    ROS_INFO_THROTTLE(1,"Test time = %s%f%s [s] (Duration = %s%f%s [s])",GREEN, t, RESET, GREEN,m_test_duration,RESET);
    double transient;
    
    if (t<m_start_stop_transient)
      transient=(3*pow(t/m_start_stop_transient,2)-2*pow(t/m_start_stop_transient,3));
    else if (t>(m_test_duration-m_start_stop_transient))
      transient=(3*pow((m_test_duration-t)/m_start_stop_transient,2)-2*pow((m_test_duration-t)/m_start_stop_transient,3));
    else
      transient=1.0;
    for (int i_ax=0;i_ax<m_nAx;i_ax++)
    {
      double trigonometric_term=0;
      for (int i_sin=0; i_sin<m_nsin; i_sin++)
      {
				trigonometric_term += m_cosinusoidal_coeffs.at(i_sin).at(i_ax) * cos(m_omega.at(i_sin)*t)+
	                      m_sinusoidal_coeffs.at(i_sin).at(i_ax)   * sin(m_omega.at(i_sin)*t);
      }
      if (m_signal==0)
				msg->effort.at(i_ax)=trigonometric_term*transient;
			else if (m_signal==1)
				msg->velocity.at(i_ax)=trigonometric_term*transient;
			else if (m_signal==2)
				msg->position.at(i_ax)=trigonometric_term*transient;
    }
    msg->header.stamp=ros::Time::now();
    target_js_pub.publish(msg);
    
  }
  ROS_INFO("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] %sEarly stopped%s", getName().c_str(), BLUE,RESET);
  std::fill(msg->position.begin(), msg->position.end(), 0);
  std::fill(msg->velocity.begin(), msg->velocity.end(), 0);
  std::fill(msg->effort.begin(), msg->effort.end(), 0);
  msg->header.stamp=ros::Time::now();
  target_js_pub.publish(msg);
  loop_rate.sleep();
  return;
}

void FeedforwardActionNodelet::stopThread()
{
  ROS_INFO("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] stopping thread", getName().c_str());
  m_stop = true;
}

bool FeedforwardActionNodelet::loadParameters()
{
  
  if (!getPrivateNodeHandle().getParam("test_duration",m_test_duration))
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/test_duration' does not exist", getName().c_str());
    return false;
  }
  if (m_test_duration<=0)
  {
    ROS_WARN("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/test_duration' (%f [s]) is not positive. The test duration is set to 1e6 [s]", getName().c_str(),m_test_duration);
    m_test_duration=1e6;
  }
  
  if (!getPrivateNodeHandle().getParam("start_stop_transient",m_start_stop_transient))
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/start_stop_transient' does not exist", getName().c_str());
    return false;
  }
  if (m_start_stop_transient<1e-6)
  {
    ROS_WARN("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/start_stop_transient'=%f, it should be greater than 1e-6. It will be set equal to 1e-6", getName().c_str(),m_start_stop_transient);
    m_start_stop_transient=1e-6;
  }
  if (m_start_stop_transient>(2*m_test_duration))
  {
    ROS_WARN("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/start_stop_transient'=%f, it should be less than the half of the test duration. It will be set equal to the half of the test duration", getName().c_str(),m_start_stop_transient);
    m_start_stop_transient=m_test_duration/2.0;
  }
  
  if (!getPrivateNodeHandle().getParam("natural_frequency",m_omega))
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/natural_frequency' does not exist", getName().c_str());
    return false;
  }
  m_nsin=m_omega.size();
  if (m_nsin<=0)
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/natural_frequency' has %d elements, it should not be empty", getName().c_str(),m_nsin);
    return false;
  }
  
  
  if (!getPrivateNodeHandle().getParam("signal_ffw",m_signal))
	{
		ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/signal_ffw' does not exist", getName().c_str());
		return false;
	}
	if ( (m_signal>2) || (m_signal<0) )
	{
		ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/signal_ffw' should be equal to: 0 (torque), 1 (Velocity), 2 (position)", getName().c_str());
	}
	
	if (!getPrivateNodeHandle().getParam("topic_name",m_topic_name))
	{
		m_topic_name="/joint_feedforward";
	}
	ROS_INFO("publishing on topic '%s'",m_topic_name.c_str());
	
  
  
  if (!itia::rutils::getParamMatrix(getPrivateNodeHandle(),"sinusoidal_coefficients",m_sinusoidal_coeffs))
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/sinusoidal_coefficients' does not exist", getName().c_str());
    return false;
  }
  if (m_nsin != m_sinusoidal_coeffs.size())
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/sinusoidal_coefficients' dimension (%d rows) is different w.r.t. '~/natural_frequency' (%d rows)", getName().c_str(),m_sinusoidal_coeffs.size(),m_nsin);
    return false;
  }
  
  m_nAx=m_sinusoidal_coeffs.at(0).size();
  if (m_nAx<=0)
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/sinusoidal_coefficients' has %d columns, it should not be empty", getName().c_str(),m_nAx);
    return false;
  }
  
  if (!itia::rutils::getParamMatrix(getPrivateNodeHandle(),"cosinusoidal_coefficients",m_cosinusoidal_coeffs))
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/cosinusoidal_coefficients' does not exist", getName().c_str());
    return false;
  }
  if (m_nsin != m_cosinusoidal_coeffs.size())
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/cosinusoidal_coefficients' dimension (%d rows) is different w.r.t. '~/natural_frequency' (%d rows)", getName().c_str(),m_cosinusoidal_coeffs.size(),m_nsin);
    return false;
  }
  if (m_nAx != m_cosinusoidal_coeffs.at(0).size())
  {
    ROS_ERROR("[IDENTIFICATION_FEEDFORWARD_ACTION '%s'] Parameter '~/cosinusoidal_coefficients' dimension (%d cols) is different w.r.t. '~/sinusoidal_coefficients' (%d cols)", getName().c_str(),m_cosinusoidal_coeffs.at(0).size(),m_nAx);
    return false;
  }
  
  
  return true;
}


}
}