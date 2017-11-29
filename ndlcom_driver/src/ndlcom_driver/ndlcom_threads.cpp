
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

#include <ndlcom_driver/ndlcom_utils.h>
#include <sys/socket.h>

namespace ndlcom_ros{


void NdlcomDriver::spin()
{
  m_communication_error=false;
  bool ndlcom_thread_stop = true;
  ROS_INFO("starting spinning");
  double read_rate = 1000;
  ros::Rate lp(read_rate);
  
  ros::ServiceServer enable_srv_server  = m_nh.advertiseService("com/enable_joints",&NdlcomDriver::callbackJointsEnable,this);
  ros::ServiceServer disable_srv_server = m_nh.advertiseService("com/disable_joints",&NdlcomDriver::callbackJointsDisable,this);
  ros::ServiceServer reset_srv_server   = m_nh.advertiseService("com/set_zero",&NdlcomDriver::callbackResetZeros,this);
  
  while (true)
  {
    m_ndlcom_thread_stop_mtx.lock();
    ndlcom_thread_stop = m_ndlcom_thread_stop;
    m_ndlcom_thread_stop_mtx.unlock();
    
    if (ndlcom_thread_stop)
      break;
    
    if (m_communication_error)
    {
      waitForDisable(ros::Duration(0.1));
      m_ndlcom_thread_stop=true;
    }
    lp.sleep();
    
    
  }
}

void NdlcomDriver::read()
{
  
  m_qmotor.resize(m_nJoints);
  m_qabsmotor.resize(m_nJoints);
  m_qlink.resize(m_nJoints);
  m_qspring.resize(m_nJoints);
  m_taumotor.resize(m_nJoints);
  
  long int n_mesg=0;
  long int n_missing_messages=0;
  
  
  bool ndlcom_thread_stop = true;
  int message_size;
  ROS_INFO("starting reading");
  double read_rate = 2000;
  ros::Rate lp(read_rate);
  
  int max_missing_package;
  int warning_missing_package;
  double warning_rate_quality;
  double error_rate_quality;
  if (!m_nh.getParam("ndlcom/error_missing_messages_threshold",max_missing_package))
    max_missing_package=100;
  if (!m_nh.getParam("ndlcom/warning_missing_messages_threshold",warning_missing_package))
    warning_missing_package=20;
  if (!m_nh.getParam("ndlcom/warning_rate_quality",warning_rate_quality))
    warning_rate_quality=0.01;
  if (!m_nh.getParam("ndlcom/error_rate_quality",error_rate_quality))
    error_rate_quality=0.10;
  
  
  while (true)
  {
    ndlcom_thread_stop = m_ndlcom_thread_stop;
    
    if (ndlcom_thread_stop)
    {
      break;
    }
    message_size =UART_receive(m_port_handle, m_serial_buf, c_MAX_BUFFER_LENGTH);
    int parsed_size_total = 0;
    while( parsed_size_total < message_size ) 
    {
      // try to parse
      int parsed_size = ndlcomParserReceive(m_ndlcom_parser, 
                                            (uint8_t*)m_serial_buf + parsed_size_total,
                                            message_size - parsed_size_total);
      parsed_size_total += parsed_size;
      
      if( ndlcomParserHasPacket( m_ndlcom_parser ) ) 
      {
        // Get data header
        m_ndlcom_header = *( ndlcomParserGetHeader( m_ndlcom_parser ) );

        // Get data pointer
        const uint8_t* data = (uint8_t*) ndlcomParserGetPacket( m_ndlcom_parser );

        // Once a packet is completed and data read destroy it
        ndlcomParserDestroyPacket( m_ndlcom_parser );

        m_ndlcom_msg.header = m_ndlcom_header;
        
        // Resize to accomodate data
        m_ndlcom_msg.data.resize((unsigned int)m_ndlcom_header.mDataLen);
        
        for (unsigned int d=0; d<(unsigned int)m_ndlcom_header.mDataLen; d++) 
          m_ndlcom_msg.data[d] = data[d];
        

        m_mngr_jnts_mtx.lock();
        int missing_messages= m_mngr_jnts.handleMessage( m_ndlcom_msg );
        m_mngr_jnts_mtx.unlock();
        if (n_mesg>10000)
        {
          if (missing_messages>max_missing_package)
          {
            ROS_FATAL_THROTTLE(5,"[ NDLCOM DRIVER ] Communication error (at cycle %ld). %d packages missing. The robot will be stopped for safety reason", n_mesg, missing_messages);
            m_communication_error=true;
            diagnostic_msgs::DiagnosticArray diag_msg;
            diag_msg.header.stamp=ros::Time::now();
            diag_msg.status.resize(1);
            diag_msg.status.at(0).name="NDLCOM COMMUNICATION";
            diag_msg.status.at(0).hardware_id="FourByThreeRobot";
            diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
            diag_msg.status.at(0).message="Communication error. The robot will be stopped for safety reason";
            diag_msg.status.at(0).values.resize(4);
            
            diag_msg.status.at(0).values.at(0).key="Cycle";
            diag_msg.status.at(0).values.at(0).value=std::to_string( n_mesg );

            diag_msg.status.at(0).values.at(1).key="Consecutive Missing packages";
            diag_msg.status.at(0).values.at(1).value=std::to_string( missing_messages);
            
            diag_msg.status.at(0).values.at(2).key="Suggestion 1";
            diag_msg.status.at(0).values.at(2).value="Check the CPU usage";

            diag_msg.status.at(0).values.at(3).key="Suggestion 2";
            diag_msg.status.at(0).values.at(3).value="try to use a shorter USB cable";
            m_diagnostics_pub.publish(diag_msg);
          }
          else if (missing_messages>warning_missing_package)
          {
            ROS_WARN_THROTTLE(5,"[ NDLCOM DRIVER ] Communication error (at cycle %ld). %d packages missing. ", n_mesg, missing_messages);
            diagnostic_msgs::DiagnosticArray diag_msg;
            diag_msg.header.stamp=ros::Time::now();
            diag_msg.status.resize(1);
            diag_msg.status.at(0).name="NDLCOM COMMUNICATION";
            diag_msg.status.at(0).hardware_id="FourByThreeRobot";
            diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::WARN;
            diag_msg.status.at(0).message="Communication error. Consecutive Missing packages";
            diag_msg.status.at(0).values.resize(4);
            
            diag_msg.status.at(0).values.at(0).key="Cycle";
            diag_msg.status.at(0).values.at(0).value=std::to_string( n_mesg );

            diag_msg.status.at(0).values.at(1).key="Consecutive Missing packages";
            diag_msg.status.at(0).values.at(1).value=std::to_string( missing_messages);
            
            diag_msg.status.at(0).values.at(2).key="Suggestion 1";
            diag_msg.status.at(0).values.at(2).value="Check the CPU usage";

            diag_msg.status.at(0).values.at(3).key="Suggestion 2";
            diag_msg.status.at(0).values.at(3).value="try to use a shorter USB cable";
            m_diagnostics_pub.publish(diag_msg);
          }
          n_missing_messages+=missing_messages;
      

          double ratio=double(n_missing_messages)/double(n_mesg);
          if (ratio>error_rate_quality)
          {
            ROS_FATAL_THROTTLE(5,"[ NDLCOM DRIVER ] Communication error  (at cycle %ld). %f%% missing packages. The robot will be stopped for safety reason", n_mesg, ratio*100);
            m_communication_error=true;
            
            diagnostic_msgs::DiagnosticArray diag_msg;
            diag_msg.header.stamp=ros::Time::now();
            diag_msg.status.resize(1);
            diag_msg.status.at(0).name="NDLCOM COMMUNICATION RATE";
            diag_msg.status.at(0).hardware_id="FourByThreeRobot";
            diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
            diag_msg.status.at(0).message="[NDLCOM DRIVER] Communication error. The robot will be stopped for safety reason";
            diag_msg.status.at(0).values.resize(4);
            
            diag_msg.status.at(0).values.at(0).key="Cycle";
            diag_msg.status.at(0).values.at(0).value=std::to_string( n_mesg );

            diag_msg.status.at(0).values.at(1).key="Missing packages ratio";
            diag_msg.status.at(0).values.at(1).value=std::to_string( ratio*100)+"%";
            
            diag_msg.status.at(0).values.at(2).key="Suggestion 1";
            diag_msg.status.at(0).values.at(2).value="Check the CPU usage";

            diag_msg.status.at(0).values.at(3).key="Suggestion 2";
            diag_msg.status.at(0).values.at(3).value="try to use a shorter USB cable";
            m_diagnostics_pub.publish(diag_msg);
          }
          else if (ratio>warning_rate_quality)
          {
            ROS_WARN_THROTTLE(5,"[ NDLCOM DRIVER ] low communication quality (at cycle %ld) %f%% missing packages", n_mesg, ratio*100);
            
            diagnostic_msgs::DiagnosticArray diag_msg;
            diag_msg.header.stamp=ros::Time::now();
            diag_msg.status.resize(1);
            diag_msg.status.at(0).name="NDLCOM COMMUNICATION RATE";
            diag_msg.status.at(0).hardware_id="FourByThreeRobot";
            diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::WARN;
            diag_msg.status.at(0).message="[NDLCOM DRIVER] Communication warning, low communication quality.";
            diag_msg.status.at(0).values.resize(4);
            
            diag_msg.status.at(0).values.at(0).key="Cycle";
            diag_msg.status.at(0).values.at(0).value=std::to_string( n_mesg );

            diag_msg.status.at(0).values.at(1).key="Missing packages ratio";
            diag_msg.status.at(0).values.at(1).value=std::to_string( ratio*100)+"%";
            
            diag_msg.status.at(0).values.at(2).key="Suggestion 1";
            diag_msg.status.at(0).values.at(2).value="Check the CPU usage";

            diag_msg.status.at(0).values.at(3).key="Suggestion 2";
            diag_msg.status.at(0).values.at(3).value="try to use a shorter USB cable";
            m_diagnostics_pub.publish(diag_msg);
          }
        }
      }

    }
    
    m_mngr_jnts_mtx.lock();
    m_mngr_jnts.getJointInfo(m_info_jnts);
    m_mngr_jnts_mtx.unlock();
    
    
    
    double total_current=0;
    for (unsigned int idx=0;idx<(m_nJoints);idx++)
    {
  
//       ROS_INFO("%d  CONFIG_NV %f",idx,m_mngr_jnts.getJoints().at(idx)->getRegister( RobotConfig::SerialElasticJoint::CONFIG_NV )->getRealActValue());
// 			m_qmotor(idx) = m_info_jnts.at(idx).actualJointState.position-m_motor_offset.at(idx);
			m_qmotor(idx) = m_mngr_jnts.getJoints().at(idx)->getRegister( RobotConfig::SerialElasticJoint::POSITION )->getRealActValue()-m_motor_offset.at(idx);
			m_qabsmotor(idx) = m_mngr_jnts.getJoints().at(idx)->getRegister( RobotConfig::SerialElasticJoint::ABS_POS )->getRealActValue()-m_motor_offset.at(idx);
      m_qspring(idx) = m_info_jnts.at(idx).telemetry.unused[2]* M_PI / pow(2.0,15)-m_spring_offset.at(idx);
      m_voltage.at(idx)=m_mngr_jnts.getJoints().at(idx)->getRegister( RobotConfig::SerialElasticJoint::VOLTAGE )->getRealActValue();
      m_current.at(idx)=m_mngr_jnts.getJoints().at(idx)->getRegister( RobotConfig::SerialElasticJoint::MOTOR_CURRENT )->getRealActValue();
      total_current+=std::abs(m_current.at(idx));

      m_qlink(idx) = m_qspring(idx)+m_qmotor(idx);
      m_taumotor(idx) = m_info_jnts.at(idx).actualJointState.effort;
  
      if (m_info_jnts.at(idx).activeJointErrors.size()>0)
      {
        int idx2=0;
        ROS_ERROR_THROTTLE(5, "Active joint errors");
        waitForDisable(ros::Duration(0.1));
        m_communication_error=true;
        diagnostic_msgs::DiagnosticArray diag_msg;
        diag_msg.header.stamp=ros::Time::now();
        diag_msg.status.resize(1);
        diag_msg.status.at(0).name="NDLCOM FPGA ERROR";
        diag_msg.status.at(0).hardware_id="FourByThreeRobot";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_msg.status.at(0).message="Error from FPGA";
        diag_msg.status.at(0).values.resize(m_info_jnts.at(idx).activeJointErrors.size());
        
        for (unsigned int e=0; e < m_info_jnts.at(idx).activeJointErrors.size(); e++)
        {
          ROS_ERROR_THROTTLE(5, "%d) %s",idx2++,m_info_jnts.at(idx).activeJointErrors[e].c_str());
          
          
          diag_msg.status.at(0).values.at(e).key="ERROR";
          diag_msg.status.at(0).values.at(e).value=m_info_jnts.at(idx).activeJointErrors[e];
          
        }
        m_diagnostics_pub.publish(diag_msg);
      }
      
      if (m_info_jnts.at(idx).activeJointWarnings.size()>0)
      {
        int idx2=0;
        ROS_WARN_THROTTLE(	100, "Active joint warnings");
        for (std::size_t e=0; e < m_info_jnts.at(idx).activeJointWarnings.size(); e++)
          ROS_WARN_THROTTLE(100, "%d) %s",idx2++,m_info_jnts.at(idx).activeJointWarnings[e].c_str());
        
        diagnostic_msgs::DiagnosticArray diag_msg;
        diag_msg.header.stamp=ros::Time::now();
        diag_msg.status.resize(1);
        diag_msg.status.at(0).name="NDLCOM FPGA WARNING";
        diag_msg.status.at(0).hardware_id="FourByThreeRobot";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::WARN;
        diag_msg.status.at(0).message="[NDLCOM DRIVER] Warning from FPGA";
        diag_msg.status.at(0).values.resize(m_info_jnts.at(idx).activeJointWarnings.size());
        
        for (unsigned int e=0; e < m_info_jnts.at(idx).activeJointWarnings.size(); e++)
        {
          ROS_WARN_THROTTLE(5, "%d) %s",idx2++,m_info_jnts.at(idx).activeJointWarnings[e].c_str());
          diag_msg.status.at(0).values.at(e).key="WARNING";
          diag_msg.status.at(0).values.at(e).value=m_info_jnts.at(idx).activeJointWarnings[e];
          
        }
        m_diagnostics_pub.publish(diag_msg);
      }
        
    }
    
    m_current.at(m_nJoints)=total_current;
    n_mesg++;
    lp.sleep();
    
  }
  ROS_INFO("stoping read thread");
  return;
}


void NdlcomDriver::publish()
{
  bool first_time = true;
  bool ndlcom_thread_stop = true;
  
  double read_rate = 1000;
  ros::Rate lp(read_rate);
  
//   ros::WallDuration(0.2).sleep(); //time required to start and load controller
  
  ros::Time t0=ros::Time::now();
  
  while (true)
  {
    lp.sleep();
    m_ndlcom_thread_stop_mtx.lock();
    ndlcom_thread_stop = m_ndlcom_thread_stop;
    m_ndlcom_thread_stop_mtx.unlock();
    
    if (ndlcom_thread_stop)
      break;
  
    if (m_info_jnts.size() == 0)
      continue;
    
    
    if (first_time)
    {
      first_time = false;
      for (unsigned int idx=0;idx<(m_nJoints);idx++)
      {
        
        for (unsigned int iTmp = 0; iTmp<100; iTmp++)
        {
          m_js_msg->position.at(4*idx)   = m_q_filter.at(4*idx  )->update(m_qmotor.block(idx, 0, 1, 1))(0);
          m_js_msg->position.at(4*idx+1) = m_q_filter.at(4*idx+1)->update(m_qabsmotor.block(idx, 0, 1, 1))(0);
          m_js_msg->position.at(4*idx+2) = m_q_filter.at(4*idx+2)->update(m_qspring.block(idx, 0, 1, 1))(0);
          m_js_msg->position.at(4*idx+3) = m_q_filter.at(4*idx+3)->update(m_qlink.block(idx, 0, 1, 1))(0);
          
          m_js_msg->velocity.at(4*idx)   = m_Dq_filter.at(4*idx  )->update(m_qmotor.block(idx, 0, 1, 1))(0);
          m_js_msg->velocity.at(4*idx+1) = m_Dq_filter.at(4*idx+1)->update(m_qabsmotor.block(idx, 0, 1, 1))(0);
          m_js_msg->velocity.at(4*idx+3) = m_Dq_filter.at(4*idx+3)->update(m_qlink.block(idx, 0, 1, 1))(0);
          m_js_msg->velocity.at(4*idx+2) = m_Dq_filter.at(4*idx+2)->update(m_qspring.block(idx, 0, 1, 1))(0);
          
          m_js_msg->effort.at(4*idx  )   = m_effort_filter.at(idx)->update(m_taumotor.block(idx, 0, 1, 1))(0);
          m_js_msg->effort.at(4*idx+1)   = 0.0;
          m_js_msg->effort.at(4*idx+3)   = 0.0;
          m_js_msg->effort.at(4*idx+2)   = 0.0;
          
          m_js_red_msg->position.at(idx) = m_js_msg->position.at(4*idx);
          m_js_red_msg->velocity.at(idx) = m_js_msg->velocity.at(4*idx);
          m_js_red_msg->effort.at(idx)   = m_effort_filter.at(idx)->update(m_taumotor.block(idx, 0, 1, 1))(0);
          
          m_js_redlink_msg->position.at(idx) = m_js_msg->position.at(4*idx+3);
          m_js_redlink_msg->velocity.at(idx) = m_js_msg->velocity.at(4*idx+3);
					m_js_redlink_msg->effort.at(idx) = -m_q_filter.at(4*idx+2)->update(m_qspring.block(idx, 0, 1, 1))(0)*m_spring_elasticity.at(idx);
          
          m_js_elastic_msg->position.at(2*idx)   = m_q_filter.at(4*idx )->update(m_qmotor.block(idx, 0, 1, 1))(0);
          m_js_elastic_msg->position.at(2*idx+1) = m_q_filter.at(4*idx+2 )->update(m_qspring.block(idx, 0, 1, 1))(0);
          
          m_js_elastic_msg->velocity.at(2*idx)   = m_Dq_filter.at(4*idx  )->update(m_qmotor.block(idx, 0, 1, 1))(0);
          m_js_elastic_msg->velocity.at(2*idx+1) = m_Dq_filter.at(4*idx+2)->update(m_qspring.block(idx, 0, 1, 1))(0);
          
          m_js_elastic_msg->effort.at(2*idx  )   = m_effort_filter.at(idx)->update(m_taumotor.block(idx, 0, 1, 1))(0);
          m_js_elastic_msg->effort.at(2*idx+1)   = 0.0;
          m_ji_msg->data.at(idx) = m_info_jnts.at(idx).telemetry.temperature/ pow(2.0,8);
          
          
          
        }
        
      }
    }
    
    m_jvoltage_msg->data=m_voltage;
    m_jcurrent_msg->data=m_current;
    
    if (m_current.at(m_nJoints)>m_power_supply_current)
    {
        diagnostic_msgs::DiagnosticArray diag_msg;
        
        diag_msg.header.stamp=ros::Time::now();
        diag_msg.status.resize(1);
        diag_msg.status.at(0).name="NDLCOM POWER SUPPLY";
        diag_msg.status.at(0).hardware_id="FourByThreeRobot";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_msg.status.at(0).message="Too much current consumption (maximum = "+std::to_string(m_power_supply_current)+" [A]";
        diag_msg.status.at(0).values.resize(m_nJoints+1);
        
        for (unsigned int i=0;i<m_current.size();i++)
        {
          diag_msg.status.at(0).values.at(i).key=m_jcurrent_msg->name.at(i)+" [A]";
          diag_msg.status.at(0).values.at(i).value=std::to_string(m_current.at(i));
        }
        m_diagnostics_pub.publish(diag_msg);
    }
    
    m_js_msg->header.stamp=ros::Time::now();
    m_ji_msg->header.stamp=ros::Time::now();
    m_jvoltage_msg->header.stamp=ros::Time::now();
    m_jcurrent_msg->header.stamp=ros::Time::now();
    m_js_red_msg->header.stamp=ros::Time::now();
    m_js_redlink_msg->header.stamp=ros::Time::now();
    m_js_elastic_msg->header.stamp=ros::Time::now();


    for (unsigned int idx=0;idx<(m_nJoints);idx++)
    {
      
      
      m_js_msg->position.at(4*idx)   = m_q_filter.at(4*idx  )->update(m_qmotor.block(idx, 0, 1, 1))(0);
      m_js_msg->position.at(4*idx+1) = m_q_filter.at(4*idx+1)->update(m_qabsmotor.block(idx, 0, 1, 1))(0);
      m_js_msg->position.at(4*idx+2) = m_q_filter.at(4*idx+2)->update(m_qspring.block(idx, 0, 1, 1))(0);
      m_js_msg->position.at(4*idx+3) = m_q_filter.at(4*idx+3)->update(m_qlink.block(idx, 0, 1, 1))(0);

      m_js_msg->velocity.at(4*idx)   = m_Dq_filter.at(4*idx  )->update(m_qmotor.block(idx, 0, 1, 1))(0);
      m_js_msg->velocity.at(4*idx+1) = m_Dq_filter.at(4*idx+1)->update(m_qabsmotor.block(idx, 0, 1, 1))(0);
      m_js_msg->velocity.at(4*idx+3) = m_Dq_filter.at(4*idx+3)->update(m_qlink.block(idx, 0, 1, 1))(0);
      m_js_msg->velocity.at(4*idx+2) = m_Dq_filter.at(4*idx+2)->update(m_qspring.block(idx, 0, 1, 1))(0);
      
      m_js_msg->effort.at(4*idx  )   = m_effort_filter.at(idx)->update(m_taumotor.block(idx, 0, 1, 1))(0);
      m_js_msg->effort.at(4*idx+1)   = 0.0;
      m_js_msg->effort.at(4*idx+3)   = 0.0;
      m_js_msg->effort.at(4*idx+2)   = 0.0;
      
      m_js_red_msg->position.at(idx)     = m_js_msg->position.at(4*idx);
      m_js_red_msg->velocity.at(idx)     = m_js_msg->velocity.at(4*idx);
      m_js_red_msg->effort.at(idx)     = m_js_msg->effort.at(4*idx);
      
      m_js_redlink_msg->position.at(idx) = m_js_msg->position.at(4*idx+3);
      m_js_redlink_msg->velocity.at(idx) = m_js_msg->velocity.at(4*idx+3);
			m_js_redlink_msg->effort.at(idx) =-m_q_filter.at(4*idx+2)->update(m_qspring.block(idx, 0, 1, 1))(0)*m_spring_elasticity.at(idx); // spring deflection is the effort of the "rigid" system
      
      m_js_elastic_msg->position.at(2*idx)   = m_js_msg->position.at(4*idx) ;
      m_js_elastic_msg->position.at(2*idx+1) = m_js_msg->position.at(4*idx+2);
      
      m_js_elastic_msg->velocity.at(2*idx)   = m_js_msg->velocity.at(4*idx);
      m_js_elastic_msg->velocity.at(2*idx+1) = m_js_msg->velocity.at(4*idx+2);
      
      m_js_elastic_msg->effort.at(2*idx  )   = m_js_msg->effort.at(4*idx);
      m_js_elastic_msg->effort.at(2*idx+1)   = 0.0;
      
      m_ji_msg->data.at(idx) = m_info_jnts.at(idx).telemetry.temperature/ pow(2.0,8);
      
      bool valid_torque=true;
      if (std::abs(m_js_red_msg->effort.at(idx))>400)
      {
        valid_torque=false;
        diagnostic_msgs::DiagnosticArray diag_msg;
        
        diag_msg.header.stamp=ros::Time::now();
        diag_msg.status.resize(1);
        diag_msg.status.at(0).name="NDLCOM FPGA";
        diag_msg.status.at(0).hardware_id="FourByThreeRobot";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_msg.status.at(0).message="receiving unrealistically high torque";
        diag_msg.status.at(0).values.resize(2);
        
        diag_msg.status.at(0).values.at(0).key="JOINT";
        diag_msg.status.at(0).values.at(0).value=m_js_red_msg->name.at(idx);
        
        if ((ros::Time::now()-t0).toSec()<2)
        {
          diag_msg.status.at(0).values.at(1).key="waiting the end of the warm-up (2sec)";
          diag_msg.status.at(0).values.at(1).value="";
        }
        else
        {
          diag_msg.status.at(0).values.at(1).key="stop the error for safety";
          diag_msg.status.at(0).values.at(1).value="";
        }
        
        m_diagnostics_pub.publish(diag_msg);
      }
      
      double deflection_estimated_from_torque=-m_js_msg->effort.at(4*idx)/ m_spring_elasticity.at(idx);
      double error=m_js_msg->position.at(4*idx+2)-deflection_estimated_from_torque;
      if (std::abs(error*m_spring_elasticity.at(idx))>50 )
      {
        diagnostic_msgs::DiagnosticArray diag_msg;
        
        diag_msg.header.stamp=ros::Time::now();
        diag_msg.status.resize(1);
        diag_msg.status.at(0).name="NDLCOM SPRING";
        diag_msg.status.at(0).hardware_id="FourByThreeRobot";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_msg.status.at(0).message=m_js_red_msg->name.at(idx)+" has a wrong spring zero value";
        diag_msg.status.at(0).values.resize(3);
        
        diag_msg.status.at(0).values.at(0).key="JOINT";
        diag_msg.status.at(0).values.at(0).value=m_js_red_msg->name.at(idx);

        diag_msg.status.at(0).values.at(1).key="Suggestion 1";
        diag_msg.status.at(0).values.at(1).value="Try to restart the robot in the bugfix position";
        diag_msg.status.at(0).values.at(2).key="Suggestion 2";
        diag_msg.status.at(0).values.at(2).value="Software will try to estimated the correct spring position. Accuracy can be affected!";
        
        m_diagnostics_pub.publish(diag_msg);
        
      }
      
      if (m_adapt_spring_zeros && valid_torque)
      {
        m_motor_offset.at(idx)  = -m_adapt_coefficient*error+m_motor_offset.at(idx);
        m_spring_offset.at(idx) = m_adapt_coefficient*error+m_spring_offset.at(idx);
      }
    }
    if ( (ros::Time::now()>(m_adapt_start_time+m_adapt_time)) && m_adapt_spring_zeros )
    {
      ROS_INFO("END OF ADAPTATION");
      m_nh.setParam("ndlcom/motor_offset",m_motor_offset);
      m_adapt_spring_zeros=false;
    }
    
    m_js_pub.publish(m_js_msg);
    m_js_red_pub.publish(m_js_red_msg);
    m_js_redlink_pub.publish(m_js_redlink_msg);
    m_js_elastic_pub.publish(m_js_elastic_msg);
    m_ji_pub.publish(m_ji_msg);
    m_jvolt_pub.publish(m_jvoltage_msg);
    m_jcurr_pub.publish(m_jcurrent_msg);
  }
}

void NdlcomDriver::write()
{
  double write_rate = 1000;
  ros::Rate lp(write_rate);
  
  bool ndlcom_thread_stop = true;
  

  while (true)
  {
    m_ndlcom_thread_stop_mtx.lock();
    ndlcom_thread_stop = m_ndlcom_thread_stop;
    m_ndlcom_thread_stop_mtx.unlock();
    
    if (ndlcom_thread_stop)
      break;
    
    // get messages to be sent
    m_mngr_jnts_mtx.lock();
    m_mngr_jnts.run();
    m_mngr_jnts.getCommandMessages( m_outmsg );
    m_mngr_jnts.getMessages( m_outmsg );
    m_mngr_jnts_mtx.unlock();

    unsigned int send_len = 0;
    std::size_t m = 0;
    
    for(m = 0; m < m_outmsg.size(); m++ ) 
    {
      // encode the message to raw data
      int toSendDataLen = ndlcomEncode( m_encoder_buf, 
                                        c_NDLCOM_PARSER_BUFFER_SIZE, 
                                        &m_outmsg[m].header,
                                        (const void *)&m_outmsg[m].data[0]);
      
      // send the raw data
      UART_send(m_port_handle, m_encoder_buf, toSendDataLen);
      send_len += toSendDataLen;
      
      // make a break if we sent a lot
      if (send_len > m_send_len_per_cycle)
      {
        ROS_WARN("[NdlcomDriver::write] too many data! send_len=%u, max_send_len=%u", send_len, m_send_len_per_cycle);
        send_len = 0;
        lp.sleep();
      }
    }
    // remove sent messages from outbox
    m_outmsg.clear();
    lp.sleep();
  }
  return;
}



}


