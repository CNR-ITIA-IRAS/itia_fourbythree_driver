
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

#ifndef __NDL_COM_ROS_UTILS__
#define __NDL_COM_ROS_UTILS__


#include <termios.h>
#include <string.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
# include <std_msgs/Bool.h>

#include <ndlcom/Parser.h>
#include <ndlcom/Encoder.h>
#include <ndlcom_serialelasticjoints/SerialElasticJoint.h>
//# include <ndlcom_aila_joints/AILAJoint.h>
#include <ndlcom_device_driver/MultiJointManagerBase.h>
#include <std_srvs/Trigger.h>
# include <thread>
# include <mutex>
#include <boost/graph/graph_concepts.hpp>
# include <itia_msgs/JointInfo.h>
# include <std_srvs/Empty.h>
# include <itia_controllers_and_filters/discrete_state_space.h>
# include <itia_rutils/itia_rutils.h>
# include <diagnostic_msgs/DiagnosticArray.h>
#include <urdf/model.h>

extern "C" {
#include <ndlcom_driver/serial_communication.h>
}


namespace ndlcom_ros{
  enum motorState {
    WAITCONF, PREPARE, WAITEN, TRAJ, END
  }; 
  
  const int c_MAX_BUFFER_LENGTH=2048; // BUG: acm-irgedwas linux-treiber hat Probleme mit ]1,2048[ buffer size (DON'T CHANGE!)
  const int c_NDLCOM_PARSER_BUFFER_SIZE=1024;
  const int c_NO_PARITY=0;
  const int c_EVEN_PARITY=1;
  const int c_ODD_PARITY=2;

  
  class NdlcomDriver
  {
  private:
    unsigned int  m_baud;
    unsigned int  m_receive_timeout_us;
    unsigned int m_send_len_per_cycle; // rough approx. of how many bytes we can send per 1 ms
    char m_parser_buf[c_NDLCOM_PARSER_BUFFER_SIZE];
    char m_encoder_buf[c_NDLCOM_PARSER_BUFFER_SIZE];
    int m_parsed_size_total;    
    int m_port_handle;
    unsigned int m_nJoints;
    std::string   m_port;
    
    bool m_adapt_spring_zeros;
    ros::Duration m_adapt_time;
    ros::Time m_adapt_start_time;
    double m_adapt_coefficient;
    
    bool m_communication_error;
    
		bool m_first_read;
    bool m_new_data;
    int m_message_size;
    char m_data_buf[c_MAX_BUFFER_LENGTH];
    char m_serial_buf[c_MAX_BUFFER_LENGTH];
    
    ros::NodeHandle         m_nh;
    ros::Publisher          m_js_pub;
    ros::Publisher          m_ji_pub;
    ros::Publisher          m_jvolt_pub;
    ros::Publisher          m_jcurr_pub;
    ros::Publisher          m_missing_data;
    ros::Publisher          m_js_red_pub;
    ros::Publisher          m_js_redlink_pub;
    ros::Publisher          m_js_elastic_pub;
    ros::Publisher          m_diagnostics_pub;
    ros::Subscriber         m_jt_sub;

    
    sensor_msgs::JointStatePtr m_js_msg;
    sensor_msgs::JointStatePtr m_js_red_msg;
    sensor_msgs::JointStatePtr m_js_redlink_msg;
    sensor_msgs::JointStatePtr m_js_elastic_msg;
    itia_msgs::JointInfoPtr m_ji_msg;
    itia_msgs::JointInfoPtr m_jvoltage_msg;
    itia_msgs::JointInfoPtr m_jcurrent_msg;
    
    
    
    std::vector<ndlcom_serialelasticjoints::SerialElasticJoint*> m_joints;
    // communcation variables
    struct termios        m_oldtio;
    NDLComHeader          m_ndlcom_header;
    std::vector<uint8_t>  m_ndlcom_data;
    struct timeval m_receive_timeout;
    
    // joint variables
    boost::shared_ptr<ndlcom_device_driver::NDLComMessage> m_ndlcom_msg_rx;
    boost::shared_ptr<ndlcom_device_driver::NDLComMessage> m_ndlcom_msg_shared;
    boost::shared_ptr<ndlcom_device_driver::NDLComMessage> m_ndlcom_msg_pub;
    
    bool m_ndlcom_msg_shared_new_data;
    bool m_ndlcom_thread_stop;
    
    std::mutex m_ndlcom_msg_shared_mtx;
    std::mutex m_ndlcom_thread_stop_mtx;
    std::mutex m_mngr_jnts_mtx;
    std::vector<double> m_motor_offset;
    std::vector<double> m_link_offset;
		std::vector<double> m_spring_offset;
		std::vector<double> m_spring_elasticity;
		double m_power_supply_current;
    
    Eigen::VectorXd m_qmotor;
    Eigen::VectorXd m_qabsmotor;
    Eigen::VectorXd m_qspring;
    Eigen::VectorXd m_qlink;
    Eigen::VectorXd m_taumotor;
    std::vector<double> m_voltage;
    std::vector<double> m_current;
    
    ndlcom_device_driver::NDLComMessage m_ndlcom_msg;
    ndlcom_device_driver::JointConfigGeneral m_cfg_gen;// General config for all joints of chain
    ndlcom_device_driver::JointsConfig m_cfg_jnts;// Define which devices to control (node id for given name must match node id of your device)
    
    ndlcom_device_driver::MultiJointManagerBase<ndlcom_serialelasticjoints::SerialElasticJoint > m_mngr_jnts;
    ndlcom_device_driver::JointsInfo m_info_jnts;
    base::commands::Joints m_cmd_jnts;// a command struct for high-level interfacing
    std::vector< ndlcom_device_driver::NDLComMessage > m_outmsg;
    NDLComParser* m_ndlcom_parser;// NDLCom Message Parser
    
    
    std::vector<itia::control::DiscreteStateSpacePtr> m_q_filter;
    std::vector<itia::control::DiscreteStateSpacePtr> m_Dq_filter;
    std::vector<itia::control::DiscreteStateSpacePtr> m_effort_filter;
  
    void callbackJointTarget(const sensor_msgs::JointState::ConstPtr& msg);
    bool callbackJointsEnable(std_srvs::Trigger::Request& req,
                                  std_srvs::Trigger::Response& res);
    bool callbackJointsDisable(std_srvs::Trigger::Request& req,
                                  std_srvs::Trigger::Response& res);
                                  
    bool callbackResetZeros(std_srvs::Trigger::Request& req,
                                  std_srvs::Trigger::Response& res);
    
    
  public:
    NdlcomDriver();
    ~NdlcomDriver();
    
    void read();
    void spin();
    void write();
    void publish();
    sensor_msgs::JointStatePtr getJointState();
    
    bool waitForConfigure(const ros::Duration& timeout);
    bool waitForEnable(const ros::Duration& timeout);
    bool waitForDisable(const ros::Duration& timeout);
    void shutdown();
    void stopThread()
    {
      ROS_INFO("STOP THREADS");
      m_ndlcom_thread_stop = true;
      
    };
  };
}

#endif
