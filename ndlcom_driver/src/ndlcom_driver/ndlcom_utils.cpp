
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
#include <itia_futils/itia_futils.h>
#include <boost/graph/graph_concepts.hpp>

namespace ndlcom_ros{
  
NdlcomDriver::NdlcomDriver():
m_ndlcom_data(256)
{
  m_js_pub          = m_nh.advertise<sensor_msgs::JointState>("fb/joint_states",1);
  m_missing_data    = m_nh.advertise<std_msgs::Bool>("missing_data",1);
  m_js_red_pub      = m_nh.advertise<sensor_msgs::JointState>("motor/joint_states",1);
  m_js_redlink_pub  = m_nh.advertise<sensor_msgs::JointState>("link/joint_states",1);
  m_js_elastic_pub  = m_nh.advertise<sensor_msgs::JointState>("elastic/joint_states",1);
  m_jt_sub          = m_nh.subscribe("sp/joint_states",1,&NdlcomDriver::callbackJointTarget,this);
  m_ji_pub          = m_nh.advertise<itia_msgs::JointInfo>("fb/temperatures",1);
  m_jvolt_pub       = m_nh.advertise<itia_msgs::JointInfo>("fb/voltages",1);
  m_jcurr_pub       = m_nh.advertise<itia_msgs::JointInfo>("fb/currents",1);
  m_diagnostics_pub = m_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  
  m_ji_msg.reset(new itia_msgs::JointInfo);
  m_jvoltage_msg.reset(new itia_msgs::JointInfo);
  m_jcurrent_msg.reset(new itia_msgs::JointInfo);
  m_js_msg.reset(new sensor_msgs::JointState);
  m_js_red_msg.reset(new sensor_msgs::JointState);
  m_js_redlink_msg.reset(new sensor_msgs::JointState);
  m_js_elastic_msg.reset(new sensor_msgs::JointState);
  
  m_message_size = 0;
  m_ndlcom_msg_rx.reset(new ndlcom_device_driver::NDLComMessage());
  m_ndlcom_msg_pub.reset(new ndlcom_device_driver::NDLComMessage());
  m_ndlcom_msg_shared.reset(new ndlcom_device_driver::NDLComMessage());
  m_ndlcom_msg_rx->data.reserve(257);
  m_ndlcom_msg_pub->data.reserve(257);
  m_ndlcom_msg_shared->data.reserve(257);
  m_ndlcom_msg_shared_new_data = false;
  m_ndlcom_thread_stop = false;
  
  std::string par_name="ndlcom/port";
  if (!m_nh.getParam(par_name,m_port))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }

  par_name="ndlcom/serialcom_timeout";
  int tmp;
  if (!m_nh.getParam(par_name,tmp))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }
  m_receive_timeout_us=tmp;

  par_name="ndlcom/jointcom_timeout";
  double dtmp;
  if (!m_nh.getParam(par_name,dtmp))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }
  ROS_DEBUG("[NdlcomDriver::NdlcomDriver] 'ndlcom/jointcom_timeout'=%f",dtmp);
  m_cfg_gen.communicationTimeoutSeconds = dtmp;
  
  m_cfg_gen.outboxMaxNumMessages = 4;
  
  m_cfg_gen.nrConfigureTriesMax = 1;
  
  
  par_name="ndlcom/configcom_timeout";
  if (!m_nh.getParam(par_name,dtmp))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }
  m_cfg_gen.configureTimeoutSeconds = dtmp;
  
  par_name="ndlcom/power_supply_current";
  if (!m_nh.getParam(par_name,m_power_supply_current))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }
  
  
  m_cfg_gen.keepSendingLastCommandTimeoutSeconds = 0;
  m_cfg_gen.disableBeforeConfigure = false;
  
  m_cfg_gen.disableBeforeConfigure=true;
  
  par_name="ndlcom/serialcom_baudrate";
  if (!m_nh.getParam(par_name,tmp))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }
  m_baud=tmp;
  
  
  m_parsed_size_total=0;
  m_receive_timeout.tv_usec = m_receive_timeout_us;

  // rough approx. of how many bytes we can send per 1 ms
  m_send_len_per_cycle = (double)m_receive_timeout.tv_usec * (double)m_baud / 10.0 / 1000000.0;
  if (m_send_len_per_cycle < 1)
    m_send_len_per_cycle = 1;
    
  
  // General config for all joints of chain
  
  std::vector<std::string> joint_names;
  par_name="controlled_joint_names";
  if (!m_nh.getParam(par_name,joint_names))
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
  }
  m_nJoints=joint_names.size();
  ROS_DEBUG("[NdlcomDriver::NdlcomDriver] Loading %d joints..",m_nJoints);
  
  
  m_motor_offset.resize(m_nJoints);
  m_link_offset.resize(m_nJoints);
  m_spring_offset.resize(m_nJoints);
	m_spring_elasticity.resize(m_nJoints);
  
  m_cfg_jnts.resize(m_nJoints);
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
  
  m_js_elastic_msg->name.resize(2*m_nJoints);
  m_js_elastic_msg->position.resize(2*m_nJoints);
  m_js_elastic_msg->velocity.resize(2*m_nJoints);
  m_js_elastic_msg->effort.resize(2*m_nJoints);

  m_ji_msg->name.resize(m_nJoints);
  m_ji_msg->data.resize(m_nJoints);
  
  m_jvoltage_msg->name.resize(m_nJoints);
  m_jvoltage_msg->data.resize(m_nJoints);
  
  m_jcurrent_msg->name.resize(m_nJoints+1);
  m_jcurrent_msg->data.resize(m_nJoints+1);
  m_jcurrent_msg->name.at(m_nJoints)="total";
  m_voltage.resize(m_nJoints);
  m_current.resize(m_nJoints+1);
  std::fill(m_voltage.begin(),m_voltage.end(),0.0);
  std::fill(m_current.begin(),m_current.end(),0.0);
  
  urdf::Model model;
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("Urdf '/robot_description' does not exist");
    throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] rdf '/robot_description' does not exist!");
  }
 
  for (int iAx=0;iAx<m_nJoints;iAx++)
  {
    std::string name_string=joint_names.at(iAx);
    
    m_ji_msg->name.at(iAx) =name_string;
    m_jcurrent_msg->name.at(iAx) =name_string;
    m_jvoltage_msg->name.at(iAx) =name_string;
    m_js_red_msg->name.at(iAx)   = name_string;
    m_js_redlink_msg->name.at(iAx)   = name_string;
    m_js_elastic_msg->name.at(2*iAx)   = name_string+"_motor";
    m_js_elastic_msg->name.at(2*iAx+1)   = name_string+"_spring";
    m_js_msg->name.at(4*iAx)   = name_string+"_motor_pos";
    m_js_msg->name.at(4*iAx+1)   = name_string+"_motor_abs_pos";
    m_js_msg->name.at(4*iAx+2) = name_string+"_spring";
    m_js_msg->name.at(4*iAx+3) = name_string+"_link";
    m_cfg_jnts.at(iAx).jointName = "FourByThree/FourByThreeArm/"+name_string; 
    m_cfg_jnts.at(iAx).configureRegisters.resize(2);
    
    std::string device_class;
    par_name="ndlcom/joint_"+std::to_string(iAx+1)+"/device_class";
    if (!m_nh.getParam(par_name,device_class))
    {
      ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
      throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
    }
    m_cfg_jnts.at(iAx).configureRegisters[0].deviceClassName = device_class;
    m_cfg_jnts.at(iAx).configureRegisters[0].registers.push_back(ndlcom_device_driver::Register("CONFIG", -1, 0.0, ""));
    m_cfg_jnts.at(iAx).configureRegisters[0].registers[0].bits.push_back(ndlcom_device_driver::RegisterBit());
    m_cfg_jnts.at(iAx).configureRegisters[0].registers[0].bits[0].name = "RESET_ERRORS";
    m_cfg_jnts.at(iAx).configureRegisters[0].registers[0].bits[0].value = 1;
    
    std::vector<std::string> ignored_errors;
    par_name="ndlcom/joint_"+std::to_string(iAx+1)+"/ignored_errors";
    if (!m_nh.getParam(par_name,ignored_errors))
    {
      ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
      throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
    }
    
    m_cfg_jnts.at(iAx).configureRegisters[0].registers.push_back(ndlcom_device_driver::Register("ERROR_IGNORE", -1, 0.0, ""));
    for (unsigned int iErr = 0;iErr<ignored_errors.size();iErr++)
      m_cfg_jnts.at(iAx).configureRegisters[0].registers[1].bits.push_back(ndlcom_device_driver::RegisterBit(ignored_errors.at(iErr), -1, 1));

    std::string controller_class;
    par_name="ndlcom/joint_"+std::to_string(iAx+1)+"/controller_class";
    if (!m_nh.getParam(par_name,controller_class))
    {
      ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
      throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
    }
    m_cfg_jnts.at(iAx).configureRegisters[1].deviceClassName = controller_class.c_str();
    int control_mode = 3;
    par_name="ndlcom/joint_"+std::to_string(iAx+1)+"/control_mode";
    if (!m_nh.getParam(par_name,control_mode))
    {
      ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
      throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
    }
    m_cfg_jnts.at(iAx).configureRegisters[1].registers.push_back(ndlcom_device_driver::Register("CTRL_MODE", -1, control_mode, ""));
		
		double elasticity;
		par_name="ndlcom/joint_"+std::to_string(iAx+1)+"/elasticity";
		if (!m_nh.getParam(par_name,elasticity))
		{
			ROS_ERROR("[NdlcomDriver::NdlcomDriver] cannot find '%s' parameter!",par_name.c_str());
			throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
		}
		m_spring_elasticity.at(iAx)=elasticity;
		
    
    std::map<std::string,double> map;
    if (m_nh.getParam("ndlcom/joint_"+std::to_string(iAx+1)+"/registers",map))
    {
      for (auto const &entry: map)
      {
        if (!entry.first.compare("POS_MAX"))
        {
          ROS_WARN("POS_MAX IS NOW SET FROM URDF FILE");
        }
        else if (!entry.first.compare("POS_MIN"))
        {
          ROS_WARN("POS_MIN IS NOW SET FROM URDF FILE");
        }
        else 
        {
          ROS_DEBUG("WRITING %s = %f",entry.first.c_str(),entry.second);
          m_cfg_jnts.at(iAx).configureRegisters[1].registers.push_back(ndlcom_device_driver::Register(entry.first, -1, entry.second, ""));
        }
      }
    }
    
    if (!model.getJoint(joint_names.at(iAx)))
    {
      ROS_ERROR_STREAM("Joint name "+joint_names.at(iAx)+" is not part of robot_description, where there are the following joints:");
      for( auto j : model.joints_ )
        std::cout << "name: " << j.first << std::endl;
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="NDLCOM_DRIVER";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_msg.status.at(0).message="Joint name "+joint_names.at(iAx)+" is not part of robot_description";
      diag_msg.status.at(0).values.resize(1);
      diag_msg.status.at(0).values.at(0).key="MISSING JOINT";
      diag_msg.status.at(0).values.at(0).value=joint_names.at(iAx);
      m_diagnostics_pub.publish(diag_msg);
      throw std::invalid_argument("[NdlcomDriver::NdlcomDriver] cannot find parameters!");
    }
    m_cfg_jnts.at(iAx).configureRegisters[1].registers.push_back(ndlcom_device_driver::Register("POS_MAX", -1, model.getJoint(joint_names.at(iAx))->limits->upper+0.1, ""));
    m_cfg_jnts.at(iAx).configureRegisters[1].registers.push_back(ndlcom_device_driver::Register("POS_MIN", -1, model.getJoint(joint_names.at(iAx))->limits->lower-0.1, ""));
    m_cfg_jnts.at(iAx).configureRegisters[1].registers.push_back(ndlcom_device_driver::Register("POS_MAX_HARD_LIMIT", -1, model.getJoint(joint_names.at(iAx))->limits->upper+0.13, ""));
    m_cfg_jnts.at(iAx).configureRegisters[1].registers.push_back(ndlcom_device_driver::Register("POS_MIN_HARD_LIMIT", -1, model.getJoint(joint_names.at(iAx))->limits->lower-0.13, ""));
    
    
    std::map<std::string,double> map2;
    if (m_nh.getParam("ndlcom/joint_"+std::to_string(iAx+1)+"/device_registers",map2))
    {
      for (auto const &entry: map2)
      {
        ROS_DEBUG("WRITING %s = %f",entry.first.c_str(),entry.second);
        m_cfg_jnts.at(iAx).configureRegisters[0].registers.push_back(ndlcom_device_driver::Register(entry.first, -1, entry.second, ""));
      }
    }
    
    m_motor_offset.at(iAx)=0.0;
    m_spring_offset.at(iAx)=0.0;
    m_link_offset.at(iAx)=0.0;

    
    
  }

  if (m_nh.getParam("ndlcom/motor_offset",m_motor_offset))
  {
    if (m_motor_offset.size()!=m_nJoints)
    {
      ROS_ERROR("ndlcom/motor_offset has wrong dimension");
      std::fill(m_motor_offset.begin(),m_motor_offset.end(),0.0);
      std::fill(m_spring_offset.begin(),m_spring_offset.end(),0.0);
    }
    else
    {
      for (unsigned int i=0;i<m_nJoints;i++)
        m_spring_offset.at(i)=-m_motor_offset.at(i);
    }
  }
  else
  {
    std::fill(m_motor_offset.begin(),m_motor_offset.end(),0.0);
    std::fill(m_spring_offset.begin(),m_spring_offset.end(),0.0);
  }
  
    // add the joint info
  m_mngr_jnts.addJoints(m_cfg_gen, m_cfg_jnts);
  
  
  std::vector<ndlcom_serialelasticjoints::SerialElasticJoint*> jnts = m_mngr_jnts.getJoints();
  
  // a command struct for high-level interfacing
  m_cmd_jnts.resize(m_cfg_jnts.size());
  // also needs the same joint name added
  for (std::size_t i=0; i< m_cmd_jnts.size(); i++)
      m_cmd_jnts.names[i] = m_cfg_jnts[i].jointName;

  m_ndlcom_parser = ndlcomParserCreate(m_parser_buf,
                                      c_NDLCOM_PARSER_BUFFER_SIZE );
  
  // check if there are Errors on creation
  if ( m_ndlcom_parser == NULL ) 
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] serial_ndlcom: did not get a parser, check buffer size ");
    stopThread();
    return;
  }
  
  // open the serail port
  ROS_DEBUG("[NdlcomDriver::NdlcomDriver] open UART connection with '%s' with baudrate=%d",m_port.c_str(),m_baud);
  m_port_handle = UART_connect( m_port.c_str(), 
                                m_baud, 
                                c_NO_PARITY, 
                                &m_oldtio);

  if (m_port_handle < 1) 
  {
    ROS_ERROR("[NdlcomDriver::NdlcomDriver] ERROR(Serialcom): UART_connect failed, 'm_port_handle=%d'.",m_port_handle);
    
    diagnostic_msgs::DiagnosticArray diag_msg;
    
    diag_msg.header.stamp=ros::Time::now();
    diag_msg.status.resize(1);
    diag_msg.status.at(0).hardware_id=m_port;
    diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
    diag_msg.status.at(0).message="Unable to open the port to connect the NDLCOM DRIVER to the robot";
    diag_msg.status.at(0).values.resize(4);
    
    diag_msg.status.at(0).values.at(0).key="PORT";
    diag_msg.status.at(0).values.at(0).value=m_port;
    diag_msg.status.at(0).values.at(1).key="BAUDRATE";
    diag_msg.status.at(0).values.at(1).value=std::to_string(m_baud);

    diag_msg.status.at(0).values.at(2).key="Suggestion 1";
    diag_msg.status.at(0).values.at(2).value="Check if the USB cable is well connected";
    diag_msg.status.at(0).values.at(3).key="Suggestion 2";
    diag_msg.status.at(0).values.at(3).value="Check if the USB port number is correct. In a terminal run 'dmesg | grep ttyUSB' and update ndlcom_drive/cfg/ndlcom_configuration.yaml";
    
    m_diagnostics_pub.publish(diag_msg);
    
    stopThread();
    return;
  }
  ROS_INFO("[NdlcomDriver::NdlcomDriver] Connection on port '%s' open",m_port.c_str());
  
  ROS_INFO("[NdlcomDriver::NdlcomDriver] Waiting for joints configuration..");
  
  m_joints = m_mngr_jnts.getJoints();



  std::string controller_name = "ndlcom";
  std::string namespace_name = controller_name + "/position_filter/";
  // position
  par_name = namespace_name+"A";
  Eigen::MatrixXd pos_filt_A;
  if (!itia::rutils::getParam(m_nh, par_name, pos_filt_A))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"B";
  Eigen::MatrixXd pos_filt_B;
  if (!itia::rutils::getParam(m_nh, par_name, pos_filt_B))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"Baw";
  Eigen::MatrixXd pos_filt_Baw;
  if (!itia::rutils::getParam(m_nh, par_name, pos_filt_Baw))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"C";
  Eigen::MatrixXd pos_filt_C;
  if (!itia::rutils::getParam(m_nh, par_name, pos_filt_C))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"D";
  Eigen::MatrixXd pos_filt_D;
  if (!itia::rutils::getParam(m_nh, par_name, pos_filt_D))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"max_output";
  Eigen::MatrixXd pos_filt_max_output;
  if (!itia::rutils::getParam(m_nh, par_name, pos_filt_max_output))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"min_output";
  Eigen::MatrixXd pos_filt_min_output;
  if (!itia::rutils::getParam(m_nh, par_name, pos_filt_min_output))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }    
        
  Eigen::MatrixXd pos_filt_initial_state(pos_filt_A.rows(), 1);
  pos_filt_initial_state.setZero();
 
  // velocity
  namespace_name = controller_name + "/velocity_filter/";
  par_name = namespace_name+"A";
  Eigen::MatrixXd vel_filt_A;
  if (!itia::rutils::getParam(m_nh, par_name, vel_filt_A))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"B";
  Eigen::MatrixXd vel_filt_B;
  if (!itia::rutils::getParam(m_nh, par_name, vel_filt_B))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }

  par_name = namespace_name+"Baw";
  Eigen::MatrixXd vel_filt_Baw;
  if (!itia::rutils::getParam(m_nh, par_name, vel_filt_Baw))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"C";
  Eigen::MatrixXd vel_filt_C;
  if (!itia::rutils::getParam(m_nh, par_name, vel_filt_C))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"D";
  Eigen::MatrixXd vel_filt_D;
  if (!itia::rutils::getParam(m_nh, par_name, vel_filt_D))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"max_output";
  Eigen::MatrixXd vel_filt_max_output;
  if (!itia::rutils::getParam(m_nh, par_name, vel_filt_max_output))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"min_output";
  Eigen::MatrixXd vel_filt_min_output;
  if (!itia::rutils::getParam(m_nh, par_name, vel_filt_min_output))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }    
        
  Eigen::MatrixXd vel_filt_initial_state(vel_filt_A.rows(), 1);
  vel_filt_initial_state.setZero();
  
  // effort
  namespace_name = controller_name + "/effort_filter/";
  par_name = namespace_name+"A";
  Eigen::MatrixXd eff_filt_A;
  if (!itia::rutils::getParam(m_nh, par_name, eff_filt_A))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"B";
  Eigen::MatrixXd eff_filt_B;
  if (!itia::rutils::getParam(m_nh, par_name, eff_filt_B))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }

  par_name = namespace_name+"Baw";
  Eigen::MatrixXd eff_filt_Baw;
  if (!itia::rutils::getParam(m_nh, par_name, eff_filt_Baw))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"C";
  Eigen::MatrixXd eff_filt_C;
  if (!itia::rutils::getParam(m_nh, par_name, eff_filt_C))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"D";
  Eigen::MatrixXd eff_filt_D;
  if (!itia::rutils::getParam(m_nh, par_name, eff_filt_D))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"max_output";
  Eigen::MatrixXd eff_filt_max_output;
  if (!itia::rutils::getParam(m_nh, par_name, eff_filt_max_output))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }
  
  par_name = namespace_name+"min_output";
  Eigen::MatrixXd eff_filt_min_output;
  if (!itia::rutils::getParam(m_nh, par_name, eff_filt_min_output))
  {
    ROS_ERROR("[DiscreteStateSpaceController] cannot find '%s' parameter!",par_name.c_str());
    return;
  }    
        
  Eigen::MatrixXd eff_filt_initial_state(eff_filt_A.rows(), 1);
  eff_filt_initial_state.setZero();

  
  m_q_filter.resize(m_nJoints*4);
  m_Dq_filter.resize(m_nJoints*4);
  m_effort_filter.resize(m_nJoints);
  
  for (int idx = 0;idx<m_nJoints;idx++)
  {
    m_q_filter.at(4*idx).reset(    new itia::control::DiscreteStateSpace(pos_filt_A, pos_filt_B, pos_filt_Baw, pos_filt_C, pos_filt_D, pos_filt_max_output.col(0), pos_filt_min_output.col(0), pos_filt_initial_state.col(0)));
    m_q_filter.at(4*idx+1).reset(  new itia::control::DiscreteStateSpace(pos_filt_A, pos_filt_B, pos_filt_Baw, pos_filt_C, pos_filt_D, pos_filt_max_output.col(0), pos_filt_min_output.col(0), pos_filt_initial_state.col(0)));
    m_q_filter.at(4*idx+2).reset(  new itia::control::DiscreteStateSpace(pos_filt_A, pos_filt_B, pos_filt_Baw, pos_filt_C, pos_filt_D, pos_filt_max_output.col(0), pos_filt_min_output.col(0), pos_filt_initial_state.col(0)));
    m_q_filter.at(4*idx+3).reset(  new itia::control::DiscreteStateSpace(pos_filt_A, pos_filt_B, pos_filt_Baw, pos_filt_C, pos_filt_D, pos_filt_max_output.col(0), pos_filt_min_output.col(0), pos_filt_initial_state.col(0)));
    
    m_Dq_filter.at(4*idx).reset(   new itia::control::DiscreteStateSpace(vel_filt_A, vel_filt_B, vel_filt_Baw, vel_filt_C, vel_filt_D, vel_filt_max_output.col(0), vel_filt_min_output.col(0), vel_filt_initial_state.col(0)));
    m_Dq_filter.at(4*idx+1).reset( new itia::control::DiscreteStateSpace(vel_filt_A, vel_filt_B, vel_filt_Baw, vel_filt_C, vel_filt_D, vel_filt_max_output.col(0), vel_filt_min_output.col(0), vel_filt_initial_state.col(0)));
    m_Dq_filter.at(4*idx+2).reset( new itia::control::DiscreteStateSpace(vel_filt_A, vel_filt_B, vel_filt_Baw, vel_filt_C, vel_filt_D, vel_filt_max_output.col(0), vel_filt_min_output.col(0), vel_filt_initial_state.col(0)));
    m_Dq_filter.at(4*idx+3).reset( new itia::control::DiscreteStateSpace(vel_filt_A, vel_filt_B, vel_filt_Baw, vel_filt_C, vel_filt_D, vel_filt_max_output.col(0), vel_filt_min_output.col(0), vel_filt_initial_state.col(0)));
    
    m_effort_filter.at(idx).reset( new itia::control::DiscreteStateSpace(eff_filt_A, eff_filt_B, eff_filt_Baw, eff_filt_C, eff_filt_D, eff_filt_max_output.col(0), eff_filt_min_output.col(0), eff_filt_initial_state.col(0)));
    
  }
  
  m_adapt_spring_zeros=false;
  m_adapt_time=ros::Duration(5);
  m_adapt_coefficient=0.01;
};

NdlcomDriver::~NdlcomDriver()
{
  shutdown();
  UART_disconnect(m_port_handle, &m_oldtio);
  m_port_handle=-1;
}



void NdlcomDriver::callbackJointTarget(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_DEBUG_THROTTLE(0.5, "NdlcomDriver::callbackJointTarget] callback received!");
  if (m_mngr_jnts.getState() == ndlcom_device_driver::JS_ENABLED)
  {
    for(int idx=0;idx<m_nJoints;idx++)
    {
      m_cmd_jnts[idx].position = msg->position.at(idx)+m_motor_offset.at(idx);
      m_cmd_jnts[idx].speed    = msg->velocity.at(idx);
      m_cmd_jnts[idx].effort   = msg->effort.at(idx);
    }
    m_mngr_jnts.handleCommand( m_cmd_jnts, true );
  }
  else
  {
    ROS_WARN_THROTTLE(0.5, "[NdlcomDriver::callbackJointTarget] Joints are not enabled!");
    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp=ros::Time::now();
    diag_msg.status.resize(1);
    diag_msg.status.at(0).name="NDLCOM DRIVER";
    diag_msg.status.at(0).hardware_id="FourByThreeRobot";
    diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::WARN;
    diag_msg.status.at(0).message="Joints are not enabled";
    diag_msg.status.at(0).values.resize(0);
    
    m_diagnostics_pub.publish(diag_msg);
    
  }
}



bool NdlcomDriver::callbackJointsEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  return waitForEnable(ros::Duration(10));
}

bool NdlcomDriver::callbackJointsDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  return waitForDisable(ros::Duration(10));
}

bool NdlcomDriver::callbackResetZeros(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  double time=10;
  if (!m_nh.getParam("configuration_time",time))
    time=10;
  m_adapt_time=ros::Duration(time);
  m_adapt_coefficient=0.001;
  m_adapt_start_time=ros::Time::now();
  m_adapt_spring_zeros=true;
  
  return true;
}



sensor_msgs::JointStatePtr NdlcomDriver::getJointState()
{
  return m_js_msg;
}

void NdlcomDriver::shutdown()
{
// TODO
}

bool NdlcomDriver::waitForConfigure(const ros::Duration& timeout)
{
  ros::Time t0 = ros::Time::now();
  ros::Rate lp(1000);
  while ( (ros::Time::now()-t0) < timeout  && ros::ok() )
  {
    lp.sleep();
    if (m_mngr_jnts.isConfigured())
      return true;
  }
  ROS_WARN("Robot configuring failed");
  
  std::vector<ndlcom_serialelasticjoints::SerialElasticJoint*> joints = m_mngr_jnts.getJoints();
  t0 = ros::Time::now();
  for (unsigned int idx = 0;idx<joints.size();idx++)
  {
    std::queue< ndlcom_device_driver::NDLComMessage > outbox;
    joints.at(idx)->getRegister(ndlcom_serialelasticjoints::JointDevice::CONFIG )->setRefBit( ndlcom_serialelasticjoints::JointDevice::CONFIG_RESET_ERRORS_BIT, true );
    joints.at(idx)->generateRegisterWriteMessage( ndlcom_serialelasticjoints::JointDevice::CONFIG, outbox );
    
    while ( (outbox.size()>0) && ( (ros::Time::now()-t0).toSec() < 2) )
    {
      m_outmsg.push_back(outbox.front());
      outbox.pop();
    }
  }
  
  diagnostic_msgs::DiagnosticArray diag_msg;
  
  diag_msg.header.stamp=ros::Time::now();
  diag_msg.status.resize(1);
  diag_msg.status.at(0).hardware_id="FourByThreeRobot";
  diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
  diag_msg.status.at(0).message="Robot configuring failed";
  diag_msg.status.at(0).values.resize(1);
  
  diag_msg.status.at(0).values.at(0).key="Suggestion 1";
  diag_msg.status.at(0).values.at(0).value="Try again";
  
  m_diagnostics_pub.publish(diag_msg);

  ROS_WARN("shutdown");
  return false;
}

bool NdlcomDriver::waitForEnable(const ros::Duration& timeout)
{
  ros::Time t0 = ros::Time::now();
  ros::Rate lp(1000);
  m_mngr_jnts.enable();
  while ( (ros::Time::now()-t0) < timeout  && ros::ok() )
  {
    lp.sleep();
    if (m_mngr_jnts.getState() == ndlcom_device_driver::JS_ENABLED)
    {
      ROS_INFO("ROBOT ENABLED IN %f seconds",(ros::Time::now()-t0).toSec());
      return true;
    }
  }
  ROS_WARN("Robot enabling failed");
  return false;
}


bool NdlcomDriver::waitForDisable(const ros::Duration& timeout)
{
  ros::Time t0 = ros::Time::now();
  ros::Rate lp(1000);
  m_mngr_jnts.disable();
  while ( (ros::Time::now()-t0) < timeout  && ros::ok() )
  {
    lp.sleep();
    if (m_mngr_jnts.getState() == ndlcom_device_driver::JS_DISABLED)
      return true;
  }
  ROS_WARN("Robot disabling failed");
  return false;
}



}
