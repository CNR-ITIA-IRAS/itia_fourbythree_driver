
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

#include <fourbythree_test/multisine_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::identification::MultisineNodelet, nodelet::Nodelet) 



namespace itia{
namespace identification{



void itia::identification::MultisineNodelet::onInit()
{
  m_stop = false;
  m_main_thread  = std::thread(&itia::identification::MultisineNodelet::mainThread, this);  
}

itia::identification::MultisineNodelet::~MultisineNodelet()
{
  stopThread();
  m_main_thread.join();
  ROS_INFO("[MULTISINE NODELET '%s'] thread STOP", getName().c_str());
  
}

void MultisineNodelet::mainThread()
{
  ros::NodeHandle nh = getPrivateNodeHandle();
  double freq_rate = 1000.0;
  ros::Rate loop_rate(freq_rate);
  
  ros::Publisher target_js_pub = nh.advertise<sensor_msgs::JointState>("/joint_feedforward", 1);
  itia::rutils::MsgReceiver<sensor_msgs::JointState> js_rec("joint_state");
  ros::Subscriber js_sub = nh.subscribe<sensor_msgs::JointState>("/fb/joint_states_reduced_motor", 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &js_rec);
  itia::rutils::MsgReceiver<sensor_msgs::JointState> complete_js_rec("joint_state");
  ros::Subscriber complete_js_sub = nh.subscribe<sensor_msgs::JointState>("/fb/joint_states", 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &complete_js_rec);
  
  ros::ServiceClient load_nodelet = nh.serviceClient<nodelet::NodeletLoad>("/fourbythree_nodelet/load_nodelet");
  ros::ServiceClient unload_nodelet = nh.serviceClient<nodelet::NodeletUnload>("/fourbythree_nodelet/unload_nodelet");
  
  
  sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
  nodelet::NodeletLoad load_srv;
  load_srv.request.name = "logger";
  load_srv.request.type = "ndlcom_ros/NdlcomLoggerNodelet";
  load_srv.request.my_argv.resize(2);
  
  nodelet::NodeletUnload unload_srv;
  unload_srv.request.name = load_srv.request.name;
  unload_nodelet.call(unload_srv);
  
  if (!js_rec.waitForANewData(10))
  {
    ROS_ERROR("[MULTISINE NODELET '%s'] timeout in '%s' receiving", getName().c_str(), complete_js_sub.getTopic().c_str());
    stopThread();
    return;
  }
  
  int nAx = 2;
  msg->position.resize(nAx);
  msg->velocity.resize(nAx);
  msg->effort.resize(nAx);
  msg->name.resize(nAx);
  std::fill(msg->position.begin(), msg->position.end(), 0);
  std::fill(msg->velocity.begin(), msg->velocity.end(), 0);
  std::fill(msg->effort.begin(), msg->effort.end(), 0);
  std::vector<double> init_pos = js_rec.getData().position;
  msg->position = init_pos;

  
  double moving_axis;
  if (!nh.getParam("/fourbythree_trj/multisine/moving_axis", moving_axis))
  {
    ROS_ERROR("[MULTISINE NODELET '%s'] Parameter fourbythree_trj/multisine/moving_axis does not exist", getName().c_str());
    stopThread();
    return;
  }  
  
  std::string test_name;
  if (!nh.getParam("/fourbythree_trj/multisine/test_name", test_name))
  {
    ROS_ERROR("[MULTISINE NODELET '%s'] Parameter fourbythree_trj/multisine/test_name does not exist", getName().c_str());
    stopThread();
    return;
  } 
  
  std::string filename;
  if (!nh.getParam("/fourbythree_trj/multisine/signal", filename))
  {
    ROS_ERROR("[MULTISINE NODELET '%s'] Parameter fourbythree_trj/multisine/signal does not exist", getName().c_str());
    stopThread();
    return;
  } 
  
  
  std::ifstream excsignal_file(filename,std::ifstream::binary);
  if (!excsignal_file)
  {
    ROS_ERROR("[MULTISINE NODELET '%s'] excting signal file '%s' does not found", getName().c_str(), filename.c_str());
    stopThread();
    return;
  }  
  excsignal_file.seekg(0, excsignal_file.end);
  int file_length = excsignal_file.tellg();
  int length = file_length/8;                            
  excsignal_file.seekg(0, excsignal_file.beg);
  
  Eigen::VectorXd data(length);
  excsignal_file.read((char*)data.data(), file_length);
  excsignal_file.close();
  
  Eigen::VectorXd positions = data.block(0, 0, length/3, 1);
  Eigen::VectorXd velocities = data.block(length/3, 0, length/3, 1);
  Eigen::VectorXd torques = data.block((length*2)/3, 0, length/3, 1);
  
  
  for (int idx = 0;idx<50;idx++)
  {
    msg->header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    loop_rate.sleep();
  }
    
  load_srv.request.my_argv.at(1) = std::to_string((100+velocities.rows())/freq_rate);
  
  int iAx = moving_axis;
  load_srv.request.my_argv.at(0) = test_name +"_jnt"+std::to_string(iAx);
  load_nodelet.call(load_srv);
  for (int idx = 0;idx<50;idx++)
  {
    msg->header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    loop_rate.sleep();
  }
  
  ROS_INFO("JOINT %d", iAx);
  
  ROS_INFO("position rows=%zu", positions.rows());
  bool stop = false;
  for (int iR = 0;iR<positions.rows();iR++)
  {
    if (!ros::ok())
      break;
    
    
    m_stop_mtx.lock();
    stop = m_stop;
    m_stop_mtx.unlock();
    if (stop)
    {
      break;
    }
    
    msg->position.at(iAx) = init_pos.at(iAx) + positions(iR, 0);
    msg->velocity.at(iAx) = velocities(iR, 0);
    msg->effort.at(iAx) = torques(iR, 0);
    
    msg->header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    loop_rate.sleep();
  }
  msg->position.at(iAx) = init_pos.at(iAx);
  msg->velocity.at(iAx) = 0;
  msg->effort.at(iAx) = 0;
  ROS_INFO("qui");
      
  for (int idx = 0;idx<50;idx++)
  {
    msg->header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    loop_rate.sleep();
  }
  if (!stop)
    unload_nodelet.call(unload_srv);
  
  msg->position = init_pos;
  std::fill(msg->velocity.begin(), msg->velocity.end(), 0);
  std::fill(msg->effort.begin(), msg->effort.end(), 0);
  for (int idx = 0;idx<50;idx++)
  {
    msg->header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    loop_rate.sleep();
  }
  ROS_INFO("[MULTISINE NODELET '%s'] end of loggin thread", getName().c_str());
      
}

void MultisineNodelet::stopThread()
{
  ROS_INFO("[MULTISINE NODELET '%s'] stopping thread", getName().c_str());
  m_stop_mtx.lock();
  m_stop = true;
  m_stop_mtx.unlock();
}


}
}