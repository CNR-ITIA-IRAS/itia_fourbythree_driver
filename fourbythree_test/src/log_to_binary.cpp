
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
#include <sensor_msgs/JointState.h>
#include <itia_rutils/itia_rutils.h>
#include <boost/graph/graph_concepts.hpp>
#include <rosbag/bag.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Bool.h>

sensor_msgs::JointStateConstPtr fb_msg, sp_msg;
bool fb_new_data = false;
bool sp_new_data = false;
std::mutex sp_mtx;
std::mutex fb_mtx;

void callback_fb(const sensor_msgs::JointStateConstPtr& msg)
{
//   fb_mtx.lock();
//   fb_msg = msg;
  fb_new_data = true;
//   fb_mtx.unlock();
}
void callback_sp(const sensor_msgs::JointStateConstPtr& msg)
{
//   sp_mtx.lock();
//   sp_msg = msg;
  sp_new_data = true;
//   sp_mtx.unlock();
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "log_4x3");
  ros::NodeHandle nh;
  ros::NodeHandle nh_fb;
  
  ros::CallbackQueue my_callback_queue;
  nh_fb.setCallbackQueue(&my_callback_queue);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::Rate loop_rate(1000);
  
  itia::rutils::MsgReceiver<sensor_msgs::JointState> fb_rec("joint_state");
  itia::rutils::MsgReceiver<sensor_msgs::JointState> sp_rec("joint_state");
  itia::rutils::MsgReceiver<std_msgs::Bool> startstop_rec("start_stop_log");
  
//   ros::Subscriber startstop_sub = nh.subscribe<std_msgs::Bool>("/start_stop_log", 1, &itia::rutils::MsgReceiver<std_msgs::Bool>::callback, &startstop_rec);
//   ros::Subscriber sp_sub = nh.subscribe<sensor_msgs::JointState>("/sp/joint_states", 1, callback_sp);
  ros::Subscriber fb_sub = nh_fb.subscribe<sensor_msgs::JointState>("/fb/joint_states_reduced_motor", 1, callback_fb);
  //fb_sub.
  sensor_msgs::JointState msg;
  ros::AsyncSpinner spinner(8); // Use 4 threads
  spinner.start();
  
  ros::AsyncSpinner spinner_fb(8, &my_callback_queue);      // Use 4 threads
  spinner_fb.start();
  
  
  unsigned int fail_sp = 0;
  unsigned int fail_fb = 0;
  unsigned int ic = 0;
  std::ofstream sp_file;
  sp_file.open ("spdata.bin", std::ios::out | std::ios::binary);
  std::ofstream fb_file;
  fb_file.open ("fbdata.bin", std::ios::out | std::ios::binary);
  sensor_msgs::JointStateConstPtr sp_msg_t, fb_msg_t;
//   while (startstop_rec.getMsgCounter() == 0)
//   {
//     
//     if (!ros::ok())
//     {
//       sp_file.close();
//       fb_file.close();
//     }
//   }
  ROS_INFO("start logging");
  ros::Time t0 = ros::Time::now();
  ros::Duration delta(5);
  while (ros::ok())
  {
//     if (!startstop_rec.getData().data)
//       break;
/*    
    if (!sp_new_data)
    {
      fail_sp++;
    }
    else
    {
      sp_mtx.lock();
      sp_new_data = false;
      sp_msg_t = sp_msg;
      sp_mtx.unlock();
      double time = sp_msg_t->header.stamp.toSec();
//       sp_file.write((char*)&(time), sizeof(double));
//       sp_file.write((char*)(sp_msg_t->position.data()), sizeof(double) * sp_msg_t->position.size());
//       sp_file.write((char*)(sp_msg_t->velocity.data()), sizeof(double) * sp_msg_t->velocity.size());
    }*/
    
    my_callback_queue.callOne();
    if (!fb_new_data)
    {
      fail_fb++;
    }
    else
    {
//       fb_mtx.lock();
      fb_new_data = false;
//       fb_msg_t = fb_msg;
//       fb_mtx.unlock();
//       double time = fb_msg_t->header.stamp.toSec();
//       fb_file.write((char*)&(time), sizeof(double));
//       fb_file.write((char*)(fb_msg_t->position.data()), sizeof(double) * fb_msg_t->position.size());
//       fb_file.write((char*)(fb_msg_t->velocity.data()), sizeof(double) * fb_msg_t->velocity.size());
//       fb_file.write((char*)(fb_msg_t->effort.data()), sizeof(double) * fb_msg_t->effort.size());
    }
    
    ic++;
    if ((ros::Time::now()-t0)>delta)
    {
      ROS_INFO("fail sp = %f%%,  fb = %f%%", (double)fail_sp/(double)ic*100.0, (double)fail_fb/(double)ic*100.0); 
      ic = 0;
      fail_sp = 0;
      fail_fb = 0;
      t0 = ros::Time::now();
    }
    loop_rate.sleep();
    
  }    
  sp_file.close();
  fb_file.close();
  return 0;
}
