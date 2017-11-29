#ifndef __NDLCOM_NODELET__
#define __NDLCOM_NODELET__

#include <nodelet/nodelet.h>
#include <ndlcom_driver/ndlcom_utils.h>
#include <iostream>
#include <fstream> 
#include <boost/graph/graph_concepts.hpp>
#include <std_srvs/Trigger.h>
namespace ndlcom_ros{
  


class NdlcomDriverNodeletFake : public nodelet::Nodelet
{
public:
  virtual void onInit();

protected:
  std::thread m_main_thread;
   std::mutex m_stop_mtx;
  bool m_stop;

  ros::Publisher          m_js_pub;
  ros::Publisher          m_ji_pub;
  ros::Publisher          m_missing_data;
  ros::Publisher          m_js_red_pub;
  ros::Publisher          m_js_redlink_pub;
  ros::Subscriber         m_jt_sub;

  sensor_msgs::JointStatePtr m_js_msg;
  sensor_msgs::JointStatePtr m_js_red_msg;
  sensor_msgs::JointStatePtr m_js_redlink_msg;
  itia_msgs::JointInfoPtr m_ji_msg;

  int m_nJoints;

  void main_thread();
  bool triggerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void callbackJointTarget(const sensor_msgs::JointState::ConstPtr& msg);
  ~NdlcomDriverNodeletFake();
};



}

# endif