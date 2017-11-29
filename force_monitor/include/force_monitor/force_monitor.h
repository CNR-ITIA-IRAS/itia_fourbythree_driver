#ifndef __FORCE_MONITOR__
#define __FORCE_MONITOR__

#include <thread>
#include <mutex>
#include <iostream>
#include <fstream> 

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <itia_rutils/itia_rutils.h>
#include <std_msgs/Int64.h>
#include <boost/graph/graph_concepts.hpp>

namespace itia
{
    
  class ForceMonitor : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    ~ForceMonitor();
  protected:
    std::thread m_thread;
    
    std::vector<double> m_max_torque_threshold;
    std::vector<double> m_torque_threshold;
    
    std::vector<double> m_max_force_threshold;
    std::vector<double> m_force_threshold;
    
    bool m_stop;
    void main();
    
    void torqueLimitCb(const sensor_msgs::JointStateConstPtr& msg);
    void forceLimitCb(const geometry_msgs::WrenchConstPtr& msg);
  };
  
}

# endif