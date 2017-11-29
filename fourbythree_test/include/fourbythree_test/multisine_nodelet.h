#ifndef __MULTISINE_NODELET__
#define __MULTISINE_NODELET__

#include<nodelet/nodelet.h>
#include<thread>
# include<mutex>
# include<ros/ros.h>
# include<itia_rutils/itia_rutils.h>
# include<sensor_msgs/JointState.h>
# include<nodelet/NodeletLoad.h>
# include<nodelet/NodeletUnload.h>
# include<nodelet/NodeletList.h>
# include<iostream>
# include<fstream>

namespace itia{
namespace identification{

class MultisineNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

protected:
  std::thread m_main_thread;                                  // NOTE: std thread is moveable!
  
  bool m_stop;
  std::mutex m_stop_mtx;
  
  void mainThread();
  void stopThread();
  ~MultisineNodelet();
};


}
}

# endif