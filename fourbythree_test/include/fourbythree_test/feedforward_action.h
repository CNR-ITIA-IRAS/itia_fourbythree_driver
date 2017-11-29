#ifndef __IDENTIFICATION_FOR_FEEDFORWARD_ACTION__
#define __IDENTIFICATION_FOR_FEEDFORWARD_ACTION__

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
#include <boost/graph/graph_concepts.hpp>

namespace itia{
namespace identification{

class FeedforwardActionNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  
protected:
  std::thread m_main_thread;                                  // NOTE: std thread is moveable!
  
  bool m_stop;

  std::vector<std::vector<double>> m_sinusoidal_coeffs;
  std::vector<std::vector<double>> m_cosinusoidal_coeffs;
  std::vector<double> m_omega;
	std::string m_topic_name;
	
  double m_start_stop_transient;
  double m_test_duration;
	
	int m_signal;
	
  int m_nAx;
  int m_nsin;
  void mainThread();
  void stopThread();
  ~FeedforwardActionNodelet();
  bool loadParameters();
};


}
}

# endif