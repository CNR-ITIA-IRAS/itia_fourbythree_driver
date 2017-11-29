#ifndef __NDLCOM_NODELET__
#define __NDLCOM_NODELET__

#include <nodelet/nodelet.h>
#include <ndlcom_driver/ndlcom_utils.h>
#include <iostream>
#include <fstream> 
#include <boost/graph/graph_concepts.hpp>

namespace ndlcom_ros{
  


class NdlcomDriverNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

protected:
  std::thread m_read_thread;                                  // NOTE: std thread is moveable!
  std::thread m_write_thread;
  std::thread m_publish_thread;
  std::thread m_ros_thread;
  ndlcom_ros::NdlcomDriver m_ndlcom_driver;

  ~NdlcomDriverNodelet();
};





}

# endif