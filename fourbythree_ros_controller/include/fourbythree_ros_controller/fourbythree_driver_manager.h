#ifndef __FOURBYTHREE_DRIVER_MANAGER__
#define __FOURBYTHREE_DRIVER_MANAGER__
#include <ros/ros.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <itia_futils/itia_futils.h>
#include <boost/graph/graph_concepts.hpp>
# include <diagnostic_msgs/DiagnosticArray.h>
namespace fourbythree
{

  
class DriverManager
{
private:
  ros::NodeHandle m_nh;
  

  ros::Publisher m_diagnostics_pub;
public:
  
  DriverManager();
  ~DriverManager();
  bool loadCallback   (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool enableCallback (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool unloadCallback (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  
  
};
  
}

#endif