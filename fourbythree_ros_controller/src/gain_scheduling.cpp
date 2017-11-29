#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <itia_rutils/itia_rutils.h>
#include <sensor_msgs/JointState.h>
#include <itia_dynamics_core/itia_primitives.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "fourbythree_gain_scheduling");
  ros::NodeHandle nh;

  
  ROS_INFO("%sFourByThree Gain Scheduling node%s",BOLDGREEN,RESET);
  
  controller_manager_msgs::SwitchController switch_ctrl_srv;
  controller_manager_msgs::ListControllers list_ctrl_srv;
  controller_manager_msgs::LoadController load_ctrl_srv;
  switch_ctrl_srv.request.start_controllers.resize(1);
  switch_ctrl_srv.request.stop_controllers.resize(1);
  switch_ctrl_srv.request.strictness=1;

  std::string hi_ns;
  if (!nh.getParam("gain_scheduling/hardware_interface_ns",hi_ns))
  {
    ROS_FATAL("gain_scheduling/hardware_interface_ns does not exist");
    return -1;
  }
  std::string controllers_prefix;
  if (!nh.getParam("gain_scheduling/controllers_prefix",controllers_prefix))
  {
    ROS_FATAL("gain_scheduling/controllers_prefix does not exist");
    return -1;
  }
  
  double overlap;
  if (!nh.getParam("gain_scheduling/overlapping",overlap))
  {
    ROS_FATAL("gain_scheduling/overlapping does not exist");
    return -1;
  }
  if (overlap>0.5)
    overlap=0.5;
  else if (overlap<0)
    overlap=0;
  
  
  ros::ServiceClient switch_ctrl = nh.serviceClient<controller_manager_msgs::SwitchController>(hi_ns+"/controller_manager/switch_controller");
  ros::ServiceClient list_ctrl = nh.serviceClient<controller_manager_msgs::ListControllers>(hi_ns+"/controller_manager/list_controllers");
  ros::ServiceClient load_ctrl = nh.serviceClient<controller_manager_msgs::LoadController>(hi_ns+"/controller_manager/load_controller");
  
  if (!switch_ctrl.waitForExistence(ros::Duration(40)))
  {
    ROS_ERROR("/fourbythree/controller_manager/switch_controller is not available");
    return 0;
  }
  

  
  
  
  itia::rutils::MsgReceiver<sensor_msgs::JointState> m_fb_js_rec;
  ros::Subscriber m_js_fb_sub     = nh.subscribe<sensor_msgs::JointState>("/link/joint_states", 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_fb_js_rec);
      
  if (!m_fb_js_rec.waitForANewData(10))
  {
    ROS_ERROR("/link/joint_states is not available");
    return 0;
  }
  ROS_INFO("%sFourByThree Gain Scheduling node started%s",BOLDGREEN,RESET);
  
  
   
  std::string base_frame = "base";
  std::string tool_frame = "flange";                  //"ee_link"; "upper_arm_link"  "forearm_link"
  int njnt = m_fb_js_rec.getData().position.size();
  
  std::vector<std::string> js(njnt);
  js=m_fb_js_rec.getData().name;
  
  
  std::vector<std::vector<double>> controllers_inertia(njnt);
  for (int iAx=0;iAx<njnt;iAx++)
  {
    if (!nh.getParam("gain_scheduling/"+js.at(iAx),controllers_inertia.at(iAx) ))
    {
      ROS_ERROR("gain_scheduling/%s  is not available",js.at(iAx).c_str());
      return 0;
    }
    for (auto inertia: controllers_inertia.at(iAx))
    {
      ROS_INFO("%s -> %f",js.at(iAx).c_str(),inertia);
    }
  }
  std::vector<int> controller_state(njnt);
  std::fill(controller_state.begin(),controller_state.end(),0.0);
  
  
  urdf::Model model;
  model.initParam("robot_description");
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
  root_link->fromUrdf(model.root_link_);
  boost::shared_ptr<itia::dynamics::Chain> chain(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
  
  chain->setInputJointsName(js);
  
  Eigen::VectorXd q(njnt);
  q.setZero();
  Eigen::MatrixXd joint_inertia;
  
  list_ctrl.call(list_ctrl_srv);

  
  
  ros::Rate lp(100);
  while (ros::ok())
  {
    

    for (int iAx=0;iAx<njnt;iAx++)
      q(iAx)=m_fb_js_rec.getData().position.at(iAx);
      
    joint_inertia = chain->getJointInertia(q);

    bool change_controller=false;
    for (int iAx=0;iAx<njnt;iAx++)
    {
      int iS=controller_state.at(iAx);
      if (iS>0)
      {
        if (joint_inertia(iAx,iAx)< (controllers_inertia.at(iAx).at(iS-1) + (0.5-overlap) * ( controllers_inertia.at(iAx).at(iS)-controllers_inertia.at(iAx).at(iS-1)) ) )
        {
          change_controller=true;
          controller_state.at(iAx)--;
        }
      }
      if (iS < (controllers_inertia.at(iAx).size()-1))
      {
        if (joint_inertia(iAx,iAx)> (controllers_inertia.at(iAx).at(iS) + (0.5+overlap) * ( controllers_inertia.at(iAx).at(iS+1)-controllers_inertia.at(iAx).at(iS)) ) )
        {
          change_controller=true;
          controller_state.at(iAx)++;
        }
      }
    }
    
    
    
    
    if (change_controller)
    {
      std::string active_controller;
      std::string new_controller=controllers_prefix;
      for (int iAx=0;iAx<njnt;iAx++)
      {
        new_controller+="_"+std::to_string(controller_state.at(iAx));
      }
      ROS_INFO("new controller: %s",new_controller.c_str());
      list_ctrl.call(list_ctrl_srv);
      bool need_loading=true;
      for (auto c: list_ctrl_srv.response.controller)
      {
        if (!c.name.compare(new_controller))
        {
          need_loading=false;
        }
        if (!c.state.compare("running"))
          active_controller=c.name;
      }
      if (need_loading)
      {
        ROS_DEBUG("Loading %s", new_controller.c_str());
        load_ctrl_srv.request.name=new_controller;
        load_ctrl.call(load_ctrl_srv);
      }
      
      switch_ctrl_srv.request.start_controllers.at(0)=new_controller;
      switch_ctrl_srv.request.stop_controllers.at(0)=active_controller;
      switch_ctrl.call(switch_ctrl_srv);
      ROS_INFO("Switch from: %s to: %s",active_controller.c_str(),new_controller.c_str());
      active_controller=new_controller; 
    }
    
    ros::spinOnce();
    lp.sleep();
  }
  return 0;
}
