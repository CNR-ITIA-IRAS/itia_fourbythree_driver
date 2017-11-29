#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <itia_rutils/itia_rutils.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


int main(int argc, char **argv) {
  
  ros::init(argc, argv, "calibration_test");
  ros::NodeHandle nh;
  nh.getNamespace();
  
  itia::rutils::MsgReceiver<sensor_msgs::JointState> js_rec("joint_states");
  ros::Subscriber js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &js_rec);
  

  bool debug=false;
  
  if (!nh.getParam("debug",debug))
    debug=false;
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("follow_joint_trajectory",true);
  ROS_INFO("Waiting for action server");
  
  if (!debug)
    ac.waitForServer(ros::Duration(0.0));
  
  
  tf::TransformListener listener;
  
  control_msgs::FollowJointTrajectoryGoal goal;
  
  double rest_time;
  
  if (!nh.getParam("rest_time",rest_time))
  {
    ROS_ERROR("rest_time does not exist");
    return false;
  }
  
  
  if (!nh.getParam("joint_names",goal.trajectory.joint_names))
  {
    ROS_ERROR("joint_names string vector does not exist");
    return false;
  }
  
  std::string first_frame,last_frame;
  if (!nh.getParam("first_frame",first_frame))
  {
    ROS_ERROR("first_frame parameter does not exist");
    return 0;
  }
  if (!nh.getParam("last_frame",last_frame))
  {
    ROS_ERROR("last_frame parameter does not exist");
    return 0;
  }
  
  std::vector<std::vector<double>> positions;
  if (!itia::rutils::getParamMatrix(nh, "positions",positions))
  {
    ROS_ERROR("positions vector does not exist");
    return false;
  }
  
  unsigned int npnt=positions.size();
  if (npnt==0)
  {
    ROS_WARN("NO POINTS LOADED");
    return 0;
  }
  std::vector<std::vector<double>> link_positions;
  std::vector<std::vector<double>> link_frames;
  
  
  goal.trajectory.points.resize(1);
  goal.trajectory.points.at(0).velocities.resize(goal.trajectory.joint_names.size());
  std::fill(goal.trajectory.points.at(0).velocities.begin(),goal.trajectory.points.at(0).velocities.end(),0.5);
  goal.trajectory.points.at(0).time_from_start = ros::Duration(0.0);
  
  control_msgs::JointTolerance tol;
  goal.path_tolerance.resize(goal.trajectory.joint_names.size());
  std::fill(goal.path_tolerance.begin(),goal.path_tolerance.end(),tol);
  goal.goal_tolerance.resize(goal.trajectory.joint_names.size());
  std::fill(goal.goal_tolerance.begin(),goal.goal_tolerance.end(),tol);
  
  for (unsigned int iPnt=0;iPnt<npnt;iPnt++)
  {
    ROS_INFO("POSITION %u of %u",iPnt+1,npnt);
    goal.trajectory.points.at(0).positions = positions.at(iPnt);
    goal.trajectory.header.stamp=ros::Time::now();
    
    if (!debug)
      ac.sendGoalAndWait(goal); 
    
    ros::Duration(rest_time).sleep();
    ros::spinOnce();
    
    tf::StampedTransform transform;
    
    bool success=false;
    for (unsigned int iTrial=0;iTrial<10;iTrial++)
    {
      try
      {
        listener.lookupTransform("/"+first_frame, "/"+last_frame,  
                                ros::Time(0), transform);
        success=true;
        break;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }
    
    if (!success)
    {
      ROS_WARN("unable to get transform, skip point");
      continue;
    }
      
    double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Vector3 v = transform.getOrigin();
    
    std::vector<double> tmp(6);
    tmp.at(0)=v.getX();
    tmp.at(1)=v.getY();
    tmp.at(2)=v.getZ();
    tmp.at(3)=roll;
    tmp.at(4)=pitch;
    tmp.at(5)=yaw;
    
    link_positions.push_back(js_rec.getData().position);
    link_frames.push_back(tmp);
    
  }
  
  itia::rutils::setParam(nh,"results/link_joint_position",link_positions);
  itia::rutils::setParam(nh,"results/link_frames",link_frames);
  
  system(("rosparam dump ~/.ros/kin_cal_results.yaml /"+nh.getNamespace()+"/results").c_str());
  return 0;
}
