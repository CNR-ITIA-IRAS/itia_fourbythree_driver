#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <itia_rutils/itia_rutils.h>
#include <itia_controllers_and_filters/tf.h>

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "test_ndl");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  double st = 1e0;
  if (!nh.getParam("st", st))
    return -1;
  ros::Rate loop_rate(1.0/st);
  
  ros::Publisher target_js_pub = nh.advertise<sensor_msgs::JointState>("/sp/joint_states", 1);
  itia::rutils::MsgReceiver<sensor_msgs::JointState> target_js_rec("joint_states_sp");
  ros::Subscriber target_js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states_sp", 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &target_js_rec);
  
  itia::rutils::MsgReceiver<sensor_msgs::JointState> js_rec("joint_state");
  ros::Subscriber js_sub = nh.subscribe<sensor_msgs::JointState>("fb/joint_states_reduced_motor", 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &js_rec);
  
  sensor_msgs::JointState msg;
  
  msg.position.resize(6);
  msg.velocity.resize(6);
  msg.effort.resize(6);
  msg.name.resize(6);
  msg.name.at(0) ="joint_1";
  msg.name.at(1) ="joint_2";
  msg.name.at(2) ="joint_3";
  msg.name.at(3) ="joint_4";
  msg.name.at(4) ="joint_5";
  msg.name.at(5) ="joint_6";
  
  std::fill(msg.position.begin(), msg.position.end(), 0);
  std::vector<double> init_pos(6);
  std::fill(init_pos.begin(), init_pos.end(), 0);
  if (!js_rec.waitForANewData(10))
    return -1;
  init_pos = js_rec.getData().position;

  std::vector<double> pos = init_pos ;
  msg.position = init_pos;
  std::fill(msg.velocity.begin(), msg.velocity.end(), 0);
  std::fill(msg.effort.begin(), msg.effort.end(), 0);
  target_js_pub.publish(msg);
  
  
  std::vector<double> filt_num, filt_den, Dfilt_num, Dfilt_den;
  if (!nh.getParam("filt_num", filt_num))
    return -1;
  if (!nh.getParam("filt_den", filt_den))
    return -1;
  if (!nh.getParam("Dfilt_num", Dfilt_num))
    return -1;
  if (!nh.getParam("Dfilt_den", Dfilt_den))
    return -1;
  
  std::vector<itia::filter::tf> lowpass(6);
  std::vector<itia::filter::tf> Dlowpass(6);
  std::vector<double> init_cond(filt_den.size()-1);
  std::vector<double> Dinit_cond(filt_den.size()-1);
  for (int idx = 0;idx<6;idx++)
  {
    lowpass.at(idx).set_st(st);
    Dlowpass.at(idx).set_st(st);
    lowpass.at(idx).setCoeffs(filt_num, filt_den);
    Dlowpass.at(idx).setCoeffs(Dfilt_num, Dfilt_den);
    
    std::fill(init_cond.begin(), init_cond.end(), init_pos.at(idx));
    std::fill(Dinit_cond.begin(), Dinit_cond.end(), 0.0);
  
    lowpass.at(idx).setInitConditions(init_cond, init_cond);
    Dlowpass.at(idx).setInitConditions(Dinit_cond, init_cond);
  }
  
  for (int iC = 0;iC<20;iC++)
  {
    msg.header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    ros::Duration(0.01).sleep();                              //pauses.at(idx)).sleep();
  }
  
  while (ros::ok())
  {
    if (target_js_rec.isANewDataAvailable())
    {
      pos = target_js_rec.getData().position;
    }
    for (int idx = 0;idx<6;idx++)
    {
      lowpass.at(idx).step(pos.at(idx));
      Dlowpass.at(idx).step(pos.at(idx));
      msg.position.at(idx) =lowpass.at(idx).tf_out;
      msg.velocity.at(idx) =Dlowpass.at(idx).tf_out;
    }
    msg.header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
