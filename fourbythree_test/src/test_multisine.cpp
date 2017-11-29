
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
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletList.h>
#include <std_msgs/Bool.h>
#include <boost/filesystem.hpp>

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "test_multifreq");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::Rate loop_rate(1000);
  
  ros::Publisher target_js_pub = nh.advertise<sensor_msgs::JointState>("/rigid/joint_states", 1);
  itia::rutils::MsgReceiver<sensor_msgs::JointState> js_rec("joint_state");
  
  sensor_msgs::JointState msg;
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  
	std::string manager_name;
	if (!nh.getParam("binary_logger/manager_name", manager_name))
	{
		ROS_ERROR("binary_logger/manager_name NOT FOUND");
		return -1;
	}
	
	ros::ServiceClient list_logger_srv = nh.serviceClient<nodelet::NodeletList>(manager_name+"/list");
	ros::ServiceClient load_logger_srv = nh.serviceClient<nodelet::NodeletLoad>(manager_name+"/load_nodelet");
	ros::ServiceClient unload_logger_srv = nh.serviceClient<nodelet::NodeletUnload>(manager_name+"/unload_nodelet");
	
	
	
	std::string test_path;
	if (!nh.getParam("binary_logger/test_path", test_path))
	{
		test_path=( boost::filesystem::current_path().string() );
		ROS_INFO("binary_logger/test_path NOT FOUND, select PWD = '%s'",test_path.c_str());
	}
	if (!test_path.compare(""))
	{
		test_path=( boost::filesystem::current_path().string() );
		ROS_INFO("binary_logger/test_path VOID, select PWD = '%s'",test_path.c_str());
	}
	

	if (!list_logger_srv.waitForExistence(ros::Duration(5)))
	{
		ROS_ERROR("Manager '%s' is not found", manager_name.c_str());
		return -1;
	}
	
	nodelet::NodeletList nodelet_list;
	nodelet::NodeletLoad nodelet_load;
	nodelet::NodeletUnload nodelet_unload;
	
	
	std::vector<std::string> test_namespace;
	if (!nh.getParam("multisine/test_namespace",test_namespace))
	{
		ROS_ERROR("binary_logger/test_namespace NOT FOUND");
		return -1;
	}
	
	bool flag = true;
	
	std::string test_name_base;
	if (!nh.getParam("binary_logger/test_name", test_name_base))
	{
		ROS_ERROR("binary_logger/test_name NOT FOUND");
		return -1;
	}
	
	std::vector<std::string> unstoppable_nodelets;
	if (!nh.getParam("binary_logger/unstoppable_nodelets", unstoppable_nodelets))
	{
		ROS_ERROR("binary_logger/unstoppable_nodelets NOT FOUND");
		return -1;
	}
	
	for (int iTest=0;iTest<test_namespace.size();iTest++)
	{
		std::string test_name=test_name_base+"_"+test_namespace.at(iTest);
		double test_duration;
		if (!nh.getParam(test_namespace.at(iTest)+"/test_duration",test_duration))
		{
			ROS_ERROR("%s does not exist",(test_namespace.at(iTest)+"/test_duration").c_str());
			return -1;
		}
		
		list_logger_srv.call(nodelet_list);
		if (nodelet_list.response.nodelets.size()>0)
			ROS_INFO("%zu logger threads will be stopped", nodelet_list.response.nodelets.size());
		
		for (int idx = 0;idx < nodelet_list.response.nodelets.size();idx++)
		{
			bool do_stop = true;
			for (int idx2 =0; idx2 < unstoppable_nodelets.size(); idx2++)
			{
				if ( !nodelet_list.response.nodelets.at(idx).compare(unstoppable_nodelets.at(idx2) ) )
					do_stop=false;
			}
			
			if (do_stop)
			{
				nodelet_unload.request.name = nodelet_list.response.nodelets.at(idx);
				unload_logger_srv.call(nodelet_unload);
				if (!nodelet_unload.response.success)
					ROS_ERROR("Unloading of '%s' unsuccessfull", nodelet_unload.request.name.c_str());
			}
		}
		
		
		
		std::vector<std::string> topic_type;
		if (!nh.getParam("binary_logger/topic_type", topic_type))
		{
			ROS_ERROR("binary_logger/topic_type NOT FOUND");
			return -1;
		}
		
		for (int idx = 0;idx<topic_type.size();idx++)
		{
			std::vector<std::string> topic_names;
			std::vector<double> duration;
			std::vector<int> decimation;
			std::string nodelet_type;
			
			
			
			if (!nh.getParam("binary_logger/"+topic_type.at(idx)+"/topic_names", topic_names))
			{
				ROS_ERROR("binary_logger/%s/topic_names NOT FOUND", topic_type.at(idx).c_str());
				return -1;
			}
			if (!nh.getParam("binary_logger/"+topic_type.at(idx)+"/decimation", decimation))
			{
				ROS_ERROR("binary_logger/%s/decimation NOT FOUND", topic_type.at(idx).c_str());
				return -1;
			}
			nodelet_type = "itia/"+topic_type.at(idx)+"BinaryLogger";
			
			for (int iT = 0;iT<topic_names.size();iT++)
			{
				nodelet_load.request.name = topic_names.at(iT);
				nodelet_load.request.type = nodelet_type;
				nodelet_load.request.my_argv.resize(4);
				nodelet_load.request.my_argv.at(1) = "/"+topic_names.at(iT);
				std::replace( topic_names.at(iT).begin(), topic_names.at(iT).end(), '/', '_');
				nodelet_load.request.my_argv.at(0) = test_path+"/"+test_name+"_"+topic_type.at(idx)+"_"+topic_names.at(iT);
				nodelet_load.request.my_argv.at(2) = std::to_string(test_duration+5);
				nodelet_load.request.my_argv.at(3) = std::to_string(decimation.at(iT));
				
				load_logger_srv.call(nodelet_load);
				if (!nodelet_load.response.success)
				{
					ROS_ERROR("Fail calling '%s' logger ", topic_names.at(iT).c_str());
				}
			}
		}
		
		nodelet_load.request.name = test_namespace.at(iTest);
		nodelet_load.request.type = "itia/identification/FeedforwardActionNodelet";
		load_logger_srv.call(nodelet_load);
		if (!nodelet_load.response.success)
		{
			ROS_ERROR("Fail calling '%s' ", nodelet_load.request.name.c_str());
		}
		ros::Duration(test_duration).sleep();
		
	}
	
	
	
	list_logger_srv.call(nodelet_list);
	if (nodelet_list.response.nodelets.size()>0)
		ROS_INFO("%zu logger threads will be stopped", nodelet_list.response.nodelets.size());
	for (int idx = 0;idx < nodelet_list.response.nodelets.size();idx++)
	{
		bool do_stop = true;
		for (int idx2 =0; idx2 < unstoppable_nodelets.size(); idx2++)
		{
			if ( !nodelet_list.response.nodelets.at(idx).compare(unstoppable_nodelets.at(idx2) ) )
				do_stop=false;
		}
		
		if (do_stop)
		{
			nodelet_unload.request.name = nodelet_list.response.nodelets.at(idx);
			unload_logger_srv.call(nodelet_unload);
			if (!nodelet_unload.response.success)
				ROS_ERROR("Unloading of '%s' unsuccessfull", nodelet_unload.request.name.c_str());
		}
	}
}
