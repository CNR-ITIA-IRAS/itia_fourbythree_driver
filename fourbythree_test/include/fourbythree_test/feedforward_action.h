
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