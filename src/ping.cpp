/*
 * ping.cpp
 *
 *  Created on: Dec 20, 2019
 *      Author: ryu
 */
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"

class Ping
{
public:
  Ping();

private:
  double ctrl_freq;
  int count_rx;
  int count_tx;
  int num=0;
  float avg=0;

  void timer_callback(const ros::TimerEvent &);
  void timeout_callback(const ros::TimerEvent &);
  void RxCallback(const std_msgs::UInt8ConstPtr& msg);
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher tx_pub_;
  ros::Subscriber rx_sub_;

  ros::Timer timer;
  ros::Timer timeout;
};

Ping::Ping() : private_nh_("~")
{

  if (!private_nh_.getParam("ctrl_freq", ctrl_freq))
  {
    ctrl_freq = 1000;
  }

  count_tx = 0;
  count_rx = 0;

  tx_pub_ = nh_.advertise<std_msgs::UInt8>("test_tx", 1);
  rx_sub_ = nh_.subscribe("test_rx", 2, &Ping::RxCallback, this);

  timeout = nh_.createTimer(ros::Duration(1), boost::bind(&Ping::timeout_callback, this, _1));
}

void Ping::timer_callback(const ros::TimerEvent &)
{
  std_msgs::UInt8 msg;
  msg.data = 0x11;
  tx_pub_.publish(msg);
  count_tx += 1;
  if (count_tx >= 100)
    timer.stop();
}

void Ping::timeout_callback(const ros::TimerEvent &)
{


  if(0 < num){
    ROS_INFO("%d packs transmitted, %d received", count_tx,count_rx);
    avg += (double)count_rx/(double)count_tx;
    count_rx = 0;
    count_tx = 0;
  }
  if(num < 10){
    timer = nh_.createTimer(ros::Duration(1 / ctrl_freq), boost::bind(&Ping::timer_callback, this, _1));
    num++;
  }
  else
  {
    ROS_INFO("%f%% packet lost", (1-avg/10)*100);
    timeout.stop();
  }
  
   
  
}

void Ping::RxCallback(const std_msgs::UInt8ConstPtr& msg)
{
  count_rx += 1;
}

boost::shared_ptr<Ping> ping_ptr;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Ping");
  ros::NodeHandle nh;

  ping_ptr.reset(new Ping());

  ros::spin();

  // Without this, our boost locks are not shut down nicely
  ping_ptr.reset();

  // To quote Morgan, Hooray!
  return (0);
}