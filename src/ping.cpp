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
#include "can_utils.hpp"
#include <can_msgs/Frame.h>

class Ping
{
public:
  Ping();

private:
  double ctrl_freq;
  int count_rx;
  int count_tx;
  int id_tx;
  int id_rx;
  int num=0;
  float avg=0;

  void timer_callback(const ros::TimerEvent &);
  void timeout_callback(const ros::TimerEvent &);
  void canRxCallback(const can_msgs::Frame::ConstPtr &msg);
  template<typename T>
  void sendData(const uint16_t id, const T data);
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher tx_pub_;
  ros::Subscriber rx_sub_;

  ros::Publisher _can_tx_pub;
  ros::Subscriber _can_rx_sub;

  ros::Timer timer;
  ros::Timer timeout;
};

Ping::Ping() : private_nh_("~"),count_rx(0),count_tx(0),ctrl_freq(1000),id_tx(0)
{
  private_nh_.getParam("ctrl_freq", ctrl_freq);
  private_nh_.getParam("id_tx", id_tx);
  
  id_rx = id_tx + 3;

  _can_tx_pub				    = nh_.advertise<can_msgs::Frame>("can_tx", 1000);
  _can_rx_sub				    = nh_.subscribe<can_msgs::Frame>("can_rx", 1000, &Ping::canRxCallback, this);

  timeout = nh_.createTimer(ros::Duration(1), boost::bind(&Ping::timeout_callback, this, _1));
}

void Ping::timer_callback(const ros::TimerEvent &)
{
  uint8_t GetStatus = 0x11;
  sendData(id_tx,GetStatus);
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

void Ping::canRxCallback(const can_msgs::Frame::ConstPtr &msg)
{
  if(msg->id == id_rx)
  {
    count_rx += 1;
  }
}

template<typename T>
void Ping::sendData(const uint16_t id, const T data)
{
  can_msgs::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;

  frame.dlc = sizeof(T);

  can_pack<T>(frame.data, data);

  _can_tx_pub.publish(frame);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Ping");
  ros::NodeHandle nh;

  Ping ping = Ping();

  ros::spin();

  return (0);
}