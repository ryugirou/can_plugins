/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_msgs/Frame.h>

#define CAN_MTU 8
#define BETA 10

#include <stdint.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace can_plugins{

  template<typename T>
    union _Encapsulator
    {
      T data;
      uint64_t i;
    };

  template <typename T>
    void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
    {
      _Encapsulator<T> _e;

      for(int i = 0; i < sizeof(T); i++)
      {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
      }

      data = _e.data;
    }

  template<typename T>
    void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
    {
      _Encapsulator<T> _e;
      _e.data = data;

      for(int i = sizeof(T); i > 0;)
      {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
      }
    }

  class CanHandler : public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      void betaCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
      void betamotor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor3CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor4CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor5CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor6CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor7CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor8CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    
      void betamotor9CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);    

      void solenoidOrderCallback(const std_msgs::UInt8::ConstPtr& msg);

      void canRxCallback(const can_msgs::Frame::ConstPtr &msg);

      void TestTxCallback(const std_msgs::UInt8::ConstPtr &msg);

      template<typename T>
        void sendData(const uint16_t id, const T data);

      ros::NodeHandle _nh;
      ros::NodeHandle pnh;
      ros::Publisher _can_tx_pub;
      ros::Subscriber _can_rx_sub;

      ros::Publisher  _base_odom_x_pub;
      ros::Publisher  _base_odom_y_pub;
      ros::Publisher  _base_odom_yaw_pub;

      ros::Publisher  _test_pub;
      ros::Subscriber _test_sub;

      ros::Subscriber _beta_cmd_sub;
      ros::Subscriber _beta_motor0_cmd_vel_sub;
      ros::Subscriber _beta_motor1_cmd_vel_sub;
      ros::Subscriber _beta_motor2_cmd_vel_sub;
      ros::Subscriber _beta_motor3_cmd_vel_sub;
      ros::Subscriber _beta_motor4_cmd_vel_sub;
      ros::Subscriber _beta_motor5_cmd_vel_sub;
      ros::Subscriber _beta_motor6_cmd_vel_sub;
      ros::Subscriber _beta_motor7_cmd_vel_sub;
      ros::Subscriber _beta_motor8_cmd_vel_sub;
      ros::Subscriber _beta_motor9_cmd_vel_sub;

      ros::Subscriber _solenoid_order_sub;

      static constexpr uint16_t id_baseOdomX              = 0x205;
      static constexpr uint16_t id_baseOdomY              = 0x206;
      static constexpr uint16_t id_baseOdomYaw            = 0x207;
      static constexpr uint16_t id_solenoid            = 0x100;
      static constexpr uint16_t id_test_tx=0x4b0;
      static constexpr uint16_t id_test_rx=0x4b3;

      int id_beta[BETA];
  };

  void CanHandler::onInit(){
    _nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    for(int i=0;i < BETA;i++){
      pnh.param<int>("beta" + std::to_string(i) , id_beta[i], 0x700 + i);
    }

    _can_tx_pub				    = _nh.advertise<can_msgs::Frame>("can_tx", 1000);

    _base_odom_x_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/x", 1);
    _base_odom_y_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/y", 1);
    _base_odom_yaw_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/yaw", 1);

    _test_pub = _nh.advertise<std_msgs::UInt8>("test_rx",1000);
    _test_sub = _nh.subscribe<std_msgs::UInt8>("test_tx",1000,&CanHandler::TestTxCallback,this);

    _can_rx_sub				    = _nh.subscribe<can_msgs::Frame>("can_rx", 1000, &CanHandler::canRxCallback, this);

    _beta_cmd_sub	        = _nh.subscribe<std_msgs::UInt8>("beta/cmd", 10, &CanHandler::betaCmdCallback, this);
    _beta_motor0_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor0_cmd_vel", 1, &CanHandler::betamotor0CmdVelCallback, this);
    _beta_motor1_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor1_cmd_vel", 1, &CanHandler::betamotor1CmdVelCallback, this);
    _beta_motor2_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor2_cmd_vel", 1, &CanHandler::betamotor2CmdVelCallback, this);
    _beta_motor3_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor3_cmd_vel", 1, &CanHandler::betamotor3CmdVelCallback, this);
    _beta_motor4_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor4_cmd_vel", 1000, &CanHandler::betamotor4CmdVelCallback, this);
    _beta_motor5_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor5_cmd_vel", 1000, &CanHandler::betamotor5CmdVelCallback, this);
    _beta_motor6_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor6_cmd_vel", 1000, &CanHandler::betamotor6CmdVelCallback, this);
    _beta_motor7_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor7_cmd_vel", 1000, &CanHandler::betamotor7CmdVelCallback, this);
    _beta_motor8_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor8_cmd_vel", 1000, &CanHandler::betamotor8CmdVelCallback, this);
    _beta_motor9_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("beta/motor9_cmd_vel", 1000, &CanHandler::betamotor9CmdVelCallback, this);

    _solenoid_order_sub = _nh.subscribe<std_msgs::UInt8>("solenoid", 1000, &CanHandler::solenoidOrderCallback, this);

    NODELET_INFO("can_handler has started.");
  }

  void CanHandler::betaCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    for(int i=0;i < BETA;i++){
      this->sendData(id_beta[i], msg->data);
    }
    this->sendData(id_solenoid, msg->data);
  }

  void CanHandler::betamotor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[0]+1, (float)(msg->data));
  }

  void CanHandler::betamotor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[1]+1, (float)(msg->data));
  }

  void CanHandler::betamotor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[2]+1, (float)(msg->data));
  }

  void CanHandler::betamotor3CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[3]+1, (float)(msg->data));
  }

  void CanHandler::betamotor4CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[4]+1, (float)(msg->data));
  }

  void CanHandler::betamotor5CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[5]+1, (float)(msg->data));
  }

  void CanHandler::betamotor6CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[6]+1, (float)(msg->data));
  }

  void CanHandler::betamotor7CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[7]+1, (float)(msg->data));
  }

  void CanHandler::betamotor8CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[8]+1, (float)(msg->data));
  }

  void CanHandler::betamotor9CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    this->sendData(id_beta[9]+1, (float)(msg->data));
  }

  void CanHandler::solenoidOrderCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    this->sendData(id_solenoid+1, msg->data);
  }

  void CanHandler::TestTxCallback(const std_msgs::UInt8::ConstPtr &msg)
  {
    this->sendData(id_test_tx,msg->data);
  }

  void CanHandler::canRxCallback(const can_msgs::Frame::ConstPtr &msg)
  {
    std_msgs::Float32 _base_odom_x_msg;
    std_msgs::Float32 _base_odom_y_msg;
    std_msgs::Float32 _base_odom_yaw_msg;

    std_msgs::UInt8 _test_msg;

    switch(msg->id)
    {
      case id_baseOdomX:
        can_unpack(msg->data, _base_odom_x_msg.data);
        _base_odom_x_pub.publish(_base_odom_x_msg);
        break;

      case id_baseOdomY:
        can_unpack(msg->data, _base_odom_y_msg.data);
        _base_odom_y_pub.publish(_base_odom_y_msg);
        break;

      case id_baseOdomYaw:
        can_unpack(msg->data, _base_odom_yaw_msg.data);
        _base_odom_yaw_pub.publish(_base_odom_yaw_msg);
        break;

      case id_test_rx:
        can_unpack(msg->data, _test_msg.data);
        _test_pub.publish(_test_msg);
        break;
      default:
        break;
    }
  }

  template<typename T>
    void CanHandler::sendData(const uint16_t id, const T data)
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

}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(can_plugins::CanHandler, nodelet::Nodelet);
