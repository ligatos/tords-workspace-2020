#include <ros/ros.h>
#include <queue>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Core>

geometry_msgs::Vector3 last_reset,last_reset_odom;
sensor_msgs::Joy cmd;
geometry_msgs::TransformStamped tff,tfh,tfrp,tfm,tfo,tfoa,tfa,tfa2,tfp,tfpos,tfb;
geometry_msgs::Vector3Stamped vlocal_gps,vglobal_gps;
geometry_msgs::Vector3Stamped armvec_min_max_radprs;
geometry_msgs::Vector3 rpy,rpyx;
nav_msgs::Odometry odom;
ros::Time time_last_v;
bool first = true;
bool par_gotref,par_lookattarget,par_pubtfa;
std_msgs::Float64 pz;
double yaw_odom,target_arm,par_latitude,par_longitude,par_dtfuture,par_altitude,vel_z,par_x_start,par_y_start,par_z_start;
tf2_ros::Buffer tfBuffer;
geometry_msgs::PointStamped ll40_stabilized;
float t_int_1,t_int_2,t_int_3;
const float deg2rad = M_PI/180.0;
const float rad2deg = 180.0/M_PI;
const float earth = 6378137.0;
int motionstate;
bool got_att;
geometry_msgs::Point t;
float tyaw;
int mainstate;
geometry_msgs::Point d;
double get_shortest(double target_yaw,double actual_yaw){
  double a = target_yaw - actual_yaw;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
double saturate(double val, double max){
    if (abs(val) >= max)
      val = (val>0) ? max : -1 * max;
    else
      val = val;
    if((std::isnan(val)) || (std::isinf(val))){
      return 0;
    }
    else{
      return val;
    }
}
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    float dt = ((ros::Time::now() - time_last_v).toSec());
    time_last_v                = ros::Time::now();
    odom.twist.twist.angular.z = msg->angular.z;
    odom.twist.twist.linear.x  = msg->linear.x;
    odom.twist.twist.linear.y  = msg->linear.y;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_hydra_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(20);

    tf2_ros::TransformListener tf2_listener(tfBuffer);

    tf2_ros::TransformBroadcaster tf_b;

    odom.header.frame_id         = "odom";
    odom.child_frame_id          = "base_perfect";
    odom.pose.pose.orientation.w = 1;

    tfo.header.frame_id          = "odom";
    tfo.child_frame_id           = "base_perfect";
    tfo.transform.rotation.w     = 1;
    tfm.header.frame_id         = "map";
    tfm.child_frame_id          = "odom";
    tfm.transform.rotation.w    = 1;

    odom.pose.covariance[0] = (1e-3);
    odom.pose.covariance[7] = (1e-3);
    odom.pose.covariance[14] = (1e-3);
    odom.pose.covariance[21] = (1e-3);
    odom.pose.covariance[28] = (1e-3);
    odom.pose.covariance[35] = (1e-3);
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[7] = 1e-3;
    odom.twist.covariance[14] = 1e-3;
    odom.twist.covariance[21] = 1e-3;
    odom.twist.covariance[35] = 1e-3;

    ros::Subscriber s8 = nh.subscribe("/cmd_vel",             100,&twist_cb);
    ros::Publisher odom_pub  = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    double dt;
    ros::Time time_last = ros::Time::now();
    while(ros::ok()){
      if((ros::Time::now() - time_last_v).toSec() > 0.5) odom.twist.twist.angular.z = odom.twist.twist.linear.x = odom.twist.twist.linear.y  = 0;
      if((ros::Time::now() - time_last).toSec() > 0.03) dt = 0.03;
      else dt= (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
      rate.sleep();
      ros::spinOnce();
      odom.pose.pose.position.x  += ((odom.twist.twist.linear.x*cos(yaw_odom)-odom.twist.twist.linear.y*sin(yaw_odom)) * dt);
      odom.pose.pose.position.y  += ((odom.twist.twist.linear.x*sin(yaw_odom)+odom.twist.twist.linear.y*cos(yaw_odom)) * dt);
      yaw_odom                   += odom.twist.twist.angular.z * dt;
      yaw_odom                    = constrainAngle(yaw_odom);

      tfo.transform.translation.x = odom.pose.pose.position.x;
      tfo.transform.translation.y = odom.pose.pose.position.y;
      tfo.transform.rotation      = odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_odom);

      odom.header.stamp = tfo.header.stamp = tfm.header.stamp = ros::Time::now();
      tf_b.sendTransform(tfo);
      tf_b.sendTransform(tfm);
      odom_pub.publish(odom);
    }
    return 0;
}
