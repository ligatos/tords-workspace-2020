#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
nav_msgs::Odometry odom,odo,odom_in;
ros::Time time_last,time_last_v;
geometry_msgs::TransformStamped tfo,tfoo,tfv;
geometry_msgs::Vector3Stamped velvec;
geometry_msgs::Point opos,vpos,oopos;
double vmax_up = 5;
double vmin_down = -3;
double target_altitude;
double valt;
geometry_msgs::Vector3 d;
bool firsto = true;
bool firstv = true;
float yaw;

void start(){
  opos = odom.pose.pose.position;
  vpos = odom.pose.pose.position;
  oopos = opos;
  ROS_INFO("STARTING at pos. %.0f %.0f %.0f",opos.x,opos.y,opos.z);
}
void odo_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if(!firsto){
    float dt = (msg->header.stamp - odo.header.stamp).toSec();
    oopos.x += msg->twist.twist.linear.x * dt;
    oopos.y += msg->twist.twist.linear.y * dt;
    oopos.z += msg->twist.twist.linear.z * dt;
    odo = *msg;
  }
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if(!firsto){
    float dt = (msg->header.stamp - odom.header.stamp).toSec();
    opos.x += msg->twist.twist.linear.x * dt;
    opos.y += msg->twist.twist.linear.y * dt;
    opos.z += msg->twist.twist.linear.z * dt;
    odom = *msg;
  }
  else if(!firstv && firsto){
    firsto = false;
    start();
    yaw = tf::getYaw(msg->pose.pose.orientation);
    odom = *msg;

  }
  else{
    firsto = false;
    odom = *msg;
  }
}
void vel_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  if(!firstv){
    float dt = (msg->header.stamp - velvec.header.stamp).toSec();
    vpos.x += msg->vector.x * dt;
    vpos.y += msg->vector.y * dt;
    vpos.z += msg->vector.z * dt;
    velvec = *msg;
  }
  else if(firstv && !firsto){
    velvec = *msg;
    firstv = false;
    start();
  }
  else{
    velvec = *msg;
    firstv = false;
  }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "tb_odomtfperf_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    tf2_ros::TransformBroadcaster tf_b;
    odom.header.frame_id           = "odom";
    odom.child_frame_id            = "base_perfect";
    odom.pose.pose.orientation.w   = 1;

    tfo.header.frame_id      = "odom";
    tfo.child_frame_id       = "tfo";
    tfo.transform.rotation.w = 1;
    tfoo.header.frame_id      = "odom";
    tfoo.child_frame_id       = "tfoo";
    tfoo.transform.rotation.w = 1;

    tfv.header.frame_id = "odom";
    tfv.child_frame_id  = "tfv";
    tfv.transform.rotation.w    = 1;

    ros::Subscriber s3 = nh.subscribe("/odom",100,&odo_cb);
    ros::Subscriber s1 = nh.subscribe("/odom_global",100,&odom_cb);
    ros::Subscriber s2 = nh.subscribe("/dji_sdk/velocity",100,&vel_cb);

    time_last = ros::Time::now();
    while(ros::ok()){

/*
      float dt = (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
      yaw += odom.twist.twist.angular.z * dt;
      if(yaw >  M_PI) yaw -= 2*M_PI;
      if(yaw < -M_PI) yaw += 2*M_PI;
      if(dt > 0.03)
        dt = 0.0;
      tfo.transform.rotation = odom_in.pose.pose.orientation= tf::createQuaternionMsgFromYaw(yaw);
      odom.pose.pose.position.x += (odom_in.twist.twist.linear.x*cos(yaw)-odom_in.twist.twist.linear.y*sin(yaw))*dt;
      odom.pose.pose.position.y += (odom_in.twist.twist.linear.x*sin(yaw)+odom_in.twist.twist.linear.y*cos(yaw))*dt;
      tfo.transform.translation.x = odom.pose.pose.position.x;
      tfo.transform.translation.y = odom.pose.pose.position.y;
      tfo.transform.rotation      = odom.pose.pose.orientation;
      tfv.header.stamp = tfmap.header.stamp = odom.header.stamp = tfo.header.stamp = ros::Time::now();

      tf_b.sendTransform(tfo);
      tf_b.sendTransform(tfv);
  */
    ROS_INFO("*******************************************************************************************************************");

    ROS_INFO("DJITWIST at pos. %.0f %.0f %.0f vel: %.1f %.1f %.1f",vpos.x,vpos.y,vpos.z,velvec.vector.x,velvec.vector.y,velvec.vector.z);
    ROS_INFO("TBINTERP at pos. %.0f %.0f %.0f vel: %.1f %.1f %.1f",opos.x,opos.y,opos.z,odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
    ROS_INFO("DELTAPOS at pos. %.0f %.0f %.0f vel: %.1f %.1f %.1f",opos.x-vpos.x,opos.y-vpos.y,opos.z-vpos.z,odom.twist.twist.linear.x-velvec.vector.x,odom.twist.twist.linear.y-velvec.vector.y,odom.twist.twist.linear.z-velvec.vector.z);
    ROS_INFO("TBINTERP at pos. %.0f %.0f      vel: %.1f %.1f ",oopos.x,oopos.y,odo.twist.twist.linear.x,odo.twist.twist.linear.y);
    ROS_INFO("DELTAPOS at pos. %.0f %.0f      vel: %.1f %.1f ",oopos.x-vpos.x,oopos.y-vpos.y,odo.twist.twist.linear.x-velvec.vector.x,odo.twist.twist.linear.y-velvec.vector.y);
    ROS_INFO("*******************************************************************************************************************");
    tfo.transform.translation.x = opos.x;
    tfo.transform.translation.y = opos.y;
    tfo.transform.translation.z = opos.z;
    tfv.transform.translation.x = vpos.x;
    tfv.transform.translation.y = vpos.y;
    tfv.transform.translation.z = vpos.z;
    tfo.header.stamp = tfoo.header.stamp = tfv.header.stamp = ros::Time::now();
    tf_b.sendTransform(tfo);
    tf_b.sendTransform(tfv);
    tf_b.sendTransform(tfoo);
    rate.sleep();
    ros::spinOnce();
    }
  }
