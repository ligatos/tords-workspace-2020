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
nav_msgs::Odometry odom,odom_in;
ros::Time time_last,time_last_v;
geometry_msgs::TransformStamped tfodom,tfmap,tfalt;
geometry_msgs::Vector3Stamped velvec;
geometry_msgs::Point opos,vpos;
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
  ROS_INFO("STARTING at pos. %.0f %.0f %.0f",opos.x,opos.y,opos.z);
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

    tfodom.header.frame_id      = "odom";
    tfodom.child_frame_id       = "odom_calc";
    tfodom.transform.rotation.w = 1;

    tfalt.header.frame_id = "odom_calc";
    tfalt.child_frame_id  = "odom_calc_alt";
    tfalt.transform.rotation.w    = 1;

    geometry_msgs::Vector3Stamped v;
    v.header.frame_id = "odom";
    ros::Subscriber s1 = nh.subscribe("/odom_global",100,&odom_cb);
    ros::Subscriber s2 = nh.subscribe("/dji_sdk/velocity",100,&vel_cb);

    double yaw;

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
      tfodom.transform.rotation = odom_in.pose.pose.orientation= tf::createQuaternionMsgFromYaw(yaw);
      odom.pose.pose.position.x += (odom_in.twist.twist.linear.x*cos(yaw)-odom_in.twist.twist.linear.y*sin(yaw))*dt;
      odom.pose.pose.position.y += (odom_in.twist.twist.linear.x*sin(yaw)+odom_in.twist.twist.linear.y*cos(yaw))*dt;
      tfodom.transform.translation.x = odom.pose.pose.position.x;
      tfodom.transform.translation.y = odom.pose.pose.position.y;
      tfodom.transform.rotation      = odom.pose.pose.orientation;
      tfalt.header.stamp = tfmap.header.stamp = odom.header.stamp = tfodom.header.stamp = ros::Time::now();

      tf_b.sendTransform(tfodom);
      tf_b.sendTransform(tfalt);
  */
    ROS_INFO("*******************************************************************************************************************");

    ROS_INFO("DJITWIST at pos. %.0f %.0f %.0f vel: %.1f %.1f %.1f",vpos.x,vpos.y,vpos.z,velvec.vector.x,velvec.vector.y,velvec.vector.z);
    ROS_INFO("TBINTERP at pos. %.0f %.0f %.0f vel: %.1f %.1f %.1f",opos.x,opos.y,opos.z,odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
    ROS_INFO("DELTAPOS at pos. %.0f %.0f %.0f vel: %.1f %.1f %.1f",opos.x-vpos.x,opos.y-vpos.y,opos.z-vpos.z,odom.twist.twist.linear.x-velvec.vector.x,odom.twist.twist.linear.y-velvec.vector.y,odom.twist.twist.linear.z-velvec.vector.z);
    ROS_INFO("*******************************************************************************************************************");

    rate.sleep();
    ros::spinOnce();
    }
  }
