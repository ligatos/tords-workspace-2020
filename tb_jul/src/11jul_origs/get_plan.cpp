#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_datatypes.h>
#include "message_filters/subscriber.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
nav_msgs::Path to_eval,to_eval_off1,to_eval_off2;
geometry_msgs::PoseStamped pose,target_to_eval;
geometry_msgs::Point pos;
ros::Publisher evaluate_fromto_pub,target_eval_pub;
double dst,hdng,max_z,offsetdst;
int tries;
tf2_ros::Buffer tfBuffer;
ros::ServiceClient path_client;

void check_plan(nav_msgs::Path pathin){

}




void target_to_eval_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  geometry_msgs::TransformStamped transformStamped;

  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  target_to_eval = *msg;

  to_eval.poses[0].pose.position.x    = transformStamped.transform.translation.x;
  to_eval.poses[0].pose.position.y    = transformStamped.transform.translation.y;
  to_eval.poses[0].pose.position.z    = transformStamped.transform.translation.z;
  to_eval.poses[0].pose.orientation.w = 1;
  to_eval.poses[1] =target_to_eval;
  to_eval_off1.poses = to_eval_off2.poses = to_eval.poses;
  float dx = to_eval.poses[1].pose.position.x - to_eval.poses[0].pose.position.x;
  float dy = to_eval.poses[1].pose.position.y - to_eval.poses[0].pose.position.y;
  dst   = sqrt(dx*dx+dy*dy);
  hdng  = atan2(dy,dx);
  ROS_INFO("TargetEval: Receivedtarget: dst%.0f/hdng:%.0f %.0f %.0f %.0f",dst,hdng,to_eval.poses[0].pose.position.x,to_eval.poses[0].pose.position.y,to_eval.poses[0].pose.position.z);
  to_eval_off1.poses[0].pose.position.x += offsetdst * cos(hdng+M_PI/2);
  to_eval_off1.poses[0].pose.position.y += offsetdst * sin(hdng+M_PI/2);
  to_eval_off1.poses[1].pose.position.x += offsetdst * cos(hdng+M_PI/2);
  to_eval_off1.poses[1].pose.position.y += offsetdst * sin(hdng+M_PI/2);
  ROS_INFO("TargetEval: Offset target at %.0f %.0f %.0f",to_eval_off1.poses[0].pose.position.x,to_eval_off1.poses[0].pose.position.y,to_eval_off1.poses[0].pose.position.z);

  to_eval_off2.poses[0].pose.position.x += offsetdst * cos(hdng-M_PI/2);
  to_eval_off2.poses[0].pose.position.y += offsetdst * sin(hdng-M_PI/2);
  to_eval_off2.poses[1].pose.position.x += offsetdst * cos(hdng-M_PI/2);
  to_eval_off2.poses[1].pose.position.y += offsetdst * sin(hdng-M_PI/2);
  ROS_INFO("TargetEval: Offset target at %.0f %.0f %.0f",to_eval_off2.poses[0].pose.position.x,to_eval_off2.poses[0].pose.position.y,to_eval_off2.poses[0].pose.position.z);

  nav_msgs::GetPlan srv;
  srv.request.start = to_eval.poses[0];
  srv.request.goal = to_eval.poses[1];
  srv.request.tolerance = 1.5;
  ROS_INFO("Make plan: %d", (path_client.call(srv) ? 1 : 0));
  ROS_INFO("Plan size: %d", srv.response.plan.poses.size());
  double dst_plan;
  if(srv.response.plan.poses.size() < 2){
    evaluate_fromto_pub.publish(to_eval);
  }
  else{
    for(int i = 0; i < srv.response.plan.poses.size()-1;i++){
      float dx = srv.response.plan.poses[i+1].pose.position.x - srv.response.plan.poses[i].pose.position.x;
      float dy = srv.response.plan.poses[i+1].pose.position.y - srv.response.plan.poses[i].pose.position.y;
      dst_plan += sqrt(dx*dx + dy*dy);
    }
    ROS_INFO("TargetEval: totplanlength %.0f vs luftlinje: %.0f",dst_plan,dst);
    if(dst_plan > dst){
      ROS_INFO("TargetEval: Testing different acceptance criteria; planlength > 1.2 airdst, skipping raytracing");
      target_eval_pub.publish(target_to_eval);
    }
    else{
      evaluate_fromto_pub.publish(to_eval);
    }
  }
}

void evaluated_fromto_cb(const nav_msgs::Path::ConstPtr& msg){
  float delta_z = msg->poses[0].pose.position.z - to_eval.poses[0].pose.position.z;
  ROS_INFO("delta_z returned: z < max_z (%.2f < %.2f) tries: %i ",delta_z,max_z,tries);

  if(delta_z < max_z){
    target_to_eval.pose.position.z += delta_z;
    target_eval_pub.publish(target_to_eval);
  }
  else if(tries == 0){
    tries++;
    evaluate_fromto_pub.publish(to_eval_off1);
  }
  else if(tries == 1){
    tries++;
    evaluate_fromto_pub.publish(to_eval_off2);
  }
  else if(tries == 2){
    tries = 0;
    target_to_eval.header.frame_id ="declined";
    ROS_INFO("target declined");
    target_eval_pub.publish(target_to_eval);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_evaltarget_node");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf2_listener(tfBuffer);

  max_z = 15;
  offsetdst = 5;
  to_eval.poses.resize(2);
  to_eval_off1.poses.resize(2);
  to_eval_off2.poses.resize(2);
  to_eval.header.frame_id = "map"; to_eval.header.stamp = ros::Time::now();
  to_eval_off1.header   = to_eval_off2.header = to_eval.header;
  ros::Subscriber s1    = nh.subscribe("/tb_obst/evaluated_fromto",10,&evaluated_fromto_cb);
   ros::Subscriber s2   = nh.subscribe("/tb_path/target_to_evaluate",10,&target_to_eval_cb);
   evaluate_fromto_pub = nh.advertise<nav_msgs::Path>("/tb_obst/evaluate_fromto",10);
   target_eval_pub      = nh.advertise<geometry_msgs::PoseStamped>("/tb_path/target_evaluation",10);
   path_client         = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

   ros::spin();
  return 0;
}
