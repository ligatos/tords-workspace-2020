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

bool first = true;
double yaw_odom,target_arm1,target_arm2_tilt,target_arm2_pan,t_int_1,t_int_2,t_int_3;
const float deg2rad = M_PI/180.0; const float rad2deg = 180.0/M_PI; const float earth = 6378137.0;
int mainstate;
bool test_odom = true;

ros::Time time_last_v,time_last_z;
tf2_ros::Buffer tfBuffer;

geometry_msgs::Vector3 rpy,rpyx;
geometry_msgs::Vector3Stamped vlocal_gps,vglobal_gps,armvec_min_max_radprs;
geometry_msgs::TransformStamped tfsdunkarm,tff,tfh,tfrp,tfm,tfo,tfoa,tfa,tfa2,tfp,tfpos,tfb,tfs,tfx,tfv,tf_up,tf_dwn,tfvt;
sensor_msgs::Joy cmd;
nav_msgs::Odometry odom,lowrate_odom;
std::string cmd_mode;
geometry_msgs::Point cmd_pos;
double vel_xy_max = 5.0;
std_msgs::Float64 arm_cmd1,arm_cmd2_tilt,arm_cmd2_pan;
geometry_msgs::Point offset,pos;
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
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_inclination(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
void att_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
  tf2::Matrix3x3 q(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));
  q.getRPY(rpy.x,rpy.y,rpy.z);
  first         = false;
  if(std::isnan(rpy.z) || std::isnan(rpy.y) || std::isnan(rpy.x))
    rpy.z = rpy.x = rpy.y = 0;
}
void pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  tff.transform.translation.x = msg->point.x;
  tff.transform.translation.y = msg->point.y;
}
void spose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  tfs.transform.translation.x = msg->pose.position.x;
  tfs.transform.translation.y = msg->pose.position.y;
  tfs.transform.translation.z = msg->pose.position.z;
  tfs.transform.rotation      = msg->pose.orientation;
}
void abs_cb(const std_msgs::Float64::ConstPtr& msg){
  if(tfoa.transform.translation.z != msg->data)
    time_last_z = ros::Time::now();
  tfoa.transform.translation.z = msg->data;
}
void height_cb(const std_msgs::Float32::ConstPtr& msg){
  tfh.transform.translation.z = msg->data;
}
void arm1_tilt_cb(const std_msgs::Float64::ConstPtr& msg){
  target_arm1 = msg->data;
}
void arm2_tilt_cb(const std_msgs::Float64::ConstPtr& msg){
  target_arm2_tilt = msg->data;
}
void arm2_pan_cb(const std_msgs::Float64::ConstPtr& msg){
  target_arm2_pan  = msg->data;
}
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  if(!first){
    float dt = ((ros::Time::now() - time_last_v).toSec());
    time_last_v                = ros::Time::now();
    if(dt > 0.3)
      dt = 0.0;
    odom.twist.twist.angular.z = saturate(msg->angular.z,10);
    odom.twist.twist.linear.x  = saturate(msg->linear.x,10);
    odom.twist.twist.linear.y  = saturate(msg->linear.y,10);


    cmd_mode = "mb";
    vel_xy_max = 6;
  }
}
void arm_cmd_cb(const geometry_msgs::Vector3::ConstPtr& msg){
  armvec_min_max_radprs.vector = *msg;
  armvec_min_max_radprs.header.stamp = ros::Time::now();
//  ROS_INFO("Arm motion set: min: %.2f -> max: %.2f, radsprsec: %.2f",armvec_min_max_radprs.vector.x,armvec_min_max_radprs.vector.y,armvec_min_max_radprs.vector.z);
  t_int_1    = abs(armvec_min_max_radprs.vector.x - target_arm1) / armvec_min_max_radprs.vector.z;
  t_int_2    = abs(armvec_min_max_radprs.vector.y - armvec_min_max_radprs.vector.x) / armvec_min_max_radprs.vector.z;
  t_int_3    = abs(armvec_min_max_radprs.vector.y - target_arm1) / armvec_min_max_radprs.vector.z;
  float t_tot = t_int_1 + t_int_2 + t_int_3;
  ROS_INFO("Arm motion set: t_tot: %.2f",t_tot);
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
void cmdpos_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    cmd_mode          = "cmd_pos";
    cmd_pos           = *msg;
    odom.pose.pose.position = pos;
    odom.pose.pose.position.z=0;
}
void velxymax_cb(const std_msgs::Float64::ConstPtr& msg){
  vel_xy_max = msg->data;
}

void update_pos(){
  pos.x = tff.transform.translation.x;
  pos.y = tff.transform.translation.y;
  pos.z = tfh.transform.translation.z;
}
void update_arms(){
  arm_cmd1.data 		 = (rpy.y - target_arm1);
  arm_cmd2_tilt.data = (rpy.y - target_arm2_tilt);
  arm_cmd2_pan.data  = get_shortest(constrainAngle(rpy.z - target_arm2_pan),0);

// --- == forward
  if((ros::Time::now() - armvec_min_max_radprs.header.stamp).toSec() < (t_int_1+t_int_2+t_int_3)){
    offset.x = offset.y = 0;
    float dt_arm = (ros::Time::now() - armvec_min_max_radprs.header.stamp).toSec();
    if(dt_arm > (t_int_1+t_int_2)){
      arm_cmd1.data = armvec_min_max_radprs.vector.y + (dt_arm-(t_int_2+t_int_1)) * armvec_min_max_radprs.vector.z;
      arm_cmd2_tilt.data = arm_cmd1.data;
    }
    else if(dt_arm > t_int_1){
      arm_cmd1.data = armvec_min_max_radprs.vector.x - (dt_arm-t_int_1)           * armvec_min_max_radprs.vector.z;
      arm_cmd2_tilt.data = arm_cmd1.data;
    }
    else{
      arm_cmd1.data = target_arm1                     +  dt_arm                    * armvec_min_max_radprs.vector.z;
      arm_cmd2_tilt.data = arm_cmd1.data;
    }
  }
  arm_cmd2_tilt.data = saturate(arm_cmd2_tilt.data,M_PI);
  arm_cmd2_pan.data = saturate(arm_cmd2_pan.data,M_PI);
  if(arm_cmd1.data > 0)saturate(arm_cmd1.data,M_PI/4);
  if(arm_cmd1.data < 0)saturate(arm_cmd1.data,M_PI/2);
}
geometry_msgs::Vector3 vel_global2local(geometry_msgs::Vector3 global_vel,float yaw_used) {
	geometry_msgs::Vector3 local_vel;
  float vel_scalar  = sqrt(global_vel.x * global_vel.x + global_vel.y * global_vel.y);
  float rpy_rel     = atan2(global_vel.y,global_vel.x) - yaw_used;
  local_vel.x = vel_scalar * cos(rpy_rel);
  local_vel.y = vel_scalar * sin(rpy_rel);
	local_vel.z = global_vel.z;
	return local_vel;
}
geometry_msgs::Quaternion get_quat(geometry_msgs::Point to, geometry_msgs::Point from){
  float incl = atan2(to.z - from.z,get_dst2d(to,from));
  float hdng = get_hdng(to,from);
  ROS_INFO("Incl: %.2f hdng: %.2f",incl,hdng);

  return tf::createQuaternionMsgFromRollPitchYaw(0,incl,hdng);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmdodomtf_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Rate rate(100);

    tf2_ros::TransformListener tf2_listener(tfBuffer);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);

    tf2_ros::TransformBroadcaster tf_b;
    nav_msgs::Odometry odom_global;
    tfvt.header.frame_id = "base_position";
    tfvt.child_frame_id = "velodyne_perfect";
    tfvt.transform.rotation.w = 1;
    tfm.header.frame_id         = "map";
    tfm.child_frame_id          = "odom";
    tfm.transform.rotation.w    = 1;

    tfa.header.frame_id         = "base_link";
    tfa.child_frame_id          = "mx64";
    tfa.transform.rotation.w    = 1;

    tfpos.header.frame_id         = "odom";
    tfpos.child_frame_id          = "base_position";
    tfpos.transform.rotation.w    = 1;

    tfx.header.frame_id         = "odom";
    tfx.child_frame_id          = "imu_x";
    tfx.transform.rotation.w    = 1;

    tff.header.frame_id         = "odom";
    tff.child_frame_id          = "base_footprint";
    tff.transform.rotation.w    = 1;

    tfh.header.frame_id         = "base_footprint";
    tfh.child_frame_id          = "base_stabilized";
    tfh.transform.rotation.w    = 1;

    tf_up.header.frame_id         = "base_stabilized";
    tf_up.child_frame_id          = "base_stabilized_up";
    tf_up.transform.translation.z = 1;
    tf_up.transform.rotation.w = 1;

    tf_dwn.header.frame_id         = "base_stabilized";
    tf_dwn.child_frame_id          = "base_stabilized_down";
    tf_dwn.transform.translation.z = -1;
    tf_dwn.transform.rotation.w    = 1;

    tfrp.header.frame_id        = "base_stabilized";
    tfrp.child_frame_id         = "base_link";
    tfrp.transform.rotation.w   = 1;

    tfp.header.frame_id        = "base_stabilized";
    tfp.child_frame_id         = "base_future";
    tfp.transform.rotation.w   = 1;

    odom.header.frame_id         = "odom";
    odom.child_frame_id          = "base_perfect";
    odom.pose.pose.orientation.w = 1;

    odom_global.header.frame_id         = "odom";
    odom_global.child_frame_id          = "global";
    odom_global.pose.pose.orientation.w = 1;

		lowrate_odom.header.frame_id         = "odom";
    lowrate_odom.child_frame_id          = "base_stabilized";
    lowrate_odom.pose.pose.orientation.w = 1;

    tfo.header.frame_id          = "odom";
    tfo.child_frame_id           = "base_perfect";
    tfo.transform.rotation.w     = 1;

    tfv.header.frame_id          = "base_position";
    tfv.child_frame_id           = "velodyne_alternative";
    tfv.transform.rotation.w     = 1;

    tfb.header.frame_id          = "map";
    tfb.child_frame_id           = "tfb";
    tfb.transform.rotation.w     = 1;

    tfs.header.frame_id          = "map";
    tfs.child_frame_id           = "pose_stereo_slam";
    tfs.transform.rotation.w     = 1;

    tfoa.header.frame_id          = "base_perfect";
    tfoa.child_frame_id           = "base_perfect_alt";
    tfoa.transform.rotation.w     = 1;
		tfsdunkarm.header.frame_id          = "base_link";
		tfsdunkarm.child_frame_id           = "sdunk_center";
		tfsdunkarm.transform.rotation.w     = 1;

    vlocal_gps.header.frame_id    = "base_stabilized";
    vglobal_gps.header.frame_id   = "odom";

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

    ros::Subscriber s22= nh.subscribe("/dji_sdk/height_above_takeoff", 100,&height_cb);
    ros::Subscriber s3 = nh.subscribe("/dji_sdk/attitude",             100,&att_cb);
    ros::Subscriber s5 = nh.subscribe("/dji_sdk/local_position",       100,&pos_cb);

    ros::Subscriber s15= nh.subscribe("/tb_cmd/arm_motion",   100,&arm_cmd_cb);
		ros::Subscriber s10= nh.subscribe("/tb_cmd/arm1_tilt",    100,&arm1_tilt_cb);
		ros::Subscriber s11= nh.subscribe("/tb_cmd/arm2_tilt",    100,&arm2_tilt_cb);
		ros::Subscriber s13= nh.subscribe("/tb_cmd/arm2_pan",     100,&arm2_pan_cb);
    ros::Subscriber s9 = nh.subscribe("/tb_cmd/alt_target",   100,&abs_cb);
    ros::Subscriber s18 = nh.subscribe("/tb_fsm/main_state",  100,&mainstate_cb);

    ros::Subscriber s8 = nh.subscribe("/cmd_vel",             100,&twist_cb);
    ros::Subscriber ss = nh.subscribe("/pose",                100,&spose_cb);
    ros::Subscriber sss = nh.subscribe("/cmd_pos",            100,&cmdpos_cb);
    ros::Subscriber sxy = nh.subscribe("/tb_cmd/vel_xy_max",  100,&velxymax_cb);


    ros::Publisher odom_pub           = nh.advertise<nav_msgs::Odometry>("/odom", 100);
		ros::Publisher pub_global_odom    = nh.advertise<nav_msgs::Odometry>("/odom_global", 100);
    ros::Publisher cmd_pub            = nh.advertise<sensor_msgs::Joy>("/tb_exeq/cmd_joy", 100);
		ros::Publisher pub_arm_cmd1				= nh.advertise<std_msgs::Float64>("/tilt_controller/command", 100);
	//	ros::Publisher pub_arm_cmd2_tilt	= nh.advertise<std_msgs::Float64>("/tilt_controller/command", 100);
	//	ros::Publisher pub_arm_cmd2_pan		= nh.advertise<std_msgs::Float64>("/pan_controller/command", 100);

		double dt;
    float lastyaw;
    ros::Time time_last = ros::Time::now();
		ros::Time lowrate_update = ros::Time::now();
    std::queue<float> vx_que;
    std::queue<float> vy_que;
    std::queue<float> vz_que;
    std::queue<float> vyaw_que;

    int que_size = 100;
    float vx_sum = 0;
    float vy_sum = 0;
    float vz_sum = 0;
    float vyaw_sum = 0;

    while(ros::ok()){

      Eigen::Vector3f pnt1_vec(pos.x,pos.y,0);
      Eigen::Vector3f stride_vec;
      geometry_msgs::Vector3 cmd_global,cmd_local;

      rate.sleep();
      ros::spinOnce();

      if((ros::Time::now() - time_last).toSec() > 0.03) dt = 0.03;
      else dt = (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();


      if(cmd_mode == "mb"){
       if((ros::Time::now() - time_last_v).toSec() > 0.5) {
         cmd_mode = "";
         odom.twist.twist.angular.z = odom.twist.twist.linear.x = odom.twist.twist.linear.y = 0;
       }
     }
     if(!test_odom){
      odom.pose.pose.position.x  += ((odom.twist.twist.linear.x*cos(yaw_odom)-odom.twist.twist.linear.y*sin(yaw_odom)) * dt);
      odom.pose.pose.position.y  += ((odom.twist.twist.linear.x*sin(yaw_odom)+odom.twist.twist.linear.y*cos(yaw_odom)) * dt);
      yaw_odom                   += odom.twist.twist.angular.z * dt;
      yaw_odom                    = constrainAngle(yaw_odom);
    }
    update_pos();
    update_arms();

    if(cmd_mode == "mb"){
      Eigen::Vector3f pnt2_vec(odom.pose.pose.position.x,odom.pose.pose.position.y,0);
      float cmd_length = fmin((pnt2_vec - pnt1_vec).norm(),odom.twist.twist.linear.x);
      stride_vec = (pnt2_vec - pnt1_vec).normalized()  * cmd_length;
    }
    if(mainstate == 1 && cmd_mode == "cmd_pos"){
      Eigen::Vector3f pnt2_vec(cmd_pos.x,cmd_pos.y,0);
      float cmd_length = fmin((pnt2_vec - pnt1_vec).norm(),vel_xy_max);
      stride_vec = (pnt2_vec - pnt1_vec).normalized()  * cmd_length;
      cmd_global.x = stride_vec.x();
      cmd_global.y = stride_vec.y();
      odom.twist.twist.angular.z = saturate(get_shortest(get_hdng(cmd_pos,pos),yaw_odom),1.5);
      odom.twist.twist.linear    = vel_global2local(cmd_global,yaw_odom);
    }

    tfo.transform.translation.x = odom.pose.pose.position.x;
    tfo.transform.translation.y = odom.pose.pose.position.y;
    tfo.transform.rotation      = odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_odom);

    tfpos.transform.translation   = tff.transform.translation;
    tfpos.transform.translation.z = tfh.transform.translation.z;

      cmd.axes[0] = stride_vec.x();
      cmd.axes[1] = stride_vec.y();
      cmd.axes[2] = tfoa.transform.translation.z;
      cmd.axes[3] = yaw_odom;
      cmd.header.frame_id = "cmd_vel";
      tfrp.transform.rotation     	= tf::createQuaternionMsgFromRollPitchYaw(rpy.x,rpy.y,0);
      tff.transform.rotation     	  = tf::createQuaternionMsgFromRollPitchYaw(0,0,rpy.z);
			tfsdunkarm.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,arm_cmd2_tilt.data,arm_cmd2_pan.data);
      tfx.transform.translation   = tfpos.transform.translation;
      tfvt.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,arm_cmd1.data,yaw_odom);
      tfx.transform.rotation      = tf::createQuaternionMsgFromRollPitchYaw(-rpyx.x,-rpyx.y,rpyx.z+M_PI/2);
      tfv.transform.rotation      = tf::createQuaternionMsgFromRollPitchYaw(-rpyx.x,-rpyx.y,rpy.z);
      odom_global.header.stamp = tfvt.header.stamp = tfsdunkarm.header.stamp = tf_up.header.stamp = tf_dwn.header.stamp =  tfv.header.stamp = tfx.header.stamp = tfs.header.stamp = tfb.header.stamp = tfa.header.stamp = tfpos.header.stamp = tfp.header.stamp = tfm.header.stamp = tfoa.header.stamp = odom.header.stamp = tfo.header.stamp = cmd.header.stamp = tfh.header.stamp = tfrp.header.stamp = tff.header.stamp = ros::Time::now();
      odom_pub.publish(odom);
      cmd_pub.publish(cmd);
			pub_arm_cmd1.publish(arm_cmd1);
		//	pub_arm_cmd2_tilt.publish(arm_cmd2_tilt);
		//	pub_arm_cmd2_pan.publish(arm_cmd2_pan);
      tf_b.sendTransform(tfoa);
      tf_b.sendTransform(tfo);
      tf_b.sendTransform(tfv);
      tf_b.sendTransform(tfrp);
      tf_b.sendTransform(tff);
      tf_b.sendTransform(tfh);
      tf_b.sendTransform(tfm);
      tf_b.sendTransform(tfp);
      tf_b.sendTransform(tfvt);
			tf_b.sendTransform(tfs);
      tf_b.sendTransform(tfx);
      tf_b.sendTransform(tf_up);
			tf_b.sendTransform(tf_dwn);
			tf_b.sendTransform(tfsdunkarm);
      if(tfb.transform.translation.x + tfb.transform.translation.y + tfb.transform.translation.z > 0)
        tf_b.sendTransform(tfb);
      tf_b.sendTransform(tfpos);
			float dt_lowrate = (ros::Time::now() - lowrate_odom.header.stamp).toSec();

      float vyaw = get_shortest(rpy.z,lastyaw)/dt;
      float vz   = (tfh.transform.translation.z - odom_global.pose.pose.position.z)/dt;
      float vy   = (tff.transform.translation.y - odom_global.pose.pose.position.y)/dt;
      float vx   = (tff.transform.translation.x - odom_global.pose.pose.position.x)/dt;
      if(!std::isnan(vz) && !std::isnan(vy) && !std::isnan(vx)){
        lastyaw = rpy.z;
        vx_que.push(vx);
        vy_que.push(vy);
        vz_que.push(vz);
        vyaw_que.push(vyaw);

        vx_sum += vx;
        vy_sum += vy;
        vz_sum += vz;
        vyaw_sum += vyaw;

        if(vz_que.size() > que_size){

          vx_sum -= vx_que.front();
          vy_sum -= vy_que.front();
          vz_sum -= vz_que.front();
          vyaw_sum -= vyaw_que.front();

          odom_global.twist.twist.linear.x = vx_sum / que_size;
          odom_global.twist.twist.linear.y = vy_sum / que_size;
          odom_global.twist.twist.linear.z  = vz_sum / que_size;
          odom_global.twist.twist.angular.z = vyaw_sum / que_size;
          vx_que.pop();
          vy_que.pop();
          vz_que.pop();
          vyaw_que.pop();
          float incl = atan2(odom_global.twist.twist.linear.z,sqrt(pow(odom_global.twist.twist.linear.x,2)+pow(odom_global.twist.twist.linear.y,2)));
          float hdng = atan2(odom_global.twist.twist.linear.y,odom_global.twist.twist.linear.x);
          odom_global.pose.pose.position.x = tff.transform.translation.x;
          odom_global.pose.pose.position.y = tff.transform.translation.y;
          odom_global.pose.pose.position.z = tfh.transform.translation.z;
          odom_global.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-incl,hdng);
          pub_global_odom.publish(odom_global);
        }

        Eigen::Vector3f pnt1_vec2(0.0,0.0,0.0);
        Eigen::Vector3f pnt2_vec2(stride_vec.x(),stride_vec.y(),odom_global.twist.twist.linear.z);
        Eigen::Vector3f stride_vec2 = (pnt2_vec2 - pnt1_vec2).normalized();

        geometry_msgs::Point test_pos,p0;
        test_pos.x = stride_vec2.x();
        test_pos.y = stride_vec2.y();
        test_pos.z = stride_vec2.z();
      }

      //tf::getYaw(yaw_odom);

      //ROS_INFO("Incl: %.2f hdng: %.2f, vx: %.2f vy: %.2f vz: %.2f vaz: %.2f",incl,hdng,odom_global.twist.twist.linear.x,odom_global.twist.twist.linear.y,odom_global.twist.twist.linear.z,odom_global.twist.twist.angular.z);

    }
    return 0;
}
