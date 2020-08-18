
#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <tf/transform_datatypes.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
tf2_ros::Buffer tfBuffer;

using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
string par_workdir_path;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos,last_pos;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
float pos_yaw;
bool par_unknownAsOccupied,get_floor,got_map;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump,par_delta_hdng,par_delta_incline;
nav_msgs::Path path_candidates,path_visited,path_candidates2d,path_floor,path_floor_2d;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::PointStamped pnt_midpoint;
geometry_msgs::PoseStamped last_pose;
float mapradtouse;
float grid_sidelength = 3;
geometry_msgs::PolygonStamped poly_vlp;
geometry_msgs::Point bbmin_vlp,bbmax_vlp;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}

double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
	ros::Time t0 = ros::Time::now();
  int cross = 0;
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > point.y) != (polyin.polygon.points[j].y > point.y))
           && (point.x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (point.y - polyin.polygon.points[i].y) /
            (polyin.polygon.points[j].y - polyin.polygon.points[i].y) + polyin.polygon.points[i].x) )
    {
      cross++;
    }
  }
//	float dt = (ros::Time::now() - t0).toSec();
//	ROS_INFO("sr: %.5f",dt);
  return bool(cross % 2);
}
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

bool dst_point_in_path_lim(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
bool dst_point_in_path_lim2d(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst2d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
bool is_within_hdng_incline(geometry_msgs::Point pnt,float maxdelta_hdng, float maxdelta_pitch){
	float hdng_shortest = get_shortest(get_hdng(pnt,pos),vlp_rpy.z);
  if(hdng_shortest < 0)
    hdng_shortest *= -1;
  if(hdng_shortest < maxdelta_hdng){
    float pitch_shortest = get_shortest(atan2(pnt.z-pos.z,get_dst2d(pnt,pos)),vlp_rpy.y);
    if(pitch_shortest < 0)
      pitch_shortest *= -1;
    if(pitch_shortest < maxdelta_pitch)
      return true;
    else
      return false;
  }
  else
    return false;
}
float get_inclination(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
bool is_within_incline(geometry_msgs::Point pnt,float maxdelta_pitch){
  float inclination       = get_inclination(pos,pnt);
  float delta_inclination = get_shortest(vlp_rpy.y,inclination);
  float pitch_shortest    = get_shortest(atan2(pnt.z-pos.z,get_dst2d(pnt,pos)),vlp_rpy.y);
  ROS_INFO("vlp_rpy: %.2f, inclination: %.2f, max_inclination: %.2f, pitch_shortest: %.2f,delta_inclination %.2f",vlp_rpy.y,inclination,maxdelta_pitch,pitch_shortest,delta_inclination);
  if(delta_inclination < maxdelta_pitch && delta_inclination > -maxdelta_pitch)
    return true;
  else
    return false;
}

nav_msgs::Path merge_paths(nav_msgs::Path path1,nav_msgs::Path path0){
  for(int i = 0; i < path1.poses.size(); i++){
    path0.poses.push_back(path1.poses[i]);
  }
  return path0;
}
bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied){
    octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
    octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
    zmin_global = int(round(bbmin_octree.z));
    zmax_global = int(round(bbmax_octree.z));

    z0 = fmin(z0,bbmax_octree.z-2);
    z1 = fmin(z1,bbmax_octree.z);
    bbmin_custom.x = midpoint.x-maprad;
    bbmin_custom.y = midpoint.y-maprad;
    bbmin_custom.z = z0;

    bbmax_custom.x = midpoint.x+maprad;
    bbmax_custom.y = midpoint.y+maprad;
    bbmax_custom.z = z1;
    float zmin_touse = fmax(bbmin_custom.z,bbmin_octree.z);
    float zmax_touse = fmin(bbmax_custom.z,bbmax_octree.z);

    if(zmax_touse < zmin_touse){
      zmax_touse = fmax(bbmin_custom.z,bbmin_octree.z);
      zmin_touse = fmin(bbmax_custom.z,bbmax_octree.z);
    }

    octomap::point3d boundary_min(fmax(bbmin_custom.x,bbmin_octree.x),fmax(bbmin_custom.y,bbmin_octree.y),zmin_touse);
    octomap::point3d boundary_max(fmin(bbmax_custom.x,bbmax_octree.x),fmin(bbmax_custom.y,bbmax_octree.y),zmax_touse);

    edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
            boundary_min,
            boundary_max,
            unknownAsOccupied));
    edf_ptr.get()->update();
    xmin    = int(round(boundary_min.x()))+1;  ymin    = int(round(boundary_min.y()))+1; zmin    = int(round(boundary_min.z()));
    xmax    = int(round(boundary_max.x()))-1;  ymax    = int(round(boundary_max.y()))-1; zmax    = int(round(boundary_max.z()));
    range_x = xmax - xmin;                     range_y = ymax - ymin;                    range_z = zmax - zmin;
    int vol = range_x*range_y*range_z;
    if(vol <= 0){
			ROS_INFO("FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
			return false;
		}
  return true;
}
bool update_edto_vlp(float collision_radius,float z0,float z1){

  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  float zmin_touse = fmax(z0,bbmin_octree.z);
  float zmax_touse = fmin(z1,bbmax_octree.z);

  if(zmax_touse < zmin_touse){
    zmax_touse = fmax(z0,bbmin_octree.z);
    zmin_touse = fmin(z1,bbmax_octree.z);
  }
  octomap::point3d boundary_min(fmax(bbmin_vlp.x,bbmin_octree.x),fmax(bbmin_vlp.y,bbmin_octree.y),zmin_touse);
  octomap::point3d boundary_max(fmin(bbmax_vlp.x,bbmax_octree.x),fmin(bbmax_vlp.y,bbmax_octree.y),zmax_touse);

  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  xmin    = int(round(boundary_min.x()))+1;  ymin    = int(round(boundary_min.y()))+1; zmin    = int(round(boundary_min.z()));
  xmax    = int(round(boundary_max.x()))-1;  ymax    = int(round(boundary_max.y()))-1; zmax    = int(round(boundary_max.z()));
  range_x = xmax - xmin;                     range_y = ymax - ymin;                    range_z = zmax - zmin;
  int vol = range_x*range_y*range_z;
  if(vol <= 0){
    ROS_INFO("FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
    return false;
  }
  ROS_INFO("FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);

return true;
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
float get_floor_z(geometry_msgs::PoseStamped pose_to_check){
  for(int i = 0; i < path_floor.poses.size(); i++){
    if(get_dst2d(path_floor.poses[i].pose.position,pose_to_check.pose.position) < grid_sidelength)
      return path_floor.poses[i].pose.position.z;
  }
  return 0;
}
float append_floor_to_candidates(){
  ros::Time t0 = ros::Time::now();
  for(int i = 0; i < path_floor.poses.size(); i++){
    if(is_within_hdng_incline(path_floor.poses[i].pose.position,par_delta_hdng,par_delta_incline))
      path_candidates.poses.push_back(path_floor.poses[i]);
  }
  float dt = (ros::Time::now() - t0).toSec();
  return dt;
}
void enlarge_bbvlp(float dxy, float dz){
  bbmin_vlp.x -= dxy;
  bbmin_vlp.y -= dxy;
  bbmax_vlp.x += dxy;
  bbmax_vlp.y += dxy;
  if(dz == 0 && abs(bbmin_vlp.z - bbmax_vlp.z) < 2)
    dz = 1;
  bbmin_vlp.z -= dz;
  bbmax_vlp.z += dz;
  ROS_INFO("bbmin_vlp %.0f %.0f %.0f max %.0f %.0f %.0f",bbmin_vlp.x,bbmin_vlp.y,bbmin_vlp.z,bbmax_vlp.x,bbmax_vlp.y,bbmax_vlp.z);
}
float get_path_candidates(){
//  enlarge_bbvlp(par_maxobs,0);
  enlarge_bbvlp(par_maxobs,2);

  ros::Time t0 = ros::Time::now();
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();
  path_candidates.poses.resize(0);
  for(int z = int(round(bbmin_vlp.z)); z < int(round(bbmax_vlp.z)); z++){
    update_edto_vlp(par_maxobs,z-1,z+1);
    for(int y = ymin; y < ymax; y++){
      for(int x = xmin; x < xmax; x++){
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        if(!dst_point_in_path_lim(path_floor_2d,ps.pose.position,8) ){
  //      if(is_within_incline(ps.pose.position,par_delta_incline) && dst_point_in_path_lim(path_candidates,ps.pose.position,3)){ //&& ){
          //ROS_INFO("get_floor: %.2f",get_floor_z(ps));

      //   && in_poly(poly_vlp,ps.pose.position.x,ps.pose.position.y) f( && &&  && ){
          octomap::point3d p(x,y,z);
          octomap::point3d closestObst;
          float d;
          edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
          if(round((par_minobs + par_maxobs)/2) == round(d)){
            ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - y,closestObst.x() - x));
            path_candidates.poses.push_back(ps);
          }
        }
      }
    }
  }
  float dt = (ros::Time::now() - t0).toSec();
  return dt;
}
float update_path_floor(){
  ros::Time t0 = ros::Time::now();
  geometry_msgs::PoseStamped ps;
  ps.header      = hdr();
  path_floor.poses.resize(0);
  for(int i = 0; i < path_floor_2d.poses.size(); i++){
    float z1 = path_floor_2d.poses[i].pose.position.z;
    float z0 = bbmin_octree.z;
    float collrad = z1 - z0 + grid_sidelength;
    update_edto(path_floor_2d.poses[i].pose.position,collrad,grid_sidelength,z0,z1,par_unknownAsOccupied);
    float d;
    octomap::point3d p(path_floor_2d.poses[i].pose.position.x,path_floor_2d.poses[i].pose.position.y,path_floor_2d.poses[i].pose.position.z);
    octomap::point3d closestObst;
    edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
    if(round(d) < round(collrad)){
      ps                    = path_floor_2d.poses[i];
      ps.pose.position.z    = closestObst.z() + 3;
      path_floor.poses.push_back(ps);
    }
  }
  float dt = (ros::Time::now() - t0).toSec();
  return dt;
}
geometry_msgs::PointStamped transformpoint(geometry_msgs::PointStamped pin,std::string frame_out){
  geometry_msgs::PointStamped pout;
  pin.header.stamp = ros::Time();
  try
  {
    pout = tfBuffer.transform(pin, frame_out);
  }
  catch (tf2::TransformException &ex)
  {
        ROS_WARN("Failure %s\n", ex.what());
  }
  return pout;
}

float get_polypath_vlp_orig(float collision_radius,float maxdelta_hdng,int num_rays,float tot_length){
  ros::Time t0 = ros::Time::now();
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();
  path_floor_2d.header = hdr();
  path_floor_2d.poses.resize(0);
  poly_vlp.polygon.points.resize(num_rays);
  poly_vlp.polygon.points[0].x  = pos.x;
  poly_vlp.polygon.points[0].y  = pos.y;
  poly_vlp.polygon.points[0].z  = pos.z;
  float rads_pr_i = maxdelta_hdng*2 / num_rays;
  bbmax_vlp.x = -1000; bbmax_vlp.y = -1000; bbmax_vlp.z = -1000;
  bbmin_vlp.x = 1000; bbmin_vlp.y = 1000; bbmin_vlp.z = 1000;
  std::vector<geometry_msgs::Point> points;
  for(int i = 1; i < num_rays; i++){
    geometry_msgs::PointStamped p,pout;
    p.header.frame_id = "velodyne_aligned";
    float a = -maxdelta_hdng + rads_pr_i * i;
    p.point.x = tot_length*cos(a);
    p.point.y = tot_length*sin(a);
    p.point.z = 0;
    p.header.stamp = ros::Time();
    pout = transformpoint(p,"map");
    if(pout.point.x > bbmax_vlp.x)
      bbmax_vlp.x = pout.point.x;
    if(pout.point.y > bbmax_vlp.y)
      bbmax_vlp.y = pout.point.y;
    if(pout.point.z > bbmax_vlp.z)
      bbmax_vlp.z = pout.point.z;
    if(pout.point.x < bbmin_vlp.x)
      bbmin_vlp.x = pout.point.x;
    if(pout.point.y < bbmin_vlp.y)
      bbmin_vlp.y = pout.point.y;
    if(pout.point.z < bbmin_vlp.z)
      bbmin_vlp.z = pout.point.z;
    points.push_back(pout.point);
  }

//  enlarge_bbvlp(collision_radius,tot_length * sin(par_delta_incline));
  float dx = bbmax_vlp.x - bbmin_vlp.x;
  float dy = bbmax_vlp.y - bbmin_vlp.y;
  float dz = bbmax_vlp.z - bbmin_vlp.z;
  pnt_midpoint.point.x = (bbmin_vlp.x + bbmax_vlp.x)/2;
  pnt_midpoint.point.y = (bbmin_vlp.y + bbmax_vlp.y)/2;
  pnt_midpoint.point.z = (bbmin_vlp.z + bbmax_vlp.z)/2;
  //mapradtouse = fmax(dx,dy)/2 + collision_radius;
//  ROS_INFO("POLYVLP: points: %i dx: %.0f dy: %.0f dz: %.0f, midpoint: x: %.0f y: %.0f z: %.0f",points.size(),dx,dy,dz,midpoint.x,midpoint.y,midpoint.z);
//  update_edto(pnt_midpoint.point,collision_radius,mapradtouse+collision_radius,bbmin_vlp.z-2,bbmax_vlp.z,false);
  update_edto_vlp(collision_radius,bbmin_vlp.z,bbmax_vlp.z);
  bbmax_vlp.x = -1000; bbmax_vlp.y = -1000; bbmax_vlp.z = -1000;
  bbmin_vlp.x = 1000; bbmin_vlp.y = 1000; bbmin_vlp.z = 1000;
  for(int i = 0; i < points.size(); i++){
    Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt2_vec(points[i].x,points[i].y,points[i].z);
    Eigen::Vector3f stride_vec,cur_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    float distance = collision_radius-1;
    float cur_ray_len = 0;
    cur_vec = pnt1_vec;
    while(distance > 2 && cur_ray_len < tot_length){
      cur_vec = cur_vec + stride_vec;
      ps.pose.position.x  = cur_vec.x();
      ps.pose.position.y  = cur_vec.y();
      ps.pose.position.z  = cur_vec.z();
      cur_ray_len = (cur_vec - pnt1_vec).norm();
      octomap::point3d closestObst;
      point3d p(cur_vec.x(),cur_vec.y(),cur_vec.z());
      distance = edf_ptr.get()->getDistance(p);
      if(dst_point_in_path_lim2d(path_floor_2d,ps.pose.position,grid_sidelength*2)){
        ps.pose.orientation.y = 0.7071;
        ps.pose.orientation.w = 0.7071;
        path_floor_2d.poses.push_back(ps);
      }
    }
    if(cur_vec.x() > bbmax_vlp.x)
      bbmax_vlp.x = cur_vec.x();
    if(cur_vec.y() > bbmax_vlp.y)
      bbmax_vlp.y = cur_vec.y();
    if(cur_vec.z() > bbmax_vlp.z)
      bbmax_vlp.z = cur_vec.z();
    if(cur_vec.x() < bbmin_vlp.x)
      bbmin_vlp.x = cur_vec.x();
    if(cur_vec.y() < bbmin_vlp.y)
      bbmin_vlp.y = cur_vec.y();
    if(cur_vec.z() < bbmin_vlp.z)
      bbmin_vlp.z = cur_vec.z();

    poly_vlp.polygon.points[i].x = cur_vec.x();
    poly_vlp.polygon.points[i].y = cur_vec.y();
    poly_vlp.polygon.points[i].z = cur_vec.z();
  }
  poly_vlp.polygon.points[num_rays-1].x = pos.x;
  poly_vlp.polygon.points[num_rays-1].y = pos.y;
  poly_vlp.polygon.points[num_rays-1].z = pos.z;
  float dt = (ros::Time::now() - t0).toSec();
  return dt;
}
float get_polypath_vlp(float collision_radius,float maxdelta_hdng,int num_rays,float tot_length){
  ros::Time t0 = ros::Time::now();
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();
  path_floor_2d.header = hdr();
  path_floor_2d.poses.resize(0);

  float rads_pr_i = maxdelta_hdng*2 / num_rays;
  bbmax_vlp.x = -1000; bbmax_vlp.y = -1000; bbmax_vlp.z = -1000;
  bbmin_vlp.x = 1000; bbmin_vlp.y = 1000; bbmin_vlp.z = 1000;
  std::vector<geometry_msgs::Point> points;

  for(int k = 0; k < 3; k++){
    for(int i = 1; i < num_rays; i++){
      geometry_msgs::PointStamped p,pout;
      p.header.frame_id = "velodyne_aligned";
      float a_k = -par_delta_incline + k * par_delta_incline;
      float a_i = -maxdelta_hdng + rads_pr_i * i;
      p.point.x = tot_length*cos(a_k)*cos(a_i),
      p.point.y = tot_length*cos(a_k)*sin(a_i),
      p.point.z = tot_length*sin(a_k);
      p.header.stamp = ros::Time();
      pout = transformpoint(p,"map");
      if(pout.point.x > bbmax_vlp.x)
        bbmax_vlp.x = pout.point.x;
      if(pout.point.y > bbmax_vlp.y)
        bbmax_vlp.y = pout.point.y;
      if(pout.point.z > bbmax_vlp.z)
        bbmax_vlp.z = pout.point.z;
      if(pout.point.x < bbmin_vlp.x)
        bbmin_vlp.x = pout.point.x;
      if(pout.point.y < bbmin_vlp.y)
        bbmin_vlp.y = pout.point.y;
      if(pout.point.z < bbmin_vlp.z)
        bbmin_vlp.z = pout.point.z;
  //    ROS_INFO("p(a: %.2f a_k: %.2f) %.0f %.0f %.0f -> %.0f %.0f %.0f)",a_i,a_k,p.point.x,p.point.y,p.point.z,pout.point.x,pout.point.y,pout.point.z);
      points.push_back(pout.point);
    }
  }
  poly_vlp.polygon.points.resize(points.size());
  poly_vlp.polygon.points[0].x  = pos.x;
  poly_vlp.polygon.points[0].y  = pos.y;
  poly_vlp.polygon.points[0].z  = pos.z;
  enlarge_bbvlp(collision_radius,0);
  float dx = bbmax_vlp.x - bbmin_vlp.x;
  float dy = bbmax_vlp.y - bbmin_vlp.y;
  float dz = bbmax_vlp.z - bbmin_vlp.z;
  pnt_midpoint.point.x = (bbmin_vlp.x + bbmax_vlp.x)/2;
  pnt_midpoint.point.y = (bbmin_vlp.y + bbmax_vlp.y)/2;
  pnt_midpoint.point.z = (bbmin_vlp.z + bbmax_vlp.z)/2;
  //mapradtouse = fmax(dx,dy)/2 + collision_radius;
  // ROS_INFO("POLYVLP: points: %i dx: %.0f dy: %.0f dz: %.0f, midpoint: x: %.0f y: %.0f z: %.0f",points.size(),dx,dy,dz,pnt_midpoint.point.x,pnt_midpoint.point.y,pnt_midpoint.point.z);
//  update_edto(pnt_midpoint.point,collision_radius,mapradtouse+collision_radius,bbmin_vlp.z-2,bbmax_vlp.z,false);
  update_edto_vlp(collision_radius,bbmin_vlp.z,bbmax_vlp.z);
  bbmax_vlp.x = -1000; bbmax_vlp.y = -1000; bbmax_vlp.z = -1000;
  bbmin_vlp.x = 1000; bbmin_vlp.y = 1000; bbmin_vlp.z = 1000;
  for(int i = 0; i < points.size(); i++){
    Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt2_vec(points[i].x,points[i].y,points[i].z);
    Eigen::Vector3f stride_vec,cur_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    float distance = collision_radius-1;
    float cur_ray_len = 0;
    cur_vec = pnt1_vec;
    while(distance > 2 && cur_ray_len < tot_length){
      cur_vec     = cur_vec + stride_vec;
      cur_ray_len = (cur_vec - pnt1_vec).norm();

      if(cur_vec.x() > float(xmin) && cur_vec.y() > float(ymin) && cur_vec.z() > float(zmin)
      && cur_vec.x() < float(xmax) && cur_vec.y() < float(ymax) && cur_vec.z() < float(zmax)){
      //  ROS_INFO("p %i",i);
        ps.pose.position.x  = cur_vec.x();
        ps.pose.position.y  = cur_vec.y();
        ps.pose.position.z  = cur_vec.z();
        octomap::point3d closestObst;
        point3d p(cur_vec.x(),cur_vec.y(),cur_vec.z());
        distance = edf_ptr.get()->getDistance(p);
        if(dst_point_in_path_lim2d(path_floor_2d,ps.pose.position,grid_sidelength*2)){
          ps.pose.orientation.y = 0.7071;
          ps.pose.orientation.w = 0.7071;
          path_floor_2d.poses.push_back(ps);
        }
        if(cur_vec.x() > bbmax_vlp.x)
          bbmax_vlp.x = cur_vec.x();
        if(cur_vec.y() > bbmax_vlp.y)
          bbmax_vlp.y = cur_vec.y();
        if(cur_vec.z() > bbmax_vlp.z)
          bbmax_vlp.z = cur_vec.z();
        if(cur_vec.x() < bbmin_vlp.x)
          bbmin_vlp.x = cur_vec.x();
        if(cur_vec.y() < bbmin_vlp.y)
          bbmin_vlp.y = cur_vec.y();
        if(cur_vec.z() < bbmin_vlp.z)
          bbmin_vlp.z = cur_vec.z();

        poly_vlp.polygon.points[i].x = cur_vec.x();
        poly_vlp.polygon.points[i].y = cur_vec.y();
        poly_vlp.polygon.points[i].z = cur_vec.z();
      }
      else{
        break;
      }
    }
  }
  enlarge_bbvlp(collision_radius,2);
  poly_vlp.polygon.points[poly_vlp.polygon.points.size()-1].x = pos.x;
  poly_vlp.polygon.points[poly_vlp.polygon.points.size()-1].y = pos.y;
  poly_vlp.polygon.points[poly_vlp.polygon.points.size()-1].z = pos.z;
  float dt = (ros::Time::now() - t0).toSec();
  return dt;
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    last_pose.pose.position    = pos;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
    path_visited.header.stamp  = ros::Time::now();
  }
  if(!get_floor && (abs(last_yaw - pos_yaw) > 1 || get_dst3d(pos,last_pos) > 5)){
    get_floor = true;
  }
}
void checktfvlp(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_pathmini_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("workdir_path",par_workdir_path);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);
  private_nh.param("par_maxobs",  par_maxobs, 20.0);//*2.0);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("par_minobs",  par_minobs, 2.0);//*2.0);
  private_nh.param("delta_hdng",   par_delta_hdng, M_PI/2);//*2.0);
  private_nh.param("delta_incline",  par_delta_incline, M_PI/24);//*2.0);

  path_floor_2d.header      = hdr();
  path_floor.header         = hdr();
  poly_vlp.header           = hdr();
  path_candidates.header    = hdr();
  path_visited.header       = hdr();
  last_pose.header          = hdr();
  pnt_midpoint.header       = hdr();
  last_pose.pose.orientation.w = 1;
  path_visited.poses.push_back(last_pose);

  tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Publisher pub_candidates = nh.advertise<nav_msgs::Path>("/tb_path/candidates",100);
  ros::Publisher pub_floor_2d   = nh.advertise<nav_msgs::Path>("/tb_path/floor_2d",100);
  ros::Publisher pub_floor      = nh.advertise<nav_msgs::Path>("/tb_path/floor",100);
  ros::Publisher pub_visited   	= nh.advertise<nav_msgs::Path>("/tb_path/visited",10);
  ros::Publisher poly_pub 			= nh.advertise<geometry_msgs::PolygonStamped>("/tb_path/poly_vlp",100);
  ros::Publisher pub_midpoint   = nh.advertise<geometry_msgs::PointStamped>("/tb_path/midpoint",100);
//  ros::Publisher pub_path             = nh.advertise<nav_msgs::Path>("/tb_path",100);

  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ROS_INFO("Ready to convert octomaps.");
  ros::Rate rate(2.0);
  ros::Time t0           = ros::Time::now();
  ros::Time last_update  = ros::Time::now();
  float collision_radius = 5;
  while(ros::ok()){
    checktf();
    checktfvlp();
    if(got_map){
      if((ros::Time::now()-last_update).toSec() > 5)
        get_floor = true;
      if(get_floor){
        int num_rays     = 32;
        float ray_length = 15;
        float dt_polypath   = get_polypath_vlp(par_maxobs,par_delta_hdng,num_rays,ray_length);
        float dt_floor      = update_path_floor();
        float dt_candidates = get_path_candidates();
      //  float dt_append     = append_floor_to_candidates();
        int floor_2d        = path_floor_2d.poses.size();
        int floor           = path_floor.poses.size();
        int candidates      = path_candidates.poses.size();
        //path_candidates     = constrain_path_bbpoly(path_candidates,poly_vlp);
        ROS_INFO("floor_2d: %i -> floor: %i -> candidates: %i -> constrained: %i (poly_vlp: %i)",floor_2d,floor,candidates,path_candidates.poses.size(),poly_vlp.polygon.points.size());
        ROS_INFO("TIMER: dt_polypath: %.4f, dt_floor: %.4f, dt_candidates: %.4f",dt_polypath,dt_floor,dt_candidates);
        last_update = ros::Time::now();
        get_floor = false;
      }
      pub_candidates.publish(path_candidates);
      pub_floor_2d.publish(path_floor_2d);
      pub_floor.publish(path_floor);
      pub_midpoint.publish(pnt_midpoint);
      pub_visited.publish(path_visited);
      poly_pub.publish(poly_vlp);
    }
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
