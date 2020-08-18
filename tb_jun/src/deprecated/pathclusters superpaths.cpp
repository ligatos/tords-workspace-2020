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
nav_msgs::Path superpath_down,superpath_side;
double par_s_r,par_d_r,par_d_zlo,par_d_zhi,par_s_zlo,par_s_zhi;

nav_msgs::Path path_raw_down,path_raw_side,path_visited_inpoly,path_vlp_full,path_candidates,path_visited,path_candidates2d,path_floor,path_vlp;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::PointStamped pnt_midpoint;
geometry_msgs::PoseStamped last_pose,base_pose;
float mapradtouse;
float grid_sidelength = 3;
geometry_msgs::PolygonStamped poly_vlp,poly_vlp_ext;
geometry_msgs::Point bbmin_vlp,bbmax_vlp;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
nav_msgs::Path spath1,spath2,spath3;

float y2r(float y, float rows,float res){
  return (rows / 2 - y / res);
}
float x2c(float x, float cols,float res){
  return (x / res + cols/2);
}
int r2y(float r, float rows,float res){
  return int((rows / 2 - r) * res);
}
int c2x(float c, float cols,float res){
  return int((c - cols / 2) * res);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
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
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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
int dst_point_in_path_index(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return i;
  }
  return 0;
}
bool dst_point_in_path_lim(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return true;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
bool dst_point_in_path_lim2d(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return true;
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
  float delta_inclination = get_shortest(-vlp_rpy.y,inclination);
  float pitch_shortest    = get_shortest(atan2(pnt.z-pos.z,get_dst2d(pnt,pos)),-vlp_rpy.y);
  //ROS_INFO("vlp_rpy: %.2f, inclination: %.2f, max_inclination: %.2f, pitch_shortest: %.2f,delta_inclination %.2f",vlp_rpy.y,inclination,maxdelta_pitch,pitch_shortest,delta_inclination);
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
nav_msgs::Path scatter_path(nav_msgs::Path pathin, float poses_spacing){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(pathout,pathin.poses[i].pose.position,poses_spacing)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
std::vector<nav_msgs::Path> scatter_clusters(std::vector<nav_msgs::Path> paths_in,float poses_spacing){
  for(int i= 0; i < paths_in.size(); i++){
    paths_in[i] = scatter_path(paths_in[i],poses_spacing);
  }
  return paths_in;
}

////////////////////////////***********************//////////////////////////////
                          //START  /*ORDERED_PATH*/ //START
////////////////////////////***********************//////////////////////////////
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}
int get_next_target(std::vector<int> vec_blacklist,nav_msgs::Path pathin,geometry_msgs::Point current_pos,float current_yaw){
  int best_i = -1;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_blacklist,i)){
      float dyaw = get_shortest(get_hdng(pathin.poses[i].pose.position,current_pos),current_yaw);
      if(dyaw < 0)
        dyaw *= -1;
      if(dyaw < M_PI/2){
        float dst = get_dst3d(current_pos,pathin.poses[i].pose.position);
        if(dst < lowest_dist){
          lowest_dist = dst;
          best_i = i;
        }
      }
    }
  }
  return best_i;
}

nav_msgs::Path create_ordered_path(nav_msgs::Path pathin){
  std::vector<int> vec_blacklist;
  geometry_msgs::Point current_pos;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() < 2){
    return pathout;
  }
  float current_yaw = vlp_rpy.z;
  current_pos = base_pose.pose.position;
  int best_i  = get_next_target(vec_blacklist,pathin,current_pos,current_yaw);
  while(best_i >= 0){
    current_yaw = get_hdng(pathin.poses[best_i].pose.position,current_pos);
    current_pos = pathin.poses[best_i].pose.position;
    vec_blacklist.push_back(best_i);
    best_i = get_next_target(vec_blacklist,pathin,current_pos,current_yaw);
  }
  for(int i = 0; i < vec_blacklist.size(); i++){
    pathout.poses.push_back(pathin.poses[vec_blacklist[i]]);
  }
  //ROS_INFO("poses_in: %i, poses out: %i",pathin.poses.size(),pathout.poses.size());
  return pathout;
}

std::vector<nav_msgs::Path> order_clusters(std::vector<nav_msgs::Path> paths_in){
  nav_msgs::Path path_temp;
  std::vector<nav_msgs::Path> paths_out;
  for(int i = 0; i < paths_in.size(); i++){
    path_temp = create_ordered_path(paths_in[i]);
    if(path_temp.poses.size() > 2){
      paths_out.push_back(path_temp);
    }
  }
  //ROS_INFO("Path_clusters_in: %i -> out: %i",paths_in.size(),paths_out.size());
  return paths_out;
}
////////////////////////////***********************//////////////////////////////
                          //END  /*ORDERED_PATH*/ //END
////////////////////////////***********************//////////////////////////////

////////////////////////////***********************//////////////////////////////
                          //START  /*CLUSTERS*/ //START
////////////////////////////***********************//////////////////////////////
std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radius, float dz_min,float dz_max){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  float yaw0 = tf::getYaw(pathin.poses[i_to_check].pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    float dyaw = get_shortest(tf::getYaw(pathin.poses[i].pose.orientation),yaw0);
    float dz   = pathin.poses[i].pose.position.z-pathin.poses[i_to_check].pose.position.z;
    if(dyaw < 0)
      dyaw *= -1;

    if(dist <= radius && dist > 0 && dyaw < M_PI/8 && dz < dz_max && dz > dz_min)
      vec_out.push_back(i);
  }
  return vec_out;
}

std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radius,float dz_min,float dz_max){
	std::vector<std::vector<int>> neighbours_at_index;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours_at_index.push_back(getinpath_indexes_inrad(pathin,i,radius,dz_min,dz_max));
  //  ROS_INFO("neighbours_at_index[%i]: %i",i,neighbours_at_index[i].size());
  }
  return neighbours_at_index;
}

std::vector<int> add_neighbours_index(std::vector<std::vector<int>> neighbours_at_indexes,std::vector<int> neighbours_in_cluster,std::vector<int> indexes_to_add){
  std::vector<int> new_neighbours;
  for(int k = 0; k < indexes_to_add.size(); k++){
    if(!in_vec(neighbours_in_cluster,indexes_to_add[k])
    && !in_vec(new_neighbours,       indexes_to_add[k]))
      new_neighbours.push_back(indexes_to_add[k]);
    for(int i = 0; i < neighbours_at_indexes[indexes_to_add[k]].size(); i++){
      if(!in_vec(neighbours_in_cluster,neighbours_at_indexes[indexes_to_add[k]][i])
      && !in_vec(new_neighbours,       neighbours_at_indexes[indexes_to_add[k]][i])){
        new_neighbours.push_back(neighbours_at_indexes[indexes_to_add[k]][i]);
      }
    }
  }
//  if(new_neighbours.size() > 2)
  //  ROS_INFO("new neighbours: count %i 0: %i N: %i",new_neighbours.size(),new_neighbours[0],new_neighbours[new_neighbours.size()-1]);
  return new_neighbours;
}

std::vector<int> get_neighbour_cluster(nav_msgs::Path pathin,float radius,int start_index,float dz_min,float dz_max){
  std::vector<std::vector<int>> neighbours_at_index;
  neighbours_at_index = getinpath_neighbours(pathin,radius,dz_min,dz_max);
  std::vector<int> neighbours_in_cluster;
  std::vector<int> indexes_to_add;
  indexes_to_add.push_back(start_index);
  while(indexes_to_add.size() > 0 && neighbours_in_cluster.size() < 30){
    for(int i = 0; i < indexes_to_add.size(); i++){
      neighbours_in_cluster.push_back(indexes_to_add[i]);
    }
    indexes_to_add = add_neighbours_index(neighbours_at_index,neighbours_in_cluster,indexes_to_add);
  }
	//ROS_INFO("Cluster size: %i",neighbours_in_cluster.size());
  return neighbours_in_cluster;
}

std::vector<int> update_neighbours_clustered(std::vector<int> neighbours_clustered,std::vector<int> neighbours_in_cluster){
  for(int i = 0; i < neighbours_in_cluster.size(); i++){
    neighbours_clustered.push_back(neighbours_in_cluster[i]);
  }
  return neighbours_clustered;
}
std::vector<int> get_neighbours_not_clustered(std::vector<int> neighbours_clustered,int path_size){
  std::vector<int> not_clustered;
  for(int i = 0; i < path_size; i++){
    if(!in_vec(neighbours_clustered,i))
      not_clustered.push_back(i);
  }
  return not_clustered;
}

std::vector<std::vector<int>> get_neighbour_clusters(nav_msgs::Path pathin,float radius,float dz_min,float dz_max){
  std::vector<int> neighbours_not_clustered;
  std::vector<int> neighbours_in_cluster;
  std::vector<int> neighbours_clustered;
  std::vector<std::vector<int>> neighbour_clusters;
  while(neighbours_clustered.size() < pathin.poses.size()){
    neighbours_not_clustered = get_neighbours_not_clustered(neighbours_clustered,pathin.poses.size());
    neighbours_in_cluster    = get_neighbour_cluster(pathin,radius,neighbours_not_clustered[0],dz_min,dz_max);
    neighbours_clustered     = update_neighbours_clustered(neighbours_clustered,neighbours_in_cluster);
    neighbour_clusters.push_back(neighbours_in_cluster);
  //  ROS_INFO("Neighbours cluster: %i / %i, neighbours_in_cluster: %i, neighbour_clusters: %i",neighbours_clustered.size(),neighbours_not_clustered.size(),neighbours_in_cluster.size(),neighbour_clusters.size());
  }

  return neighbour_clusters;
}
std::vector<nav_msgs::Path> paths_from_clusters(nav_msgs::Path pathin,std::vector<std::vector<int>> clusters_in){
//  ROS_INFO("Clusters_in: %i, pathposes_in: %i",clusters_in.size(),pathin.poses.size());
  std::vector<nav_msgs::Path> path_clusters;
  for(int i = 0; i < clusters_in.size(); i++){
    if(clusters_in[i].size() > 1){
      nav_msgs::Path path_cluster;
      path_cluster.header = pathin.header;
      for(int k = 0; k < clusters_in[i].size(); k++){
        path_cluster.poses.push_back(pathin.poses[clusters_in[i][k]]);
      }
  //    ROS_INFO("Cluster: %i path_size: %i",i,path_cluster.poses.size());
      path_clusters.push_back(path_cluster);
    }
  }
  return path_clusters;
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
  return true;
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
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
}



nav_msgs::Path get_path_down(){
  nav_msgs::Path pathout;
  geometry_msgs::PoseStamped ps;
  pathout.header = hdr(); ps.header = hdr();
  ps.pose.orientation.x = ps.pose.orientation.z = 0.0;
  ps.pose.orientation.y = ps.pose.orientation.w = 0.7071;
  update_edto_vlp(1.1,bbmin_vlp.z,bbmax_vlp.z);
  for(int z = int(round(bbmin_vlp.z)); z < int(round(bbmax_vlp.z)); z++){
    for(int y = ymin; y < ymax; y++){
      for(int x = xmin; x < xmax; x++){
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        if(is_within_incline(ps.pose.position,par_delta_incline) && in_poly(poly_vlp_ext,ps.pose.position.x,ps.pose.position.y) && dst_point_in_path_lim(path_visited_inpoly,ps.pose.position,8)){
          octomap::point3d p(x,y,z);
          octomap::point3d closestObst;
          float d;
          edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
          if(round(d) == 1 && d < 1.1 && closestObst.z() <= z)
            path_candidates.poses.push_back(ps);
        }
      }
    }
  }
  return pathout;
}
nav_msgs::Path get_path_side(){
  nav_msgs::Path pathout;
  geometry_msgs::PoseStamped ps;
  update_edto_vlp(10,bbmin_vlp.z,bbmax_vlp.z);
  for(int z = int(round(bbmin_vlp.z)); z < int(round(bbmax_vlp.z)); z++){
    for(int y = ymin; y < ymax; y++){
      for(int x = xmin; x < xmax; x++){
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        if(is_within_incline(ps.pose.position,par_delta_incline) && in_poly(poly_vlp,ps.pose.position.x,ps.pose.position.y) && dst_point_in_path_lim(path_visited_inpoly,ps.pose.position,8)){
          octomap::point3d p(x,y,z);
          octomap::point3d closestObst;
          float d;
          edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
          float dz = abs(closestObst.z() - z);
          if(round(d) == 6 && dz < 2){
            ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - p.y(),closestObst.x() - p.x()));
            pathout.poses.push_back(ps);
          }
        }
      }
    }
  }
  return pathout;
}
nav_msgs::Path create_superpath_clusters(std::vector<nav_msgs::Path> path_clusters){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  geometry_msgs::PoseStamped ps_holder;
	ps_holder = base_pose;
  ps_holder.header.frame_id = "next_path";
  for(int i = 0; i < path_clusters.size(); i++){
    pathout.poses.push_back(ps_holder);
    for(int k = 0; k < path_clusters[i].poses.size(); k++){
      pathout.poses.push_back(path_clusters[i].poses[k]);
    }
  }
  return pathout;
}
nav_msgs::Path get_superpath(nav_msgs::Path pathin,float r,float dz_min,float dz_max){
  return create_superpath_clusters(order_clusters(paths_from_clusters(pathin,get_neighbour_clusters(pathin,r,dz_min,dz_max))));
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
  base_pose.pose.position.x = transformStamped.transform.translation.x;
  base_pose.pose.position.y = transformStamped.transform.translation.y;
  base_pose.pose.position.z = transformStamped.transform.translation.z;
  base_pose.pose.orientation.x = transformStamped.transform.rotation.x;
  base_pose.pose.orientation.y = transformStamped.transform.rotation.y;
  base_pose.pose.orientation.z = transformStamped.transform.rotation.z;
  base_pose.pose.orientation.w = transformStamped.transform.rotation.w;
  base_pose.header.stamp       = transformStamped.header.stamp;
}
float get_polypath_vlp(float collision_radius,float maxdelta_hdng,int num_rays,float tot_length){
  ros::Time t0 = ros::Time::now();
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();
  path_vlp.header = hdr();
  path_vlp.poses.resize(0);
  checktfvlp();
  float rads_pr_i = maxdelta_hdng*2 / num_rays;
  bbmax_vlp.x = -1000; bbmax_vlp.y = -1000; bbmax_vlp.z = -1000;
  bbmin_vlp.x = 1000; bbmin_vlp.y = 1000; bbmin_vlp.z = 1000;
  std::vector<geometry_msgs::Point> points;

  for(int k = 0; k < 1; k++){
    for(int i = 1; i < num_rays; i++){
      geometry_msgs::PointStamped p,pout;
      p.header.frame_id = "velodyne_aligned";
      int kk;
      if(k == 0)
        kk = 1;
      if(k == 1)
        kk = 0;
      if(k == 2)
        kk = 2;

      float a_k = -par_delta_incline + kk * par_delta_incline;
      float a_i = -maxdelta_hdng + rads_pr_i * i;
  //    a_k = vlp_rpy.y;
//      if(vlp_rpy.y < 0)
      if(a_k < -par_delta_incline)
        a_k = - par_delta_incline;
      if(a_k > par_delta_incline)
        a_k = par_delta_incline;
      p.point.x = tot_length*cos(a_k)*cos(a_i);
      p.point.y = tot_length*cos(a_k)*sin(a_i);

      p.point.z = p.point.x*sin(a_k);
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
  poly_vlp_ext.polygon.points.resize(points.size());
  enlarge_bbvlp(collision_radius,tot_length * sin(par_delta_incline));
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
  ps.pose    = base_pose.pose;
  path_vlp_full = path_vlp;

  path_vlp.poses.push_back(ps);

  for(int i = 0; i < points.size(); i++){
    Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt2_vec(points[i].x,points[i].y,points[i].z);
    Eigen::Vector3f stride_vec,cur_vec,cur_vec_poly;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    float distance = collision_radius-1;
    float cur_ray_len = 0;
    cur_vec = pnt1_vec;

    while(distance > 2 && cur_ray_len < tot_length){

      cur_vec          = cur_vec + stride_vec;
      cur_vec_poly     = cur_vec + stride_vec*3;
      cur_ray_len = (cur_vec - pnt1_vec).norm();

      if(cur_vec.x() > float(xmin) && cur_vec.y() > float(ymin) && cur_vec.z() > float(zmin)
      && cur_vec.x() < float(xmax) && cur_vec.y() < float(ymax) && cur_vec.z() < float(zmax)){
        ps.pose.position.x  = cur_vec.x();
        ps.pose.position.y  = cur_vec.y();
        ps.pose.position.z  = cur_vec.z();

        octomap::point3d closestObst;
        point3d p(cur_vec.x(),cur_vec.y(),cur_vec.z());
        edf_ptr.get()->getDistanceAndClosestObstacle(p,distance,closestObst);
        if(dst_point_in_path_lim(path_vlp,ps.pose.position,grid_sidelength)){
          float dz = closestObst.z()-p.z();
          if(abs(dz) > 2){
            ps.pose.orientation.x = 0;
            ps.pose.orientation.z = 0;
            ps.pose.orientation.y = 0.7071;
            ps.pose.orientation.w = 0.7071;
          }
          else{
            ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y()-ps.pose.position.y,closestObst.x() - ps.pose.position.x));
          }
          path_vlp.poses.push_back(ps);
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
          poly_vlp_ext.polygon.points[i].x = cur_vec_poly.x();
          poly_vlp_ext.polygon.points[i].y = cur_vec_poly.y();
          poly_vlp_ext.polygon.points[i].z = cur_vec_poly.z();
          poly_vlp.polygon.points[i].x = cur_vec.x();
          poly_vlp.polygon.points[i].y = cur_vec.y();
          poly_vlp.polygon.points[i].z = cur_vec.z();
        }
        else
          break;
    }
		if(cur_ray_len < tot_length){
			cur_vec          = cur_vec + stride_vec*distance;
			ps.pose.position.x  = cur_vec.x();
			ps.pose.position.y  = cur_vec.y();
			ps.pose.position.z  = cur_vec.z();
			path_vlp_full.poses.push_back(ps);
		}
  }

  enlarge_bbvlp(collision_radius,2);

  poly_vlp.polygon.points[0].x  = base_pose.pose.position.x;
  poly_vlp.polygon.points[0].y  = base_pose.pose.position.y;
  poly_vlp.polygon.points[0].z  = base_pose.pose.position.z;
  poly_vlp_ext.polygon.points[poly_vlp_ext.polygon.points.size()-1] = poly_vlp_ext.polygon.points[0] = poly_vlp.polygon.points[poly_vlp.polygon.points.size()-1] = poly_vlp.polygon.points[0];
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
}

void getpaths_cb(const std_msgs::UInt8::ConstPtr& msg){
  get_floor = true;
}

std::vector<int> test_neigbour_config(nav_msgs::Path pathin,float radius,float dz_min,float dz_max){
	std::vector<std::vector<int>> clusters = get_neighbour_clusters(pathin,radius,dz_min,dz_max);
	std::vector<int> vec_out;
	int total_count = 0;
	int max_count = 0;
	for(int i = 0; i < clusters.size(); i++){
		int csize = clusters[i].size();
		if(csize > 2){
			total_count += csize;
			if(csize > max_count)
				max_count = csize;
		}
	}
	int percentage_clustered = total_count / pathin.poses.size() * 100;
	int ave_cluster = total_count / clusters.size();
	vec_out.push_back(total_count);
	vec_out.push_back(ave_cluster);
	vec_out.push_back(clusters.size());
	vec_out.push_back(percentage_clustered);
	vec_out.push_back(max_count);
  return vec_out;
}
void test_neigbouring_configs(nav_msgs::Path pathin){
  std::vector<int> res1,res2,res3;
  res1 = test_neigbour_config(pathin,4,-2,2);
  res2 = test_neigbour_config(pathin,4,0,1);
  res3 = test_neigbour_config(pathin,2,-1,-1);

  ROS_INFO("c1(4,-2, 2)-pnts: %i. clusters: %i ave-max: %i-%i ",res1[0],res1[2],res1[1],res1[4]);
  ROS_INFO("c2(4, 0, 1)-pnts: %i. clusters: %i ave-max: %i-%i ",res2[0],res2[2],res2[1],res2[4]);
  ROS_INFO("c3(2,-1,-1)-pnts: %i. clusters: %i ave-max: %i-%i ",res3[0],res3[2],res3[1],res3[4]);

  spath1 = create_superpath_clusters(order_clusters(paths_from_clusters(pathin,get_neighbour_clusters(pathin,4,-2,2))));
  spath2 = create_superpath_clusters(order_clusters(paths_from_clusters(pathin,get_neighbour_clusters(pathin,4,0,1))));
  spath3 = create_superpath_clusters(order_clusters(paths_from_clusters(pathin,get_neighbour_clusters(pathin,2,-1,-1))));
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_pathclusters_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("workdir_path",par_workdir_path);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);
  private_nh.param("par_maxobs",  par_maxobs, 20.0);//*2.0);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("par_minobs",  par_minobs, 2.0);//*2.0);
  private_nh.param("delta_hdng",   par_delta_hdng, M_PI/2);//*2.0);
  private_nh.param("delta_incline",  par_delta_incline, M_PI/24);//*2.0);

  private_nh.param("side_radius", par_s_r, 4.0);//*2.0);
  private_nh.param("down_radius", par_d_r, 2.0);//*2.0);
  private_nh.param("par_d_zlo",   par_d_zlo, -1.1);//*2.0);
  private_nh.param("par_d_zhi",   par_d_zhi, 1.1);//*2.0);
  private_nh.param("par_s_zlo",   par_s_zlo, -1.1);//*2.0);
  private_nh.param("par_s_zhi",   par_s_zhi, 1.1);//*2.0);


  path_vlp.header      = hdr();
  path_floor.header         = hdr();
  poly_vlp.header           = hdr();
  superpath_down.header     = hdr();
  superpath_side.header    = hdr();
  path_visited.header       = hdr();
  last_pose.header          = hdr();
  pnt_midpoint.header       = hdr();
  base_pose.header          = hdr();
  last_pose.pose.orientation.w = 1;
  path_visited.poses.push_back(last_pose);
  tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Publisher pub_candidates = nh.advertise<nav_msgs::Path>("/tb_path/down",100);
  ros::Publisher pub_candidates_side = nh.advertise<nav_msgs::Path>("/tb_path/side",100);
  ros::Publisher pub_visited   	   = nh.advertise<nav_msgs::Path>("/tb_path/visited",10);
  ros::Publisher pub_midpoint      = nh.advertise<geometry_msgs::PointStamped>("/tb_path/midpoint",100);
  ros::Publisher pub_base_pose     = nh.advertise<geometry_msgs::PoseStamped>("/tb_path/base_pose",100);
  ros::Publisher pub_poly 			   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_path/cleared_poly",100);
  ros::Publisher pub_path_vlp      = nh.advertise<nav_msgs::Path>("/tb_path/cleared",100);
  ros::Publisher pub_path_vlp_full = nh.advertise<nav_msgs::Path>("/tb_path/obstacles",100);
  ros::Publisher pub_superpath_down  = nh.advertise<nav_msgs::Path>("/tb_path/superpath_down",100);
  ros::Publisher pub_superpath_side = nh.advertise<nav_msgs::Path>("/tb_path/superpath_side",100);
  ros::Publisher pub_path_side = nh.advertise<nav_msgs::Path>("/tb_clusters/side",10);
  ros::Publisher pub_path_down = nh.advertise<nav_msgs::Path>("/tb_clusters/down",10);
  ros::Publisher pub_superpath1 = nh.advertise<nav_msgs::Path>("/tb_path_super1",10);
  ros::Publisher pub_superpath2 = nh.advertise<nav_msgs::Path>("/tb_path_super2",10);
  ros::Publisher pub_superpath3 = nh.advertise<nav_msgs::Path>("/tb_path_super3",10);
  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Subscriber s2 = nh.subscribe("/tb_path/request",1,getpaths_cb);
  ros::Rate rate(5.0);
  ros::Time t0           = ros::Time::now();
  float collision_radius = 5;
  int num_rays     = 32;
  float ray_length = 30;
int count = 0;
  bool no_superpaths;
  while(ros::ok()){
    checktf();
    pub_visited.publish(path_visited);
    if(got_map){
      float dt_polypath = get_polypath_vlp(par_maxobs,par_delta_hdng,num_rays,ray_length);
			pub_path_vlp.publish(path_vlp);
			pub_path_vlp_full.publish(path_vlp_full);
      pub_midpoint.publish(pnt_midpoint);
      pub_poly.publish(poly_vlp);

      if(path_raw_side.poses.size() > 0 && path_raw_down.poses.size() > 0){
        if(path_raw_side.poses.size() > 50  || (path_raw_side.poses.size() > path_raw_down.poses.size()))
          test_neigbouring_configs(path_raw_side);
        else
          test_neigbouring_configs(path_raw_down);
      }
      else if(path_raw_down.poses.size() > 0)
        test_neigbouring_configs(path_raw_down);
      else if(path_raw_side.poses.size() > 0)
        test_neigbouring_configs(path_raw_side);

      pub_superpath1.publish(spath1);
      pub_superpath2.publish(spath2);
      pub_superpath3.publish(spath3);

      if(get_floor){
        path_visited_inpoly = constrain_path_bbpoly(path_visited,poly_vlp);

        path_raw_down = get_path_side();
        path_raw_side = get_path_down();

        superpath_down  = get_superpath(path_raw_down,par_d_r,par_d_zlo,par_d_zhi);
        superpath_side = get_superpath(path_raw_side,par_s_r,par_s_zlo,par_s_zhi);

        pub_superpath_down.publish(superpath_down);
        pub_superpath_side.publish(superpath_side);

        pub_candidates.publish(path_raw_down);
        pub_candidates_side.publish(path_raw_side);
        pub_base_pose.publish(base_pose);
        get_floor = false;
      }
    }
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
