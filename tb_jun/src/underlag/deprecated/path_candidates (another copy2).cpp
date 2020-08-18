//path_service:
// example showing how to receive a nav_msgs/Path request
// run with complementary path_client
// this could be useful for
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
int zlvl;
bool par_unknownAsOccupied;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump;
nav_msgs::Path path_candidates,path_floor;
std::vector<int> z_lvls;
bool got_map,get_floor;
geometry_msgs::PointStamped midpoint;
geometry_msgs::PointStamped closest_obstacle;
geometry_msgs::PoseStamped last_pose;
geometry_msgs::PolygonStamped poly_heading,poly_safe,poly_vstd;
nav_msgs::Path path_visited,path;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float dst_visited_area(float x,float y, float z){
  float res,dst;
  geometry_msgs::Point pin;
  pin.x = x; pin.y = y; pin.z = z;
  res = 1000;
  if(path_visited.poses.size() == 0)
    return res;
  for(int i = 0; i < path_visited.poses.size(); i++){
    if(abs(path_visited.poses[i].pose.position.z - pin.z) < 2){
      dst = get_dst3d(path_visited.poses[i].pose.position,pin);
      if(dst < res)
        res = dst;
    }
  }
  return res;
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
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

void update_path(geometry_msgs::Point midpoint)
{
  geometry_msgs::PoseStamped ps;
  last_pos = pos;
  last_yaw = pos_yaw;
  ps.header.frame_id ="map";
  path_candidates.poses.resize(0);
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  int min_zlvl = fmax(bbmin_octree.z/par_zjump + 3,zlvl-5);
  int max_zlvl = fmin(bbmax_octree.z/par_zjump + 3,zlvl+5);
  if(min_zlvl == max_zlvl)
    return;
  for(int zn = min_zlvl; zn < max_zlvl; zn++){
    int z = z_lvls[zn];
    update_edto(midpoint,par_maxobs+1,par_maprad,z-0.5,z+0.5,par_unknownAsOccupied);
      for(int y = ymin; y < ymax; y++){
        for(int x = xmin; x < xmax; x++){
          ps.pose.position.x = x;
          ps.pose.position.y = y;
          ps.pose.position.z = z;
          if(dst_point_in_path_lim(path_candidates,ps.pose.position,5)){
            octomap::point3d p(x,y,z);
            octomap::point3d closestObst;
            float d;
            edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
            if(round((par_minobs + par_maxobs)/2) == round(d) && in_poly(poly_safe,ps.pose.position.x,ps.pose.position.y)){
              ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - y,closestObst.x() - x));
              path_candidates.poses.push_back(ps);
            }
          }
        }
      }
    }
  return;
}

void update_path_floor(geometry_msgs::Point midpoint,int n_grids, float grid_sidelength)
{
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id ="map";
  path_floor.poses.resize(0);

  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  int x0 = int(round(midpoint.x)) - grid_sidelength * n_grids / 2 + grid_sidelength / 2;
  int y0 = int(round(midpoint.y)) - grid_sidelength * n_grids / 2 + grid_sidelength / 2;

  ROS_INFO("PathFloor: x0: %i y0: %i grid_Sides/Number %.2f/%i",x0,y0,grid_sidelength,n_grids);

  for(int yn = 0; yn < n_grids; yn++){
    for(int xn = 0; xn < n_grids; xn++){
      geometry_msgs::Point pmid;
      pmid.x = x0 + xn * grid_sidelength;
      pmid.y = y0 + yn * grid_sidelength;
      pmid.z = midpoint.z + par_zjump;
        //ROS_INFO("PathFloor: xn: %i yn: %i x: %.2f y: %.2f",xn,yn,pmid.x,pmid.y);
        float collrad = pmid.z-bbmin_octree.z;
        update_edto(pmid,collrad,grid_sidelength,bbmin_octree.z,pmid.z,par_unknownAsOccupied);
        float d;
        octomap::point3d p(pmid.x,pmid.y,pmid.z);
        octomap::point3d closestObst;
        edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
        if(round(d) < round(collrad) && in_poly(poly_safe,pmid.x,pmid.y)){
          ps.pose.position.x    = closestObst.x();
          ps.pose.position.y    = closestObst.y();
          ps.pose.position.z    = closestObst.z() + 5;
          ps.pose.orientation.y = 0.7071;
          ps.pose.orientation.w = 0.7071;
          path_floor.poses.push_back(ps);
      }
    }
  }
  return;
}
/*
update_edto(pos,collision_radius,maprad,pos.z-10,pos.z+50,false,false);

void get_zdown(float collision_radius){
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
			int z = zmax;
			float dst = collision_radius;
				octomap::point3d p(x,y,z);
				dst = edf_ptr.get()->getDistance(p);
				zn--;
			}
			int r = y2r(y,mapimg.rows,1);
			int c = x2c(x,mapimg.cols,1);
			mapimg.at<cv::Vec3b>(r,c)[2] = z;
  	}
	}
	ROS_INFO("get_zdown complete");
}*/
geometry_msgs::PointStamped get_point_in_front(float dist_from_base){
  geometry_msgs::PointStamped pnt;
	try
	{
		pnt.header.frame_id		 = "base_stabilized";
		pnt.point.x 			 	   = dist_from_base;
		pnt                    = tfBuffer.transform(pnt, "map");
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("Failure %s\n", ex.what());
	}
  ROS_INFO("Pos: [x: %.2f y: %.2f z: %.2f] ->front-> [x: %.2f y: %.2f z: %.2f]",pos.x,pos.y,pos.z,pnt.point.x,pnt.point.y,pnt.point.z);
  return pnt;
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
  zlvl  = round(pos.z / par_zjump) + 3;
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
  midpoint = get_point_in_front(15);
}
geometry_msgs::PolygonStamped get_poly_surround(float collision_radius,float mapradused,int num_rays){
  geometry_msgs::PolygonStamped polygon;
  polygon.header = hdr();
  update_edto(last_pose.pose.position,collision_radius,mapradused,last_pose.pose.position.z+1,last_pose.pose.position.z-1,false);
  octomap::point3d p(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
  octomap::point3d closestObst;
  float d;
  edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
  if(d < collision_radius && d > 0){
    closest_obstacle.point.x      = closestObst.x();
    closest_obstacle.point.y      = closestObst.y();
    closest_obstacle.point.z      = closestObst.z();
    closest_obstacle.header.stamp = ros::Time::now();
  }
  float rads_pr_i = 2*M_PI / num_rays;
  polygon.polygon.points.resize(num_rays);
  for(int i  = 0; i < num_rays; i++){
    Eigen::Vector3f pnt1_vec(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
    Eigen::Vector3f pnt2_vec(last_pose.pose.position.x+mapradused*cos(i*rads_pr_i),mapradused*sin(i*rads_pr_i)+last_pose.pose.position.y,last_pose.pose.position.z);
    Eigen::Vector3f cur_vec = pnt1_vec;
    float tot_length = (pnt2_vec - pnt1_vec).norm();
    Eigen::Vector3f stride_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    bool visited_clear = true;
    float cur_ray_len=0;
    float distance = collision_radius-1;
    float next_check = collision_radius;
    while(distance > 2 && cur_ray_len < tot_length){
      cur_vec = cur_vec + stride_vec;
      cur_ray_len = (cur_vec-pnt1_vec).norm();
      point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
      distance = edf_ptr.get()->getDistance(stridep);
    }
    polygon.polygon.points[i].x = cur_vec.x();
    polygon.polygon.points[i].y = cur_vec.y();
    polygon.polygon.points[i].z = cur_vec.z();
  }
  return polygon;
}
geometry_msgs::PolygonStamped get_poly_vstd(float collision_radius,float mapradused,int num_rays){
  geometry_msgs::PolygonStamped polygon;
  polygon.header = hdr();
  float rads_pr_i = 2*M_PI / num_rays;
  polygon.polygon.points.resize(num_rays);
  for(int i  = 0; i < num_rays; i++){
    Eigen::Vector3f pnt1_vec(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
    Eigen::Vector3f pnt2_vec(last_pose.pose.position.x+mapradused*cos(i*rads_pr_i),mapradused*sin(i*rads_pr_i)+last_pose.pose.position.y,last_pose.pose.position.z);
    Eigen::Vector3f cur_vec = pnt1_vec;
    float tot_length = (pnt2_vec - pnt1_vec).norm();
    Eigen::Vector3f stride_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    bool visited_clear = true;
    float cur_ray_len=0;
    float distance = 5;
    float next_check = collision_radius;
    while(distance > 2 && cur_ray_len < tot_length){
      cur_vec = cur_vec + stride_vec;
      cur_ray_len = (cur_vec-pnt1_vec).norm();
      point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
      if(cur_ray_len >= next_check){
       next_check = cur_ray_len + dst_visited_area(cur_vec.x(),cur_vec.y(),cur_vec.z());
       if(dst_visited_area(cur_vec.x(),cur_vec.y(),cur_vec.z()) < 2)
         break;
     }
    }
    polygon.polygon.points[i].x = cur_vec.x();
    polygon.polygon.points[i].y = cur_vec.y();
    polygon.polygon.points[i].z = cur_vec.z();
  }
  return polygon;
}
geometry_msgs::PolygonStamped get_poly_heading(float hdng_delta,int num_points,std::string frame){
  geometry_msgs::PolygonStamped poly;
  poly.header = hdr();
  poly.polygon.points.resize(num_points+1);
  poly.polygon.points[0].x = last_pose.pose.position.x;
  poly.polygon.points[0].y = last_pose.pose.position.y;
  poly.polygon.points[0].z = last_pose.pose.position.z;
  float rads_pr_i = hdng_delta /num_points;
  geometry_msgs::PointStamped p,pout;
  p.header.frame_id = frame;
  p.header.stamp = ros::Time();
  for(int i = 1; i < num_points; i++){
    float a = -hdng_delta/2 + rads_pr_i * i;
    p.point.x = 50*cos(a);
    p.point.y = 50*sin(a);
    p.point.z = last_pose.pose.position.z;
    pout = transformpoint(p,"map");
    poly.polygon.points[i].x = pout.point.x;
    poly.polygon.points[i].y = pout.point.y;
    poly.polygon.points[i].z = last_pose.pose.position.z;
  }
  poly.polygon.points[num_points].x = last_pose.pose.position.x;
  poly.polygon.points[num_points].y = last_pose.pose.position.y;
  poly.polygon.points[num_points].z = last_pose.pose.position.z;
  return poly;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_pathcand_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("workdir_path",par_workdir_path);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);
  private_nh.param("par_maxobs",  par_maxobs, 20.0);//*2.0);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("par_minobs",  par_minobs, 2.0);//*2.0);

  path_candidates.header.frame_id ="map";
  path_floor.header.frame_id="map";
  last_pose.header        = hdr();
  path_visited.header     = hdr();
  closest_obstacle.header = hdr();
  last_pose.pose.orientation.w = 1;
  path_visited.poses.push_back(last_pose);

  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
  }

  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Publisher pub_orig             = nh.advertise<nav_msgs::Path>("/tb_path_filtered",100);
  ros::Publisher pub_midpoint         = nh.advertise<geometry_msgs::PointStamped>("/tb_path_midpoint",100);
  ros::Publisher pub_floor            = nh.advertise<nav_msgs::Path>("/tb_path_floor",100);
  ros::Publisher pub_path_visited   	= nh.advertise<nav_msgs::Path>("/tb_nav/path_visited",10);

	ros::Publisher polysafe_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_safe",100);
  ros::Publisher polyhdng_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_hdng",100);
  ros::Publisher polyvstd_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_vstd",100);
  ros::Publisher pub_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obs/closest",100);

  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ROS_INFO("Ready to convert octomaps.");
  ros::Rate rate(1.0);
  while(ros::ok()){
    checktf();
    if(got_map){
      if(get_floor){
        int n_grids            = 10;
        float grid_sidelength = par_maprad*2/n_grids;
        ROS_INFO("Par_maprad = grids*sidelength/2: %.2f = %.2f,grids_sidelength: %.2f, n_grids: %i, ",par_maprad,grid_sidelength*n_grids,grid_sidelength,n_grids);
        update_path(midpoint.point);
        update_path_floor(midpoint.point,n_grids,grid_sidelength);
        get_floor = false;
      }

      poly_heading     = get_poly_heading(M_PI/2,16,"base_stabilized");
      poly_safe        = get_poly_surround(10,50,36);
      poly_vstd        = get_poly_vstd(10,50,36);
      polyhdng_pub.publish(poly_heading);
      polysafe_pub.publish(poly_safe);
      polyvstd_pub.publish(poly_vstd);

      pub_closest_obstacle.publish(closest_obstacle);
      pub_path_visited.publish(path_visited);
      pub_midpoint.publish(midpoint);
      pub_floor.publish(path_floor);
      pub_orig.publish(path_candidates);
    }
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}