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
ros::Publisher pub;
string par_workdir_path;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos,last_pos;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
float pos_yaw;
int zlvl;
bool par_unknownAsOccupied;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump;
nav_msgs::Path path_candidates;
std::vector<int> z_lvls;
bool got_map;
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

void update_path()
{
  geometry_msgs::PoseStamped ps;
  last_pos = pos;
  last_yaw = pos_yaw;
  ps.header.frame_id ="map";
  nav_msgs::Path path_candidates_to_send;
  path_candidates_to_send.header.frame_id="map";
  path_candidates.header.frame_id ="map";
  //path_candidates.poses.resize(0);
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  int min_zlvl = fmax(bbmin_octree.z/par_zjump + 3,zlvl-5);
  int max_zlvl = fmin(bbmax_octree.z/par_zjump + 3,zlvl+5);
  if(min_zlvl == max_zlvl)
    return;
  for(int zn = min_zlvl; zn < max_zlvl; zn++){
    int z = z_lvls[zn];
    update_edto(pos,par_maxobs+1,par_maprad,z-0.5,z+0.5,par_unknownAsOccupied);
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
            if(round((par_minobs + par_maxobs)/2) == round(d)){
              ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - y,closestObst.x() - x));
              path_candidates.poses.push_back(ps);
              path_candidates_to_send.poses.push_back(ps);
            }
          }
        }
      }
    }
  if(path_candidates_to_send.poses.size() > 0)
    pub.publish(path_candidates_to_send);
  return;
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
  if(abs(last_yaw - pos_yaw) > 1 || get_dst3d(pos,last_pos) > 5){
    update_path();
  }
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

  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
  }

  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Publisher pub_orig = nh.advertise<nav_msgs::Path>("/tb_path_filtered",100);
  pub = nh.advertise<nav_msgs::Path>("/tb_path_updates",100);
  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ROS_INFO("Ready to convert octomaps.");
  ros::Rate rate(1.0);
  while(ros::ok()){
    checktf();
    pub_orig.publish(path_candidates);
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
