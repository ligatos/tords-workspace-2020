#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Joy.h>
#include <queue>
#include <vector>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_datatypes.h>
#include "message_filters/subscriber.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <chrono>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tb_msgsrv/Paths.h>

tf2_ros::Buffer tfBuffer;
int pathfull_i = 0;
std::vector<int> pathfull_ints_sent;
std::string par_workdir = "/home/nuc/brain/pathfull/heights0.png";
sensor_msgs::LaserScan scan_lo,scan_mi,scan_hi,scan_up,scan_dn,scan_st;
geometry_msgs::Vector3 vlp_rpy,rpy_vel;
geometry_msgs::Point pos,pnt_ref;
geometry_msgs::PoseStamped last_pose;
std::vector<float> vec_pathfull;
geometry_msgs::PointStamped forward_point,scanpoint_mid,scanpoint_ave,frontier_centroid,frontier_minpnt,frontier_minpnt_infront,frontier_closest_infront,frontier_outofsight_infront;
nav_msgs::Odometry odom;
double par_hdngcutoff;
int global_i = 0;
int counter = 0;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_update(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_update_copy(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
float deg2rad = M_PI/180;
///////////********CTRL***********//////////
float vz,vxy,vxyz,vaz;
int mainstate = 0;
geometry_msgs::PoseStamped target_frontier,target,target_last;
int lead_num = 1;
int count_target_paths = 0;
int current_frontier_target = 2;
float current_frontier_radians = 0;
bool target_active = false;
bool got_map = false;
///////////********CTRL***********//////////
std::vector<geometry_msgs::PolygonStamped> gridpolys;

nav_msgs::Path pathfull,pathfull_ordered,path_lo,path_mi,path_hi,path_up,path_dn,path_st,path_full;
geometry_msgs::PolygonStamped poly_frontier;
nav_msgs::Path path_elevation_edto,path_targets,path_global_base,path_grid,path_visited,path_heightpath,path_frontier,path_scanzone,path_surround,path_forward,path_right,path_left;
tb_msgsrv::Paths heightpaths;
sensor_msgs::LaserScan scan_frontier;
ros::Publisher pub_cmdmb,pub_cmd,pub_cmdpose,pub_path_requested,pub_path_elevated,pub_viewpoint,pub_path_elevation_update,pub_path_elevation_update_edto;
ros::Time process_start;
std::string active_process;
using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
geometry_msgs::PointStamped closest_obstacle;
float closest_obstacle_dst = 10;
float min_dst = 5.0;
float max_dst = 10.0;

float par_res = 1.0;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
void start_process(std::string name){
  float dt = (ros::Time::now() - process_start).toSec();
  if(active_process != "")
    ROS_INFO("Process: %s took %.4f sec",active_process.c_str(),dt);
  active_process = name;
  process_start  = ros::Time::now();
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
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
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}


std::vector<float> vec_to_min_max_ave(std::vector<float> vec_in){
  std::vector<float> min_max_ave;
  min_max_ave.push_back(143131);
  min_max_ave.push_back(-143131);
  min_max_ave.push_back(0);
  float vec_sum = 0;
  for(int i = 0; i < vec_in.size(); i++){
    vec_sum += vec_in[i];
    if(vec_in[i] > min_max_ave[1])
     min_max_ave[1] = vec_in[i];
    if(vec_in[i] < min_max_ave[0])
     min_max_ave[0] = vec_in[i];
  }
  min_max_ave[2] = vec_sum / vec_in.size();
  return min_max_ave;
}
float y2r(float y){
  return (img_height.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img_height.cols/2);
}
int r2y(float r){
  return int((img_height.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - img_height.cols / 2) * par_res);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
std::vector<float> get_vec_attribute(nav_msgs::Path pathin,std::string type){
  std::vector<float> vec_out;
  geometry_msgs::Point p0;
  for(int i = 0; i < pathin.poses.size(); i++){
    float val = 0;
    if(type == "dst_2d0")
      val = get_dst2d(p0,pathin.poses[i].pose.position);
    else if(type == "inclination")
      val = get_inclination(pathin.poses[i].pose.position,pos);
    else if(type == "hdng_abs")
   		val = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z) * rad2deg);
    else if(type == "dst_2d")
      val = get_dst2d(pos,pathin.poses[i].pose.position);
    else if(type == "dst_3d")
      val = get_dst3d(pos,pathin.poses[i].pose.position);
    else if(type == "z")
      val = pathin.poses[i].pose.position.z;
    else if(type == "zrel")
      val = pathin.poses[i].pose.position.z - pos.z;
       vec_out.push_back(val);
  }
  return vec_out;
}
void draw_path_by_score(nav_msgs::Path pathin,std::vector<float> score,int primary_color,int secondary_color,int tertiary_color,float color_intensity){
  std::vector<float> mma = vec_to_min_max_ave(score);
  if(pathin.poses.size() < 1)
    return;
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    cv::Scalar color;
    float rel_score = color_intensity * (score[i] - mma[0]) / (mma[1]-mma[0]);
    if(rel_score > 0.9)
      color[tertiary_color] = 255*rel_score;
    if(rel_score > 0.75)
      color[secondary_color] = 255*rel_score;
    if(rel_score > 0.3)
      color[primary_color] = 255*rel_score;
    if(rel_score < 0.3)
      color[primary_color] = 100*rel_score;
    if(pathin.poses.size() > 25){
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
    }
    else
      cv::circle(img,pnt2cv(pnt),2,color,1);
	}
}
void draw_path(nav_msgs::Path pathin,cv::Scalar color, int size){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    if(size == 0){
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
    }
    else
      cv::circle(img,pnt2cv(pnt),size,color,1);
	}
}
bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
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
  return bool(cross % 2);
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
}


void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  for(int i = 1; i < polyin.polygon.points.size(); i++){
    geometry_msgs::Point p1,p2;
    p1.x = polyin.polygon.points[i-1].x;
    p1.y = polyin.polygon.points[i-1].y;
    p2.x = polyin.polygon.points[i].x;
    p2.y = polyin.polygon.points[i].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
  if(polyin.polygon.points.size() > 2){
    geometry_msgs::Point p1,p2;
    p1.x = polyin.polygon.points[polyin.polygon.points.size()-1].x;
    p1.y = polyin.polygon.points[polyin.polygon.points.size()-1].y;
    p2.x = polyin.polygon.points[0].x;
    p2.y = polyin.polygon.points[0].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
}





////////////////////////////**********************************/////////////////////////////
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
std::vector<geometry_msgs::Point> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<geometry_msgs::Point> bbmnbbmx;

  geometry_msgs::Point bbtotmax,bbtotmin;
  bbtotmax.x = bbtotmax.y = bbtotmax.z = -100;
  bbtotmin.x = bbtotmin.y = bbtotmin.z = 100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x > bbtotmax.x)bbtotmax.x = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y > bbtotmax.y)bbtotmax.y = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z > bbtotmax.z)bbtotmax.z = pathin.poses[i].pose.position.z;
    if(pathin.poses[i].pose.position.x < bbtotmin.x)bbtotmin.x = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y < bbtotmin.y)bbtotmin.y = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z < bbtotmin.z)bbtotmin.z = pathin.poses[i].pose.position.z;
  }
  bbmnbbmx.push_back(bbtotmin);
  bbmnbbmx.push_back(bbtotmax);

  float diag = sqrt(pow(bbtotmax.x-bbtotmin.x,2)+pow(bbtotmax.y-bbtotmin.y,2)+pow(bbtotmax.z-bbtotmin.z,2));
  ROS_INFO("GENERICNODE: LIMITS: diagonal: %.2f,max(%.2f %.2f %.2f) min(%.2f %.2f %.2f)",diag,bbtotmax.x,bbtotmax.y,bbtotmax.z,bbtotmin.x,bbtotmin.y,bbtotmin.z);
  return bbmnbbmx;
}

nav_msgs::Path get_path_within_m(nav_msgs::Path pathin,float meters){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(path_visited.poses.size() > 0)
    pathout.poses.push_back(path_visited.poses[path_visited.poses.size()-1]);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(meters > get_dst2d(pathin.poses[i].pose.position,pos))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
nav_msgs::Path update_edto(nav_msgs::Path pathin,float collision_radius){
  geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,bbmin,bbmax;
  pathin = scatter_path(get_path_within_m(pathin,20),2);
  std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(pathin);
  bbmin_custom = bbmnmx[0];
  bbmax_custom = bbmnmx[1];
  bbmin_custom.z -= 5.0;
  bbmax_custom.z += 5.0;
  bbmin_custom.x -= 5.0;
  bbmax_custom.x += 5.0;
  bbmin_custom.y -= 5.0;
  bbmax_custom.y += 5.0;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  if(bbmin_custom.z >= bbmax_octree.z)
    return pathin;
	bbmin.x = fmax(bbmin_custom.x,bbmin_octree.x);
	bbmin.y = fmax(bbmin_custom.y,bbmin_octree.y);
	bbmin.z = fmax(bbmin_custom.z,bbmin_octree.z);

	bbmax.x = fmin(bbmax_custom.x,bbmax_octree.x);
	bbmax.y = fmin(bbmax_custom.y,bbmax_octree.y);
	bbmax.z = fmin(bbmax_custom.z,bbmax_octree.z);
  octomap::point3d boundary_min(bbmin.x,bbmin.y,bbmin.z);
  octomap::point3d boundary_max(bbmax.x,bbmax.y,bbmax.z);

  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  closest_obstacle.header = hdr();
  closest_obstacle.point.x = 0;
  closest_obstacle.point.y = 0;
  closest_obstacle.point.z = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    if(pnt.x < bbmax.x && pnt.x > bbmin.x
    && pnt.y < bbmax.y && pnt.y > bbmin.y){
      point3d closestObst;
      point3d p(pnt.x,pnt.y,pnt.z);
      float dst = collision_radius;
      edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);

      if(i == 0){
        if(dst < collision_radius){
          closest_obstacle.point.x = closestObst.x();
          closest_obstacle.point.y = closestObst.y();
          closest_obstacle.point.z = closestObst.z();
          closest_obstacle_dst = dst;
          ROS_INFO("Closest obstacle: %.0f %.0f %.0f dst: %.0f",closest_obstacle.point.x,closest_obstacle.point.y,closest_obstacle.point.z,closest_obstacle_dst);
        }
      }
      else if(dst < min_dst){
        while(dst < min_dst){
          p.z() += 1.0;
          edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
        }
      }
      else if(dst > max_dst){
        while(dst >= max_dst){
          p.z() -= 1;
          edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
        }
      }
      ROS_INFO("Pathin: %i moved from Z: %.0f to %.0f",i,pnt.z,p.z());
      pathin.poses[i].pose.position.z = p.z();
    }
  }
  return pathin;
}

////////////////////////////**********************************/////////////////////////////
////////////////////////////**********************************/////////////////////////////


geometry_msgs::PolygonStamped create_gridpoly(geometry_msgs::Point midpoint, float radlen_xy){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header = hdr();
  poly.polygon.points[0].x = round(midpoint.x + radlen_xy);
  poly.polygon.points[0].y = round(midpoint.y + radlen_xy);
  poly.polygon.points[1].x = round(midpoint.x - radlen_xy);
  poly.polygon.points[1].y = round(midpoint.y + radlen_xy);
  poly.polygon.points[2].x = round(midpoint.x - radlen_xy);
  poly.polygon.points[2].y = round(midpoint.y - radlen_xy);
  poly.polygon.points[3].x = round(midpoint.x + radlen_xy);
  poly.polygon.points[3].y = round(midpoint.y - radlen_xy);
  poly.polygon.points[4]   = poly.polygon.points[0];
 return poly;
}

nav_msgs::Path create_path_ordered(float area_sidelength,float radlen_xy){
  float centroid_sides = 2*radlen_xy;
  int num_grids    = area_sidelength / centroid_sides;
  float true_range = num_grids * centroid_sides;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int y = 0; y < num_grids; y++){
    for(int x = 0; x < num_grids; x++){
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x    = x * centroid_sides - true_range / 2 + centroid_sides/2;
      pose.pose.position.y    = y * centroid_sides - true_range / 2 + centroid_sides/2;
      pose.pose.orientation.w = 1;
      gridpolys.push_back(create_gridpoly(pose.pose.position,radlen_xy-1));
      pose.header = hdr();
      pathout.poses.push_back(pose);
    }
  }
  return pathout;
}
nav_msgs::Path create_path(float area_sidelength,float total_sidelength){
  nav_msgs::Path pathout;
  pathout.header = hdr();

	int num_grids = total_sidelength / area_sidelength-1;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z    = 10;
  pose.pose.orientation.w = 1;
  pose.header = hdr();
  float len = area_sidelength;

  for(int i = 0; i < num_grids; i++){
    for(int k = 0; k < i; k++){
			pose.pose.position.x += pow(-1,i) * len;
      ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
    for(int l = 0; l < i; l++){
      pose.pose.position.y += pow(-1,i) * len;
      ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
  }
	return pathout;
}
int get_zmax_path(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return int(zmx);
}
int get_zmin_path(nav_msgs::Path pathin){
  float zmn = 1100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z < zmn){
      zmn = pathin.poses[i].pose.position.z;
    }
  }
  return int(zmn);
}
nav_msgs::Path get_path_aroundpnt(nav_msgs::Path pathin,geometry_msgs::Point midpoint,float radlen_xy){
  nav_msgs::Path pathout;
  float xmn = midpoint.x - radlen_xy;
  float ymn = midpoint.y - radlen_xy;
  float xmx = midpoint.x + radlen_xy;
  float ymx = midpoint.y + radlen_xy;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x < xmx
    && pathin.poses[i].pose.position.y < ymx
    && pathin.poses[i].pose.position.x > xmn
    && pathin.poses[i].pose.position.y > ymn)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
int get_area_visits(geometry_msgs::Point midpoint, int radlen_xy,bool zmax_not_size){
  nav_msgs::Path vstd = get_path_aroundpnt(path_visited,midpoint,radlen_xy);
  if(zmax_not_size)
    return get_zmax_path(vstd);
  else
    return vstd.poses.size();
}
int get_area_coverage(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int pnts = 0; int pnts_tot = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      pnts_tot++;
      if(img_height.at<cv::Vec3b>(r,c)[1] > 0)
        pnts++;
    }
  }
  int coverage = 100 * pnts / (pnts_tot+1);
  return coverage;
}
int get_zmin_grid(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int zmn = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      if(img_height.at<cv::Vec3b>(r,c)[2] > zmn)
        zmn = img_height.at<cv::Vec3b>(r,c)[2];
    }
  }
  return zmn;
}
int get_zmax_grid(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int zmx = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      if(img_height.at<cv::Vec3b>(r,c)[2] > zmx)
        zmx = img_height.at<cv::Vec3b>(r,c)[2];
    }
  }

  return zmx;
}
std::vector<geometry_msgs::Point> bbrc_to_bbxy(std::vector<int> bbmnmnx_rc){
  std::vector<geometry_msgs::Point> bbmnmx_xy;
  bbmnmx_xy.resize(2);
  bbmnmx_xy[0].x = c2x(bbmnmnx_rc[1]);
  bbmnmx_xy[0].y = r2y(bbmnmnx_rc[2]);
  bbmnmx_xy[1].x = c2x(bbmnmnx_rc[3]);
  bbmnmx_xy[1].y = r2y(bbmnmnx_rc[0]);
  return bbmnmx_xy;
}
std::vector<int> get_bbmnmnx(bool get_height){
  std::vector<int> bbvec; //rmn_cmn_rmx_cmx
  bbvec.resize(4);
  bbvec[0] = img_height.rows;
  bbvec[1] = img_height.cols;
  bbvec[2] = 0;
  bbvec[3] = 0;
  for(int r = 0; r < img_height.rows; r++){
    for(int c = 0; c < img_height.cols; c++){
      int val = img_update_copy.at<cv::Vec3b>(r,c)[1];
      if(get_height)
        val = img_height.at<cv::Vec3b>(r,c)[1];
      if(val > 0){
        if(r < bbvec[0])
          bbvec[0] = r;
        if(c < bbvec[1])
          bbvec[1] = c;
        if(c > bbvec[3])
          bbvec[3] = c;
        if(r > bbvec[2])
          bbvec[2] = r;
      }
    }
  }
  return bbvec;
}

void merge_update_with_grid(){
  for(int c = 0; c < img_height.cols; c++){
    for(int r = 0; r < img_height.rows; r++){
      img_height.at<cv::Vec3b>(r,c)[1] = fmin(255,img_height.at<cv::Vec3b>(r,c)[1]+img_update.at<cv::Vec3b>(r,c)[1]);
      if(img_update.at<cv::Vec3b>(r,c)[0] > 0 && img_update.at<cv::Vec3b>(r,c)[0] < img_height.at<cv::Vec3b>(r,c)[0])
        img_height.at<cv::Vec3b>(r,c)[0] = img_update.at<cv::Vec3b>(r,c)[0];
      if(img_update.at<cv::Vec3b>(r,c)[2] > img_height.at<cv::Vec3b>(r,c)[2])
        img_height.at<cv::Vec3b>(r,c)[2] = img_update.at<cv::Vec3b>(r,c)[2];
    }
  }
}


void test(){
  start_process("get_bbs");
  std::vector<int> bb_upd = get_bbmnmnx(false);
  std::vector<int> bb_hgt = get_bbmnmnx(true);

  std::vector<geometry_msgs::Point> bbxy_upd = bbrc_to_bbxy(bb_upd);
  std::vector<geometry_msgs::Point> bbxy_hgt = bbrc_to_bbxy(bb_hgt);
  start_process("get_zmax_zmin_area_vstd");

  ROS_INFO("BB_upd_ r,c:(%i,%i)->r,c:(%i,%i) height r,c:(%i,%i)->r,c:(%i,%i)", bb_upd[0],bb_upd[1],bb_upd[2],bb_upd[3],bb_hgt[0],bb_hgt[1],bb_hgt[2],bb_hgt[3]);  ROS_INFO("BB UPD: XY:(%.0f,%.0f)->XY:(%.0f,%.0f) BB_hgt: XY:(%.0f,%.0f)->XY:(%.0f,%.0f)",bbxy_upd[0].x,bbxy_upd[0].y,bbxy_upd[1].x,bbxy_upd[1].y,  bbxy_hgt[0].x,bbxy_hgt[0].y,bbxy_hgt[1].x,bbxy_hgt[1].y);
  float radlen_xy = 20;
  start_process("draw_rect");
  cv::rectangle(img, pnt2cv(bbxy_upd[0]),pnt2cv(bbxy_upd[1]),get_color(0,100,100),1,8,0);
  cv::rectangle(img, pnt2cv(bbxy_hgt[0]),pnt2cv(bbxy_hgt[1]),get_color(0,100,100),1,8,0);
  start_process("pathfull_ordered");


  std::vector<float> zmax;
  std::vector<float> zmin;
  std::vector<float> area;
  std::vector<float> vstd;


  start_process("push_back");
  for(int i = 0; i < pathfull_ordered.poses.size(); i++){
    int r1 = get_area_coverage(pathfull_ordered.poses[i].pose.position,radlen_xy);
    int r2 = get_zmax_grid(pathfull_ordered.poses[i].pose.position,radlen_xy);
    int r3 = get_zmin_grid(pathfull_ordered.poses[i].pose.position,radlen_xy);
    int r4 = get_area_visits(pathfull_ordered.poses[i].pose.position,radlen_xy,false);
//    ROS_INFO("i[%i]: %i %i %i %i xyz %.0f %.0f %.0f",i,r1,r2,r3,r4,pathfull_ordered.poses[i].pose.position.x,pathfull_ordered.poses[i].pose.position.y,pathfull_ordered.poses[i].pose.position.z);
    zmax.push_back(float(r1));
    zmin.push_back(float(r2));
    area.push_back(float(r3));
    vstd.push_back(float(r4));
  }
  start_process("normalize_and_draw");

  std::vector<float> zmax_mma = vec_to_min_max_ave(zmax);
  std::vector<float> zmin_mma = vec_to_min_max_ave(zmin);
  std::vector<float> area_mma = vec_to_min_max_ave(area);
  std::vector<float> vstd_mma = vec_to_min_max_ave(vstd);
  start_process("push_back");

  float zmax_range = zmax_mma[1]-zmax_mma[0];
  float zmin_range = zmin_mma[1]-zmin_mma[0];
  float area_range = area_mma[1]-area_mma[0];
  float vstd_range = vstd_mma[1]-vstd_mma[0];
  float zmax_rel,zmin_rel,area_rel,vstd_rel;
  int r = 0; int g = 0; int b = 0;
  for(int i = 0; i < pathfull_ordered.poses.size(); i++){
      zmax_rel  = (zmax[i] - zmax_mma[0]) / (zmax_range+0.0001);
      zmin_rel  = (zmin[i] - zmin_mma[0]) / (zmin_range+0.0001);
      area_rel  = (area[i] - area_mma[0]) / (area_range+0.0001);
      vstd_rel  = (vstd[i] - vstd_mma[0]) / (vstd_range+0.0001);
      if(zmax_range > 0.0)
        r = int(zmax_rel*255);
      if(area_range > 0.0)
        g = int(area_rel * 255);
      if(zmin_range > 0.0)
        b = int(zmin_rel*255);
      else if(vstd_range > 0.0)
        b = int(vstd_rel*255);
    draw_poly(gridpolys[i],get_color(b,g,r));
  }
  start_process("write_img");
  count_target_paths++;
  draw_path(path_visited,get_color(0,200,0),2);
  draw_path(pathfull_ordered,get_color(0,0,200),2);
  //draw_path(pathfull,get_color(200,0,0),2);
	cv::circle(img,pnt2cv(target.pose.position),2,get_color(255,255,200),1);
	cv::circle(img,pnt2cv(pos),2,get_color(255,255,0),1);
	cv::line (img, pnt2cv(pos), pnt2cv(target.pose.position),get_color(0,0,255),1,cv::LINE_8,0);
  std::string s ="test_img"+std::to_string(count_target_paths);
  cv::imwrite("/home/nuc/brain/fullpath/"+s+".png",img);
}
void drawimg(){
  start_process("merge_with_grid");
  merge_update_with_grid();
  start_process("");
  start_process("copy_and_write");
  img_update.copyTo(img_update_copy);
  img_blank.copyTo(img_update);
  cv::imwrite("/home/nuc/brain/fullpath/heights.png",img_height);
  start_process("");
  img_height.copyTo(img);
  test();
}

void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",

  //  transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00 || abs(get_shortest(vlp_rpy.z,tf::getYaw(last_pose.pose.orientation)) * rad2deg) > 30){
    last_pose.pose.position    = pos;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
    path_visited.header = hdr();
  }
}

void update_gridpath(geometry_msgs::Point pnt){
  int r = y2r(pnt.y);
  int c = x2c(pnt.x);
  if(pnt.z < img_update.at<cv::Vec3b>(r,c)[0] || img_update.at<cv::Vec3b>(r,c)[0] == 0)
    img_update.at<cv::Vec3b>(r,c)[0] = pnt.z;
  if(255 > img_update.at<cv::Vec3b>( r,c )[1] )
    img_update.at<cv::Vec3b>(r,c )[1]++;
  if(pnt.z > img_update.at<cv::Vec3b>( r,c )[2] )
    img_update.at<cv::Vec3b>( r,c )[2] = pnt.z;
}

nav_msgs::Path get_pathfinal(sensor_msgs::LaserScan scanin){
  nav_msgs::Path pathout;
  geometry_msgs::PointStamped pnt,pnt_out;
  pathout.header = hdr();
  geometry_msgs::TransformStamped transformStamped;
//  transformStamped = tfBuffer.lookupTransform("map", scanin.header.frame_id, scanin.header.stamp);
  pnt.header.frame_id = scanin.header.frame_id;
  for(int i = 0; i < scanin.ranges.size(); i++){
    if(std::isinf(scanin.ranges[i])){
    }
    else{
      float a = scanin.angle_min + scanin.angle_increment * i;
      pnt.point.x = scanin.ranges[i] * cos(a);
      pnt.point.y = scanin.ranges[i] * sin(a);
      pnt.header.stamp  = scanin.header.stamp;
      try{
          pnt_out = tfBuffer.transform(pnt, "map");
          geometry_msgs::PoseStamped ps;
          ps.header = hdr();
          ps.pose.position    = pnt_out.point;
          ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(pnt_out.point,pos));
          pathout.poses.push_back(ps);
          update_gridpath(pnt_out.point);
      }
      catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
      }
    ///  pnt_out = tfBuffer.transform("map",scanin.header.stamp,pnt,"map",pnt_out);

    }
  }
  return pathout;
}
void process_odom(){
  tf2::Matrix3x3 q(tf2::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
  q.getRPY(rpy_vel.x,rpy_vel.y,rpy_vel.z);
  rpy_vel.y *= -1;
  vz   = odom.twist.twist.linear.z;
  vxy  = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
  vxyz = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2)+pow(odom.twist.twist.linear.z,2));
  vaz  = odom.twist.twist.angular.z;
//  ROS_INFO("Velocity hdng: %.2f incl: %.2f speed: vxyz: %.2f vxy: %.2f vz: %.2f angular: %.2f",rpy_vel.z,rpy_vel.y,vxyz,vxy,vz,vaz);
  forward_point.point.x = odom.twist.twist.linear.x * 3;//  pos.x + 15 * cos(vlp_rpy.z);
  forward_point.point.y = odom.twist.twist.linear.y * 3;// pos.y + 15 * sin(vlp_rpy.z);
  forward_point.header = hdr();
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}

void rayranges_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_mi = get_pathfinal(*msg);
  int half_size = msg->ranges.size()/2;
  int best_dst =1111;
  float best_rng = 1111;
  float best_ang = 0;
  for(int i = 0;i < msg->ranges.size(); i++){
    int dst_from_mid = abs(i - half_size);
    if(dst_from_mid < best_dst && msg->ranges[i] < best_dst*1.3){
      best_ang = msg->angle_min + msg->angle_increment * i;
      best_dst = dst_from_mid;
      best_rng = msg->ranges[i];
    }
  }
  geometry_msgs::PointStamped pnt;
  pnt.point.x = best_rng * cos(best_ang);
  pnt.point.y = best_rng * sin(best_ang);
  pnt.header.stamp   = ros::Time(0);
  pnt.header.frame_id = msg->header.frame_id;
  scanpoint_mid = tfBuffer.transform(pnt, "map");
//  update_frontier(scanpoint_mid.point);
}
void rayranges_hi_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_hi = get_pathfinal(*msg);
}
void rayranges_lo_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_lo = get_pathfinal(*msg);
}
void scan_up_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_up = get_pathfinal(*msg);
}
void scan_dn_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_dn = get_pathfinal(*msg);
}
void scan_stab_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_st = get_pathfinal(*msg);
}
nav_msgs::Path get_path_elevation(nav_msgs::Path pathin){
  for(int i = global_i; i < pathin.poses.size(); i++){
    pathin.poses[i].pose.position.z = get_zmax_grid(pathin.poses[i].pose.position,5);
  }
  return pathin;
}
nav_msgs::Path get_path_elevated(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header =hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z == 0){
      return pathout;
    }
    else
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

void globali_cb(const std_msgs::UInt8::ConstPtr& msg){
  global_i = msg->data;
}

void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
 if(mainstate == 1 && new_target_pose.pose.position.x == 0 && new_target_pose.pose.position.y == 0){
   ROS_INFO("TARGETZERO WARNING");
 }
 else{
   float dst_target     = get_dst3d(target.pose.position,pos);
   float dst_new_target = get_dst3d(new_target_pose.pose.position,pos);
    target_last = target;
    ROS_INFO("Current target: %.0f %.0f %.0f dst: %.0f NEW: %.0f %.0f %.0f dst: %.0f",target.pose.position.x,target.pose.position.y,target.pose.position.z,dst_target,new_target_pose.pose.position.x,new_target_pose.pose.position.y,new_target_pose.pose.position.z,dst_new_target);
	  target.pose.position = new_target_pose.pose.position;
  	target.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(new_target_pose.pose.position,target_last.pose.position));
    new_target_pose = target;
    new_target_pose.pose.position.z = 0;
    new_target_pose.header = hdr();
    target.header = hdr();
    path_targets.poses.push_back(new_target_pose);
    bool par_usemovebase = true;

    if(par_usemovebase)
      pub_cmdpose.publish(new_target_pose);
    else
      pub_cmd.publish(new_target_pose.pose.position);
  }
}
////////////////////////////***********************//////////////////////////////
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	//ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}

int get_next_target(std::vector<int> vec_blacklist,nav_msgs::Path pathin,geometry_msgs::Point current_pos,float current_yaw){
  int best_i = -1;
  float lowest_dist = 10009;
  geometry_msgs::Point p0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_blacklist,i)){
      float dyaw = get_shortest(get_hdng(pathin.poses[i].pose.position,current_pos),current_yaw);
      if(dyaw < 0)
      dyaw *= -1;

      float dst  = get_dst2d(current_pos,pathin.poses[i].pose.position);
      float dst0 = get_dst2d(p0,pathin.poses[i].pose.position);
      if(dst0*dyaw*dst < lowest_dist){
        lowest_dist = dst0*dyaw*dst;
        best_i = i;
      }
    }
  }
  ROS_INFO("best i : %i",best_i);
  return best_i;
}
void set_next_target(){
//int closest_i = getinpath_closestindex3d(pathfull,pos);
  pathfull_i = get_next_target(pathfull_ints_sent,pathfull_ordered,pos,vlp_rpy.z);
  pathfull_ints_sent.push_back(pathfull_i);
  set_target_pose(pathfull_ordered.poses[pathfull_i]);
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data != mainstate){
    ROS_INFO("MAInstate change: %i -> %i",mainstate,msg->data);
    mainstate = msg->data;
  }
}
void request_pathelevation_cb(const nav_msgs::Path::ConstPtr& msg){
  path_global_base = *msg;
  global_i = 0;
  if(msg->poses.size() == 0){
    target_active = false;
    set_next_target();
  }
  else{
    target_active = true;
  }
  nav_msgs::Path path_elevation = get_path_elevation(*msg);
  if(got_map){
    path_elevation_edto = update_edto(path_elevation,20);
  }
  pub_path_elevated.publish(path_elevation);
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

int main(int argc, char** argv)

{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("heading_cutoff",par_hdngcutoff, M_PI/4);
  private_nh.getParam("workdir_path", par_workdir);//*2.0);
//img_height = cv::imread(par_workdir,CV_LOAD_IMAGE_COLOR);
  //  srand (time(NULL));
  //   5 + rand() % 25;
	tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Subscriber s0 = nh.subscribe("/tb_fsm/main_state",10,mainstate_cb);

  ros::Publisher pub_path_targets    = nh.advertise<nav_msgs::Path>("/tb_world/path_targets",10);
  ros::Publisher pub_path_visited    = nh.advertise<nav_msgs::Path>("/tb_world/path_visited",10);
  ros::Publisher pub_maxalt        	 = nh.advertise<std_msgs::UInt8>("/tb_world/maxalt_pos",10);
  ros::Publisher pub_path_pos     	 = nh.advertise<nav_msgs::Path>("/tb_world/path_pos",10);
  pub_path_elevation_update  = nh.advertise<nav_msgs::Path>("/tb_world/path_elevation_update",10);
  pub_path_elevation_update_edto  = nh.advertise<nav_msgs::Path>("/tb_world/path_elevation_update_edto",10);
  pub_path_elevated	 = nh.advertise<nav_msgs::Path>("/tb_world/path_elevated",10);
  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);

  ros::Subscriber ss3  = nh.subscribe("/tb_cmd/global_i",10,globali_cb);

  ros::Subscriber as1       = nh.subscribe("/velodyne_scan",10,rayranges_cb);
  ros::Subscriber a1        = nh.subscribe("/scan_down",10,scan_dn_cb);
  ros::Subscriber a3        = nh.subscribe("/scan_up",10,scan_up_cb);
  ros::Subscriber a6        = nh.subscribe("/scan_stabilized",10,scan_stab_cb);
  ros::Subscriber a2        = nh.subscribe("/scan_tilt_up",10,rayranges_hi_cb);
  ros::Subscriber as4       = nh.subscribe("/scan_tilt_down",10,rayranges_lo_cb);
//  ros::Subscriber aa       = nh.subscribe("/tb_world/request_path_elevated",10,request_pathelevation_cb);
  ros::Subscriber aa1       = nh.subscribe("/tb_mb/path_plan",10,request_pathelevation_cb);
  ros::Publisher pub_target   	= nh.advertise<geometry_msgs::PoseStamped>("/tb_target",10);

  ros::Publisher pub_closest = nh.advertise<geometry_msgs::PointStamped>("/tb_obs/closest",10);
  pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  pub_cmdpose       	= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/posemb",10);
  ros::Time start = ros::Time::now();
  float  area_sidelength  = 50;
	float  total_sidelength = 250;
//  pathfull = create_path(area_sidelength,total_sidelength);
  pathfull_ordered = create_path_ordered(250,25);
	ros::Rate rate(5.0);
  ros::Time last_check;

  while(ros::ok()){
      rate.sleep();
      ros::spinOnce();
      checktf();
      float dst_target     = get_dst2d(target.pose.position,pos);
      process_odom();

    if((ros::Time::now() - start).toSec() > 8 && mainstate == 1){
      if(dst_target < 10)
        set_next_target();
    }
    if((ros::Time::now() - target.header.stamp).toSec() > 10 && mainstate == 1){
      set_next_target();
    }
    if((ros::Time::now() - last_check).toSec() > 1.0){
      last_check = ros::Time::now();
      drawimg();
    }

    if(mainstate == 2){
      target.pose.position.x = 0;
      target.pose.position.y = 0;
      set_target_pose(target);
    }
    ROS_INFO("Current target: %.0f %.0f %.0f dst: %.0f",target.pose.position.x,target.pose.position.y,target.pose.position.z,dst_target);

    nav_msgs::Path path_elevation = get_path_elevation(path_global_base);
    if(got_map){
      start_process("edto_elevation");
      path_elevation_edto = update_edto(path_elevation,25);
      start_process("");
    }
    pub_path_targets.publish(path_targets);

    pub_target.publish(target);
    path_global_base.header = hdr();
    pub_path_elevation_update.publish(path_elevation);
    pub_path_elevation_update_edto.publish(path_elevation_edto);
    nav_msgs::Path path_pos;
    std_msgs::UInt8 maxalt_pos;

    maxalt_pos.data = fmax(get_zmax_grid(pos,10),closest_obstacle.point.z);
    pub_maxalt.publish(maxalt_pos);
    pub_path_visited.publish(path_visited);
  }
  return 0;
}
