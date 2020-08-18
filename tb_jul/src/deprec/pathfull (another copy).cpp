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

std::string inspection_type = "idle";
sensor_msgs::LaserScan scan_lo,scan_mi,scan_hi,scan_up,scan_dn,scan_st;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point pos;
geometry_msgs::PoseStamped last_pose;
geometry_msgs::PointStamped scanpoint_mid,scanpoint_ave,frontier_centroid,frontier_minpnt,frontier_minpnt_infront,frontier_closest_infront,frontier_outofsight_infront;
nav_msgs::Odometry odom;
double par_hdngcutoff;
int counter = 0;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;

int count_target_paths = 0;
nav_msgs::Path path_lo,path_mi,path_hi,path_up,path_dn,path_st,path_full;
geometry_msgs::PolygonStamped poly_frontier;
nav_msgs::Path path_visited,path_heightpath,path_frontier,path_scanzone,path_surround,path_forward,path_right,path_left;
tb_msgsrv::Paths heightpaths;
sensor_msgs::LaserScan scan_frontier;
ros::Publisher pub_path_requested;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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
float vec_to_ave(std::vector<float> vec_in){
  float vec_sum = 0;
  for(int i = 0; i < vec_in.size(); i++){
    vec_sum += vec_in[i];
  }
  return vec_sum / vec_in.size();
}
float vec_to_max(std::vector<float> vec_in){
  float vec_max = -12310;
  for(int i = 0; i < vec_in.size(); i++){
    if(vec_in[i] > vec_max)
     vec_max = vec_in[i];
  }
  return vec_max;
}
float vec_to_min(std::vector<float> vec_in){
  float vec_min = 133330;
  for(int i = 0; i < vec_in.size(); i++){
    if(vec_in[i] < vec_min)
     vec_min = vec_in[i];
  }
  return vec_min;
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
nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(pos,pathin.poses[i].pose.position)));
    else if(sort_by == "inclination")
      i_dst.push_back(std::make_tuple(i,get_inclination(pathin.poses[i].pose.position,pos)));
    else if(sort_by == "dst_3d")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pos,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_abs")
			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z) * rad2deg)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z)));
    else if(sort_by == "z")
      i_dst.push_back(std::make_tuple(i,pathin.poses[i].pose.position.z));
	}
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
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
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
}

void draw_path_by_score(nav_msgs::Path pathin,std::vector<float> score,int primary_color,int secondary_color,int tertiary_color,float color_intensity){
  geometry_msgs::Point maxmin;
  maxmin.x = vec_to_max(score);
  maxmin.y = vec_to_min(score);
	geometry_msgs::Point pyaw,prect0,prect1,pnt;
  if(pathin.poses.size() < 1)
    return;
	for(int i = 0; i < pathin.poses.size(); i++){
		pnt = pathin.poses[i].pose.position;
    cv::Scalar color;
    float rel_score = color_intensity * (score[i] - maxmin.y) / (maxmin.x - maxmin.y);
    if(rel_score > 0.9)
      color[tertiary_color] = 255*rel_score;
    if(rel_score > 0.75)
      color[secondary_color] = 255*rel_score;
    if(rel_score > 0.3)
      color[primary_color] = 255*rel_score;
    if(rel_score < 0.3)
      color[primary_color] = 100*rel_score;

    if(pathin.poses.size() > 25){
      img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
    }
    else{
      float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
      pyaw.x = pnt.x + 3 * cos(yaw);
      pyaw.y = pnt.y + 3 * sin(yaw);
      cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
      cv::circle(img,pnt2cv(pnt),2,color,1);
    }
	}
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
void draw_rectangle(geometry_msgs::Point midpoint,float size, cv::Scalar color){
  geometry_msgs::Point p0,p1;
  p0.x = midpoint.x-size; p1.x = midpoint.x+size*2;
  p0.y = midpoint.y-size; p1.y = midpoint.y+size*2;
  cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
}
void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}
void drawimg(){
  std::vector<float> v1 = get_vec_attribute(path_full,"z");
  std::vector<float> v2 = get_vec_attribute(path_surround,"z");
  std::vector<float> v3 = get_vec_attribute(path_forward,"z");
  std::vector<float> vr = get_vec_attribute(path_right,"z");
  std::vector<float> vl = get_vec_attribute(path_left,"z");
  std::vector<float> vh = get_vec_attribute(path_heightpath,"z");


  count_target_paths++;
  draw_path_by_score(path_full,v1,2,2,2,0.4);
  draw_path_by_score(path_surround,v2,1,1,1,0.4);
  draw_path_by_score(path_right,vr,0,0,0,0.7);
  draw_path_by_score(path_left,vl,0,0,0,0.7);
  draw_path_by_score(path_forward,v3,2,2,2,1.0);
  draw_path_by_score(path_heightpath,vh,2,1,0,1.0);

  draw_rectangle(frontier_outofsight_infront.point,3,get_color(150,25,200));
  draw_rectangle(frontier_minpnt.point,3,get_color(150,25,65));
  draw_rectangle(frontier_minpnt_infront.point,3,get_color(150,25,65));
  draw_rectangle(frontier_closest_infront.point,3,get_color(150,25,65));
  cv::circle(img,pnt2cv(scanpoint_ave.point),2,get_color(200,0,200),1);
  cv::circle(img,pnt2cv(scanpoint_mid.point),2,get_color(200,200,200),1);
  draw_poly(poly_frontier,get_color(0,0,200));
  geometry_msgs::Point pyaw,pyaw_mn,pyaw_mx;
  float a1 = constrainAngle(vlp_rpy.z + 2*par_hdngcutoff);
  float a2 = constrainAngle(vlp_rpy.z - 2*par_hdngcutoff);
  float a0 = fmin(a1,a2);
  float aN = fmax(a1,a2);
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  draw_line(pos,vlp_rpy.z,100,get_color(100,100,0));
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  cv::imwrite("/home/nuc/brain/fullpath/full_"+std::to_string(count_target_paths)+ ".png",img);
  img_blank.copyTo(img);
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

float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}
float get_zmin(nav_msgs::Path pathin){
  float zmn = 1100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z < zmn){
      zmn = pathin.poses[i].pose.position.z;
    }
  }
  return zmn;
}
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin,std::string type,float val_0,float val_N){
  if(pathin.poses.size() < 3)
    return pathin;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(type == "hdng"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng = get_hdng(pathin.poses[i].pose.position,pos);
      if(hdng >= val_0 && hdng <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "hdng_rel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng     = get_hdng(pathin.poses[i].pose.position,pos);
      float hdng_rel = get_shortest(hdng,vlp_rpy.z);
      if(hdng_rel >= val_0 && hdng_rel <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "zrel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      if((pathin.poses[i].pose.position.z - pos.z) >= val_0 && (pathin.poses[i].pose.position.z - pos.z) <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "z"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng = get_hdng(pathin.poses[i].pose.position,pos);
      if(hdng >= pathin.poses[i].pose.position.z && pathin.poses[i].pose.position.z <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  return pathout;
}
std::vector<geometry_msgs::Point> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<geometry_msgs::Point> bbmnbbmx;

  if(pathin.poses.size() == 0){
    ROS_INFO("GENERICNODE: pathin is empty");
    return bbmnbbmx;
  }
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
geometry_msgs::PolygonStamped create_bbpoly(float dmin,float dmax,float z,float a0,float an){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header = hdr();
  poly.polygon.points[0].x = pos.x + dmin * cos(a0+vlp_rpy.z);
  poly.polygon.points[0].y = pos.y + dmin * sin(a0+vlp_rpy.z);
  poly.polygon.points[0].z = z;

  poly.polygon.points[1].x = pos.x + dmin * cos(an+vlp_rpy.z);
  poly.polygon.points[1].y = pos.y + dmin * sin(an+vlp_rpy.z);
  poly.polygon.points[1].z = z;

  poly.polygon.points[2].x = pos.x + dmax * cos(an+vlp_rpy.z);
  poly.polygon.points[2].y = pos.y + dmax * sin(an+vlp_rpy.z);
  poly.polygon.points[2].z = z;

  poly.polygon.points[3].x = pos.x + dmax * cos(a0+vlp_rpy.z);
  poly.polygon.points[3].y = pos.y + dmax * sin(a0+vlp_rpy.z);
  poly.polygon.points[3].z = z;

  poly.polygon.points[4] = poly.polygon.points[0];
  return poly;
}

geometry_msgs::Point get_ave_pnt(nav_msgs::Path pathin){
  geometry_msgs::Point pnt;
  for(int i = 0; i < pathin.poses.size(); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= pathin.poses.size();
  pnt.y /= pathin.poses.size();
  pnt.z /= pathin.poses.size();
  return pnt;
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
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


geometry_msgs::Point get_poly_centroidarea(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;

    if(polyin.polygon.points.size() < 2){
      return centroid;
    }
    for (i=0; i<polyin.polygon.points.size()-1; ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[i+1].x;
        y1 = polyin.polygon.points[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    // Do last vertex separately to avoid performing an expensive
    // modulus operation in each iteration.
    x0 = polyin.polygon.points[i].x;
    y0 = polyin.polygon.points[i].y;
    x1 = polyin.polygon.points[0].x;
    y1 = polyin.polygon.points[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);
    centroid.z = signedArea;

    return centroid;
}

nav_msgs::Path merge_paths(std::vector<nav_msgs::Path> paths){
  nav_msgs::Path pathout;
  for(int i = 0; i < paths.size(); i++){
    for(int k = 0; k < paths[i].poses.size(); k++){
      pathout.poses.push_back(paths[i].poses[k]);
    }
  }
  pathout.header = hdr();
  return pathout;
}
nav_msgs::Path merge_tbpaths(tb_msgsrv::Paths paths){
  nav_msgs::Path pathout;
  for(int i = 0; i < paths.paths.size(); i++){
    for(int k = 0; k < paths.paths[i].poses.size(); k++){
      pathout.poses.push_back(paths.paths[i].poses[k]);
    }
  }
  pathout.header = hdr();
  return pathout;
}
std::vector<nav_msgs::Path> get_pathvector(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi){
  std::vector<nav_msgs::Path> paths;
  paths.push_back(p_dn);
  paths.push_back(p_lo);
  paths.push_back(p_mi);
  paths.push_back(p_up);
  paths.push_back(p_hi);
  return paths;
}

int getinpath_closestindex3d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  int lowest_dist_i = 0;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst3d(pathin.poses[i].pose.position,pnt);
    if(dst < lowest_dist){
      lowest_dist_i = i;
      lowest_dist = dst;
    }
  }
  return lowest_dist_i;
}
nav_msgs::Path get_path_aroundpnt(nav_msgs::Path pathin,geometry_msgs::Point midpoint,float dxy){
  nav_msgs::Path pathout;
  float xmn = midpoint.x - dxy;
  float ymn = midpoint.y - dxy;
  float xmx = midpoint.x + dxy;
  float ymx = midpoint.y + dxy;
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

nav_msgs::Path get_pathfull_aroundline(geometry_msgs::Point p2,geometry_msgs::Point p1,float dxy){
  nav_msgs::Path pathout;
  pathout.header = hdr();

  Eigen::Vector3f cur_vec(p1.x,p1.y,p1.z);
  Eigen::Vector3f pnt1(p1.x,p1.y,p1.z);
  Eigen::Vector3f pnt2(p2.x,p2.y,p2.z);
  float tot_ray_len = (pnt2 - pnt1).norm();
  float cur_ray_len = 0;
  Eigen::Vector3f stride_vec = (pnt2 - pnt1).normalized() * dxy;
  geometry_msgs::Point midpoint;

  while(cur_ray_len < tot_ray_len){
    cur_vec = cur_vec + stride_vec;
    midpoint.x = cur_vec.x();
    midpoint.y = cur_vec.y();
    midpoint.z = cur_vec.z();
    nav_msgs::Path path_segment = get_path_aroundpnt(path_full,midpoint,dxy);
    for(int i = 0; i < path_segment.poses.size(); i++){
      pathout.poses.push_back(path_segment.poses[i]);
    }
    cur_ray_len = (cur_vec - pnt1).norm();
  }
  return pathout;
}


void get_frontier(float num_is,float stride_length){

  path_frontier.header = hdr();
  poly_frontier.header = hdr();
  float rads_pr_i = 2*M_PI / num_is;
  path_frontier.poses.resize(num_is);
  poly_frontier.polygon.points.resize(num_is);
  scan_frontier.ranges.resize(num_is);
  scan_frontier.angle_min = -M_PI;
  scan_frontier.angle_max = M_PI;
  scan_frontier.angle_increment = rads_pr_i;
  scan_frontier.header = hdr();
  scan_frontier.range_min = 1.0;
  scan_frontier.range_max = 500;
  float lowest_rf = 10000;
float lowest_d = 10000;
float lowest_r = 10000;
float lowest_outofsight_infront = 10000;
  float a0 = -M_PI;
  for(int i = 0; i < num_is; i++){
    float a = a0 + i * rads_pr_i;
    float cur_ray_len = 0;
    float tot_ray_len = 500;
    nav_msgs::Path last_segment_hit;
    geometry_msgs::Point midpoint;
    while(cur_ray_len < tot_ray_len){
      midpoint.x = cur_ray_len * cos(a);
      midpoint.y = cur_ray_len * sin(a);
      nav_msgs::Path path_segment = get_path_aroundpnt(path_full,midpoint,stride_length);
      if(path_segment.poses.size() == 0){
        break;
      }
      else{
        last_segment_hit = path_segment;
        cur_ray_len += stride_length;
      }
    }
    float apos = get_hdng(midpoint,pos);
    float da = get_shortest(apos,vlp_rpy.z);
    if(da < M_PI && da > -M_PI){
      if(get_dst2d(pos,midpoint) > 50
      && get_dst2d(pos,midpoint) < 150
      && cur_ray_len < lowest_outofsight_infront){
        lowest_outofsight_infront = cur_ray_len;
        frontier_outofsight_infront.point = midpoint;
      }
      if(cur_ray_len < lowest_rf){
        lowest_rf = cur_ray_len;
        frontier_minpnt_infront.point = midpoint;
      }
      if(get_dst2d(pos,midpoint) < lowest_d){
        lowest_d = get_dst2d(pos,midpoint);
        frontier_closest_infront.point = midpoint;
      }
    }
    if(cur_ray_len < lowest_r){
      lowest_r = cur_ray_len;
      frontier_minpnt.point = midpoint;
    }

    scan_frontier.ranges[i] = cur_ray_len;
    poly_frontier.polygon.points[i].z = path_frontier.poses[i].pose.position.z = get_zmax(last_segment_hit);
    poly_frontier.polygon.points[i].x = path_frontier.poses[i].pose.position.x = midpoint.x;
    poly_frontier.polygon.points[i].y = path_frontier.poses[i].pose.position.y = midpoint.y;
  }
  frontier_minpnt.header = hdr();
  frontier_minpnt_infront.header = hdr();
  frontier_closest_infront.header = hdr();
  frontier_outofsight_infront.header = hdr();
}


//******************************************************************************************************************//
//***************************FRONTIER END******************************************//
//***************************HEIGHTPATHS START******************************************//
//******************************************************************************************************************//

geometry_msgs::PolygonStamped create_linepoly(geometry_msgs::Point p0, float line_length,float line_heading,float poly_width){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header = hdr();
  geometry_msgs::Point p1;
  p1.x = p0.x + line_length * cos(line_heading);
  p1.y = p0.y + line_length * sin(line_heading);

  poly.polygon.points[0].x = p0.x + poly_width/2*cos(line_heading+M_PI/2);
  poly.polygon.points[0].y = p0.y + poly_width/2*sin(line_heading+M_PI/2);
  poly.polygon.points[1].x = p0.x + poly_width/2*cos(line_heading-M_PI/2);
  poly.polygon.points[1].y = p0.y + poly_width/2*sin(line_heading-M_PI/2);
  poly.polygon.points[2].x = p1.x + poly_width/2*cos(line_heading-M_PI/2);
  poly.polygon.points[2].y = p1.y + poly_width/2*sin(line_heading-M_PI/2);
  poly.polygon.points[3].x = p1.x + poly_width/2*cos(line_heading+M_PI/2);
  poly.polygon.points[3].y = p1.y + poly_width/2*sin(line_heading+M_PI/2);
  poly.polygon.points[4] = poly.polygon.points[0];
 return poly;
}

nav_msgs::Path create_linepath(geometry_msgs::Point p0, float line_length,float line_heading,float interval_meters){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int num_intervals = line_length / interval_meters;
  for(int i = 0; i < num_intervals+1; i++){
    geometry_msgs::PoseStamped ps;
    ps.header = hdr();
    ps.pose.position.x = p0.x + interval_meters * i * cos(line_heading);
    ps.pose.position.y = p0.y + interval_meters * i * sin(line_heading);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(line_heading);
    pathout.poses.push_back(ps);
  }
 return pathout;
}
geometry_msgs::Point get_linepoly_p0(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point start;
  if(polyin.polygon.points.size() > 2){
    start.x = (polyin.polygon.points[0].x + polyin.polygon.points[1].x)/2;
    start.y = (polyin.polygon.points[0].y + polyin.polygon.points[1].y)/2;
  }
  else{
    ROS_INFO("LINEPOLY P0 ERROR");
  }
  return start;
}
geometry_msgs::Point get_linepoly_p1(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point end;
  if(polyin.polygon.points.size() > 3){
    end.x   = (polyin.polygon.points[2].x + polyin.polygon.points[3].x)/2;
    end.y   = (polyin.polygon.points[2].y + polyin.polygon.points[3].y)/2;
  }
  else{
    ROS_INFO("LINEPOLY P1 ERROR");
  }
  return end;
}

float get_linepoly_heading(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point end,start;
  start = get_linepoly_p0(polyin);
  end   = get_linepoly_p1(polyin);
  return get_hdng(end,start);
}
float get_linepoly_length(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point end,start;
  start = get_linepoly_p0(polyin);
  end   = get_linepoly_p1(polyin);
  return get_dst2d(end,start);
}

std::vector<nav_msgs::Path> segment_paths_with_linepolys(nav_msgs::Path pathin,std::vector<geometry_msgs::PolygonStamped> linepolys){
  std::vector<nav_msgs::Path> pathsout;
  pathsout.resize(linepolys.size());
  for(int i = 0; i < linepolys.size(); i++){
    pathsout[i] = constrain_path_bbpoly(pathin,linepolys[i]);
  }
  return pathsout;
}
std::vector<geometry_msgs::PolygonStamped> create_linepolys(geometry_msgs::Point p0,float a0,float aN,float len_polys,float width_polys,int num_is,bool get_shortest_angle){
  std::vector<geometry_msgs::PolygonStamped> linepolys;
  linepolys.resize(num_is);
  float rads_pr_i = (aN-a0)/num_is;
  if(get_shortest_angle)
    rads_pr_i = get_shortest(aN,a0) / num_is;
  for(int i = 0; i < num_is; i++){
    float a_i    = constrainAngle(a0 + rads_pr_i * i);
    linepolys[i] = create_linepoly(p0,len_polys,a_i,width_polys);
  }
  return linepolys;
}
std::vector<nav_msgs::Path> create_linepaths(geometry_msgs::Point p0,float a0,float aN,float len_polys,float interval_meters,int num_is,bool get_shortest_angle){
  std::vector<nav_msgs::Path> linepaths;
  linepaths.resize(num_is);
  float rads_pr_i = (aN-a0)/num_is;
  if(get_shortest_angle)
    rads_pr_i = get_shortest(aN,a0) / num_is;
  for(int i = 0; i < num_is; i++){
    float a_i    = constrainAngle(a0 + rads_pr_i * i);
    linepaths[i] = create_linepath(p0,len_polys,a_i,interval_meters);
  }
  return linepaths;
}
std::vector<std::vector<int>> get_intervals(std::vector<float> vec_dst2d,float m_pr_interval){
  std::vector<std::vector<int>> is_in_intervals;

  //ROS_INFO("dst0 -> N : %i dsts %.0f %.0f mprint: %.0f",vec_dst2d.size(),vec_dst2d[0],vec_dst2d[vec_dst2d.size()-1],m_pr_interval);
  int intervals_d2d = 1+(vec_dst2d[vec_dst2d.size()-1]) / m_pr_interval;

  is_in_intervals.resize(intervals_d2d);
  for(int i = 0; i < vec_dst2d.size();i++){
    int interval = vec_dst2d[i] / m_pr_interval;
    is_in_intervals[interval].push_back(i);
  }
  return is_in_intervals;
}

std::vector<nav_msgs::Path> get_path_intervals(nav_msgs::Path pathin,std::vector<std::vector<int>> intervals){
  std::vector<nav_msgs::Path> path_intervals;
  path_intervals.resize(intervals.size());
  for(int k = 0; k < intervals.size(); k++){
    for(int i = 0; i < intervals[k].size(); i++){
      path_intervals[k].poses.push_back(pathin.poses[intervals[k][i]]);
    }
  }
  return path_intervals;
}
tb_msgsrv::Paths remove_empty_intervals(std::vector<nav_msgs::Path> pathsin){
  tb_msgsrv::Paths pathsout;
  for(int k = 0; k < pathsin.size(); k++){
    nav_msgs::Path pathout;
    pathout.header = hdr();
    for(int i = 0; i < pathsin[k].poses.size(); i++){
      if(pathsin[k].poses[i].pose.position.z > 0)
        pathout.poses.push_back(pathsin[k].poses[i]);
    }
    if(pathout.poses.size() > 0)
      pathsout.paths.push_back(pathout);
  }
  return pathsout;
}

tb_msgsrv::Paths create_heightpaths(nav_msgs::Path pathin,float m_pr_interval,int num_polys,float len_polys,float width_polys,float a1,float a2,bool get_shortest_angle){
//  ROS_INFO("pathin: %i m_pr_interval: %.2f num_polys %i len_polys: %.2f width_polys: %.2f a1: %.2f,a2: %.2f",pathin.poses.size(),m_pr_interval,num_polys,len_polys,width_polys,a1,a2);

  float a0 = fmin(a1,a2);
  float aN = fmax(a1,a2);
  std::vector<geometry_msgs::PolygonStamped> linepolys  = create_linepolys(pos,a0,aN,len_polys,width_polys,num_polys,get_shortest_angle);
//  ROS_INFO("CREATED linepolys: %i",linepolys.size());
  std::vector<nav_msgs::Path>                linepaths  = create_linepaths(pos,a0,aN,len_polys,m_pr_interval,num_polys,get_shortest_angle);
//  ROS_INFO("CREATED linepaths: %i",linepaths.size());
  std::vector<nav_msgs::Path>            path_segments  = segment_paths_with_linepolys(pathin,linepolys);
//  ROS_INFO("CREATED path_segments: %i",path_segments.size());

  for(int i = 0; i < path_segments.size(); i++){
    nav_msgs::Path path_segment                        = sort_path(path_segments[i],"dst_2d");
    if(path_segment.poses.size() > 0){
      std::vector<float> vec_dst2d                       = get_vec_attribute(path_segment,"dst_2d");
      std::vector<std::vector<int>> intervals            = get_intervals(vec_dst2d,m_pr_interval);
      std::vector<nav_msgs::Path> path_segment_intervals = get_path_intervals(path_segment,intervals);
  //    ROS_INFO("path_segment:  %i path_segment: %i vec_dst2d: %i  intervals: %i  path_segment_intervals: %i ",path_segment.poses.size(),vec_dst2d.size(),intervals.size(),path_segment_intervals.size());
      for(int k = 0; k < path_segment_intervals.size(); k++){
        if(path_segment_intervals[k].poses.size() > 1)
          linepaths[i].poses[k].pose.position.z = get_zmax(path_segment_intervals[k]);
      }
    }
  }
  ROS_INFO("DONE");
 return remove_empty_intervals(linepaths);
}
//******************************************************************************************************************//
//***************************HEIGHTPATHS END******************************************//
//***************************FULLPATH START******************************************//
//******************************************************************************************************************//

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
nav_msgs::Path get_new_path(nav_msgs::Path path_old,nav_msgs::Path pathin,float cutoff_dst){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(path_old,pathin.poses[i].pose.position,cutoff_dst))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

nav_msgs::Path get_path_inrad(nav_msgs::Path pathin,geometry_msgs::Point midpoint,float maxrad){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(midpoint,pathin.poses[i].pose.position) < maxrad){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
void add_new(nav_msgs::Path pathin){
  for(int i = 0; i < pathin.poses.size(); i++){
    path_full.poses.push_back(pathin.poses[i]);
  }
  path_full.header = hdr();
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

void update_path_full(){
  ros::Time t0 = ros::Time::now();
  nav_msgs::Path path_scanned = merge_paths(get_pathvector(path_hi,path_lo,path_up,path_dn,path_mi));
  path_scanzone = scatter_path(path_scanned,1);
  scanpoint_ave.point = get_ave_pnt(path_scanzone);
  scanpoint_ave.header = hdr();
  nav_msgs::Path path_full_inrad  = get_path_inrad(path_full,pos,50);
  nav_msgs::Path path_scanned_new = get_new_path(path_full_inrad,path_scanzone,1);
  add_new(path_scanned_new);
  float dt = (ros::Time::now()-t0).toSec();
  ROS_INFO("PATH_FULL UPDATE: %.4 sec",dt);
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
  update_path_full();
  path_st = get_pathfinal(*msg);
}
void polyrequest_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  pub_path_requested.publish(constrain_path_bbpoly(path_full,*msg));
}
void midpointrequest_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  pub_path_requested.publish(get_path_aroundpnt(path_full,msg->point,msg->point.z));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("heading_cutoff",par_hdngcutoff, M_PI/12);

	tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Publisher pub_scan_frontier = nh.advertise<sensor_msgs::LaserScan>("/tb_world/scan_frontier",10);
  ros::Publisher pub_poly_frontier  	 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_world/poly_frontier",10);

  ros::Publisher pub_heightpaths     = nh.advertise<tb_msgsrv::Paths>("/tb_world/heightpaths",10);

  ros::Publisher pub_path_visited    = nh.advertise<nav_msgs::Path>("/tb_world/path_visited",10);
  ros::Publisher pub_path_heightpath = nh.advertise<nav_msgs::Path>("/tb_world/path_heightpath",10);
  ros::Publisher pub_path_scanzone 	 = nh.advertise<nav_msgs::Path>("/tb_world/path_scanzone",10);
  ros::Publisher pub_path_surround 	 = nh.advertise<nav_msgs::Path>("/tb_world/path_surround",10);
  ros::Publisher pub_path_forward 	 = nh.advertise<nav_msgs::Path>("/tb_world/path_forward",10);
  ros::Publisher pub_path_right    	 = nh.advertise<nav_msgs::Path>("/tb_world/path_right",10);
  ros::Publisher pub_path_left     	 = nh.advertise<nav_msgs::Path>("/tb_world/path_left",10);
  ros::Publisher pub_path_frontier	 = nh.advertise<nav_msgs::Path>("/tb_world/path_frontier",10);
                 pub_path_requested 	 = nh.advertise<nav_msgs::Path>("/tb_world/path_forward",10);
  ros::Subscriber a1s1 = nh.subscribe("/tb_world/request_poly",10,polyrequest_cb);
  ros::Subscriber s3  = nh.subscribe("/tb_world/request_midpoint",10,midpointrequest_cb);


  ros::Subscriber as1       = nh.subscribe("/velodyne_scan",10,rayranges_cb);
  ros::Subscriber a1        = nh.subscribe("/scan_down",10,scan_dn_cb);
  ros::Subscriber a3        = nh.subscribe("/scan_up",10,scan_up_cb);
  ros::Subscriber a6        = nh.subscribe("/scan_stabilized",10,scan_stab_cb);
  ros::Subscriber a2        = nh.subscribe("/scan_tilt_up",10,rayranges_hi_cb);
  ros::Subscriber as4       = nh.subscribe("/scan_tilt_down",10,rayranges_lo_cb);

  ros::Publisher pub_scanpoint_ave   	           = nh.advertise<geometry_msgs::PointStamped>("/tb_world/scanpoint_ave",10);
  ros::Publisher pub_scanpoint_mid  	           = nh.advertise<geometry_msgs::PointStamped>("/tb_world/scanpoint_mid",10);
  ros::Publisher pub_frontier_minpnt             = nh.advertise<geometry_msgs::PointStamped>("/tb_world/frontier_minpnt",10);
  ros::Publisher pub_frontier_minpnt_infront  	 = nh.advertise<geometry_msgs::PointStamped>("/tb_world/frontier_minpnt_infront",10);
  ros::Publisher pub_frontier_closest_infront  	 = nh.advertise<geometry_msgs::PointStamped>("/tb_world/frontier_closest_infront",10);
  ros::Publisher pub_frontier_outofsight_infront = nh.advertise<geometry_msgs::PointStamped>("/tb_world/frontier_outofsight_infront",10);
  ros::Publisher pub_frontier_centroid           = nh.advertise<geometry_msgs::PointStamped>("/tb_world/poly_frontier_centroid",10);


	ros::Rate rate(5.0);
  ros::Time last_check;
  ros::Time start= ros::Time::now();

  float current_revolution = 0;
  float current_radlength  = 10;
  float leadlength_radians = M_PI/4;
  float frontier_area = 0;
  bool draw = true;

    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();
      checktf();
      ros::Time t0 = ros::Time::now();
      get_frontier(32,10);
      float dt = (ros::Time::now()-t0).toSec();
      ROS_INFO("FRONTIER UPDATE: %.4f sec",dt);
      t0 = ros::Time::now();
      nav_msgs::Path pathfull_copy = constrain_path_bbpoly(path_full,poly_frontier);
      dt = (ros::Time::now()-t0).toSec();
      ROS_INFO("PATHFULL_CONSTRAIN POLY [%i poses] UPDATE: %.4f sec",pathfull_copy.poses.size(),dt);
      t0 = ros::Time::now();

      if(path_surround.poses.size() > 10){
      //  constrainAngle(vlp_rpy.z - par_hdngcutoff*4)
  //constrainAngle(vlp_rpy.z + par_hdngcutoff*4)
        heightpaths = create_heightpaths(path_surround,5.0,32,50.0,7.5,-M_PI,M_PI,false);
        path_heightpath = merge_tbpaths(heightpaths);
        dt = (ros::Time::now()-t0).toSec();
        ROS_INFO("PATHFULL_HEIGHTPATH [%i poses] UPDATE: %.4f sec",path_heightpath.poses.size(),dt);
        t0 = ros::Time::now();
      }

  //nav_msgs::Path pathin,float m_pr_interval,int num_polys,float len_polys,float width_polys,float a1,float a2


  //  path_surround = get_path_aroundpnt(path_full,pos,50);
    path_surround = get_path_inrad(path_full,pos,50);
    path_forward  = cutoff_abs(path_surround,"hdng_rel",-2*par_hdngcutoff,par_hdngcutoff*2);
    path_right    = cutoff_abs(path_surround,"hdng_rel",-par_hdngcutoff*6,-par_hdngcutoff*2);
    path_left     = cutoff_abs(path_surround,"hdng_rel",par_hdngcutoff*2,par_hdngcutoff*6);
    dt = (ros::Time::now()-t0).toSec();
    ROS_INFO("PATHFULL_CONSTRAIN POLY [path_surround %i path_forward %i path_right %i path_left %i poses] UPDATE: %.4f sec",path_surround.poses.size(),path_forward.poses.size(),path_right.poses.size(),path_left.poses.size(),dt);
    if(draw){
      t0 = ros::Time::now();
      drawimg();
      dt = (ros::Time::now()-t0).toSec();
      ROS_INFO("DRAW IT ALL: %.4f sec",dt);
    }
    frontier_centroid.point = get_poly_centroidarea(poly_frontier);

    frontier_area             = abs(frontier_centroid.point.z);
    frontier_centroid.point.z = pos.z;
    pub_path_visited.publish(path_visited);
    pub_path_heightpath.publish(path_heightpath);
    pub_heightpaths.publish(heightpaths);
    pub_frontier_centroid.publish(frontier_centroid);
    pub_poly_frontier.publish(poly_frontier);
    pub_scanpoint_ave.publish(scanpoint_ave);
    pub_scanpoint_mid.publish(scanpoint_mid);
    pub_path_scanzone.publish(path_scanzone);
    pub_path_surround.publish(path_surround);
    pub_path_forward.publish(path_forward);
    pub_path_right.publish(path_right);
    pub_path_left.publish(path_left);
    pub_path_frontier.publish(path_frontier);
    pub_scan_frontier.publish(scan_frontier);
    pub_frontier_minpnt.publish(frontier_minpnt);
    pub_frontier_minpnt_infront.publish(frontier_minpnt_infront);
    pub_frontier_closest_infront.publish(frontier_closest_infront);
    pub_frontier_outofsight_infront.publish(frontier_outofsight_infront);
  }
  return 0;
}
