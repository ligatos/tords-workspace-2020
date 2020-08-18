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
geometry_msgs::Vector3 vlp_rpy,rpy_vel;
geometry_msgs::Point pos,pnt_ref;
geometry_msgs::PointStamped forward_point,scanpoint_mid,scanpoint_ave,frontier_centroid,frontier_minpnt,frontier_minpnt_infront,frontier_closest_infront,frontier_outofsight_infront;
nav_msgs::Odometry odom;
float vz,vxy,vxyz,vaz;
double par_hdngcutoff,par_zclearing;
int counter = 0;
float rad2deg = 180.0/M_PI;
float deg2rad = M_PI/180;
int mainstate = 0;
tb_msgsrv::Paths heightpaths;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
int count_target_paths = 0;
int current_frontier_target = 2;
float current_frontier_radians = 0;
nav_msgs::Path path_pos,path_elevated,global_plan_scattered,local_plan_scattered,heightpath_practical,path_targets,path_above_pos,heightpath,path_scanzone,path_surround,path_forward,path_right,path_left,path_visited,path_frontier;
ros::Publisher pub_path,pub_global_plan_scattered,pub_local_plan_scattered,pub_path_elevation_request;
ros::Publisher pub_cmdexplore,pub_cmdmb,pub_get_next_path,pub_cmd,pub_cmdpose;
geometry_msgs::PoseStamped target_frontier,target,target_last;
std_msgs::Float64 arm1_tilt_msg;
std_msgs::Float64 target_alt_msg;
bool par_usemovebase;
int lead_num = 1;

std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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
float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / (get_dst2d(p1,p2));
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
float vec_to_last_ave(std::vector<float> vec_in){
  float ave_sum = 0;
  if(vec_in.size() > 7){
    for(int i = vec_in.size() - 4; i < vec_in.size(); i++){
      ave_sum += vec_in[i];
    }
  }
  return ave_sum/4.0;
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
std::vector<float> get_vec_attribute(nav_msgs::Path pathin,std::string type){
  std::vector<float> vec_out;
  geometry_msgs::Point p0;
  for(int i = 0; i < pathin.poses.size(); i++){
    float val = 0;
    if(type == "dst_2d0")
      val = get_dst2d(pnt_ref,pathin.poses[i].pose.position);
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

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
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

void draw_path_at_img(nav_msgs::Path pathin,geometry_msgs::Point p0,
	 bool path_line,bool pose_yawline,bool pose_rectangle,bool pose_circle,bool pose_pnt,
  cv::Scalar color, int pose_size)
  {
	geometry_msgs::Point pyaw,prect0,prect1,pnt;
  if(pathin.poses.size() < 1)
    return;
	if(p0.x == 0 && p0.y == 0)
		p0 = pathin.poses[0].pose.position;

	for(int i = 0; i < pathin.poses.size(); i++){
		pnt = pathin.poses[i].pose.position;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
		if(path_line)
			cv::line (img,  pnt2cv(p0), pnt2cv(pnt),color,1,cv::LINE_8,0);
		p0 = pnt;
		if(pose_yawline){
			pyaw.x = pnt.x + pose_size*2 * cos(yaw);
			pyaw.y = pnt.y + pose_size*2 * sin(yaw);
			cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
		}
		if(pose_rectangle){
			prect0.x = pnt.x-pose_size;
			prect0.y = pnt.y-pose_size;
			prect1.x = pnt.x+pose_size*2;
			prect1.y = pnt.y+pose_size*2;
			cv::rectangle(img, pnt2cv(prect0),pnt2cv(prect1),color,1,8,0);
		}
		if(pose_circle){
			cv::circle(img,pnt2cv(pnt),pose_size,color,1);
		}
		if(pose_pnt){
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
		}
	}
}

void draw_paths_by_scores(std::vector<nav_msgs::Path> pathsin,std::vector<float> scores){
  float scores_max = vec_to_max(scores);
  float scores_min = vec_to_min(scores);
  for(int i = 0; i < pathsin.size(); i++){
    float rel_score  = (scores[i] - scores_min) / (scores_max-scores_min);
    std::vector<float> vvz = get_vec_attribute(pathsin[i],"z");
    if(rel_score == 1.0){
      draw_path_by_score(pathsin[i],vvz,2,1,0,rel_score);
    }
    else if(rel_score > 0.66){
      draw_path_by_score(pathsin[i],vvz,1,1,1,rel_score);
    }
    else if(rel_score > 0.33){
      draw_path_by_score(pathsin[i],vvz,2,2,2,rel_score);
    }
    else{
      draw_path_by_score(pathsin[i],vvz,0,0,0,rel_score*2);
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
void drawimg(std::string name){
  bool draw_stuff = true;
  count_target_paths++;


  geometry_msgs::Point pyaw,pyaw_mn,pyaw_mx,p0;
  float a1 = constrainAngle(vlp_rpy.z + par_hdngcutoff);
  float a2 = constrainAngle(vlp_rpy.z - par_hdngcutoff);
  float a0 = fmin(a1,a2);
  float aN = fmax(a1,a2);
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  draw_line(pos,vlp_rpy.z,100,get_color(100,100,0));
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  draw_line(p0,current_frontier_radians,100,get_color(0,0,200));
  cv::imwrite("/home/nuc/brain/ctrl/ctrl_" + name +"_"+std::to_string(count_target_paths)+ ".png",img);
  img_blank.copyTo(img);

  if(draw_stuff){
    std::vector<float> v2 = get_vec_attribute(path_surround,"z");
    std::vector<float> v3 = get_vec_attribute(path_forward,"z");
    std::vector<float> vr = get_vec_attribute(path_right,"z");
    std::vector<float> vl = get_vec_attribute(path_left,"z");
    draw_path_by_score(path_surround,v2,1,1,1,0.3);
    draw_path_by_score(path_right,vr,0,0,0,0.5);
    draw_path_by_score(path_left,vl,0,0,0,0.5);
    draw_path_by_score(path_forward,v3,1,1,1,0.5);

    draw_rectangle(frontier_outofsight_infront.point,3,get_color(250,250,200));
    draw_rectangle(frontier_minpnt.point,3,get_color(150,25,65));
    draw_rectangle(frontier_minpnt_infront.point,3,get_color(150,25,65));
    draw_rectangle(frontier_closest_infront.point,3,get_color(150,25,65));
    draw_rectangle(target_frontier.pose.position,3,get_color(0,0,200));
    cv::circle(img,pnt2cv(scanpoint_ave.point),2,get_color(200,0,200),1);
    cv::circle(img,pnt2cv(scanpoint_mid.point),2,get_color(200,200,200),1);
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
      if(hdng >= val_0 && hdng <= val_N)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "hdng_rel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng     = get_hdng(pathin.poses[i].pose.position,pos);
      float hdng_rel = get_shortest(hdng,vlp_rpy.z);
      if(hdng_rel >= val_0 && hdng_rel <= val_N)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "zrel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      if((pathin.poses[i].pose.position.z - pos.z) >= val_0 && (pathin.poses[i].pose.position.z - pos.z) <= val_N)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "z"){
    for(int i = 0; i < pathin.poses.size(); i++){
      if(pathin.poses[i].pose.position.z >= val_0 && pathin.poses[i].pose.position.z <= val_N)
        pathout.poses.push_back(pathin.poses[i]);
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
  /*  tf2::Matrix3x3 q(tf2::Quaternion(pathin.poses[i].pose.orientation.x, pathin.poses[i].pose.orientation.y, pathin.poses[i].pose.orientation.z, pathin.poses[i].pose.orientation.w));
    geometry_msgs::Vector3 vv;
    q.getRPY(vv.x,vv.y,vv.z);
    ROS_INFO("%.2f %.2f %.2f",vv.x,vv.y,vv.z);*/
  }
  bbmnbbmx.push_back(bbtotmin);
  bbmnbbmx.push_back(bbtotmax);

  float diag = sqrt(pow(bbtotmax.x-bbtotmin.x,2)+pow(bbtotmax.y-bbtotmin.y,2)+pow(bbtotmax.z-bbtotmin.z,2));
  ROS_INFO("GENERICNODE: LIMITS: diagonal: %.2f,max(%.0f %.0f %.0f) min(%.0f %.0f %.0f)",diag,bbtotmax.x,bbtotmax.y,bbtotmax.z,bbtotmin.x,bbtotmin.y,bbtotmin.z);
  return bbmnbbmx;
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
float getinpath_closestdst2d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pnt);
    if(dst < lowest_dist){
      lowest_dist = dst;
    }
  }
  return lowest_dist;
}
float get_closest_dst_between_p1p2(geometry_msgs::Point p2, geometry_msgs::Point p1){
  Eigen::Vector3f pnt2(p2.x,p2.y,p2.z);
  Eigen::Vector3f pnt1(p1.x,p1.y,p1.z);
  Eigen::Vector3f cur_vec(p1.x,p1.y,p1.z);
  float tot_ray_len = (pnt2 - pnt1).norm();
  float cur_ray_len = 0;
  Eigen::Vector3f stride_vec = (pnt2 - pnt1).normalized() * 3;
  geometry_msgs::Point midpoint;
  float closest_dst = 100000;
  while(cur_ray_len < tot_ray_len){
    cur_ray_len = (cur_vec - pnt1).norm();
    cur_vec = cur_vec + stride_vec;
    midpoint.x = cur_vec.x();
    midpoint.y = cur_vec.y();
    midpoint.z = cur_vec.z();
    float closestdst = getinpath_closestdst2d(path_above_pos,midpoint);
  //  ROS_INFO("p0: %.0f %.0f %.0f CLosest: %.2f ",p2.x,p2.y,p2.z,closestdst);
    if(closest_dst > closestdst)
      closest_dst = closestdst;
  }
  return closest_dst;
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
nav_msgs::Path get_path_inrad3d(nav_msgs::Path pathin,geometry_msgs::Point midpoint,float maxrad){
  nav_msgs::Path pathout;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst3d(midpoint,pathin.poses[i].pose.position) < maxrad){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
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
nav_msgs::Path get_new_path(nav_msgs::Path path_old,nav_msgs::Path pathin,float cutoff_dst){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(path_old,pathin.poses[i].pose.position,cutoff_dst))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

nav_msgs::Path get_path_inrad(nav_msgs::Path path_old,geometry_msgs::Point midpoint,float maxrad){
  nav_msgs::Path pathout;
  for(int i = 0; i < path_old.poses.size(); i++){
    if(get_dst2d(midpoint,path_old.poses[i].pose.position) < maxrad){
      pathout.poses.push_back(path_old.poses[i]);
    }
  }
  return pathout;
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

nav_msgs::Path remove_visited(nav_msgs::Path pathin, float poses_spacing){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(path_visited,pathin.poses[i].pose.position,poses_spacing)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}

void heightpaths_cb(const tb_msgsrv::Paths::ConstPtr& msg){
  heightpaths = *msg;
}

void path_scanzone_cb(const nav_msgs::Path::ConstPtr& msg){
  path_scanzone = *msg;
}
void path_surround_cb(const nav_msgs::Path::ConstPtr& msg){
  path_surround = *msg;
}
void path_forward_cb(const nav_msgs::Path::ConstPtr& msg){
  path_forward = *msg;
}
void path_right_cb(const nav_msgs::Path::ConstPtr& msg){
  path_right = *msg;
}
void path_left_cb(const nav_msgs::Path::ConstPtr& msg){
  path_left = *msg;
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}
void frontier_cb(const nav_msgs::Path::ConstPtr& msg){
  path_frontier = *msg;
}

void frontier_outofsight_infront_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  frontier_outofsight_infront = *msg;
}
void frontier_minpnt_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  frontier_minpnt = *msg;
}
void scanpoint_ave_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  scanpoint_ave = *msg;
}
void scanpoint_mid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  scanpoint_mid = *msg;
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
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


void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
 if(new_target_pose.pose.position.x == 0 && new_target_pose.pose.position.y == 0){
   ROS_INFO("TARGETZERO WARNING");
 }
 else{
    target_last = target;
  	target.pose.position = new_target_pose.pose.position;
  	target.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(new_target_pose.pose.position,target_last.pose.position));
    new_target_pose = target;
    new_target_pose.pose.position.z = 0;
    new_target_pose.header = hdr();
    target.header = hdr();
    path_targets.poses.push_back(new_target_pose);
    if(par_usemovebase)
      pub_cmdpose.publish(new_target_pose);
    else
      pub_cmd.publish(new_target_pose.pose.position);
  }
}
void checkpath(nav_msgs::Path pathin){
  for(int i = 1; i < pathin.poses.size(); i++){
    if( std::isnan(pathin.poses[i].pose.position.x) || std::isnan(pathin.poses[i].pose.position.y) || std::isnan(pathin.poses[i].pose.position.z)
    || std::isnan(pathin.poses[i].pose.orientation.x) || std::isnan(pathin.poses[i].pose.orientation.y) || std::isnan(pathin.poses[i].pose.orientation.z) || std::isnan(pathin.poses[i].pose.orientation.w)){
      ROS_INFO("NANWARNING %.2f %.2f %.2f %.2f %.2f %.2f %.2f",pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y,pathin.poses[i].pose.position.z,
      pathin.poses[i].pose.orientation.x,pathin.poses[i].pose.orientation.y,pathin.poses[i].pose.orientation.z,pathin.poses[i].pose.orientation.w);
    }
  }
}
geometry_msgs::Quaternion get_or(int n){
  geometry_msgs::Quaternion q;
  if(n == 1){
    q.x = 0; q.y = 0.7071; q.z = 0; q.w = 0.7071;
  }
  else if(n == 3){
    q.x = 0; q.y = 0; q.z = 0; q.w = 1.0;
  }
  else{
    q = tf::createQuaternionMsgFromYaw(deg2rad*n);
  }
  return q;
}
nav_msgs::Path get_heightpath_practical(nav_msgs::Path pathin){
  geometry_msgs::PoseStamped ps;
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1){
    return pathin;
  }
  float dst2d = get_dst2d(pathin.poses[0].pose.position,pos);
  if(dst2d < 5){
    ps = pathin.poses[1];
  }
  int deg = round(rad2deg * get_hdng(pathin.poses[1].pose.position,pos));

  pathout.header =hdr();
  ps.pose.orientation = get_or(deg);
  pathout.poses.push_back(ps);
  bool above_pos = false;
  for(int i = 1; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > ps.pose.position.z){
      if(pathin.poses[i].pose.position.z > pos.z){
        if(!above_pos){
          ps = pathin.poses[i-1];
          ps.pose.position.z = pos.z;
          above_pos = true;
      //    ps.pose.orientation.y = 0.7071;
      //    ps.pose.orientation.w = 0.7071;
          ps.pose.orientation = get_or(1);
          pathout.poses.push_back(ps);
        }
      }
      else{
        ps = pathin.poses[i-1];
        ps.pose.position.z = pathin.poses[i].pose.position.z;
        ps.pose.orientation = get_or(deg);
        pathout.poses.push_back(ps);
      }
    }
    else if(get_dst2d(pathin.poses[i].pose.position,ps.pose.position) > 150){
      float zv = fmax(pathin.poses[i-1].pose.position.z,pathin.poses[i].pose.position.z);
    //  if(i < pathin.poses.)
  //    float zv = fmax(pathin.poses[i+1].pose.position.z,pathin.poses[i].pose.position.z);
      ps = pathin.poses[i];
      ps.pose.position.z = zv;
    }
  }
  if(!above_pos){
    ps = pathin.poses[pathin.poses.size()-2];
    ps.pose.orientation = get_or(3);
    pathout.poses.push_back(ps);
  }
//  ROS_INFO("Returns pathout: %i (pathin: %i)",pathout.poses.size(),pathin.poses.size());
  return pathout;
}
nav_msgs::Path get_path_above_pos(nav_msgs::Path pathin){
  return cutoff_abs(pathin,"z",pos.z-1,pos.z+100);
}
void create_all_points_in_bbox(nav_msgs::Path pathin){
  geometry_msgs::Point bbmn,bbmx;
  std::vector<geometry_msgs::Point> bbmnmx;
  bbmnmx = getinpath_boundingbox(pathin);
  bbmn = bbmnmx[0];
  bbmx = bbmnmx[1];
  ROS_INFO("BBOX: %.0f %.0f %.0f -> %.0f %.0f %.0f",bbmn.x,bbmn.y,bbmn.z,bbmx.x,bbmx.y,bbmx.z);

  for(int x = 0; x < 100; x++){
    for(int y = 0; y < 100; y++){

    }
  }
}
void update_frontier_target(){
  int best_i = 0;
  float closest_rads = 100;
  geometry_msgs::Point p0;
  for(int i = 0; i < path_frontier.poses.size(); i++){
    float frontier_radians = atan2(path_frontier.poses[i].pose.position.y,path_frontier.poses[i].pose.position.x);
    float frontier_radians_d = get_shortest(frontier_radians,current_frontier_radians);
    float frontier_dst = get_dst2d(path_frontier.poses[i].pose.position,pos);
    float frontier_dst0 = get_dst2d(path_frontier.poses[i].pose.position,p0);
  //  ROS_INFO("frontier[%i]: %.2f rads/ %.0f m_pos / %.0f m_start (%.2f ahead of current: %.2f rads)" ,i,frontier_radians,frontier_dst,frontier_dst0,frontier_radians_d,current_frontier_radians);
    if(frontier_radians_d < 0)
      frontier_radians_d *= -1;
    if(frontier_radians_d < closest_rads){
      closest_rads = frontier_radians_d;
      best_i = i;
    }
  }
  int lead_position = best_i + lead_num;
  if(lead_position >= path_frontier.poses.size())
    lead_position -= path_frontier.poses.size();
  ROS_INFO("Frontier[%i] + %i lead intervals: current lead: %i",best_i,lead_num,lead_position);
  target_frontier = path_frontier.poses[lead_position];
  float dst_target     = get_dst2d(target.pose.position,pos);
  if(dst_target < 10)
    set_target_pose(target_frontier);
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
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
  current_frontier_radians = atan2(pos.y,pos.x);
  //q.getRPY(vlp_rpy.x,-vlp_rpy.y,vlp_rpy.z);
}
std::vector<float> get_path_score(nav_msgs::Path pathin,float weight_hdng,float weight_z, float weight_dst2d, float weight_inclination,float weight_startdst, float weight_closest){
  std::vector<float> vec_z = get_vec_attribute(pathin,"z");
  std::vector<float> vec_d = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> vec_0 = get_vec_attribute(pathin,"dst_2d0");
  std::vector<float> vec_h = get_vec_attribute(pathin,"hdng_abs");
  std::vector<float> vec_i = get_vec_attribute(pathin,"inclination");
  std::vector<float> vec_c;
  vec_c.resize(pathin.poses.size());
  for(int i = 0; i < pathin.poses.size(); i++){
    vec_c[i] = get_closest_dst_between_p1p2(pathin.poses[i].pose.position,pos);
  }
  std::vector<float> mma_c = vec_to_min_max_ave(vec_c);
  std::vector<float> mma_z = vec_to_min_max_ave(vec_z);
  std::vector<float> mma_d = vec_to_min_max_ave(vec_d);
  std::vector<float> mma_h = vec_to_min_max_ave(vec_h);
  std::vector<float> mma_i = vec_to_min_max_ave(vec_i);
  std::vector<float> mma_0 = vec_to_min_max_ave(vec_0);

  std::vector<float> scores;
  std::vector<float> scores_rel;
  for(int i = 0; i < pathin.poses.size(); i++){
    float rel_c  = (vec_c[i] - mma_c[0]) / (mma_c[1]-mma_c[0]);
    float rel_z  = (vec_z[i] - mma_z[0]) / (mma_z[1]-mma_z[0]);
    float rel_d  = (vec_d[i] - mma_d[0]) / (mma_d[1]-mma_d[0]);
    float rel_h  = (vec_h[i] - mma_h[0]) / (mma_h[1]-mma_h[0]);
    float rel_i  = (vec_i[i] - mma_i[0]) / (mma_i[1]-mma_i[0]);
    float rel_0  = (vec_0[i] - mma_0[0]) / (mma_0[1]-mma_0[0]);
    float score = weight_z * rel_z + weight_dst2d * rel_d + weight_hdng * rel_h + rel_i * weight_inclination + rel_0 * weight_startdst + rel_c * weight_closest;
    scores.push_back(score);
    //ROS_INFO("Score: %.2f rel_c,%.2f rel_z,%.2f rel_d,%.2f rel_h,%.2f rel_i,%.2f rel_0,%.2f",score,rel_c,rel_z,rel_d,rel_h,rel_i,rel_0);
  }
  scores_rel.resize(scores.size());
  std::vector<float> mma_s = vec_to_min_max_ave(scores);
  for(int i = 0; i < scores.size(); i++){
    scores_rel[i]  = (scores[i] - mma_s[0]) / (mma_s[1]-mma_s[0]);
    //ROS_INFO("scores_rel[%i]: %.2f ",i, scores_rel[i]);
  }
  return scores_rel;
}

nav_msgs::Path heightpaths_to_heightpath_practical(tb_msgsrv::Paths pathsin){
  std::vector<nav_msgs::Path> pathsout;
  pathsout.resize(heightpaths.paths.size());
  for(int i=0; i < heightpaths.paths.size(); i++){
    pathsout[i] = get_heightpath_practical(heightpaths.paths[i]);
  }
  return merge_paths(pathsout);
}
geometry_msgs::PoseStamped draw_and_get_best(nav_msgs::Path pathin,std::vector<float>scores){
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();
  pnt_ref = frontier_outofsight_infront.point;

  if(pathin.poses.size() == 0)
    return ps;
  int best_i = 0;
  for(int i= 0; i < pathin.poses.size(); i++){
    float d2d = get_dst2d(pos,pathin.poses[i].pose.position);
    float in = get_inclination(pathin.poses[i].pose.position,pos);
    float h = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
    float z = pathin.poses[i].pose.position.z;
    if(scores[i] < 0.05 || scores[i] > 0.95){
      if(scores[i] > 0.5)
        cv::circle(img,pnt2cv(pathin.poses[i].pose.position),5,get_color(0,0,200),1);
      else
        cv::circle(img,pnt2cv(pathin.poses[i].pose.position),5,get_color(0,200,0),1);
    }
    if(scores[i] == 1.00){
      cv::circle(img,pnt2cv(pathin.poses[i].pose.position),5,get_color(255,255,255),1);
      best_i = i;
    }
  }
  return pathin.poses[best_i];
}
geometry_msgs::PoseStamped get_best(nav_msgs::Path pathin,std::vector<float>scores){
  for(int i= 0; i < pathin.poses.size(); i++){
    if(scores[i] == 1.00)
      return pathin.poses[i];
  }
  ROS_INFO("ERROR _ GET BEST DIDINT FIND BEST");
  if(pathin.poses.size() == 0){
    geometry_msgs::PoseStamped ps;
    return ps;
  }
  else
    return pathin.poses[0];
}
geometry_msgs::PoseStamped get_next_target(){
  float weight_z = -2.0;
  float weight_dst2d = 0.5;
  float weight_inclination = 0.0;
  float weight_startdst = -2.0;
  float weight_closest  = 1.0;
  float weight_hdng     = -5.0;
  geometry_msgs::PoseStamped ps;
  std::vector<float> scores = get_path_score(heightpath_practical,weight_hdng,weight_z,weight_dst2d,weight_inclination,weight_startdst,weight_closest);
  bool draw = true;
  if(draw)
    ps = draw_and_get_best(heightpath_practical,scores);
  else
    ps = get_best(heightpath_practical,scores);
  return ps;
}

float get_tilt(bool look_down,bool look_up,bool look_forw){
  float desired_tilt = M_PI/14;
  if(look_forw)
    return 0;
  else if(look_down)
    return arm1_tilt_msg.data - 0.5;
  else if(look_up)
    return arm1_tilt_msg.data + 0.5;
  else
    return M_PI/14 - rpy_vel.y;
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data != mainstate){
    ROS_INFO("MAInstate change: %i -> %i",mainstate,msg->data);
    mainstate = msg->data;
  }
}

void global_plan_cb(const nav_msgs::Path::ConstPtr& msg){
  float global_plan_dst  = 0;
  if(msg->poses.size() > 2){
    global_plan_dst = get_dst2d(msg->poses[0].pose.position,msg->poses[msg->poses.size()-1].pose.position);
    global_plan_scattered = scatter_path(*msg,3);
    global_plan_scattered.poses.push_back(msg->poses[msg->poses.size()-1]);
  }
  ROS_INFO("GlobalPlan received: %i poses, %.2f meters reduced to %i poses",msg->poses.size(),global_plan_dst,global_plan_scattered);
  pub_path_elevation_request.publish(global_plan_scattered);
//  pub_global_plan_scattered.publish(global_plan_scattered);
}

void local_plan_cb(const nav_msgs::Path::ConstPtr& msg){
  float local_plan_dst  = 0;
  if(msg->poses.size() > 2){
    local_plan_dst = get_dst2d(msg->poses[0].pose.position,msg->poses[msg->poses.size()-1].pose.position);
    local_plan_scattered = scatter_path(*msg,3);
    local_plan_scattered.poses.push_back(msg->poses[msg->poses.size()-1]);
  }
  ROS_INFO("LocalPlan received: %i poses, %.2f meters reduced to %i poses",msg->poses.size(),local_plan_dst,local_plan_scattered);
  pub_local_plan_scattered.publish(local_plan_scattered);
}
void path_elevated_cb(const nav_msgs::Path::ConstPtr& msg){
  path_elevated = *msg;
}
void path_pos_cb(const nav_msgs::Path::ConstPtr& msg){
  path_pos = *msg;
}
void mbfeedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){
//  last_feedback = ros::Time::now();
//  motionstate = 2;
}
void mbres_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  //  getpath_within_rad(centroid_sides*2);

  ROS_INFO("MoveBaseRes; %i",msg->status.status);

  if(msg->status.status == 3){
    ROS_INFO("Target complete");
  }
  if(msg->status.status == 4){
    lead_num++;

    if(lead_num > 20)
      lead_num = 0;
    ROS_INFO("Target complete");
  }
  /*
  uint8 PENDING=0
uint8 ACTIVE=1
uint8 PREEMPTED=2
uint8 SUCCEEDED=3
uint8 ABORTED=4
uint8 REJECTED=5
uint8 PREEMPTING=6
uint8 RECALLING=7
uint8 RECALLED=8*/
//  uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
    //                          #    to some failure (Terminal State)
}


//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_ctrl_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("heading_cutoff",par_hdngcutoff, M_PI/12);
  private_nh.param("heading_cutoff",par_zclearing,5.0);
  private_nh.param("use_movebase", par_usemovebase, true);//*2.0);

	tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Subscriber s0 = nh.subscribe("/tb_fsm/main_state",10,mainstate_cb);
  ros::Subscriber s1 = nh.subscribe("/tb_world/frontier_outofsight_infront",10,frontier_outofsight_infront_cb);
  ros::Subscriber s2 = nh.subscribe("/tb_world/frontier_minpnt",10,frontier_minpnt_cb);
  ros::Subscriber s3 = nh.subscribe("/tb_world/scanpoint_ave",10,scanpoint_ave_cb);
  ros::Subscriber s4 = nh.subscribe("/tb_world/scanpoint_mid",10,scanpoint_mid_cb);
  ros::Subscriber s6 = nh.subscribe("/tb_world/path_scanzone",10,path_scanzone_cb);
  ros::Subscriber s7 = nh.subscribe("/tb_world/path_surround",10,path_surround_cb);
  ros::Subscriber s8 = nh.subscribe("/tb_world/path_forward",10,path_forward_cb);
  ros::Subscriber s9 = nh.subscribe("/tb_world/path_right",10,path_right_cb);
  ros::Subscriber s10 = nh.subscribe("/tb_world/path_left",10,path_left_cb);
  ros::Subscriber s13 = nh.subscribe("/tb_world/heightpaths",10,heightpaths_cb);
  ros::Subscriber s11 = nh.subscribe("/tb_world/path_visited",10,pathvstd_cb);
  ros::Subscriber s14 = nh.subscribe("/tb_world/path_frontier",10,frontier_cb);
  ros::Subscriber s16 = nh.subscribe("/tb_world/path_elevated",10,path_elevated_cb);
  ros::Subscriber s17 = nh.subscribe("/tb_world/path_pos",10,path_pos_cb);
  ros::Subscriber s12 = nh.subscribe("/odom_global",10,odom_cb);
  pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  pub_cmdpose       	= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/posemb",10);
  pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);
  pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);
  ros::Subscriber smbb = nh.subscribe("/move_base/feedback",  100,&mbfeedback_cb);
  ros::Subscriber sb = nh.subscribe("/move_base/result",  100,&mbres_cb);
  ros::Subscriber sm1 = nh.subscribe("/move_base/DWAPlannerROS/global_plan",  100,&global_plan_cb);
  //ros::Subscriber sm = nh.subscribe("/move_base/DWAPlannerROS/local_plan",  100,&local_plan_cb);
  pub_local_plan_scattered	 = nh.advertise<nav_msgs::Path>("/tb_ctrl/local_plan_scattered",10);
  pub_global_plan_scattered	 = nh.advertise<nav_msgs::Path>("/tb_ctrl/global_plan_scattered",10);
  pub_path_elevation_request = nh.advertise<nav_msgs::Path>("/tb_world/request_path_elevated",10);

  ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
  ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
  ros::Publisher pub_path_targets = nh.advertise<nav_msgs::Path>("/tb_targetpath", 10);
  ros::Publisher pub_target   	= nh.advertise<geometry_msgs::PoseStamped>("/tb_target",10);
  ros::Publisher pub_forward    = nh.advertise<geometry_msgs::PointStamped>("/tb_pnt/forward",10);
  pub_path	 = nh.advertise<nav_msgs::Path>("/path_test",10);
  ros::Rate rate(1.0);
  int count_i = 0;int count_k = 0;
  float setpoint_altitude = 15;
  target.pose.position.z = 15;
  float target_angle = 0;
  ros::Time start = ros::Time::now();
  while(ros::ok()){

    rate.sleep();
    ros::spinOnce();
    checktf();
    process_odom();
    heightpath     = merge_tbpaths(heightpaths);
    path_above_pos = get_path_above_pos(heightpath);
    heightpath_practical = heightpaths_to_heightpath_practical(heightpaths);
  //  heightpath_practical = remove_visited(heightpath_practical,8);
    heightpath_practical = remove_visited(heightpath,8);
  //  heightpath_practical = cutoff_abs(heightpath_practical,"hdng_rel",vlp_rpy.z-M_PI/3,vlp_rpy.z+M_PI/3);
  //  draw_path_at_img(heightpath,pos,false,false,false,false,true,get_color(100,100,100),1);
    draw_path_at_img(path_above_pos,pos,false,false,false,false,true,get_color(0,0,100),1);
    draw_path_at_img(heightpath_practical,pos,false,false,true,false,false,get_color(200,200,0),1);
  //  create_all_points_in_bbox(heightpath_practical);
//    ROS_INFO("MErgedheightpaths: %i -> %i",heightpath.poses.size(),heightpath_practical.poses.size());'
    float dt_start = (ros::Time::now() - start).toSec();

    drawimg("ctrl_"+ std::to_string(count_target_paths) );
    geometry_msgs::PoseStamped ps;
    ps = get_next_target();
    float dst_target     = get_dst3d(target.pose.position,pos);
    float dst_new_target = get_dst3d(ps.pose.position,pos);
    ROS_INFO("Current target: %.0f %.0f %.0f dst: %.0f NEW: %.0f %.0f %.0f dst: %.0f",target.pose.position.x,target.pose.position.y,target.pose.position.z,dst_target,ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,dst_new_target);

  //  if(dst_target < 10)
  //    set_target_pose(ps);
  //target.pose.position.z
    if(path_elevated.poses.size() > 0){
      //int closest_i = getinpath_closestindex3d(path_elevated,pos);

    }
    //else
    //  target_alt_msg.data = get_zmax(path_forward) + par_zclearing;
    target_alt_msg.data = fmax(get_zmax(path_pos),get_zmax(path_elevated)) + par_zclearing;

    if(mainstate == 0){
      target_alt_msg.data = 15;
    }
    if(dt_start > 8 && mainstate == 1){
      update_frontier_target();
    }
    if(mainstate == 2){
      target.pose.position.x = 0;
      target.pose.position.y = 0;
      target_alt_msg.data = 15;
      set_target_pose(target);
    }
    if(mainstate == 3){
      target.pose.position.x = 0;
      target.pose.position.y = 0;
      target_alt_msg.data = 5;
    }

    arm1_tilt_msg.data = get_tilt(false,false,false);
    pub_path.publish(heightpath_practical);
    pub_altcmd.publish(target_alt_msg);
    pub_tiltvlp.publish(arm1_tilt_msg);
    pub_forward.publish(forward_point);
    pub_path_targets.publish(path_targets);
    pub_target.publish(target);

  }
  return 0;
}
