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
double par_hdngcutoff;
int counter = 0;
float rad2deg = 180.0/M_PI;
float deg2rad = M_PI/180;

tb_msgsrv::Paths heightpaths;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
int count_target_paths = 0;
nav_msgs::Path path_above_pos,heightpath,path_scanzone,path_surround,path_forward,path_right,path_left,path_visited;
ros::Publisher pub_path,pub_path2;
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


  geometry_msgs::Point pyaw,pyaw_mn,pyaw_mx;
  float a1 = constrainAngle(vlp_rpy.z + par_hdngcutoff);
  float a2 = constrainAngle(vlp_rpy.z - par_hdngcutoff);
  float a0 = fmin(a1,a2);
  float aN = fmax(a1,a2);
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  draw_line(pos,vlp_rpy.z,100,get_color(100,100,0));
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
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
  //q.getRPY(vlp_rpy.x,-vlp_rpy.y,vlp_rpy.z);
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
    ROS_INFO("p0: %.0f %.0f %.0f CLosest: %.2f ",p2.x,p2.y,p2.z,closestdst);
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


std::vector<float> get_path_score2(tb_msgsrv::Paths pathin_segments, nav_msgs::Path pathin, std::vector<float> weight){
  std::vector<float> vec_z = get_vec_attribute(pathin,"z");
  std::vector<float> vec_d = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> vec_0 = get_vec_attribute(pathin,"dst_2d0");
  std::vector<float> vec_h = get_vec_attribute(pathin,"hdng_abs");
  std::vector<float> vec_i = get_vec_attribute(pathin,"inclination");
  std::vector<float> mma_z = vec_to_min_max_ave(vec_z);
  std::vector<float> mma_d = vec_to_min_max_ave(vec_d);
  std::vector<float> mma_h = vec_to_min_max_ave(vec_h);
  std::vector<float> mma_i = vec_to_min_max_ave(vec_i);
  std::vector<float> mma_0 = vec_to_min_max_ave(vec_0);
  std::vector<float> scores;
  std::vector<float> scores_rel;

  scores.resize(pathin_segments.paths.size());
  for(int i = 0; i < pathin_segments.paths.size(); i++){
    std::vector<float> vec_z_seg = get_vec_attribute(pathin_segments.paths[i],"z");
    std::vector<float> vec_d_seg = get_vec_attribute(pathin_segments.paths[i],"dst_2d");
    std::vector<float> vec_0_seg = get_vec_attribute(pathin_segments.paths[i],"dst_2d0");
    std::vector<float> vec_h_seg = get_vec_attribute(pathin_segments.paths[i],"hdng_abs");
    std::vector<float> vec_i_seg = get_vec_attribute(pathin_segments.paths[i],"inclination");

    std::vector<float> mma_z_seg = vec_to_min_max_ave(vec_z_seg);
    std::vector<float> mma_d_seg = vec_to_min_max_ave(vec_d_seg);
    std::vector<float> mma_h_seg = vec_to_min_max_ave(vec_h_seg);
    std::vector<float> mma_i_seg = vec_to_min_max_ave(vec_i_seg);
    std::vector<float> mma_0_seg = vec_to_min_max_ave(vec_0_seg);

    float rel_z_min = (mma_z_seg[0] - mma_z[0]) / (mma_z[1]-mma_z[0]); //weight_z_min
    float rel_z_max = (mma_z_seg[1] - mma_z[0]) / (mma_z[1]-mma_z[0]); //weight_z_max
    float rel_z_ave = (mma_z_seg[2] - mma_z[0]) / (mma_z[1]-mma_z[0]); //weight_z_ave

    float rel_i_min = (mma_i_seg[0] - mma_i[0]) / (mma_i[1]-mma_i[0]); //weight_i_min
    float rel_i_max = (mma_i_seg[1] - mma_i[0]) / (mma_i[1]-mma_i[0]); //weight_i_max
    float rel_i_ave = (mma_i_seg[2] - mma_i[0]) / (mma_i[1]-mma_i[0]); //weight_i_ave

    float rel_d_min = (mma_d_seg[0] - mma_d[0]) / (mma_d[1]-mma_d[0]); //weight_d_min
    float rel_d_max = (mma_d_seg[1] - mma_d[0]) / (mma_d[1]-mma_d[0]); //weight_d_max
    float rel_d_ave = (mma_d_seg[2] - mma_d[0]) / (mma_d[1]-mma_d[0]); //weight_d_ave

    float rel_h_min = (mma_h_seg[0] - mma_h[0]) / (mma_h[1]-mma_h[0]); //weight_h_min
    float rel_h_max = (mma_h_seg[1] - mma_h[0]) / (mma_h[1]-mma_h[0]); //weight_h_max
    float rel_h_ave = (mma_h_seg[2] - mma_h[0]) / (mma_h[1]-mma_h[0]); //weight_h_ave

    float rel_0_min = (mma_0_seg[0] - mma_0[0]) / (mma_0[1]-mma_0[0]); //weight_0_min
    float rel_0_max = (mma_0_seg[1] - mma_0[0]) / (mma_0[1]-mma_0[0]); //weight_0_max
    float rel_0_ave = (mma_0_seg[2] - mma_0[0]) / (mma_0[1]-mma_0[0]); //weight_0_ave

    rel_h_min = saturate(rel_h_min,100);
    rel_h_max = saturate(rel_h_max,100);
    rel_h_ave = saturate(rel_h_ave,100);
   scores[i] = rel_z_min * weight[0] +rel_z_max * weight[1] +rel_z_ave * weight[2] +rel_i_min * weight[3] +rel_i_max * weight[4]
  +rel_i_ave * weight[5] +rel_d_min * weight[6] +rel_d_max * weight[7] +rel_d_ave * weight[8] +rel_h_min * weight[9]
  +rel_h_max * weight[10]+rel_h_ave * weight[11]+rel_0_min * weight[12]+rel_0_max * weight[13]+rel_0_ave * weight[14];
  //ROS_INFO("scores[%i]: %.2f rel_z_min: %.2f,rel_z_max: %.2f,rel_z_ave: %.2f,rel_i_min: %.2f,rel_i_max: %.2f,rel_i_ave: %.2f,rel_d_min: %.2f,rel_d_max: %.2f,rel_d_ave: %.2f,rel_h_min: %.2f,rel_h_max: %.2f,rel_h_ave: %.2f,rel_0_min: %.2f,rel_0_max: %.2f,rel_0_ave: %.2f",i,scores[i],rel_z_min,rel_z_max,rel_z_ave,rel_i_min,rel_i_max,rel_i_ave,rel_d_min,rel_d_max,rel_d_ave,rel_h_min,rel_h_max,rel_h_ave,rel_0_min,rel_0_max,rel_0_ave);
  }

  scores_rel.resize(scores.size());
  std::vector<float> mma_s = vec_to_min_max_ave(scores);
  for(int i = 0; i < scores.size(); i++){
    scores_rel[i]  = (scores[i] - mma_s[0]) / (mma_s[1]-mma_s[0]);
  //  ROS_INFO("SCORE %i %.2f - rel: %.2f  (%.2f - %.2f)",i,scores[i],scores_rel[i],mma_s[0],mma_s[1]);
  }
  return scores;
}
std::vector<float> get_weights(std::string z,std::string d,std::string h, std::string i, std::string d0){
  float rel_z_min = 0;float rel_z_max = 0;float rel_z_ave = 0;float rel_i_min = 0;float rel_i_max = 0;
  float rel_i_ave = 0;float rel_d_min = 0;float rel_d_max = 0;float rel_d_ave = 0;float rel_h_min = 0;
  float rel_h_max = 0;float rel_h_ave = 0;float rel_0_min = 0;float rel_0_max = 0;float rel_0_ave = 0;

  if(z == "lowest_all"){
    rel_z_min = -1;
    rel_z_max = -1;
    rel_z_ave = -1;
  }
  if(z == "highest"){
    rel_z_max = 1;
  }
  if(i == "climb"){
    rel_i_max = 1.0;
  }
  if(i == "descend"){
    rel_i_min = -1.0;
  }
  if(d == "closest"){
    rel_d_max = -1.0;
    rel_d_ave = -1.0;
  }
  if(d == "farthest"){
    rel_d_max = 1.0;
    rel_d_ave = 1.0;
  }
  if(d0 == "startclose"){
    rel_0_ave = -1.0;
  }
  if(d0 == "startfar"){
    rel_0_ave = 1.0;
  }
  if(h == "straightest"){
    rel_h_ave = -1.0;
  }
  if(h == "startfar"){
    rel_0_ave = 1.0;
  }

std::vector<float> weight;
weight.resize(15);
 weight[0] = rel_z_min;
 weight[1] = rel_z_max;
 weight[2] = rel_z_ave;
 weight[3] = rel_i_min;
 weight[4] = rel_i_max;
 weight[5] = rel_i_ave;
 weight[6] = rel_d_min;
 weight[7] = rel_d_max;
 weight[8] = rel_d_ave;
 weight[9] = rel_h_min;
 weight[10] = rel_h_max;
 weight[11] = rel_h_ave;
 weight[12] = rel_0_min;
 weight[13] = rel_0_max;
 weight[14] = rel_0_ave;
 return weight;
}
nav_msgs::Path test(nav_msgs::Path pathin,float dxy,float dz, float min_dst){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  std::vector<float> sizes;
  std::vector<float> lengths;
  std::vector<nav_msgs::Path> paths;
  paths.resize(pathin.poses.size());
  sizes.resize(pathin.poses.size());
  lengths.resize(pathin.poses.size());
  pathout.poses.resize(pathin.poses.size());
  for(int i = 0; i < pathin.poses.size(); i++){
    Eigen::Vector3f pnt2(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt1(pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y,pathin.poses[i].pose.position.z);
    Eigen::Vector3f cur_vec(pos.x,pos.y,pos.z);
    std::vector<nav_msgs::Path> path_segments;
    float tot_ray_len = (pnt2 - pnt1).norm();
    float cur_ray_len = 0;
    Eigen::Vector3f stride_vec = (pnt2 - pnt1).normalized();
    geometry_msgs::Point midpoint;
    int sum = 0;
    int cnt = 0;
    float ave;
    bool empty_streak = false;
    float empty_streak_start = 0;
    cur_vec = pnt1 + stride_vec * 5;
    while(cur_ray_len < tot_ray_len){
      cur_vec = cur_vec + stride_vec;
      midpoint.x = cur_vec.x();
      midpoint.y = cur_vec.y();
      midpoint.z = cur_vec.z();
      nav_msgs::Path path_segment  = get_path_aroundpnt(path_forward,midpoint,dxy);
      sizes.push_back(path_segment.poses.size());
      cnt++;
      sum  += path_segment.poses.size();
      int closest_i = getinpath_closestindex3d(path_segment,pos);
      float closest_dst = 1000;
      if(path_segment.poses.size() > 0 && closest_i < path_segment.poses.size())
          closest_dst = get_dst3d(path_segment.poses[closest_i].pose.position,midpoint);
      if(closest_dst < min_dst)
          break;

      if(path_segment.poses.size() == 0){
        if(!empty_streak){
          empty_streak = true;
          empty_streak_start = cur_ray_len;
        }
        else if(cur_ray_len- empty_streak_start > 5){
          ROS_INFO("EMPTY SINCE %.0f (cur at: %.0f)",empty_streak_start,cur_ray_len);
          break;
        }
      }
      else
        empty_streak = false;
      if(cur_ray_len > 7.5){
        float last_ave = vec_to_last_ave(sizes);
        if(last_ave > 2 * path_segment.poses.size()){
          ROS_INFO("Last_Average %.0f last this size: %i ( < 2*last_ave)",last_ave,path_segment.poses.size());
          break;
        }
      }
      path_segments.push_back(path_segment);
      cur_ray_len = (cur_vec - pnt1).norm();
    }
    pathout.poses[i].pose.position = midpoint;
    pathout.poses[i].pose.orientation.w = 1.0;
    pathout.poses[i].header = hdr();

    paths[i]  = scatter_path(merge_paths(path_segments),1);
    sizes[i]  = float(paths[i].poses.size());
    lengths[i] = cur_ray_len;
    ROS_INFO(" %i of %i path_segment: %i len: %.0f / %.0f",i,pathin.poses.size(),paths[i].poses.size(),cur_ray_len,tot_ray_len);
  }
  std::vector<float> scores;
  std::vector<float> mma_sbl = vec_to_min_max_ave(sizes);
  std::vector<float> mma_l   = vec_to_min_max_ave(lengths);
  scores.resize(pathin.poses.size());
  for(int i = 0; i < pathin.poses.size(); i++){
    float rel_bl  = (sizes[i] - mma_sbl[0]) / (mma_sbl[1]-mma_sbl[0]);
    float rel_l  = (lengths[i] - mma_l[0]) / (mma_l[1]-mma_l[0]);
    scores[i] = rel_bl+rel_l;
  }

  draw_paths_by_scores(paths,scores);

  draw_path_by_score(pathout,scores,0,0,0,1.0);
 count_target_paths++;
  cv::imwrite("/home/nuc/brain/fullpath/weights_"+std::to_string(count_target_paths)+ ".png",img);
  return pathout;
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
  ROS_INFO("Returns pathout: %i (pathin: %i)",pathout.poses.size(),pathin.poses.size());
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
    ROS_INFO("Score: %.2f rel_c,%.2f rel_z,%.2f rel_d,%.2f rel_h,%.2f rel_i,%.2f rel_0,%.2f",score,rel_c,rel_z,rel_d,rel_h,rel_i,rel_0);
  }
  scores_rel.resize(scores.size());
  std::vector<float> mma_s = vec_to_min_max_ave(scores);
  for(int i = 0; i < scores.size(); i++){
    scores_rel[i]  = (scores[i] - mma_s[0]) / (mma_s[1]-mma_s[0]);
    ROS_INFO("scores_rel[%i]: %.2f ",i, scores_rel[i]);
  }
  return scores_rel;
}
geometry_msgs::PoseStamped test_scoring(nav_msgs::Path pathin,std::string scoretype){
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();
  pnt_ref = frontier_outofsight_infront.point;
  float weight_z = 0.0;
  float weight_dst2d = 0.0;
  float weight_inclination = 0.0;
  float weight_startdst = 0.0;
  float weight_closest  = 1.0;
  float weight_hdng     = 0.0;

  if(scoretype == "closest_lowest_straightest_closeststart"){
    weight_startdst = -1.0;
    weight_hdng = -1;
    weight_z = -1;
    weight_dst2d = -1;
  }
  if(scoretype == "closest_lowest_closeststart"){
    weight_startdst = -1.0;
    weight_z = -1.0;
    weight_dst2d = -1.0;
  }
  if(scoretype == "farthest_highest_straightest"){
      weight_dst2d = 1.0;
      weight_hdng = -1.0;
      weight_z = 1.0;
  }
  if(scoretype == "farthest_closeststart"){
      weight_dst2d = 1.0;
      weight_startdst = -1.0;
  }
  if(scoretype == "closeststart")
      weight_startdst = -1.0;
  if(scoretype == "highest")
      weight_z = 1.0;
  if(scoretype == "lowest")
      weight_z = -1.0;
  ROS_INFO("TEST SCORING: %i",pathin.poses.size());
  if(pathin.poses.size() == 0)
    return ps;
  std::vector<float> scores = get_path_score(pathin,weight_hdng,weight_z,weight_dst2d,weight_inclination,weight_startdst,weight_closest);
  int best_i = 0;
  for(int i= 0; i < pathin.poses.size(); i++){
    float d2d = get_dst2d(pos,pathin.poses[i].pose.position);
    float in = get_inclination(pathin.poses[i].pose.position,pos);
    float h = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
    float z = pathin.poses[i].pose.position.z;
    if(scores[i] < 0.1 || scores[i] > 0.9){
      if(scores[i] > 0.5)
        cv::circle(img,pnt2cv(pathin.poses[i].pose.position),5,get_color(0,0,200),1);
      else
        cv::circle(img,pnt2cv(pathin.poses[i].pose.position),5,get_color(0,200,0),1);
      ROS_INFO("SCORE: %.2f - hdng: %.2f dst2d: %.0f z: %.0f incl: %.2f",scores[i],h,d2d,z,in);
    }
    if(scores[i] == 1.00){
      cv::circle(img,pnt2cv(pathin.poses[i].pose.position),5,get_color(255,255,255),1);
      ROS_INFO("WINNER: %.2f - hdng: %.2f dst2d: %.0f z: %.0f incl: %.2f",scores[i],h,d2d,z,in);
      best_i = i;
    }
  }
  draw_path_by_score(pathin,scores,0,2,1,1);
  return pathin.poses[best_i];
}

geometry_msgs::PoseStamped draw_and_get_best(nav_msgs::Path pathin,std::vector<float>scores){
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();
  pnt_ref = frontier_outofsight_infront.point;

  ROS_INFO("TEST SCORING: %i",pathin.poses.size());
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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_ctrl_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("heading_cutoff",par_hdngcutoff, M_PI/12);

	tf2_ros::TransformListener tf2_listener(tfBuffer);
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
  ros::Subscriber s12 = nh.subscribe("/odom_global",10,odom_cb);


  pub_path	 = nh.advertise<nav_msgs::Path>("/path_test",10);
  ros::Rate rate(1.0);
  int count_i = 0;int count_k = 0;
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
    checktf();
    process_odom();
    heightpath = merge_tbpaths(heightpaths);
    geometry_msgs::PoseStamped ps;
    ps.header = hdr();
    path_above_pos = get_path_above_pos(heightpath);
    std::string zw,dw,hw,iw,d0w;
    std::vector<nav_msgs::Path> pathsout;
    pathsout.resize(heightpaths.paths.size());
    for(int i=0; i < heightpaths.paths.size(); i++){
      ROS_INFO("********");
      pathsout[i] = get_heightpath_practical(heightpaths.paths[i]);
      create_all_points_in_bbox(heightpaths.paths[i]);
      create_all_points_in_bbox(pathsout[i]);
      ROS_INFO("********");
    }
    nav_msgs::Path heightpath2 = merge_paths(pathsout);
    draw_path_at_img(heightpath,pos,false,false,false,false,true,get_color(100,100,100),1);
    draw_path_at_img(path_above_pos,pos,false,false,false,false,true,get_color(0,0,100),1);
    draw_path_at_img(heightpath2,pos,false,false,false,false,true,get_color(200,200,0),1);
  //  create_all_points_in_bbox(heightpath2);
//    ROS_INFO("MErgedheightpaths: %i -> %i",heightpath.poses.size(),heightpath2.poses.size());

    float weight_z = -2.0;
    float weight_dst2d = 0.5;
    float weight_inclination = 0.0;
    float weight_startdst = -2.0;
    float weight_closest  = 1.0;
    float weight_hdng     = -1.0;

    std::vector<float> scores = get_path_score(heightpath2,weight_hdng,weight_z,weight_dst2d,weight_inclination,weight_startdst,weight_closest);
      ps = draw_and_get_best(heightpath2,scores);
    //draw_path_by_score(pathin,scores,0,2,1,1);
    drawimg("best_before"+ std::to_string(count_target_paths) );
    pub_path.publish(heightpath2);

    bool show_scoring= false;
    if(show_scoring){
      ps = test_scoring(heightpath2,"closeststart");
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
      drawimg("closeststart"+ std::to_string(count_target_paths) );

      ps = test_scoring(heightpath2,"highest");
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
      drawimg("highest"+ std::to_string(count_target_paths) );

      ps = test_scoring(heightpath2,"lowest");
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
      drawimg("lowest"+ std::to_string(count_target_paths) );

      ps = test_scoring(heightpath2,"farthest_highest_straightest");
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
      drawimg("farthest_highest_straightest"+ std::to_string(count_target_paths) );

      ps = test_scoring(heightpath2,"closest_lowest_straightest");
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
      drawimg("closest_lowest_straightest"+ std::to_string(count_target_paths) );

      ps = test_scoring(heightpath2,"farthest_closeststart");
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
      drawimg("farthest_closeststart"+ std::to_string(count_target_paths) );
    }
  }
  return 0;
}
