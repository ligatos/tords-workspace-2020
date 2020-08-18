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

geometry_msgs::Vector3 vlp_rpy;
nav_msgs::Odometry odom;
double par_zclearing,par_scanrange;
const float deg2rad = M_PI/180.0;
const float rad2deg = 180.0/M_PI;
const float earth = 6378137.0;
geometry_msgs::Point pnt_ref,cmd_pos,pos,scanpoint_actual,scanpoint_actual_lo,scanpoint_actual_hi,obs_p,obs_l,obs_r,obs_m;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
float altmax,scanrange_actual,scanrange_actual_hi,scanrange_actual_lo;
sensor_msgs::LaserScan scan_lo,scan_mid,scan_hi;
geometry_msgs::PolygonStamped poly_lo,poly_mid,poly_hi;
geometry_msgs::PoseStamped base_pose;
int counter = 0;
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
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
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
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
geometry_msgs::Point max_score(std::vector<float> scores){
  geometry_msgs::Point minmax;
  minmax.x = -100;
  minmax.y = 100;
  for(int i = 0; i < scores.size(); i++){
    if(minmax.x < scores[i]){
      minmax.x = scores[i];
    }
    if(minmax.y > scores[i]){
      minmax.y = scores[i];
    }
  }
  return minmax;
}
float rel_score(std::vector<float> scores, float score){
  geometry_msgs::Point minmax = max_score(scores);
  return (score - minmax.y) / (minmax.x - minmax.y);
}

cv::Scalar rel_color(std::vector<float> scores, float score, std::string rgb){
  cv::Scalar c;
  float relscore = rel_score(scores,score);
  if(rgb == "r")
    c[2] = relscore;
  if(rgb == "g")
    c[1] = relscore;
  if(rgb == "b")
    c[0] = relscore;
  return c;
 }

void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  geometry_msgs::Point p1,p2;

  for(int i = 1; i < polyin.polygon.points.size(); i++){
    p1.x = polyin.polygon.points[i-1].x;
    p1.y = polyin.polygon.points[i-1].y;
    p2.x = polyin.polygon.points[i].x;
    p2.y = polyin.polygon.points[i].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
  p1.x = polyin.polygon.points[0].x;
  p1.y = polyin.polygon.points[0].y;
  p2.x = polyin.polygon.points[polyin.polygon.points.size()-1].x;
  p2.y = polyin.polygon.points[polyin.polygon.points.size()-1].y;
  cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
}
void draw_poly_pnts(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  geometry_msgs::Point p1,p2;
  for(int i = 1; i < polyin.polygon.points.size(); i++){
    p2.x = polyin.polygon.points[i].x;
    p2.y = polyin.polygon.points[i].y;
    cv::circle(img,pnt2cv(p2),1,color,1);
  }
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
float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / get_dst2d(p1,p2);
}
float get_inclination(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_a_from_dxy_r(float dxy, float r){
  return acos(dxy/r);
}
float get_dz_from_dxy_a(float dxy, float r){
  return acos(dxy/r);
}
float get_dz_from_r_a(float a, float r){
  return r * sin(a);
}
float get_dxy_from_r_a(float a, float r){
  return r * cos(a);
}
float get_a_from_dz_r(float dz, float r){
  return asin(dz/r);
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

void cmdpos_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  cmd_pos = *msg;
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
geometry_msgs::Point get_ave_pnt32(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point pnt;
  for(int i = 1; i < polyin.polygon.points.size()-1; i++){
    pnt.x += polyin.polygon.points[i].x;
    pnt.y += polyin.polygon.points[i].y;
    pnt.z += polyin.polygon.points[i].z;
  }
  pnt.x /= polyin.polygon.points.size();
  pnt.y /= polyin.polygon.points.size();
  pnt.z /= polyin.polygon.points.size();
  ROS_INFO("AVEPNT: [%i polygonnts] - %.0f %.0f %.0f ",polyin.polygon.points.size(),pnt.x,pnt.y,pnt.z);
  return pnt;
}
float get_zmax32(geometry_msgs::PolygonStamped polyin){
  float zmx = 0;
  for(int i = 1; i < polyin.polygon.points.size()-1; i++){
    if(polyin.polygon.points[i].z > zmx){
      zmx = polyin.polygon.points[i].z;
    }
  }
  return zmx;
}
geometry_msgs::PolygonStamped get_polyfinal(sensor_msgs::LaserScan scanin){
  geometry_msgs::PolygonStamped poly;
  geometry_msgs::PointStamped pnt,pnt_out;
  poly.header = hdr();
  poly.polygon.points.resize(scanin.ranges.size() + 2);
  pnt.header.frame_id = scanin.header.frame_id;
  for(int i = 0; i < scanin.ranges.size(); i++){
    if(std::isinf(scanin.ranges[i])){
      poly.polygon.points[i+1].x = 0;
      poly.polygon.points[i+1].y = 0;
      poly.polygon.points[i+1].z = 0;
    }
    else{
      float a = scanin.angle_min + scanin.angle_increment * i;
      pnt.point.x = scanin.ranges[i] * cos(a);
      pnt.point.y = scanin.ranges[i] * sin(a);
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      poly.polygon.points[i+1].x = pnt_out.point.x;
      poly.polygon.points[i+1].y = pnt_out.point.y;
      poly.polygon.points[i+1].z = pnt_out.point.z;
    }
  }
  pnt.point.x = 0;
  pnt.point.y = 0;
  pnt_out = tfBuffer.transform(pnt, "map");
  poly.polygon.points[0].x = pnt_out.point.x;
  poly.polygon.points[0].y = pnt_out.point.y;
  poly.polygon.points[0].z = pnt_out.point.z;
  poly.polygon.points[poly.polygon.points.size()-1] = poly.polygon.points[0];
  return poly;
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
nav_msgs::Path get_pathfinal(sensor_msgs::LaserScan scanin){
  nav_msgs::Path pathout;
  geometry_msgs::PointStamped pnt,pnt_out;
  pathout.header = hdr();
  pnt.header.frame_id = scanin.header.frame_id;
  for(int i = 0; i < scanin.ranges.size(); i++){
    if(std::isinf(scanin.ranges[i])){
    }
    else{
      float a = scanin.angle_min + scanin.angle_increment * i;
      pnt.point.x = scanin.ranges[i] * cos(a);
      pnt.point.y = scanin.ranges[i] * sin(a);
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      geometry_msgs::PoseStamped ps;
      ps.header = hdr();
      ps.pose.position    = pnt_out.point;
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(pnt_out.point,pos));
      pathout.poses.push_back(ps);
      }
  }
  return pathout;
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
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

nav_msgs::Path get_new_path(nav_msgs::Path path_old,nav_msgs::Path pathin,float cutoff_dst){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(path_old,pathin.poses[i].pose.position,cutoff_dst))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

nav_msgs::Path remove_old(nav_msgs::Path path_old,geometry_msgs::Point midpoint,float maxrad){
  nav_msgs::Path pathout;
  for(int i = 0; i < path_old.poses.size(); i++){
    if(get_dst2d(midpoint,path_old.poses[i].pose.position) < maxrad){
      pathout.poses.push_back(path_old.poses[i]);
    }
  }
  return pathout;
}
nav_msgs::Path add_new(nav_msgs::Path path_old,nav_msgs::Path pathin){
  for(int i = 0; i < pathin.poses.size(); i++){
      path_old.poses.push_back(pathin.poses[i]);
  }
  return path_old;
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
bool is_within_incline(geometry_msgs::Point pnt,float maxdelta_pitch){
  float inclination       = get_inclination(pos,pnt);
  float delta_inclination = get_shortest(-vlp_rpy.y,inclination);
  float pitch_shortest    = get_shortest(atan2(pnt.z-pos.z,get_dst2d(pnt,pos)),-vlp_rpy.y);
  ////ROS_INFO("vlp_rpy: %.2f, inclination: %.2f, max_inclination: %.2f, pitch_shortest: %.2f,delta_inclination %.2f",vlp_rpy.y,inclination,maxdelta_pitch,pitch_shortest,delta_inclination);
  if(delta_inclination < maxdelta_pitch && delta_inclination > -maxdelta_pitch)
    return true;
  else
    return false;
}

int getinpath_neighbours(nav_msgs::Path pathin,int i0,float radius){
  int count = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pathin.poses[i0].pose.position) <= radius)
      count++;
  }
  return count;
}
nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  float base_yaw = tf::getYaw(base_pose.pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(base_pose.pose.position,pathin.poses[i].pose.position)));
    else if(sort_by == "inclination")
        i_dst.push_back(std::make_tuple(i,get_shortest(-vlp_rpy.y,get_inclination(pos,pathin.poses[i].pose.position))));
    else if(sort_by == "dst_3d_ref")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pnt_ref,pathin.poses[i].pose.position)));
    else if(sort_by == "neighbours")
      i_dst.push_back(std::make_tuple(i,getinpath_neighbours(pathin,i,2)));
    else if(sort_by == "dst_3d")
      i_dst.push_back(std::make_tuple(i,get_dst3d(base_pose.pose.position,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_abs")
			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw) * rad2deg)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw)));
    else if(sort_by == "zabs")
  		i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - base_pose.pose.position.z)));
    else if(sort_by == "z")
      i_dst.push_back(std::make_tuple(i,pathin.poses[i].pose.position.z));
	}
      sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
std::vector<float> get_vec_attribute(nav_msgs::Path pathin,std::string type){
  std::vector<float> vec_out;
  for(int i = 0; i < pathin.poses.size(); i++){
    float val = 0;
    if(type == "inclination_abs")
      val = get_inclination(pos,pathin.poses[i].pose.position);
    else if(type == "dst_2d")
      val = get_dst2d(base_pose.pose.position,pathin.poses[i].pose.position);
    else if(type == "neighbours")
      val = getinpath_neighbours(pathin,i,3);
    else if(type == "inclination")
      val = get_shortest(-vlp_rpy.y,get_inclination(pos,pathin.poses[i].pose.position));
    else if(type == "dst_3d_ref")
      val = get_dst3d(pnt_ref,pathin.poses[i].pose.position);
    else if(type == "dst_3d")
      val = get_dst3d(base_pose.pose.position,pathin.poses[i].pose.position);
		else if(type == "hdng_abs")
			val = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),vlp_rpy.z) * rad2deg);
    else if(type == "hdng")
      val = get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),vlp_rpy.z);
    else if(type == "zabs")
  		val = abs(pathin.poses[i].pose.position.z - base_pose.pose.position.z);
    else if(type == "z")
      val = pathin.poses[i].pose.position.z;
    vec_out.push_back(val);
  }
  return vec_out;
}

nav_msgs::Path cutoff_top(nav_msgs::Path pathin,std::string type,int percent_start,int percent_end){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pathin                  = sort_path(pathin,type);
  std::vector<float> vals = get_vec_attribute(pathin,type);
  float vmx = vec_to_max(vals);
  float vmn = vec_to_min(vals);
  float ave = vec_to_ave(vals);
  float vd = vmx - vmn;
  std::vector<float> valsout;
  float v0 = float(percent_start/100.0) * (vmx - vmn) + vmn;
  float vN = float(percent_end/100.0) * (vmx - vmn) + vmn;
  for(int i = 0; i < pathin.poses.size(); i++){
    float rel_val = (vals[i] - vmn) / vd;
    if(vals[i] >= v0 && vals[i] <= vN){
      pathout.poses.push_back(pathin.poses[i]);
      valsout.push_back(vals[i]);
    }
  }
  float pp = float(pathout.poses.size()) / float(pathin.poses.size());
  int percent_poses = pp * 100;
  ROS_INFO("Cutoff %s , [min->max: ave] [%.2f->%.2f: %.2f] in->out: [%i->%i](%i percent)",type.c_str(),vmn,vmx,ave,pathin.poses.size(),pathout.poses.size(),percent_poses);
  vmx = vec_to_max(valsout);
  vmn = vec_to_min(valsout);
  ave = vec_to_ave(valsout);
  ROS_INFO("Cutoff %s,  [min->max: ave] [%.2f->%.2f: %.2f] (v0 %.0f-> vN %.0f)",type.c_str(),vmn,vmx,ave,v0,vN);
  return pathout;
}
void check(){
  img_blank.copyTo(img);
  poly_lo  = get_polyfinal(scan_lo);
  poly_mid = get_polyfinal(scan_mid);
  poly_hi  = get_polyfinal(scan_hi);
  //path_new = cutoff_top(path_new,"z",60,100);
  //path_new = cutoff_top(path_new,"neighbours",60,100);

  nav_msgs::Path path_lo  = get_pathfinal(scan_lo);
  nav_msgs::Path path_mid = get_pathfinal(scan_mid);
  nav_msgs::Path path_hi  = get_pathfinal(scan_hi);
  draw_path_at_img(path_lo,pos,false,false,true,false,false,get_color(100,0,0),1);
  draw_path_at_img(path_mid,pos,false,false,true,false,false,get_color(0,100,0),1);
  draw_path_at_img(path_hi,pos,false,false,true,false,false,get_color(0,0,100),1);
  cv::imwrite("/home/nuc/brain/"+std::to_string(counter)+"raw.png",img);
  img_blank.copyTo(img);

  nav_msgs::Path path_lo_mid   = cutoff_top(path_lo,"hdng_abs",0,50);
  nav_msgs::Path path_mid_mid  = cutoff_top(path_mid,"hdng_abs",0,50);
  nav_msgs::Path path_hi_mid   = cutoff_top(path_hi,"hdng_abs",0,50);
  draw_path_at_img(path_lo_mid,pos,false,false,true,false,false,get_color(100,0,0),1);
  draw_path_at_img(path_mid_mid,pos,false,false,true,false,false,get_color(0,100,0),1);
  draw_path_at_img(path_hi_mid,pos,false,false,true,false,false,get_color(0,0,100),1);
  cv::imwrite("/home/nuc/brain/"+std::to_string(counter)+"cut.png",img);

  //float zmax = get_zmax(path_mid);
  scanpoint_actual = get_ave_pnt(path_mid_mid);
//  scanpoint_actual.z = zmax;
  scanrange_actual = get_dst3d(pos,scanpoint_actual);
  counter++;
  geometry_msgs::Point pyaw;
  pyaw.x = pos.x + 40 * cos(vlp_rpy.z);
  pyaw.y = pos.y + 40 * sin(vlp_rpy.z);
  cv::circle(img,pnt2cv(pos),3,get_color(200,200,200),1);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),get_color(200,200,200),1,cv::LINE_8,0);
  cv::imwrite("/home/nuc/brain/"+std::to_string(counter)+"clear_.png",img);
  img_blank.copyTo(img);
}
void rayranges_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_mid = *msg;
  if(scan_hi.ranges.size() > 0 && scan_lo.ranges.size() > 0)
    check();
}
void rayranges_hi_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_hi = *msg;
  if(scan_mid.ranges.size() > 0 && scan_lo.ranges.size() > 0)
    check();
}
void rayranges_lo_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_lo = *msg;
  if(scan_mid.ranges.size() > 0 && scan_hi.ranges.size() > 0)
    check();
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
  base_pose.pose.position = pos;
  base_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  base_pose.header = hdr();
}

float get_tilt_error(){
  float target_angle = get_a_from_dz_r(par_zclearing,par_scanrange);
  float error_angle  = target_angle - vlp_rpy.y;
  return error_angle;
}
void closest_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_p = msg->point;
  if(msg->header.frame_id != "map"){
    obs_p.z = -100;
    obs_p.z = pos.z - par_zclearing;
  }
}
void closest_left_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_l = msg->point;
  if(msg->header.frame_id != "map")
    obs_l.z = -100;
}
void closest_right_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_r = msg->point;
  if(msg->header.frame_id != "map")
    obs_r.z = -100;
}
void closest_mid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_m = msg->point;
  if(msg->header.frame_id == "map")
    obs_m.z = -100;
}
void altmax_cb(const std_msgs::Float64::ConstPtr& msg){
  altmax = msg->data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmdaltarm_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param("setpoint_scanrange", par_scanrange, 25.0);//*2.0);
    private_nh.param("setpoint_zclearing", par_zclearing, 5.0);//*2.0);

    ros::Rate rate(50);

    tf2_ros::TransformListener tf2_listener(tfBuffer);

    tf2_ros::TransformBroadcaster tf_b;
    ros::Subscriber z1        = nh.subscribe("/tb_path/altmax",10,altmax_cb);
    ros::Subscriber as1        = nh.subscribe("/velodyne_scan",10,rayranges_cb);
    ros::Subscriber a2        = nh.subscribe("/scan_tilt_up",10,rayranges_hi_cb);
    ros::Subscriber as4        = nh.subscribe("/scan_tilt_down",10,rayranges_lo_cb);
    ros::Subscriber sss        = nh.subscribe("/cmd_pos",               100,&cmdpos_cb);
    ros::Subscriber asb4 = nh.subscribe("/tb_obs/closest_pos",10,closest_pos_cb);
    ros::Subscriber asbb4 = nh.subscribe("/tb_obs/closest_left",10,closest_left_cb);
    ros::Subscriber assaa4 = nh.subscribe("/tb_obs/closest_right",10,closest_right_cb);
    ros::Subscriber assa4 = nh.subscribe("/tb_obs/closest_mid",10,closest_mid_cb);
    ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
  	ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
    cmd_pos.z = 15;
    float dt;
    float setpoint_altitude;
    float setpoint_scanrange;
    float z_err,s_err,t_err;
    float z_i,s_i,t_i;
    float z_d,s_d,t_d;
    float pz_P,ps_P,pt_P;
    float pz_D,ps_D,pt_D;
    float pz_I,ps_I,pt_I;
    pz_P = ps_P = pt_P = 1;
    pz_D = ps_D = pt_D = 0.3;
    pz_I = ps_I = pt_I = 0.01;

    float z_err_last,s_err_last,t_err_last;

    ros::Time time_last = ros::Time::now();

    std_msgs::Float64 arm1_tilt_msg;
    std_msgs::Float64 target_alt_msg;

    double syaw;
    while(ros::ok()){
      dt= (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
      rate.sleep();
      ros::spinOnce();

      checktf();

      float inc_cmd     = get_inclination(cmd_pos,pos);
      float inc_scanpnt = get_inclination(scanpoint_actual,pos);
      float velxy = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
      bool use_cmd = true;
      float inc_used = inc_cmd;
      if(inc_scanpnt > inc_cmd){
        inc_used = inc_scanpnt;
        use_cmd = false;
      }
  //    float max_obsz = fmax(obs_p.z,obs_m.z);
    //  ROS_INFO("Altitude target inc_cmd:  %.0f xyz: %.0f %.0f %.0f",inc_cmd,cmd_pos.x,cmd_pos.y,cmd_pos.z);
    //  ROS_INFO("Altitude scapnt inc_pnt:  %.0f xyz: %.0f %.0f %.0f",inc_scanpnt,scanpoint_actual.x,scanpoint_actual.y,scanpoint_actual.z);
      bool use_altitude_pnt_directly = true;
      float vz          = velxy * inc_used;
      if(use_altitude_pnt_directly){
        setpoint_altitude = fmax(cmd_pos.z,scanpoint_actual.z + par_zclearing);
      }
      else
        setpoint_altitude += vz * dt;
    //  ROS_INFO("Altitude scapnt inc_used: %.0f vxy: %.2f vz: %.2f z: %.2f",inc_used,velxy,vz,setpoint_altitude);
  //    if(max_obsz > setpoint_altitude)
    //    setpoint_altitude = max_obsz + 3;
      float target_angle = get_a_from_dz_r(par_zclearing,par_scanrange);
      //ROS_INFO("Target angle %.2f from %.2f clearing %.2f scantarget",target_angle,par_zclearing,par_scanrange);
      z_err = setpoint_altitude - pos.z;
      s_err = par_scanrange - scanrange_actual;
      t_err = target_angle - vlp_rpy.y;
      ROS_INFO("z_err: %.2f (%.2f - %.2f)",z_err,setpoint_altitude,pos.z);
      ROS_INFO("s_err: %.2f (%.2f - %.2f)",s_err,par_scanrange,scanrange_actual);
      ROS_INFO("t_err: %.2f (%.2f - %.2f)",t_err,target_angle,vlp_rpy.y);
      if(std::isnan(t_err))
        t_err = 0;
      if(z_i > 5.0 || z_i < -5.0)
        z_i = 0;
      if(s_i > 5.0 || s_i < -5.0)
        s_i += 0;
      if(t_i > 5.0 || t_i < -5.0)
        t_i = 0;
      if(z_d > 5.0 || z_d < -5.0)
        z_d = 0;
      if(s_d > 5.0 || s_d < -5.0)
        s_d = 0;
      if(t_d > 5.0 || t_d < -5.0)
        t_d = 0;
      z_i += z_err*dt; s_i += s_err*dt;  t_i += t_err*dt;

      z_d = (z_err - z_err_last) / dt;
      s_d = (s_err - s_err_last) / dt;
      t_d = (t_err - t_err_last) / dt;

      z_err_last = z_err;
      s_err_last = s_err;
      t_err_last = t_err;

      float cmd_z   = (z_err * pz_P      + z_d * pz_D     + z_i * pz_I);
      float cmd_s   = (s_err * ps_P      + s_d * pz_D     + s_i * ps_I);
      float cmd_t   = (t_err * pt_P      + t_d * pz_D     + t_i * pt_I);

      arm1_tilt_msg.data  += (cmd_t*dt);
      target_alt_msg.data += (cmd_z*dt);
      if(target_alt_msg.data > altmax + 5){
        target_alt_msg.data = altmax+5;
      }
      ROS_INFO("z_cmd:  %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",z_err,z_d,z_i,cmd_z,target_alt_msg.data);
      ROS_INFO("scan:   %.2f d: %.2f, i %.2f, cmd: %.2f ",s_err,s_d,s_i,cmd_s);
      ROS_INFO("tilt:   %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",t_err,t_d,t_i,cmd_t,arm1_tilt_msg.data);
      if(std::isnan(target_alt_msg.data)){
        ROS_INFO("NANWARN target alt");
        target_alt_msg.data = 15;
        pub_altcmd.publish(target_alt_msg);
      }
      else
        pub_altcmd.publish(target_alt_msg);
      if(std::isnan(arm1_tilt_msg.data))
        ROS_INFO("NANWARN target tilt");
      else
        pub_tiltvlp.publish(arm1_tilt_msg);

    }
    return 0;
}
