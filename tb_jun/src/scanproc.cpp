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
cv::Mat img_blankish(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
float altmax,scanrange_actual,scanrange_actual_hi,scanrange_actual_lo;
sensor_msgs::LaserScan scan_lo,scan_mi,scan_hi,scan_up,scan_dn,scan_st;
geometry_msgs::PolygonStamped poly_lo,poly_mi,poly_hi,poly_cleared;
geometry_msgs::PoseStamped base_pose;
int counter = 0;
cv::Scalar c_lo,c_dn,c_mi,c_up,c_hi;
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


nav_msgs::Path get_pathfinalclear(sensor_msgs::LaserScan scanin){
  nav_msgs::Path pathout;
  geometry_msgs::PointStamped pnt,pnt_out;
  pathout.header = hdr();
  pnt.header.frame_id = scanin.header.frame_id;
  for(int i = 0; i < scanin.ranges.size(); i++){
    if(std::isinf(scanin.ranges[i])){
      float a = scanin.angle_min + scanin.angle_increment * i;
      pnt.point.x = scanin.range_max * cos(a);
      pnt.point.y = scanin.range_max * sin(a);
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      geometry_msgs::PoseStamped ps;
      ps.header = hdr();
      ps.pose.position    = pnt_out.point;
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(pnt_out.point,pos));
      pathout.poses.push_back(ps);
    }
    else{

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
std::vector<int> getinpath_neighbours_point(nav_msgs::Path pathin,geometry_msgs::Point pnt,float radius){
  int count = 0;
  std::vector<int> vec;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pnt) <= radius)
      vec.push_back(i);
  }
  return vec;
}
int getinpath_neighbours(nav_msgs::Path pathin,int i0,float radius){
  int count = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pathin.poses[i0].pose.position) <= radius)
      count++;
  }
  return count;
}
geometry_msgs::Point get_maxmin_in_path(nav_msgs::Path pathin,bool get_max,std::string type){
  geometry_msgs::Point pnt;
  float best_val;
  int best_i = 0;
  if(get_max)
    best_val = -99999;
  else
    best_val = 999999;
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
    else if(type == "zrel")
    	val = pathin.poses[i].pose.position.z - base_pose.pose.position.z;
    else if(type == "z")
      val = pathin.poses[i].pose.position.z;
    else
      ROS_INFO("UNKNOWN");
    if(get_max){
      if(best_val < val){
        best_val = val;
        best_i   = i;
      }
    }
    else{
      if(best_val > val){
        best_val = val;
        best_i = i;
      }
    }
  }
  if(best_i < pathin.poses.size())
    pnt = pathin.poses[best_i].pose.position;
  return pnt;
}
nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
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
			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),vlp_rpy.z) * rad2deg)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),vlp_rpy.z)));
    else if(sort_by == "zabs")
  		i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - base_pose.pose.position.z)));
    else if(sort_by == "zrel")
  		i_dst.push_back(std::make_tuple(i,(pathin.poses[i].pose.position.z - base_pose.pose.position.z)));
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
    else if(type == "zrel")
    	val = pathin.poses[i].pose.position.z - base_pose.pose.position.z;
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
  ROS_INFO("Cutoff_percent %s , [min->max: ave] [%.2f->%.2f: %.2f] in->out: [%i->%i](%i percent)",type.c_str(),vmn,vmx,ave,pathin.poses.size(),pathout.poses.size(),percent_poses);
  vmx = vec_to_max(valsout);
  vmn = vec_to_min(valsout);
  ave = vec_to_ave(valsout);
  ROS_INFO("Cutoff_percent %s,  [min->max: ave] [%.2f->%.2f: %.2f] (v0 %.0f-> vN %.0f)",type.c_str(),vmn,vmx,ave,v0,vN);
  return pathout;
}
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin,std::string type,float val_0,float val_N){
  if(pathin.poses.size() < 3)
    return pathin;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pathin                  = sort_path(pathin,type);
  std::vector<float> vals = get_vec_attribute(pathin,type);
  float vmx = vec_to_max(vals);
  float vmn = vec_to_min(vals);
  float ave = vec_to_ave(vals);
  float vd = vmx - vmn;
  std::vector<float> valsout;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(vals[i] >= val_0 && vals[i] <= val_N){
      pathout.poses.push_back(pathin.poses[i]);
      valsout.push_back(vals[i]);
    }
  }
  float pp = float(pathout.poses.size()) / float(pathin.poses.size());
  int percent_poses = pp * 100;
  ROS_INFO("Cutoff_abs %s , [min->max: ave] [%.2f->%.2f: %.2f] in->out: [%i->%i](%i percent)",type.c_str(),vmn,vmx,ave,pathin.poses.size(),pathout.poses.size(),percent_poses);
  vmx = vec_to_max(valsout);
  vmn = vec_to_min(valsout);
  ave = vec_to_ave(valsout);
  ROS_INFO("Cutoff_abs %s,  [min->max: ave] [%.2f->%.2f: %.2f] (v0 %.2f-> vN %.2f)",type.c_str(),vmn,vmx,ave,val_0,val_N);
  return pathout;
}
void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}
void drawimg(float hdng_cutoff,std::string name){
  geometry_msgs::Point pyaw,pyaw_mn,pyaw_mx;
  float a0 = constrainAngle(vlp_rpy.z + hdng_cutoff);
  float a1 = constrainAngle(vlp_rpy.z - hdng_cutoff);
  draw_line(pos,a0,50,get_color(200,200,200));
  draw_line(pos,a1,50,get_color(200,200,200));
  draw_line(pos,vlp_rpy.z,50,get_color(200,200,200));

  ROS_INFO("vlp_rpy_Z: %.2f, hdng_cutoff: %.2f, a0: %.2f a1: %.2f",vlp_rpy.z,hdng_cutoff,a0,a1);

  draw_poly(poly_cleared,get_color(200,200,200));
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),get_color(100,100,100),1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw_mn),get_color(200,0,200),1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw_mx),get_color(200,0,200),1,cv::LINE_8,0);
  cv::circle(img,pnt2cv(pos),3,get_color(200,200,200),1);
  cv::circle(img,pnt2cv(cmd_pos),3,get_color(0,0,200),1);
  cv::line (img, pnt2cv(pos), pnt2cv(cmd_pos),get_color(0,0,200),1,cv::LINE_8,2);
  cv::imwrite("/home/nuc/brain/scanproc/"+std::to_string(counter)+ name + ".png",img);
  img_blank.copyTo(img);
}
float relval(float min,float max,float val){
  return 255*(val - min) / (max - min);
}
sensor_msgs::LaserScan merge_scans(sensor_msgs::LaserScan s1,sensor_msgs::LaserScan s2,sensor_msgs::LaserScan s3){
  sensor_msgs::LaserScan scanout = s2;
  scanout.ranges.resize(fmax(s1.ranges.size(),s2.ranges.size()));
  float v1,v2,v3;
  for(int i = 0; 0 < s1.ranges.size(); i++){
    if(s2.ranges.size() > i && s3.ranges.size() > i){
      if(std::isinf(s1.ranges[i]) && std::isinf(s2.ranges[i]) && std::isinf(s3.ranges[i]))
        scanout.ranges[i] = s1.ranges[i];
      else if(!std::isinf(s1.ranges[i]) && !std::isinf(s2.ranges[i]) && !std::isinf(s3.ranges[i]))
        scanout.ranges[i] = fmin(fmin(s1.ranges[i],s2.ranges[i]),fmin(s2.ranges[i],s3.ranges[i]));
      else if(std::isinf(s1.ranges[i]) && !std::isinf(s2.ranges[i]) && !std::isinf(s3.ranges[i]))
        scanout.ranges[i] = fmin(s2.ranges[i],s3.ranges[i]);
      else if(!std::isinf(s1.ranges[i]) && std::isinf(s2.ranges[i]) && !std::isinf(s3.ranges[i]))
        scanout.ranges[i] = fmin(s1.ranges[i],s3.ranges[i]);
      else if(!std::isinf(s1.ranges[i]) && !std::isinf(s2.ranges[i]) && std::isinf(s3.ranges[i]))
        scanout.ranges[i] = fmin(s1.ranges[i],s2.ranges[i]);
      else if(!std::isinf(s1.ranges[i]) && std::isinf(s2.ranges[i]) && std::isinf(s3.ranges[i]))
        scanout.ranges[i] =s1.ranges[i];
      else if(std::isinf(s1.ranges[i]) && !std::isinf(s2.ranges[i]) && std::isinf(s3.ranges[i]))
        scanout.ranges[i] =s2.ranges[i];
      else if(std::isinf(s1.ranges[i]) && std::isinf(s2.ranges[i]) && !std::isinf(s3.ranges[i]))
        scanout.ranges[i] =s3.ranges[i];
    }
  }
  return scanout;
}
float get_i_min(sensor_msgs::LaserScan scanin,int i0, int iN){
  float mn = scanin.ranges[i0];
  for(int i = i0; i < iN; i++){
    if(std::isinf(scanin.ranges[i])){
      if(scanin.ranges[i] < mn)
        mn = scanin.ranges[i];
    }
  }
  return mn;
}
sensor_msgs::LaserScan get_ranges2(sensor_msgs::LaserScan scanin,int scans_pr_min){
  sensor_msgs::LaserScan scanin_old = scanin;
  std::vector<float> ranges;
  std::queue<float> ranges_segment;
  ranges.resize(scanin.ranges.size());

  for(int i = scans_pr_min/2; i < scanin.ranges.size()-scans_pr_min/2; i++){
    ranges[i] = get_i_min(scanin,i-scans_pr_min/2,i+scans_pr_min/2);
  }

  float ranges_segment_sum = 0;
  for(int i = 0; i < ranges.size(); i++){
    ranges_segment.push(ranges[i]);
    ranges_segment_sum += ranges[i];
    if(ranges_segment.size() > scans_pr_min)
    {
      ranges_segment_sum -= ranges_segment.front();
      float ranges_ave = ranges_segment_sum / ranges_segment.size();
      scanin.ranges[i] = ranges_ave;
      ranges_segment.pop();
    }
    else
      scanin.ranges[i] = 0;
  }
  for(int i  = 0; i < scanin.ranges.size(); i++){
    ROS_INFO("Sanin[%i] old: %.0f new: %.0f",i,scanin_old.ranges[i],scanin.ranges[i]);
  }
  return scanin;
}
sensor_msgs::LaserScan get_ranges(sensor_msgs::LaserScan scanin,int scans_pr_min){
  sensor_msgs::LaserScan scanout;
  scanout = scanin;
  for(int i = scans_pr_min/2; i < scanin.ranges.size()-scans_pr_min/2; i++){
    scanout.ranges[i] = get_i_min(scanin,i-scans_pr_min/2,i+scans_pr_min/2);
  }
  return scanout;
}
//std::deque<int> q{5, 1, 3};
//std::deque<int>::iterator it = std::min_element(q.begin(), q.end());
//std::vector<int>::iterator result = std::min_element(v.begin(), v.end());
//std::cout << *it << std::endl;


void initialscan(sensor_msgs::LaserScan scanin){
		geometry_msgs::Point current_object;
		std::vector<geometry_msgs::Point>cluster;
    std::vector<float> hdngs;
    std::vector<bool> is_clear;
    std::vector<float> hdng_start_obs;
    std::vector<float> hdng_end_obs;
    std::vector<float> obstacle_clusters_r;
    std::vector<float> obstacle_clusters_z;
    std::vector<float> cleared_clusters_ends;
    std::vector<float> obstacle_clusters_starts;
    std::vector<float> obstacle_clusters_ends;
    std::vector<float> cleared_clusters_starts;

    std::vector<geometry_msgs::Point> obstacle_clusters_center;
    std::vector<std::vector<geometry_msgs::Point>>cleared_clusters;
		std::vector<std::vector<geometry_msgs::Point>>obstacle_clusters;
    bool last_clear = false;
    float zmx = 0;
    bool clear;
    float r_sum = 0;
    int largest_cleared_cluster = 0;
    int largest_obstacle_cluster = 0;

    float cleared_lim = 30;
    if(std::isinf(scanin.ranges[0]))
      last_clear = true;
    if(last_clear)
      cleared_clusters_starts.push_back(scanin.angle_min);
    else
      obstacle_clusters_starts.push_back(scanin.angle_min);
    int count_ten = 0;
    for(int i  = 1; i < scanin.ranges.size()-1; i++){
      geometry_msgs::PointStamped pnt,pnt_out;
      float current_angle = scanin.angle_min + scanin.angle_increment * i;
      hdngs.push_back(current_angle);
      //ROS_INFO("Current angle: %.2f range: %.2f cluster: %i",current_angle,scanin.ranges[i],cluster.size());
      clear = false;
      pnt.point.x = scanin.ranges[i] * cos(current_angle);
      pnt.point.y = scanin.ranges[i] * sin(current_angle);
      pnt.header.frame_id = scanin.header.frame_id;
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      if(scanin.ranges[i] > scanin.ranges[i+1] && scanin.ranges[i] > scanin.ranges[i-1] && scanin.ranges[i] > cleared_lim){
        obstacle_clusters_center.push_back(pnt_out.point);
        cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,200),1);
      }
      count_ten++;
      if(scanin.ranges[i] > cleared_lim)
        clear = true;
      if(count_ten > 10){
        count_ten = 0;
      if(clear)
        draw_line(pos,current_angle,scanin.ranges[i],get_color(relval(0,50,scanin.ranges[i]),relval(0,50,scanin.ranges[i]),0));
      else
        draw_line(pos,current_angle,scanin.ranges[i],get_color(0,0,relval(0,50,scanin.ranges[i])));
      }

    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,0),1);

        // std::isinf(scanin.ranges[i-1]) && std::isinf(scanin.ranges[i]) && std::isinf(scanin.ranges[1+i]))


    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(0,0,relval(0,50,pnt_out.point.z)),1);

      is_clear.push_back(clear);
      if(last_clear && !clear){
        zmx = 0;
        cleared_clusters_ends.push_back(current_angle);
        obstacle_clusters_starts.push_back(current_angle);
        cleared_clusters.push_back(cluster);
        if(cluster.size() > largest_cleared_cluster)
          largest_cleared_cluster = cluster.size();
        cluster.resize(0);
      }
      else if(!last_clear && clear){
        float ranges_ave = r_sum / cluster.size();
        r_sum = 0;
        obstacle_clusters_z.push_back(zmx);
        obstacle_clusters_r.push_back(ranges_ave);
        if(cluster.size() > largest_obstacle_cluster)
          largest_obstacle_cluster = cluster.size();
        obstacle_clusters_ends.push_back(current_angle);
        cleared_clusters_starts.push_back(current_angle);
        obstacle_clusters.push_back(cluster);
        cluster.resize(0);
      }
      else if(last_clear && clear){

      }
      else if(!last_clear && !clear){

      }
      else{
        ROS_INFO("ERROR - impossible combinatio (CLEAR vs OBST)");
      }

      cluster.push_back(pnt_out.point);
      last_clear = clear;
      if(!clear){
        r_sum += scanin.ranges[i];
        if(pnt_out.point.z > zmx)
          zmx = pnt_out.point.z;
      }
    }
  for(int i  = 0; i < cleared_clusters.size(); i++){
    int num  = cleared_clusters[i].size();
    float a0 = cleared_clusters_starts[i];
    float aN = cleared_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    draw_line(pos,a,50,get_color(0,200*num/largest_cleared_cluster,0));
    float ad = aN - a0;
    ROS_INFO("Cluster[%i], size: %i a: %.2f (delta. %.2f rads) (a0 %.2f->%.2f an)",i,num,a,ad,a0,aN);
  }
  for(int i  = 0; i < obstacle_clusters.size(); i++){
    int num  = obstacle_clusters[i].size();
    float a0 = obstacle_clusters_starts[i];
    float aN = obstacle_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    float r  = (aN+a0)/2.0;
    float z  = (aN+a0)/2.0;
    draw_line(pos,a,r,get_color(0,0,200*num/largest_obstacle_cluster));

    ROS_INFO("Obstacles[%i], size: %i a: %.2f, zmax: %.2f dst_ave: %.2f",i,num,a,obstacle_clusters_z[i],obstacle_clusters_r[i]);
  }
}
void initialscan2(sensor_msgs::LaserScan scanin){
		geometry_msgs::Point current_object;
		std::vector<geometry_msgs::Point>cluster;
    std::vector<float> hdngs;
    std::vector<bool> is_clear;
    std::vector<float> hdng_start_obs;
    std::vector<float> hdng_end_obs;
    std::vector<float> obstacle_clusters_r;
    std::vector<float> obstacle_clusters_z;
    std::vector<float> cleared_clusters_ends;
    std::vector<float> obstacle_clusters_starts;
    std::vector<float> obstacle_clusters_ends;
    std::vector<float> cleared_clusters_starts;

    std::vector<geometry_msgs::Point> obstacle_clusters_center;
    std::vector<std::vector<geometry_msgs::Point>>cleared_clusters;
		std::vector<std::vector<geometry_msgs::Point>>obstacle_clusters;
    bool last_clear = false;
    float zmx = 0;
    bool clear;
    float r_sum = 0;
    int largest_cleared_cluster = 0;
    int largest_obstacle_cluster = 0;

    scanin = get_ranges(scanin,15);
    float cleared_lim = 30;
    if(std::isinf(scanin.ranges[0]))
      last_clear = true;
    if(last_clear)
      cleared_clusters_starts.push_back(scanin.angle_min);
    else
      obstacle_clusters_starts.push_back(scanin.angle_min);
    int count_ten = 0;
    for(int i  = 1; i < scanin.ranges.size()-1; i++){
      geometry_msgs::PointStamped pnt,pnt_out;
      float current_angle = scanin.angle_min + scanin.angle_increment * i;
      hdngs.push_back(current_angle);
      //ROS_INFO("Current angle: %.2f range: %.2f cluster: %i",current_angle,scanin.ranges[i],cluster.size());
      clear = false;
      pnt.point.x = scanin.ranges[i] * cos(current_angle);
      pnt.point.y = scanin.ranges[i] * sin(current_angle);
      pnt.header.frame_id = scanin.header.frame_id;
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      if(scanin.ranges[i] > scanin.ranges[i+1] && scanin.ranges[i] > scanin.ranges[i-1] && scanin.ranges[i] > cleared_lim && scanin.ranges[i] < scanin.range_max){
        obstacle_clusters_center.push_back(pnt_out.point);
        cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,200),1);
      }
      count_ten++;
      if(scanin.ranges[i] > cleared_lim)
        clear = true;
      if(count_ten > 10){
        count_ten = 0;
      if(clear)
        draw_line(pos,current_angle,scanin.ranges[i],get_color(relval(0,50,scanin.ranges[i]),relval(0,50,scanin.ranges[i]),0));
      else
        draw_line(pos,current_angle,scanin.ranges[i],get_color(0,0,relval(0,50,scanin.ranges[i])));
      }

    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,0),1);

        // std::isinf(scanin.ranges[i-1]) && std::isinf(scanin.ranges[i]) && std::isinf(scanin.ranges[1+i]))


    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(0,0,relval(0,50,pnt_out.point.z)),1);

      is_clear.push_back(clear);
      if(last_clear && !clear){
        zmx = 0;
        cleared_clusters_ends.push_back(current_angle);
        obstacle_clusters_starts.push_back(current_angle);
        cleared_clusters.push_back(cluster);
        if(cluster.size() > largest_cleared_cluster)
          largest_cleared_cluster = cluster.size();
        cluster.resize(0);
      }
      else if(!last_clear && clear){
        float ranges_ave = r_sum / cluster.size();
        r_sum = 0;
        obstacle_clusters_z.push_back(zmx);
        obstacle_clusters_r.push_back(ranges_ave);
        if(cluster.size() > largest_obstacle_cluster)
          largest_obstacle_cluster = cluster.size();
        obstacle_clusters_ends.push_back(current_angle);
        cleared_clusters_starts.push_back(current_angle);
        obstacle_clusters.push_back(cluster);
        cluster.resize(0);
      }
      else if(last_clear && clear){

      }
      else if(!last_clear && !clear){

      }
      else{
        ROS_INFO("ERROR - impossible combinatio (CLEAR vs OBST)");
      }

      cluster.push_back(pnt_out.point);
      last_clear = clear;
      if(!clear){
        r_sum += scanin.ranges[i];
        if(pnt_out.point.z > zmx)
          zmx = pnt_out.point.z;
      }
    }
  for(int i  = 0; i < cleared_clusters.size(); i++){
    int num  = cleared_clusters[i].size();
    float a0 = cleared_clusters_starts[i];
    float aN = cleared_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    draw_line(pos,a,50,get_color(0,200*num/largest_cleared_cluster,0));
    float ad = aN - a0;
    ROS_INFO("Cluster[%i], size: %i a: %.2f (delta. %.2f rads) (a0 %.2f->%.2f an)",i,num,a,ad,a0,aN);
  }
  for(int i  = 0; i < obstacle_clusters.size(); i++){
    int num  = obstacle_clusters[i].size();
    float a0 = obstacle_clusters_starts[i];
    float aN = obstacle_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    float r  = (aN+a0)/2.0;
    float z  = (aN+a0)/2.0;
    draw_line(pos,a,r,get_color(0,0,200*num/largest_obstacle_cluster));

    ROS_INFO("Obstacles[%i], size: %i a: %.2f, zmax: %.2f dst_ave: %.2f",i,num,a,obstacle_clusters_z[i],obstacle_clusters_r[i]);
  }
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
std::vector<nav_msgs::Path> get_pathvector(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi){
  std::vector<nav_msgs::Path> paths;
  paths.push_back(p_dn);
  paths.push_back(p_lo);
  paths.push_back(p_mi);
  paths.push_back(p_up);
  paths.push_back(p_hi);
  return paths;
}
std::vector<nav_msgs::Path> get_paths_in_a0aN(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi,float a0, float aN){
  nav_msgs::Path p_dn_h = cutoff_abs(p_dn,"hdng",a0,aN);
  nav_msgs::Path p_lo_h = cutoff_abs(p_lo,"hdng",a0,aN);
  nav_msgs::Path p_mi_h = cutoff_abs(p_mi,"hdng",a0,aN);
  nav_msgs::Path p_up_h = cutoff_abs(p_up,"hdng",a0,aN);
  nav_msgs::Path p_hi_h = cutoff_abs(p_hi,"hdng",a0,aN);
  std::vector<nav_msgs::Path> paths;
  paths.push_back(p_dn_h);
  paths.push_back(p_lo_h);
  paths.push_back(p_mi_h);
  paths.push_back(p_up_h);
  paths.push_back(p_hi_h);
  return paths;
}

geometry_msgs::PointStamped analyze_merged_pathsegment0(nav_msgs::Path pathin){
  std::vector<float> v1 = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> v2 = get_vec_attribute(pathin,"z");
  std::vector<float> v3 = get_vec_attribute(pathin,"neighbours");
  float ave_dst = vec_to_ave(v1);
  float ave_z   = vec_to_ave(v2);
  float ave_n   = vec_to_ave(v3);
  float mx_dst = vec_to_max(v1);
  float mx_z   = vec_to_max(v2);
  float mx_n   = vec_to_max(v3);
  float mn_dst = vec_to_min(v1);
  float mn_z   = vec_to_min(v2);
  float mn_n   = vec_to_min(v3);
  float dz   = mx_z - mn_z;
  float ddst = mx_dst - mn_dst;
  geometry_msgs::PointStamped pave;
  pave.point = get_ave_pnt(pathin);
  pave.point.z = mx_z;
  ROS_INFO("analyze_merged_pathsegment dst: [%.2f->%.2f: %.2f] z: [%.2f->%.2f: %.2f] neighbours: [%.2f->%.2f: %.2f] tot: [%i] ",mn_dst,mx_dst,ave_dst,mn_z,mx_z,ave_z,mn_n,mx_n,ave_n,pathin.poses.size());
  if(mx_n > 2 && ddst < dz*3)
    pave.header.frame_id = "wall";
  else if(dz < 3){
    pave.header.frame_id = "plane";
  }
  else{
    pave.header.frame_id = "mess";
  }
  return pave;
}
void analyze_merged_pathsegment(nav_msgs::Path pathin){
  pathin = sort_path(pathin,"hdng");
  float current_heading = get_hdng(pathin.poses[0].pose.position,base_pose.pose.position);
  nav_msgs::Path  current_pathsegment;
  for(int i = 0; i < pathin.poses.size(); i++){
    float candidate_heading = get_hdng(pathin.poses[i].pose.position,base_pose.pose.position);
    float delta_hdng        = get_shortest(candidate_heading,current_heading);
    int delta_hdng_degs     = abs(delta_hdng * rad2deg);
    if(delta_hdng_degs > 10){
      current_heading = candidate_heading;
      ROS_INFO("PATH_COMB[%i / %i] - hdng: %.2f (%i degs change)",i,pathin.poses.size(),candidate_heading,delta_hdng_degs);
      ROS_INFO("PATH_COMB-segment: %i",current_pathsegment.poses.size());
      current_pathsegment = sort_path(current_pathsegment,"dst_2d");
      float sum_dst2d = 0; float sum_dz = 0;
      for(int k = 0; k < current_pathsegment.poses.size(); k++){
        float dst2d = get_dst2d(current_pathsegment.poses[k].pose.position,base_pose.pose.position);
        float dz    = current_pathsegment.poses[k].pose.position.z - base_pose.pose.position.z;
        sum_dst2d += dst2d;
        sum_dz += dz;
      }
      current_pathsegment.poses.resize(0);
    }
    current_pathsegment.poses.push_back(pathin.poses[i]);
  }
}
std::vector<float> get_vec_attribute_indexes(nav_msgs::Path pathin,std::vector<float> vec_attribute,std::vector<int> indexes){
  std::vector<float> vals;
  vals.resize(indexes.size());
   for(int i = 0; i < indexes.size(); i++){
    vals[i] = vec_attribute[indexes[i]];
  }
  return vals;
}
void draw_background(nav_msgs::Path pathin){
 bool draw_density,draw_z_ave;
 float zmin = get_zmin(pathin);
 float zmax = get_zmax(pathin);
 std::vector<float> vec_z = get_vec_attribute(pathin,"z");
 for(int x = 0; x < 100; x++){
   for(int y = 0; y < 100; y++){
     geometry_msgs::Point p;
     p.x = pos.x - 50 + x;
     p.y = pos.y - 50 + y;
     std::vector<int> neighbours = getinpath_neighbours_point(pathin,p,2);
     std::vector<float> zvals    = get_vec_attribute_indexes(pathin,vec_z,neighbours);

     float z_ave = vec_to_ave(zvals);
     float z_max = vec_to_max(zvals);
     float density = neighbours.size();
     cv::Scalar color;
     color[0] = 100 * neighbours.size() / 50;
     color[1] = 100 * (z_ave - zmin) / (zmax - zmin);
     //ROS_INFO("Neighbours: %i ", neighbours.size());
   //  color[2] = 100 * (z_max - zmin) / (zmax - zmin);
     img.at<cv::Vec3b>( y2r(p.y,img.rows,1),x2c(p.x,img.cols,1) )[0] = color[0];
     img.at<cv::Vec3b>( y2r(p.y,img.rows,1),x2c(p.x,img.cols,1) )[1] = color[1];
     img.at<cv::Vec3b>( y2r(p.y,img.rows,1),x2c(p.x,img.cols,1) )[2] = color[2];
   }
 }
}
float get_rel_score(std::vector<float> scores, float score){
  geometry_msgs::Point minmax = max_score(scores);
  return (score - minmax.y) / (minmax.x - minmax.y);
}

cv::Scalar get_rel_color(std::vector<float> scores, float score, std::string rgb){
  cv::Scalar c;
  float relscore = get_rel_score(scores,score);
  if(rgb == "r")
    c[2] = relscore;
  if(rgb == "g")
    c[1] = relscore;
  if(rgb == "b")
    c[0] = relscore;
  return c;
 }

void draw_path_by_attribute(nav_msgs::Path pathin,std::string type){
 geometry_msgs::Point maxmin;
 std::vector<float> vals = get_vec_attribute(pathin,type);
 maxmin.x = vec_to_max(vals);
 maxmin.y = vec_to_min(vals);
 geometry_msgs::Point pyaw,prect0,prect1,pnt;

 if(pathin.poses.size() < 1)
   return;
 for(int i = 0; i < pathin.poses.size(); i++){
   pnt = pathin.poses[i].pose.position;
   cv::Scalar color;
   float rel_score = (vals[i] - maxmin.y) / (maxmin.x - maxmin.y);
   if(type == "z" && vals[i] > pos.z-1)
     color[2] = rel_score*255;
   else
     color[1] = rel_score*255;
   img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
   img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
   img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
 }
}
void draw_point_by_z(geometry_msgs::Point pnt,float zmax,float zmin,bool circle){
  cv::Scalar color;
  float rel_score = (pnt.z - zmin) / (zmax - zmin);
  if(pnt.z > pos.z-1)
    color[0] = rel_score*255;
  else
    color[0] = rel_score*255;
  if(circle)
    cv::circle(img,pnt2cv(pnt),1,color,1);
  else{
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
  }
}
geometry_msgs::Point path2point(nav_msgs::Path pathin){
  geometry_msgs::Point ave_pnt = get_ave_pnt(pathin);
  ave_pnt.z = get_zmax(pathin);
  return ave_pnt;
}
void eval_area_simple(nav_msgs::Path pathin,float hdng_cutoff){
  int num_is = 64;
  float tot_rads = hdng_cutoff * 2;
  float rads_pr_i = tot_rads / num_is;
  float aN;
  float a0 = -hdng_cutoff;
  std::vector<float> vals = get_vec_attribute(pathin,"z");
  float zmx = vec_to_max(vals);
  float zmn = vec_to_min(vals);
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(num_is*2+2);
  poly.header = hdr();
  for(int i = 0; i < num_is; i++){
    aN = a0 + rads_pr_i;
    nav_msgs::Path path_hdng = cutoff_abs(pathin,"hdng",a0,aN);
    a0 = aN;

    geometry_msgs::Point dst_max_pnt = get_maxmin_in_path(path_hdng,true,"dst_2d");
    geometry_msgs::Point dst_min_pnt = get_maxmin_in_path(path_hdng,false,"dst_2d");
    geometry_msgs::Point z_max   = get_maxmin_in_path(path_hdng,true,"z");
    geometry_msgs::Point z_min   = get_maxmin_in_path(path_hdng,false,"z");
    float dst_maxmin = get_dst2d(dst_max_pnt,dst_min_pnt);
    float dst_max = get_dst2d(dst_max_pnt,pos);
    float dst_min = get_dst2d(dst_min_pnt,pos);
    draw_point_by_z(dst_max_pnt,zmx,zmn,true);
    draw_point_by_z(dst_min_pnt,zmx,zmn,false);
    draw_point_by_z(z_max,zmx,zmn,false);
    draw_point_by_z(z_min,zmx,zmn,false);
    ROS_INFO("analyze_merged_pathsegment: hdng: %.2f z: [%.2f->%.2f: %.2f] dst: [%.2f->%.2f: %.2f] tot: [%i] ",aN,z_min.z,z_max.z,z_max.z-z_min.z,dst_min,dst_max,dst_maxmin,pathin.poses.size());
    /*std::vector<nav_msgs::Path> paths_a0aN = get_paths_in_a0aN(p_dn,p_lo,p_mi,p_up,p_hi,a0,aN);
    hdngs.push_back(a0 + aN / 2.0);
    ROS_INFO("PATHS_A0AN[%i] (%.2f -> %.2f) p1: %i p2: %i p3: %i p4: %i p5: %i ",a0,aN,paths_a0aN.size(),paths_a0aN[0].poses.size(),paths_a0aN[1].poses.size(),paths_a0aN[2].poses.size(),paths_a0aN[3].poses.size(),paths_a0aN[4].poses.size());
    geometry_msgs::PointStamped pave = analyze_merged_pathsegment0(path_comb);
    //analyze_merged_pathsegment(path_comb);
    if(paths_a0aN[4].poses.size() > 0)
      is_hi[i] = true;
    */
    poly.polygon.points[i].x = dst_max_pnt.x;
    poly.polygon.points[i].y = dst_max_pnt.y;
    poly.polygon.points[i].z = dst_max_pnt.z;
    poly.polygon.points[poly.polygon.points.size()-1-i].x = dst_min_pnt.x;
    poly.polygon.points[poly.polygon.points.size()-1-i].y = dst_min_pnt.y;
    poly.polygon.points[poly.polygon.points.size()-1-i].z = dst_min_pnt.z;
  }
  //draw_poly(poly,get_color(100,100,0));
}
void draw_basepaths(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi){
  draw_path_at_img(p_dn,pos,false,false,false,false,true,c_dn,1);
  draw_path_at_img(p_lo,pos,false,false,false,false,true,c_lo,1);
  draw_path_at_img(p_mi,pos,false,false,false,false,true,c_mi,1);
  draw_path_at_img(p_up,pos,false,false,false,false,true,c_up,1);
  draw_path_at_img(p_hi,pos,false,false,false,false,true,c_hi,1);
}
void get_above_pos(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi){
  nav_msgs::Path path_all   = merge_paths(get_pathvector(p_dn,p_lo,p_mi,p_up,p_hi));
  nav_msgs::Path path_above_z = cutoff_abs(path_all,"zrel",-1,100);
  nav_msgs::Path path_below_z = cutoff_abs(path_all,"zrel",-100,-1);
  draw_path_at_img(path_above_z,pos,false,false,false,false,true,get_color(0,0,200),1);
  draw_path_at_img(path_below_z,pos,false,false,false,false,true,get_color(0,200,0),1);
  drawimg(M_PI/3,"above_pos");
}
void get_hi(nav_msgs::Path p_hi,nav_msgs::Path p_hic,float hdng_cutoff){
  p_hi         = cutoff_abs(p_hi,"hdng",-M_PI/3,M_PI/3);
  p_hic        = cutoff_abs(p_hic,"hdng",-M_PI/3,M_PI/3);
  nav_msgs::Path path_above_z = cutoff_abs(p_hi,"zrel",-1,100);
  nav_msgs::Path p_hicabove_z = cutoff_abs(p_hic,"zrel",-1,100);
  draw_path_at_img(path_above_z,pos,false,false,false,false,true,get_color(0,0,200),1);
  draw_path_at_img(p_hicabove_z,pos,false,false,false,false,true,get_color(200,0,200),1);
  drawimg(hdng_cutoff,"above_z");
}
void get_zabove(nav_msgs::Path pathin,float z0,float zN,float hdng_cutoff){
  nav_msgs::Path path_above_z = cutoff_abs(pathin,"zrel",5,-6);
  draw_path_at_img(path_above_z,pos,false,false,false,false,true,get_color(0,0,200),1);
  drawimg(hdng_cutoff,"above_z");
}/*
std::deque<int> q{5, 1, 3};
std::deque<int>::iterator it = std::min_element(q.begin(), q.end());
std::vector<int>::iterator result = std::min_element(v.begin(), v.end());
#include <iostream>
#include <queue>
float height,yaw;
std::queue<float> sonar_down;
std::queue<float> vision_down;
vision_down.push(msg->data);
vision_down_sum += msg->data;
if(vision_down.size() > smoothingWindowSize)
{
  vision_down_sum -= vision_down.front();
  vision_down.pop();
}  */
void check(){
  float hdng_cutoff = M_PI/2;
  counter++;
  ros::Time t0 = ros::Time::now();
 //sensor_msgs::LaserScan scan_comb = merge_scans(scan_dn,scan_mi,scan_up);
  nav_msgs::Path path_lo  = get_pathfinal(scan_lo);
  nav_msgs::Path path_mi  = get_pathfinal(scan_mi);
  nav_msgs::Path path_hi  = get_pathfinal(scan_hi);
  nav_msgs::Path path_up  = get_pathfinal(scan_up);
  nav_msgs::Path path_dn  = get_pathfinal(scan_dn);
  nav_msgs::Path path_st  = get_pathfinal(scan_st);
  nav_msgs::Path path_hic = get_pathfinalclear(scan_hi);
  nav_msgs::Path path_all = merge_paths(get_pathvector(path_lo,path_mi,path_hi,path_up,path_dn));
  nav_msgs::Path path_all_mi = cutoff_abs(path_all,"hdng",-hdng_cutoff,hdng_cutoff);
  //eval_area_simple(path_all,hdng_cutoff);
  //draw_path_by_attribute(path_all_mi,"inclination");
  //drawimg(hdng_cutoff,"inclination");
  //drawimg(hdng_cutoff,"initscan");
  draw_basepaths(path_lo,path_mi,path_hi,path_up,path_dn);
  std::vector<float> vals = get_vec_attribute(path_all_mi,"z");

  float zmx = vec_to_max(vals);
  float zmn = vec_to_min(vals);
  geometry_msgs::Point z_max = get_maxmin_in_path(path_all_mi,true,"z");
  geometry_msgs::Point z_min = get_maxmin_in_path(path_all_mi,false,"z");
  draw_point_by_z(z_max,zmx,zmn,true);
  draw_point_by_z(z_min,zmx,zmn,false);
  drawimg(hdng_cutoff,"smax");
  initialscan(scan_hi);
  drawimg(hdng_cutoff,"initscan1");
  initialscan2(scan_hi);
  drawimg(hdng_cutoff,"initscan2");

  //get_hi(path_hi,path_hic,hdng_cutoff);

//  drawimg(hdng_cutoff,"base");

//  drawimg(hdng_cutoff,"all_z");
//get_above_pos(path_lo,path_mi,path_hi,path_up,path_dn);

  ROS_INFO("Path RAW: dn: %i lo: %i mi: %i hi: %i up: %i",path_dn.poses.size(),path_lo.poses.size(),path_mi.poses.size(),path_up.poses.size(),path_hi.poses.size());




/*  nav_msgs::Path path_dn_r  = cutoff_abs(path_dn,"hdng",hdng_cutoff/4,hdng_cutoff);
  nav_msgs::Path path_lo_r  = cutoff_abs(path_lo,"hdng",hdng_cutoff/4,hdng_cutoff);
  nav_msgs::Path path_mi_r  = cutoff_abs(path_mi,"hdng",hdng_cutoff/4,hdng_cutoff);
  nav_msgs::Path path_up_r  = cutoff_abs(path_up,"hdng",hdng_cutoff/4,hdng_cutoff);
  nav_msgs::Path path_hi_r  = cutoff_abs(path_hi,"hdng",hdng_cutoff/4,hdng_cutoff);

  nav_msgs::Path path_dn_0  = cutoff_abs(path_dn,"hdng",-hdng_cutoff/4,hdng_cutoff/4);
  nav_msgs::Path path_lo_0  = cutoff_abs(path_lo,"hdng",-hdng_cutoff/4,hdng_cutoff/4);
  nav_msgs::Path path_mi_0  = cutoff_abs(path_mi,"hdng",-hdng_cutoff/4,hdng_cutoff/4);
  nav_msgs::Path path_up_0  = cutoff_abs(path_up,"hdng",-hdng_cutoff/4,hdng_cutoff/4);
  nav_msgs::Path path_hi_0  = cutoff_abs(path_hi,"hdng",-hdng_cutoff/4,hdng_cutoff/4);

  nav_msgs::Path path_dn_l  = cutoff_abs(path_dn,"hdng",-hdng_cutoff,-hdng_cutoff/4);
  nav_msgs::Path path_lo_l  = cutoff_abs(path_lo,"hdng",-hdng_cutoff,-hdng_cutoff/4);
  nav_msgs::Path path_mi_l  = cutoff_abs(path_mi,"hdng",-hdng_cutoff,-hdng_cutoff/4);
  nav_msgs::Path path_up_l  = cutoff_abs(path_up,"hdng",-hdng_cutoff,-hdng_cutoff/4);
  nav_msgs::Path path_hi_l  = cutoff_abs(path_hi,"hdng",-hdng_cutoff,-hdng_cutoff/4);
  drawimg(hdng_cutoff,"raw");

  draw_path_at_img(path_dn_2,pos,false,false,false,false,true,c_dn,1);
  draw_path_at_img(path_lo_2,pos,false,false,false,false,true,c_lo,1);
  draw_path_at_img(path_mi_2,pos,false,false,false,false,true,c_mi,1);
  draw_path_at_img(path_up_2,pos,false,false,false,false,true,c_up,1);
  draw_path_at_img(path_hi_2,pos,false,false,false,false,true,c_hi,1);

  ROS_INFO("Path PROC: dn: %i lo: %i mi: %i hi: %i up: %i",path_dn_2.poses.size(),path_lo_2.poses.size(),path_mi_2.poses.size(),path_up_2.poses.size(),path_hi_2.poses.size());

  drawimg(hdng_cutoff,"proc");*/
  scan_mi.ranges.resize(0);
  scan_hi.ranges.resize(0);
  scan_lo.ranges.resize(0);
  scan_up.ranges.resize(0);
  scan_dn.ranges.resize(0);

  float dt_last_check = (ros::Time::now()-t0).toSec();
  ROS_INFO("CHECK COMPLETE IN %.4f",dt_last_check);
}
void rayranges_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_mi.ranges.size() == 0)
    scan_mi = *msg;
}
void rayranges_hi_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_hi.ranges.size() == 0)
    scan_hi = *msg;
}
void rayranges_lo_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_lo.ranges.size() == 0)
    scan_lo = *msg;
}
void scan_up_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_up.ranges.size() == 0)
    scan_up = *msg;
}

void scan_dn_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_dn.ranges.size() == 0)
    scan_dn = *msg;
}
void scan_stab_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(scan_st.ranges.size() == 0)
    scan_st = *msg;
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

void define_colors(){
  c_lo[0] = 200;
  c_lo[1] = 0;
  c_lo[2] = 0;

  c_dn[0] = 100;
  c_dn[1] = 100;
  c_dn[2] = 0;

  c_mi[0] = 0;
  c_mi[1] = 50;
  c_mi[2] = 50;

  c_up[0] =  0;
  c_up[1] =  100;
  c_up[2] =  100;

  c_hi[0] = 0;
  c_hi[1] = 0;
  c_hi[2] = 200;
}


void pathcleared_cb(const nav_msgs::Path::ConstPtr& msg){
  img_blank.copyTo(img);

  nav_msgs::Path path_ll  = cutoff_abs(*msg,"hdng",-M_PI,-M_PI/4);
  nav_msgs::Path path_l   = cutoff_abs(*msg,"hdng",-M_PI/4,-M_PI/12);
  nav_msgs::Path path_m   = cutoff_abs(*msg,"hdng",-M_PI/12,M_PI/12);
  nav_msgs::Path path_r   = cutoff_abs(*msg,"hdng",M_PI/12,M_PI/4);
  nav_msgs::Path path_rr  = cutoff_abs(*msg,"hdng",M_PI/4,M_PI);
  draw_path_at_img(path_ll,pos,false,false,false,false,true,c_dn,1);
  draw_path_at_img(path_l,pos,false,false,false,false,true,c_up,1);
  draw_path_at_img(path_m,pos,false,false,false,false,true,c_mi,1);
  draw_path_at_img(path_r,pos,false,false,false,false,true,c_up,1);
  draw_path_at_img(path_rr,pos,false,false,false,false,true,c_dn,1);
  drawimg(M_PI,"cleared");
}
  void poylcleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  //ROS_INFO("CBpolycb");
  poly_cleared = *msg;

  //poly_cleared_centroid      = get_poly_centroidarea(poly_cleared);
  //poly_cleared_centroid_area = poly_cleared_centroid.z;
  //poly_cleared_centroid.z    = base_pose.pose.position.z;
}
void cmdpos_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  cmd_pos = *msg;
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_scanproc_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param("setpoint_scanrange", par_scanrange, 25.0);//*2.0);
    private_nh.param("setpoint_zclearing", par_zclearing, 5.0);//*2.0);

    ros::Rate rate(10);

    tf2_ros::TransformListener tf2_listener(tfBuffer);

    tf2_ros::TransformBroadcaster tf_b;
    ros::Subscriber s10 = nh.subscribe("/tb_path/cleared_poly",1,poylcleared_cb);
    ros::Subscriber s0 = nh.subscribe("/tb_path/cleared",1,pathcleared_cb);
    ros::Subscriber as1       = nh.subscribe("/velodyne_scan",10,rayranges_cb);
    ros::Subscriber a1        = nh.subscribe("/scan_down",10,scan_dn_cb);
    ros::Subscriber a3        = nh.subscribe("/scan_up",10,scan_up_cb);
    ros::Subscriber a6        = nh.subscribe("/scan_stabilized",10,scan_stab_cb);
    ros::Subscriber a2        = nh.subscribe("/scan_tilt_up",10,rayranges_hi_cb);
    ros::Subscriber as4       = nh.subscribe("/scan_tilt_down",10,rayranges_lo_cb);
    ros::Subscriber sss       = nh.subscribe("/cmd_pos",               100,&cmdpos_cb);
    float z_err_last,s_err_last,t_err_last;

    ros::Time time_last = ros::Time::now();
    ros::Time last_check;
    define_colors();
    double syaw;
    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();
      float dt_last_check = (ros::Time::now()-last_check).toSec();
      checktf();

      if(scan_mi.ranges.size() > 0 && scan_hi.ranges.size() > 0
      && scan_lo.ranges.size() > 0 && scan_up.ranges.size() > 0
      && scan_dn.ranges.size() > 0 && dt_last_check > 1.0){
        check();
        last_check = ros::Time::now();
      }
    }
    return 0;
}
