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

cv::Scalar c_lo,c_dn,c_mi,c_up,c_h,c_hi,c_sz;
geometry_msgs::PointStamped obs_p,obs_r,obs_l,obs_m,obs_b;

ros::Publisher pub_cmdexplore,pub_cmdmb,pub_target_dash,pub_path_best,pub_get_next_path,pub_cmd,pub_cmdpose;
geometry_msgs::PoseStamped pose_down,pose_side,target,base_pose,target_last,target_final,target_dash;
ros::Time activity_change,last_tilt,path_complete_time;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point cmd_pos,poly_cleared_centroid,pos,pnt_midpoint,pnt_ref;
nav_msgs::Odometry odom;
nav_msgs::Path scanzone_path,path_ds_full,raw_full,path_raw_side,path_raw_down,path_target_full,path_clear_vlp,path_down_best_in_poly,path_side_best_in_poly,path_full,path_world_visible,path_down_best,path_side_best,path_vlp,path_obs,path_side_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_side,path_down;
int mainstate;
int counter = 0;
int blankdraw_counter = 0;
int path_targets_i = 0;
int inspection_count = 0;
double par_vlpmaxtilt,par_vlpmintilt,par_vlptiltinterval,par_takeoffaltitude;
bool path_complete,tiltlimit_reached,tilting,side_empty;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super_down(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super_side(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
float hdng_cutoff = M_PI/8;
bool override = false;
int count_target_paths = 0;
int targets_down_in_poly,targets_side_in_poly;
int dash_stage = 0;
bool got_one,got_one_raw;
float altmax,dst_target,poly_cleared_centroid_area,pos_yaw,scanrange_actual;
std::vector<int> targets_complete;
tb_msgsrv::Paths paths_active_down,down_in_poly;
tb_msgsrv::Paths paths_active_side,side_in_poly;
geometry_msgs::PolygonStamped poly_cleared,target_final_poly;
tb_msgsrv::Paths paths_down,paths_side;
double par_zclearing,par_scanrange;
geometry_msgs::Point scanpoint_actual2,scanpoint_actual,scanpoint_actual_lo,scanpoint_actual_hi;
float target_angle,setpoint_altitude,setpoint_scanrange;
nav_msgs::Path path_lo;
nav_msgs::Path path_mi;
nav_msgs::Path path_hi;
nav_msgs::Path path_up;
nav_msgs::Path path_dn;
nav_msgs::Path path_st;
nav_msgs::Path path_up_ri,path_up_le,path_mi_ri,path_mi_le,path_hi_ri,path_hi_le,path_lo_ri,path_lo_le,path_dn_ri,path_dn_le,path_st_ri,path_st_le;
bool use_old;
bool use_raw_for_target = true;
int override_tilt = 0;
int override_alt = 0;
int scanzone_size_pnts;
float scanzone_size_area,scanzone_size_ddst,scanzone_size_dalt,scanzone_dst_min,scanzone_dst_max,scanzone_z_min,scanzone_z_max;
geometry_msgs::Point scanzone_centroid,scanzone_avepnt;
geometry_msgs::Point scanzone_bbmin,scanzone_bbmax;
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
float relval(float min,float max,float val){
  return 255*(val - min) / (max - min);
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
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
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
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
nav_msgs::Path add_new(nav_msgs::Path path_old,nav_msgs::Path pathin){
  for(int i = 0; i < pathin.poses.size(); i++){
      path_old.poses.push_back(pathin.poses[i]);
  }
  return path_old;
}
bool is_within_incline(geometry_msgs::Point pnt,float maxdelta_pitch){
  float inclination       = get_inclination(pos,pnt);
  float delta_inclination = get_shortest(vlp_rpy.y,inclination);
  float pitch_shortest    = get_shortest(atan2(pnt.z-pos.z,get_dst2d(pnt,pos)),vlp_rpy.y);
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
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(pos,pathin.poses[i].pose.position)));
    else if(sort_by == "inclination")
        i_dst.push_back(std::make_tuple(i,get_shortest(vlp_rpy.y,get_inclination(pos,pathin.poses[i].pose.position))));
    else if(sort_by == "dst_3d_ref")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pnt_ref,pathin.poses[i].pose.position)));
    else if(sort_by == "neighbours")
      i_dst.push_back(std::make_tuple(i,getinpath_neighbours(pathin,i,2)));
    else if(sort_by == "dst_3d")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pos,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_abs")
			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw) * rad2deg)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw)));
    else if(sort_by == "zabs")
  		i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - pos.z)));
    else if(sort_by == "z")
      i_dst.push_back(std::make_tuple(i,pathin.poses[i].pose.position.z));
	}
      sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
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



geometry_msgs::Point max_score(std::vector<float> scores){
  geometry_msgs::Point maxmin;
  maxmin.x = -100;
  maxmin.y = 100;
  for(int i = 0; i < scores.size(); i++){
    if(maxmin.x < scores[i]){
      maxmin.x = scores[i];
    }
    if(maxmin.y > scores[i]){
      maxmin.y = scores[i];
    }
  }
  return maxmin;
}
float rel_pnt(geometry_msgs::Point maxmin,float score){
  return (255 * (score - maxmin.y) / (maxmin.x - maxmin.y));
}
float rel_score(std::vector<float> scores, float score){
  geometry_msgs::Point maxmin = max_score(scores);
  return rel_pnt(maxmin,score);
}

 std::vector<float> get_vec_attribute_indexes(nav_msgs::Path pathin,std::vector<float> vec_attribute,std::vector<int> indexes){
   std::vector<float> vals;
   vals.resize(indexes.size());
    for(int i = 0; i < indexes.size(); i++){
     vals[i] = vec_attribute[indexes[i]];
   }
   return vals;
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
 std::vector<float> get_vec_attribute(nav_msgs::Path pathin,std::string type){
   std::vector<float> vec_out;
   geometry_msgs::Point p0;
   for(int i = 0; i < pathin.poses.size(); i++){
     float val = 0;
     if(type == "inclination_abs")
       val = get_inclination(pos,pathin.poses[i].pose.position);
     else if(type == "dst_2d0")
       val = get_dst2d(p0,pathin.poses[i].pose.position);
     else if(type == "dst_2d")
       val = get_dst2d(pos,pathin.poses[i].pose.position);
     else if(type == "neighbours")
       val = getinpath_neighbours(pathin,i,3);
     else if(type == "inclination")
       val = get_inclination(pathin.poses[i].pose.position,pos);
     else if(type == "dst_3d_ref")
       val = get_dst3d(pnt_ref,pathin.poses[i].pose.position);
     else if(type == "dst_3d")
       val = get_dst3d(pos,pathin.poses[i].pose.position);
 		else if(type == "hdng_abs")
 			val = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z) * rad2deg);
    else if(type == "hdng_rel")
      val = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
    else if(type == "hdng")
       val = get_hdng(pathin.poses[i].pose.position,pos);
     else if(type == "zabs")
   		val = abs(pathin.poses[i].pose.position.z - pos.z);
     else if(type == "zrel")
     	val = pathin.poses[i].pose.position.z - pos.z;
     else if(type == "z")
       val = pathin.poses[i].pose.position.z;
     vec_out.push_back(val);
   }
   return vec_out;
 }
 float get_elevation_below2(){
   std::vector<int> neighbours = getinpath_neighbours_point(path_full,pos,10);
   nav_msgs::Path path_n;
   std::vector<float> zvals;

   for(int i= 0; i < neighbours.size(); i++){
     zvals.push_back(path_full.poses[neighbours[i]].pose.position.z);
   }
   float z_ave = vec_to_ave(zvals);
   float z_max = vec_to_max(zvals);
   float z_min = vec_to_max(zvals);
   return z_max;
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



bool pose_is_side(geometry_msgs::PoseStamped pose){
  if(pose.pose.orientation.y == 0.7071)
    return false;
  else
    return true;
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

void closest_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_p =*msg;
}
void closest_left_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_l =*msg;
}
void closest_right_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_r =*msg;
}
void closest_mid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_m =*msg;
}
void closest_blw_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_b =*msg;
}
void hdngclear_cb(const nav_msgs::Path::ConstPtr& msg){
  path_clear_vlp = *msg;
}


geometry_msgs::PolygonStamped get_polyfinal(float l0,float l1,float delta_hdng){
  geometry_msgs::PolygonStamped poly;
  poly.header.frame_id = "map";
  int num_rays    = 16;
  poly.polygon.points.resize(num_rays*2);
  float rads_pr_i = delta_hdng/num_rays;
  float hdng0     = get_hdng(target.pose.position,target_last.pose.position) - delta_hdng/2;
  for(int i = 0; i < num_rays; i++){
    float a = hdng0 + rads_pr_i * i;
    poly.polygon.points[i].x = target.pose.position.x + l0 * cos(a);
    poly.polygon.points[i].y = target.pose.position.y + l0 * sin(a);
    poly.polygon.points[poly.polygon.points.size()-1-i].x = target.pose.position.x + l1 * cos(a);
    poly.polygon.points[poly.polygon.points.size()-1-i].y = target.pose.position.y + l1 * sin(a);
  }
  return poly;
}
void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
	target_last = target;
  if(new_target_pose.pose.position.x != 0){
  	target.pose.position = new_target_pose.pose.position;
  	target.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(new_target_pose.pose.position,target_last.pose.position));
  	cmd_pos = new_target_pose.pose.position;
  	target_final_poly = get_polyfinal(1,30,M_PI);
    new_target_pose = target;
    new_target_pose.pose.position.z = 0;
    new_target_pose.header = hdr();
    target.header = hdr();
    //pub_cmdpose.publish(new_target_pose);
    if(!override)
      pub_cmd.publish(new_target_pose.pose.position);
  }
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

nav_msgs::Path cutoff_below(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = 0; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  //ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}

nav_msgs::Path cutoff_percentage(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = percent_length; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  //ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
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

void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
  if(msg->poses.size() > 0){
    base_pose = msg->poses[0];
	}
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
void poylcleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  poly_cleared = *msg;
}
void pntmid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
}
float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / get_dst2d(p1,p2);
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
	 dst_target = get_dst2d(target.pose.position,pos);
	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
  //q.getRPY(vlp_rpy.x,-vlp_rpy.y,vlp_rpy.z);
	pos_yaw = vlp_rpy.z;
}
void cmdpos_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	target_last = target;
	target.pose.position = *msg;
	cmd_pos = *msg;
	target_final_poly = get_polyfinal(1,15,M_PI/2);
}
void request_paths(){
  std_msgs::UInt8 get_path;
  pub_get_next_path.publish(get_path);
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
      float hdng_rel = get_shortest(hdng,pos_yaw);
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
void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
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

geometry_msgs::PointStamped get_danger(){
  geometry_msgs::PointStamped pnt_out;
  float dst_danger = 100;
  for(int i = 0; i < path_obs.poses.size(); i++){
    float dst = get_dst3d(path_obs.poses[i].pose.position,pos);
    if(path_obs.poses[i].header.frame_id == "closed"){
      pnt_out.header.frame_id = "danger";
      if(dst < dst_danger){
        dst_danger = dst;
        pnt_out.point = path_obs.poses[i].pose.position;
      }
    }
  }
  return pnt_out;
}


float get_elevation_below(){
  std::vector<geometry_msgs::Point> valid_points;
  if(obs_p.header.frame_id == "map")
    valid_points.push_back(obs_p.point);
  if(obs_m.header.frame_id == "map")
    valid_points.push_back(obs_m.point);
  if(obs_b.header.frame_id == "map")
    valid_points.push_back(obs_b.point);
  float highest_point = 0;
  for(int i = 0; i < valid_points.size(); i++){
    float hdng = get_shortest(get_hdng(valid_points[i],pos),pos_yaw);
    float dst  = get_dst2d(valid_points[i],pos);
    if(valid_points[i].z > highest_point)
      highest_point = valid_points[i].z;
    ROS_INFO("Alternative hdng: %.2f dst: %.2f point: %.0f %.0f %.0f",hdng,dst,valid_points[i].x,valid_points[i].y,valid_points[i].z);
  }
  return highest_point;
}
geometry_msgs::Point get_closepnts(){
  std::vector<geometry_msgs::Point> valid_points;
  if(obs_l.header.frame_id == "map")
    valid_points.push_back(obs_l.point);
  if(obs_r.header.frame_id == "map")
    valid_points.push_back(obs_r.point);
  if(obs_p.header.frame_id == "map")
    valid_points.push_back(obs_p.point);
  if(obs_m.header.frame_id == "map")
    valid_points.push_back(obs_m.point);
  if(obs_b.header.frame_id == "map")
    valid_points.push_back(obs_b.point);
  float lowest_dst = 0;
  geometry_msgs::Point best_point;
  if(valid_points.size() > 0)
    best_point = valid_points[0];
  for(int i = 0; i < valid_points.size(); i++){
    float dst  = get_dst2d(valid_points[i],pos);
    if(valid_points[i].z > best_point.z && best_point.z < pos.z)
      best_point = valid_points[i];
    else if(get_dst2d(best_point,pos) > get_dst2d(valid_points[i],pos))
      best_point = valid_points[i];
  }
  return best_point;
}

void draw_rectangle(geometry_msgs::Point midpoint,float size, cv::Scalar color){
  geometry_msgs::Point p0,p1;
  p0.x = midpoint.x-size; p1.x = midpoint.x+size*2;
  p0.y = midpoint.y-size; p1.y = midpoint.y+size*2;
  cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
}
void draw_scanzone(int size){
  geometry_msgs::Point p0,p1,scanzone_avepnt0,scanzone_centroid0,scanzone_avepnt1,scanzone_centroid1;

  p0.x = pos.x + scanzone_dst_min * cos(pos_yaw);
  p0.y = pos.y + scanzone_dst_min * sin(pos_yaw);

  p1.x = pos.x + scanzone_dst_max * cos(pos_yaw);
  p1.y = pos.y + scanzone_dst_max * sin(pos_yaw);

  cv::circle(img,pnt2cv(scanzone_centroid),size*2,get_color(200,0,0),1);
  cv::circle(img,pnt2cv(scanzone_avepnt),size*2,get_color(0,200,0),1);

  cv::line (img, pnt2cv(p0), pnt2cv(p1),c_sz,1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(scanzone_centroid),c_sz,1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(scanzone_centroid),c_sz,1,cv::LINE_8,0);

  draw_rectangle(scanzone_avepnt,size,c_sz);
  draw_rectangle(scanzone_centroid,size,c_sz);
  cv::rectangle(img, pnt2cv(scanzone_bbmin),pnt2cv(scanzone_bbmax),c_sz,1,8,0);
}
void drawimg(float hdng_cutoff,float a,std::string name){
  geometry_msgs::Point pyaw,pyaw_mn,pyaw_mx;
  float a1 = constrainAngle(a + hdng_cutoff);
  float a2 = constrainAngle(a - hdng_cutoff);
  float a0 = fmin(a1,a2);
  float aN = fmax(a1,a2);
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  draw_line(pos,pos_yaw,100,get_color(100,100,0));

  //ROS_INFO("vlp_rpy_Z: %.2f, hdng_cutoff: %.2f, a0: %.2f a1: %.2f",vlp_rpy.z,hdng_cutoff,a0,aN);
  draw_path_at_img(path_visited,pos,false,false,false,false,true,get_color(0,50,0),1);

  draw_poly(poly_cleared,get_color(200,200,200));
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),get_color(100,100,100),1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw_mn),get_color(200,0,200),1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw_mx),get_color(200,0,200),1,cv::LINE_8,0);
  cv::circle(img,pnt2cv(pos),3,get_color(200,200,200),1);
  cv::circle(img,pnt2cv(cmd_pos),3,get_color(0,0,200),1);
  cv::line (img, pnt2cv(pos), pnt2cv(cmd_pos),get_color(0,0,200),1,cv::LINE_8,2);
  draw_scanzone(2);
  cv::imwrite("/home/nuc/brain/scanproc/"+std::to_string(counter)+ name + ".png",img);
  img_blank.copyTo(img);
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
nav_msgs::Path create_path(float total_sidelength,float area_sidelength){
  nav_msgs::Path pathout;
  pathout.header = hdr();
	int num_grids = total_sidelength / area_sidelength-1;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z    = 10;
  pose.pose.orientation.w = 1;
  pose.header = hdr();
  for(int i = 0; i < num_grids; i++){
    for(int k = 0; k < i; k++){
			pose.pose.position.x += pow(-1,i) * area_sidelength;
      pathout.poses.push_back(pose);
    }
    for(int l = 0; l < i; l++){
      pose.pose.position.y += pow(-1,i) * area_sidelength;
      pathout.poses.push_back(pose);
    }
  }
	return pathout;
}
geometry_msgs::PolygonStamped create_gridpoly(geometry_msgs::Point midpoint, float dxy){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header = hdr();
  poly.polygon.points[0].x = round(midpoint.x + dxy);
  poly.polygon.points[0].y = round(midpoint.y + dxy);
  poly.polygon.points[1].x = round(midpoint.x - dxy);
  poly.polygon.points[1].y = round(midpoint.y + dxy);
  poly.polygon.points[2].x = round(midpoint.x - dxy);
  poly.polygon.points[2].y = round(midpoint.y - dxy);
  poly.polygon.points[3].x = round(midpoint.x + dxy);
  poly.polygon.points[3].y = round(midpoint.y - dxy);
  poly.polygon.points[4]   = poly.polygon.points[0];
 return poly;
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
      pathout.poses.push_back(path_full.poses[i]);
  }
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
std::vector<float> score_paths_by_attributes(std::vector<nav_msgs::Path> pathsin,std::vector<std::string> vec_attributes,std::vector<std::string> vec_attributes_type,std::vector<float> vec_weights){
  std::vector<float> vec_att_ave;
  std::vector<float> vec_att_max;
  std::vector<float> vec_att_min;
  std::vector<float> scores;
  std::vector<float> path_sizes;
  scores.resize(vec_attributes.size());
  vec_att_ave.resize(vec_attributes.size());
  vec_att_max.resize(vec_attributes.size());
  vec_att_min.resize(vec_attributes.size());
  for(int i = 0; i < pathsin.size(); i++){
    path_sizes.push_back(float(pathsin[i].poses.size()));
  }
  float size_max_all = vec_to_max(path_sizes);
  float size_min_all = vec_to_min(path_sizes);
  float s_r = size_max_all - size_min_all;

  nav_msgs::Path path_combined = merge_paths(pathsin);
  for(int k = 0; k < vec_attributes.size(); k++){
    std::vector<float> vec_att = get_vec_attribute(path_combined,vec_attributes[k]);
    vec_att_ave[k] = vec_to_ave(vec_att);
    vec_att_max[k] = vec_to_max(vec_att);
    vec_att_min[k] = vec_to_min(vec_att);
  }
  for(int i= 0; i < pathsin.size(); i++){
    float tot_score = 0;
    for(int k = 0; k < vec_attributes.size(); k++){
      float s = 0;
      if(vec_attributes[k] == "size")
          s = (path_sizes[i] - size_min_all) / s_r;
      else{
        std::vector<float> vec_att = get_vec_attribute(path_combined,vec_attributes[k]);
        if(vec_attributes_type[k] == "max")
          s   = (vec_to_max(vec_att) - vec_att_min[k]) / (vec_att_max[k] - vec_att_min[k]);
        else if(vec_attributes_type[k] == "ave")
          s   = (vec_to_ave(vec_att) - vec_att_min[k]) / (vec_att_max[k] - vec_att_min[k]);
        else if(vec_attributes_type[k] == "min")
          s   = (vec_to_min(vec_att) - vec_att_min[k]) / (vec_att_max[k] - vec_att_min[k]);
      }
      tot_score += s * vec_weights[k];
    }
    scores[i] = tot_score;
  }
  return scores;
}
void draw_grids(){
  std::vector<float> vz_full = get_vec_attribute(path_full,"z");
  float vz_max = vec_to_max(vz_full);
  float vz_min = vec_to_min(vz_full);
  float vz_rng = vz_max - vz_min;
  float dxy = 10;
  nav_msgs::Path path_core = create_path(300,dxy);
  nav_msgs::Path path_core_areas = create_path(300,50);
  float grid_area = dxy *dxy;
  int grids_empty = 0;
  int grids_visited = 0;
  int grids_partially_explored = 0;
  int grids_minimally_explored = 0;
  std::vector<float> grids_empty_area;
  std::vector<float> grids_partial_area;
  std::vector<float> grids_visited_area;
  grids_empty_area.resize(path_core_areas.poses.size());
  grids_partial_area.resize(path_core_areas.poses.size());
  grids_visited_area.resize(path_core_areas.poses.size());


/*
  std::vector<float> core_pnts;
  std::vector<float> core_zmax;
  std::vector<float> core_zmin;
  std::vector<float> core_density;
  std::vector<float> core_vstd;
  core_pnts.resize(path_core.poses.size());
  core_zmax.resize(path_core.poses.size());
  core_zmin.resize(path_core.poses.size());
  core_density.resize(path_core.poses.size());
  core_vstd.resize(path_core.poses.size());
  core_pnts[i] = 0;
  core_zmax[i] = 0;
  core_zmin[i] = 0;
  core_density[i] = 0;
  core_vstd[i] = 0;*/
  for(int i = 0; i < path_core.poses.size(); i++){

    int i_area = getinpath_closestindex3d(path_core_areas,path_core.poses[i].pose.position);
    geometry_msgs::PolygonStamped gridpoly = create_gridpoly(path_core.poses[i].pose.position,dxy-1);
    nav_msgs::Path path_in_grid = get_path_aroundpnt(path_full,path_core.poses[i].pose.position,dxy);
    if(path_in_grid.poses.size() == 0){
      grids_empty++;
      grids_empty_area[i_area] = grids_empty_area[i_area] + 1.0;
    //  draw_poly(gridpoly,get_color(50,0,0));
    }
    else{
      std::vector<float> vz_grid = get_vec_attribute(path_in_grid,"z");
      float rel_score  = (vec_to_max(vz_grid) - vz_min) / vz_rng;
      if(rel_score > 0.5)
        draw_path_by_score(path_in_grid,vz_grid,2,2,2,rel_score);
      else
        draw_path_by_score(path_in_grid,vz_grid,1,1,1,rel_score);
      nav_msgs::Path visited_in_grid = get_path_aroundpnt(path_visited,path_core.poses[i].pose.position,dxy);

      float density_in_grid = float(path_in_grid.poses.size()) / grid_area;
      float zmax_grid = vec_to_max(vz_grid);
      float zmin_grid = vec_to_min(vz_grid);
      float zrng_grid = zmax_grid-zmin_grid;


      ROS_INFO("DENSITY: %.2f size: %i,zmax_grid: %.0f zmin_grid: %.0f zrng_grid: %.0f",density_in_grid,path_in_grid.poses.size(),zmax_grid,zmin_grid,zrng_grid);
      if(visited_in_grid.poses.size() > 0){
        grids_visited++;
        grids_visited_area[i_area] = grids_visited_area[i_area] + 1.0;
      //  draw_poly(gridpoly,get_color(0,50,0));
      }
      else{
        grids_partial_area[i_area] = grids_partial_area[i_area] + 1.0;
        draw_poly(gridpoly,get_color(0,density_in_grid*20,0));
      //  draw_poly(gridpoly,get_color(0,density_in_grid*100,zrng_grid * 10));
      }
      if(density_in_grid > 3.0){
        draw_poly(gridpoly,get_color(255,255,255));

      }
    }
  }
  std::vector<float> dst_areas;
  std::vector<float> hdng_areas;
  std::vector<float> dst0_areas;
  std::vector<float> zto_areas;
  geometry_msgs::Point p0;
 for(int i = 0; i < path_core_areas.poses.size(); i++){
  //  nav_msgs::Path path_to_area = get_pathfull_aroundline(path_core_areas.poses[i].pose.position,pos);
    //std::vector<float> vz_path = get_vec_attribute(path_to_area,"z");
  //std::vector<float> vd_path = get_vec_attribute(path_to_area,"dst_2d");
    dst_areas.push_back(get_dst2d(path_core_areas.poses[i].pose.position,pos));
    dst0_areas.push_back(get_dst2d(path_core_areas.poses[i].pose.position,p0));
    hdng_areas.push_back(abs(get_shortest(get_hdng(path_core_areas.poses[i].pose.position,pos),pos_yaw))*10);
  //  dstknown_areas.push_back(vec_to_max(vd_path));
  //  zto_areas.push_back(vec_to_max(vz_path));
  }
  std::vector<float> mma_d2        = vec_to_min_max_ave(dst_areas);
  std::vector<float> mma_hdng      = vec_to_min_max_ave(hdng_areas);
  std::vector<float> mma_d2known   = vec_to_min_max_ave(dst0_areas);
//  std::vector<float> mma_zto_known = vec_to_min_max_ave(zto_areas);
  std::vector<float> mma_e = vec_to_min_max_ave(grids_empty_area);
  std::vector<float> mma_p = vec_to_min_max_ave(grids_partial_area);
  std::vector<float> mma_v = vec_to_min_max_ave(grids_visited_area);
  std::vector<float> scores;
  for(int i = 0; i < path_core_areas.poses.size(); i++){
    float rel_e  = (grids_empty_area[i] - mma_e[0]) / (mma_e[1]-mma_e[0]);
    float rel_p  = (grids_partial_area[i] - mma_p[0]) / (mma_p[1]-mma_p[0]);
    float rel_v  = (grids_visited_area[i] - mma_v[0]) / (mma_v[1]-mma_v[0]);
    float rel_d  = (dst_areas[i] - mma_d2[0]) / (mma_d2[1]-mma_d2[0]);
    float rel_h  = (hdng_areas[i] - mma_hdng[0]) / (mma_hdng[1]-mma_hdng[0]);
    float rel_d0  = (dst0_areas[i] - mma_d2known[0]) / (mma_d2known[1]-mma_d2known[0]);
    float rel_d01  = (dst0_areas[i] - vec_to_min(dst0_areas)) / (vec_to_max(dst0_areas)-vec_to_min(dst0_areas));
    //float score = 2*rel_e - 2*rel_d - rel_d0 - rel_h*0.5;
    ROS_INFO("rel_d0: %.2f rel_d01: %.2f ",rel_d0,rel_d01);

    float score = rel_e;
    ROS_INFO("score[%i]: %.2f rel_e: %.2f rel_d: -%.2f rel_d0: -%.2f rel_h: -%.2f ",i,score,rel_e,rel_d,rel_d0,rel_h);
    scores.push_back(score);
  //  ROS_INFO("Grid[%i], empty: %.2f partial: %.2f visited: %.2f",i,grids_empty_area[i],grids_partial_area[i],grids_visited_area[i]);
  }
  std::vector<float> mma_scores = vec_to_min_max_ave(scores);
  geometry_msgs::PoseStamped ps;

  for(int i = 0; i < path_core_areas.poses.size(); i++){
    geometry_msgs::PolygonStamped gridpoly = create_gridpoly(path_core_areas.poses[i].pose.position,24);
    float rel_score  = (scores[i] - mma_scores[0]) / (mma_scores[1]-mma_scores[0]);
    ROS_INFO(" TOTAL: %i, grids_empty_area %.2f,dst_areas %.2f,hdng_areas %.2f,dst0_areas %.2f score: %.2f (rel)", i,grids_empty_area[i],dst_areas[i],hdng_areas[i],dst0_areas[i],rel_score);
    if(rel_score == 1.0){
      ps = path_core_areas.poses[i];
      cv::circle(img,pnt2cv(path_core_areas.poses[i].pose.position),5,get_color(rel_score *200,rel_score *200,rel_score *200),1);
      draw_poly(gridpoly,get_color(rel_score *200,rel_score *200,rel_score *200));
    }
    else if(rel_score > 0.8){
      cv::circle(img,pnt2cv(path_core_areas.poses[i].pose.position),5,get_color(0,rel_score *200,rel_score *200),1);
      draw_poly(gridpoly,get_color(0,rel_score *200,rel_score *200));
    }
    else if(rel_score > 0.6){
      cv::circle(img,pnt2cv(path_core_areas.poses[i].pose.position),5,get_color(0,0,rel_score *200),1);
      draw_poly(gridpoly,get_color(0,0,rel_score *200));
    }
    else if(rel_score > 0.33){
      cv::circle(img,pnt2cv(path_core_areas.poses[i].pose.position),5,get_color(rel_score *200,0,0),1);
      draw_poly(gridpoly,get_color(rel_score *200,0,0));
    }
    else{
      cv::circle(img,pnt2cv(path_core_areas.poses[i].pose.position),5,get_color(rel_score *200,0,0),1);
      draw_poly(gridpoly,get_color(rel_score * 255,0,0));
    }
  }
  ps.header = hdr();
  float dst_target_2d = get_dst2d(target.pose.position,pos);
  if(dst_target_2d < 10){
    cv::circle(img,pnt2cv(ps.pose.position),1,get_color(0,0,255),1);
    cv::line(img,pnt2cv(pos),pnt2cv(ps.pose.position),get_color(0,0,255),1,cv::LINE_8,0);
  //  set_target_pose(ps);
  }
  drawimg(M_PI,pos_yaw,"grids"+ std::to_string(count_target_paths) );
}


void draw_paths_by_scores(std::vector<nav_msgs::Path> pathsin,std::vector<float> scores){
  float scores_max = vec_to_max(scores);
  float scores_min = vec_to_min(scores);
  for(int i = 0; i < pathsin.size(); i++){
    float rel_score  = (scores[i] - scores_min) / (scores_max-scores_min);
    std::vector<float> vvz = get_vec_attribute(pathsin[i],"z");
  //  ROS_INFO("REL SCORE: %.2f",rel_score);

  //    img_blank.copyTo(img);

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
//    cv::circle(img,pnt2cv(pos),3,get_color(200,200,200),1);
//    cv::line (img, pnt2cv(pos), pnt2cv(cmd_pos),get_color(0,0,200),1,cv::LINE_8,2);
//  cv::imwrite("/home/nuc/brain/scanproc/"+std::to_string(i)+ "path" + ".png",img);
  }
}


void scores_testing(std::vector<nav_msgs::Path> pathsin){
  std::vector<std::string> vec_att;
  std::vector<std::string> vec_att_type;
  std::vector<float> vec_weights;
  vec_att.push_back("inclination");
  vec_att_type.push_back("max");
  vec_weights.push_back(-1.0);
  vec_att.push_back("dst_2d");
  vec_att_type.push_back("max");
  vec_weights.push_back(-2.0);
  vec_att.push_back("dst_2d");
  vec_att_type.push_back("ave");
  vec_weights.push_back(-2.0);
  vec_att.push_back("size");
  vec_att_type.push_back("");
  vec_weights.push_back(-2.0);
  std::vector<float> scores = score_paths_by_attributes(pathsin,vec_att,vec_att_type,vec_weights);
}

void look_in_directions(float num_rays, float len, float dxy){
  float rads_pr_i = M_PI / num_rays;
  float min_yaw = constrainAngle(pos_yaw - M_PI/2) + rads_pr_i / 2;
  geometry_msgs::Point midpoint;
  std::vector<float> angles;
  std::vector<float> dangles;
  std::vector<float> path_empty_len;
  std::vector<float> path_vstd_len;
  std::vector<float> zaves;
  std::vector<float> zmaxs;
  std::vector<float> zmins;
  std::vector<float> inc_ave;
  std::vector<float> inc_max;
  std::vector<float> inc_min;
  std::vector<float> path_sizes;
  std::vector<nav_msgs::Path> paths_full_angles;
  std::vector<nav_msgs::Path> paths_vstd_angles;
  paths_full_angles.resize(num_rays);
  paths_vstd_angles.resize(num_rays);
  angles.resize(num_rays);
  dangles.resize(num_rays);
  zaves.resize(num_rays);
  zmaxs.resize(num_rays);
  zmins.resize(num_rays);
  inc_ave.resize(num_rays);
  inc_max.resize(num_rays);
  inc_min.resize(num_rays);
  path_vstd_len.resize(num_rays);
  path_empty_len.resize(num_rays);
  path_sizes.resize(num_rays);
  for(int i = 0; i < num_rays; i++){
    float a_i = constrainAngle(min_yaw + rads_pr_i * i);
    float cur_len = dxy;
    angles[i] = a_i;
    dangles[i] = -M_PI/2 + rads_pr_i * i;
    std::vector<nav_msgs::Path> paths_visited_ang;
    std::vector<nav_msgs::Path> paths_full_ang;
    while(cur_len < len){
      cur_len += dxy;
      midpoint.x = pos.x + cur_len*cos(a_i);
      midpoint.y = pos.y + cur_len*sin(a_i);
      nav_msgs::Path path_visited_pnt  = get_path_aroundpnt(path_visited,midpoint,dxy);
      nav_msgs::Path path_full_pnt     = get_path_aroundpnt(path_full,midpoint,dxy);
      paths_full_ang.push_back(path_full_pnt);
      paths_visited_ang.push_back(path_visited_pnt);

      if(path_full_pnt.poses.size() == 0){
        path_empty_len[i] = cur_len;
        break;
      }
      if(path_visited_pnt.poses.size() > 5){
        path_vstd_len[i] = cur_len;
        break;
      }
    }
    paths_vstd_angles[i] = merge_paths(paths_visited_ang);
    paths_full_angles[i] = merge_paths(paths_full_ang);
    path_sizes[i] = float(paths_vstd_angles[i].poses.size());
  }

  nav_msgs::Path path1 = merge_paths(paths_full_angles);
  std::vector<float> vdd = get_vec_attribute(path1,"dst_2d");
  std::vector<float> vzz = get_vec_attribute(path1,"z");
  std::vector<float> vii = get_vec_attribute(path1,"inclination");
  float zave_all = vec_to_ave(vzz);
  float zmax_all = vec_to_max(vzz);
  float zmin_all = vec_to_min(vzz);
  float inc_ave_all = vec_to_ave(vii);
  float inc_max_all = vec_to_max(vii);
  float inc_min_all = vec_to_min(vii);
  float dst_ave_all = vec_to_ave(vdd);
  float dst_max_all = vec_to_max(vdd);
  float dst_min_all = vec_to_min(vdd);
  float size_max_all = vec_to_max(path_sizes);
  float size_min_all = vec_to_min(path_sizes);

  float d_r = dst_max_all - dst_min_all;
  float z_r = zmax_all - zmin_all;
  float i_r = inc_max_all - inc_min_all;
  float s_r = size_max_all - size_min_all;

  //ROS_INFO("size: %i inc_ave %.2f inc_max: %.0f zaves: %.0f, zmaxs: %.2f, zmins: %.2f",path1.poses.size(),inc_ave_all,inc_max_all,inc_min_all,zave_all,zmax_all,zmin_all);

  int best_i = 0;
  float best_score = -1000;
  std::vector<float> tot_scores;
  tot_scores.resize(num_rays);
  for(int i = 0; i < num_rays; i++){
    std::vector<float> vd = get_vec_attribute(paths_full_angles[i],"dst_2d");
    std::vector<float> vz = get_vec_attribute(paths_full_angles[i],"z");
    std::vector<float> vi = get_vec_attribute(paths_full_angles[i],"inclination");
    float zmax_v = vec_to_max(vz);
    float zave_v = vec_to_ave(vz);
    float incm_v = vec_to_max(vi);
    float inca_v = vec_to_ave(vi);
    float dstm_v = vec_to_max(vd);
    float dsta_v = vec_to_ave(vd);

    float zmax_s   = (zmax_v - zmin_all) / z_r;
    float zave_s   = (zave_v - zmin_all) / z_r;
    float incm_s   = -1*(incm_v - inc_min_all) / i_r;
    float inca_s   = -1*(inca_v - inc_min_all) / i_r;
    float dstm_s   = -2*(dstm_v - dst_min_all) / d_r;
    float dsta_s   = -1*(dsta_v - dst_min_all) / d_r;
    float size_s   = -1*(path_sizes[i] - size_min_all) / s_r;
    float hdng_score = -3*abs(dangles[i]) / (M_PI/2);
    tot_scores[i] = zmax_s + zave_s + incm_s + inca_s + dstm_s + dsta_s + hdng_score + size_s;
    if(tot_scores[i] > best_score){
      best_score = tot_scores[i];
      best_i = i;
      //ROS_INFO("New Best Score [z: %.2f,%.2f], [inc: %.2f,%.2f],[dst: %.2f,%.2f],[ang: %.2f] ",zmax_v,zave_v,incm_v,inca_v,dstm_v,dsta_v,dangles[i]);
      //ROS_INFO("New Best Score [z: %.2f,%.2f], [inc: %.2f,%.2f],[dst: %.2f,%.2f],[size: %.2f][ang: %.2f]",path1.poses.size(),zmax_s,zave_s,incm_s,inca_s,dstm_s,dsta_s,size_s,hdng_score);
    }
  }
  draw_paths_by_scores(paths_full_angles,tot_scores);
  target_angle = dangles[best_i];
  geometry_msgs::PoseStamped ps;
  ps.pose.position.x = pos.x + 20*cos(angles[best_i]);
  ps.pose.position.y = pos.y + 20*sin(angles[best_i]);
  cv::circle(img,pnt2cv(ps.pose.position),1,get_color(0,0,255),1);
  cv::line(img,pnt2cv(pos),pnt2cv(ps.pose.position),get_color(0,0,255),1,cv::LINE_8,0);
  drawimg(M_PI,pos_yaw,"fullpath_scores"+ std::to_string(count_target_paths) );
  set_target_pose(ps);
}
void draw_path_full(){
  img_blank.copyTo(img);
  std::vector<float> v1 = get_vec_attribute(path_full,"z");
  std::vector<float> v2 = get_vec_attribute(path_full,"z");
  count_target_paths++;
  draw_path_by_score(path_full,v1,0,1,2,0.6);
  drawimg(M_PI,pos_yaw,"fullpath"+ std::to_string(count_target_paths) );
}
void update_path_full(){
  nav_msgs::Path path_scanned = merge_paths(get_pathvector(path_hi,path_lo,path_up,path_dn,path_mi));
  path_scanned = scatter_path(path_scanned,1);
  nav_msgs::Path path_full_inrad  = get_path_inrad(path_full,pos,50);
  nav_msgs::Path path_scanned_new = get_new_path(path_full_inrad,path_scanned,1);
  path_full = add_new(path_full,path_scanned_new);
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
geometry_msgs::PolygonStamped create_bbpoly(float dmin,float dmax,float z,float a0,float an){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header = hdr();
  poly.polygon.points[0].x = pos.x + dmin * cos(a0+pos_yaw);
  poly.polygon.points[0].y = pos.y + dmin * sin(a0+pos_yaw);
  poly.polygon.points[0].z = z;

  poly.polygon.points[1].x = pos.x + dmin * cos(an+pos_yaw);
  poly.polygon.points[1].y = pos.y + dmin * sin(an+pos_yaw);
  poly.polygon.points[1].z = z;

  poly.polygon.points[2].x = pos.x + dmax * cos(an+pos_yaw);
  poly.polygon.points[2].y = pos.y + dmax * sin(an+pos_yaw);
  poly.polygon.points[2].z = z;

  poly.polygon.points[3].x = pos.x + dmax * cos(a0+pos_yaw);
  poly.polygon.points[3].y = pos.y + dmax * sin(a0+pos_yaw);
  poly.polygon.points[3].z = z;

  poly.polygon.points[4] = poly.polygon.points[0];
  return poly;
}

void pathhdngcleared_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
}
void cutop_path(nav_msgs::Path pathin){
  nav_msgs::Path path_ll  = cutoff_abs(pathin,"hdng_rel",-M_PI,-M_PI/4);
  nav_msgs::Path path_l   = cutoff_abs(pathin,"hdng_rel",-M_PI/4,-M_PI/12);
  nav_msgs::Path path_m   = cutoff_abs(pathin,"hdng_rel",-M_PI/12,M_PI/12);
  nav_msgs::Path path_r   = cutoff_abs(pathin,"hdng_rel",M_PI/12,M_PI/4);
  nav_msgs::Path path_rr  = cutoff_abs(pathin,"hdng_rel",M_PI/4,M_PI);
  geometry_msgs::Point pnt_ll = get_ave_pnt(path_ll);
  geometry_msgs::Point pnt_l  = get_ave_pnt(path_l);
  geometry_msgs::Point pnt_m  = get_ave_pnt(path_m);
  geometry_msgs::Point pnt_r  = get_ave_pnt(path_r);
  geometry_msgs::Point pnt_rr = get_ave_pnt(path_rr);

  draw_path_at_img(path_ll,pos,false,false,false,false,true,get_color(200,200,0),1);
  draw_path_at_img(path_l,pos,false,false,false,false,true,get_color(200,0,200),1);
  draw_path_at_img(path_m,pos,false,false,false,false,true,get_color(200,100,200),1);
  draw_path_at_img(path_r,pos,false,false,false,false,true,get_color(200,0,200),1);
  draw_path_at_img(path_rr,pos,false,false,false,false,true,get_color(200,200,0),1);
  drawimg(M_PI/4,pos_yaw,"targets");
  float fac1 = 1.5;
  float fac2 = 2.5;
  int size_rr = path_rr.poses.size(); int size_ll = path_ll.poses.size();
  int size_r = path_r.poses.size() * fac1; int size_l = path_l.poses.size() * fac1;
  int size_m = path_m.poses.size() * fac2;
  ROS_INFO("Size  ll: %i l: %i m: %i r: %i rr: %i",path_ll.poses.size(),path_l.poses.size(),path_m.poses.size(),path_r.poses.size(),path_rr.poses.size());
  ROS_INFO("Score ll: %i l: %i m: %i r: %i rr: %i",size_ll,size_l,size_m,size_r,size_rr);
  geometry_msgs::Point pnt;
  if(size_rr > size_ll && size_rr > size_l && size_rr > size_m && size_rr > size_r)
    pnt = get_ave_pnt(path_rr);
  else if(size_ll > size_rr && size_ll > size_l && size_ll > size_m && size_ll > size_r)
    pnt = get_ave_pnt(path_rr);
  else if(size_l > size_rr && size_l > size_r && size_l > size_m && size_l > size_ll)
    pnt = get_ave_pnt(path_rr);
  else if(size_r > size_rr && size_r > size_l && size_r > size_m && size_r > size_ll)
    pnt = get_ave_pnt(path_rr);
  else
    pnt  = get_ave_pnt(path_m);
  ROS_INFO("Pnt: %.0f %.0f %.0f",pnt.x,pnt.y,pnt.z);
  if(std::isnan(pnt.x) || std::isnan(pnt.y) || std::isnan(pnt.z)){
    ROS_INFO("NANWARN");
  }
}
void pathcleared_cb(const nav_msgs::Path::ConstPtr& msg){
  path_clear_vlp = *msg;
}

geometry_msgs::PolygonStamped path_to_bb_poly(nav_msgs::Path pathin,bool use_data){
  std::vector<float> vals_dst  = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> vals_z    = get_vec_attribute(pathin,"z");
  std::vector<float> vals_hdng = get_vec_attribute(pathin,"hdng_rel");

  float av_dst = vec_to_ave(vals_dst);
  float mn_dst = vec_to_min(vals_dst);
  float mx_dst = vec_to_max(vals_dst);
  float av_z = vec_to_ave(vals_z);
  float mn_z = vec_to_min(vals_z);

  float mx_z = vec_to_max(vals_z);
  float av_hdng = vec_to_ave(vals_hdng);
  float mn_hdng = vec_to_min(vals_hdng);
  float mx_hdng = vec_to_max(vals_hdng);

  geometry_msgs::PolygonStamped poly_bottom = create_bbpoly(mn_dst,mx_dst,mn_z,mn_hdng,mx_hdng);
  geometry_msgs::PolygonStamped poly_top    = create_bbpoly(mn_dst,mx_dst,mx_z,mn_hdng,mx_hdng);
  ROS_INFO("POLY FRONT: dst2d: %.0f -> %.0f a0aN: %.2f -> %.2f z0zN: %.0f -> %.0f",mn_dst,mx_dst,mn_hdng,mx_hdng,mn_z,mx_z);
  ROS_INFO("POLY FRONT: dst2d:     %.0f     a0aN:     %.2f          z0zN: %.0f",mx_dst-mn_dst,get_shortest(mn_hdng,mx_hdng),mx_z-mn_z);

  geometry_msgs::Point poly_centroid = get_poly_centroidarea(poly_bottom);
  float poly_area   = abs(poly_centroid.z);
  float poly_volume = poly_area * (mx_z - mn_z);
  poly_centroid.z = (mx_z + mn_z)/2.0;
  ROS_INFO("POLY FRONT: poly: area:  %.0f volume: %.0f centroid: [%.0f %.0f %.0f]",poly_area,poly_volume,poly_centroid.x,poly_centroid.y,poly_centroid.z);
  if(use_data){
    std::vector<geometry_msgs::Point> bbvec = getinpath_boundingbox(pathin);
    scanzone_bbmin = bbvec[0];
    scanzone_bbmax = bbvec[1];
    scanzone_size_pnts = pathin.poses.size();
    scanzone_size_area = poly_area;
    scanzone_size_ddst = mx_dst-mn_dst;
    scanzone_size_dalt = mx_z-mn_z;
    scanzone_avepnt   = get_ave_pnt(pathin);
    scanzone_centroid = poly_centroid;
    scanzone_dst_min = mn_dst;
    scanzone_dst_max = mx_dst;
    scanzone_z_min = mn_z;
    scanzone_z_max = mx_z;
    ROS_INFO("SCANZONE: pnts %i, mm: %.0f, ddst: %.0f, zdst: %.0f pnt_ave: [%.0f %.0f %.0f] pnt_cen: [%.0f %.0f %.0f]",scanzone_size_pnts,scanzone_size_ddst,scanzone_size_ddst,scanzone_size_dalt,scanzone_avepnt.x,scanzone_avepnt.y,scanzone_avepnt.z,scanzone_centroid.x,scanzone_centroid.y,scanzone_centroid.z);
  }
  return poly_bottom;
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
  pnt = tfBuffer.transform(pnt, "map");
  scanpoint_actual2 = pnt.point;
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

  c_sz[0] = 135;
  c_sz[1] = 125;
  c_sz[2] = 255;
}
void altmax_cb(const std_msgs::Float64::ConstPtr& msg){
  altmax = msg->data;
}


void check_front(){
  float front_cutoff = M_PI/12;
  nav_msgs::Path path_all = merge_paths(get_pathvector(path_mi,path_hi,path_lo,path_up,path_dn));

  nav_msgs::Path path_all_mi = cutoff_abs(path_all,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  nav_msgs::Path path_all_ri = cutoff_abs(path_all,"hdng_rel",-hdng_cutoff*2,-hdng_cutoff);
  nav_msgs::Path path_all_le = cutoff_abs(path_all,"hdng_rel",hdng_cutoff,hdng_cutoff*2);

  scanzone_path    = path_all_mi;

  scanpoint_actual = get_ave_pnt(scanzone_path);

  if(path_all_mi.poses.size() > 10 && path_all_ri.poses.size() > 10 && path_all_le.poses.size() > 10){
    std::vector<float> vals_mi = get_vec_attribute(path_all_mi,"z");
    std::vector<float> vals_ri = get_vec_attribute(path_all_ri,"z");
    std::vector<float> vals_le = get_vec_attribute(path_all_le,"z");

    draw_path_by_score(path_all_mi,vals_mi,0,2,0,1.0);
    draw_path_by_score(path_all_ri,vals_ri,0,1,0,1.0);
    draw_path_by_score(path_all_le,vals_le,0,1,0,1.0);

    geometry_msgs::PolygonStamped  poly_all_mi = path_to_bb_poly(path_all_mi,true);
    geometry_msgs::PolygonStamped  poly_all_ri = path_to_bb_poly(path_all_ri,false);
    geometry_msgs::PolygonStamped  poly_all_le = path_to_bb_poly(path_all_le,false);

    scanpoint_actual = get_ave_pnt(path_all_mi);
    scanrange_actual = get_dst3d(pos,scanpoint_actual);
    count_target_paths++;
  }
  std::string s ="points_front"+std::to_string(count_target_paths);
  drawimg(hdng_cutoff,pos_yaw,s);
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}

void override_cb(const std_msgs::String::ConstPtr& msg){
  override = true;
  if(msg->data == "tilt_up"){
    override_tilt++;
  }
  if(msg->data == "tilt_dn"){
    override_tilt--;
  }
  if(msg->data == "alt_up"){
    override_alt++;
  }
  if(msg->data == ""){
    override_alt = 10;
  }
  if(msg->data == "alt_dn"){
    override_alt--;
  }
}
float set_tilt_scanpoint(geometry_msgs::Point target_scanpoint){
  return get_inclination(target_scanpoint,pos);
}
float set_tilt_terrain(geometry_msgs::Point target_terrain){
  return get_inclination(target_terrain,pos);
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
    return -M_PI/14;
}
nav_msgs::Path offset_path(nav_msgs::Path pathin,geometry_msgs::Point offset_pnt){
  for(int i = 0; i < pathin.poses.size(); i++){
    pathin.poses[i].pose.position.x += offset_pnt.x;
    pathin.poses[i].pose.position.y += offset_pnt.y;
    pathin.poses[i].pose.position.z += offset_pnt.z;
  }
  return pathin;
}

nav_msgs::Path consider_targets(){
  float area_sidelength = 50;
  float dxy = 5;
  nav_msgs::Path path_cand   = offset_path(create_path(area_sidelength,dxy),pos);
                 path_cand   = cutoff_abs(path_cand,"hdng_rel",-M_PI/2,M_PI/2);

  nav_msgs::Path path_full_inrad       = get_path_inrad(path_full,pos,area_sidelength);
  nav_msgs::Path path_full_inrad_atlvl = cutoff_abs(path_full,"z",pos.z-1,100);
  std::vector<float> vz_full = get_vec_attribute(path_full_inrad,"z");
  float vz_max = vec_to_max(vz_full);
  float vz_min = vec_to_min(vz_full);
  float vz_rng = vz_max - vz_min;
  float grid_area = pow((dxy * 2),2);
    nav_msgs::Path path_cand_out;
  for(int i = 0; i < path_cand.poses.size(); i++){
    nav_msgs::Path path_in_grid = get_path_aroundpnt(path_full_inrad,path_cand.poses[i].pose.position,dxy);
    if(path_in_grid.poses.size() > 0){
      geometry_msgs::PolygonStamped gridpoly = create_gridpoly(path_cand.poses[i].pose.position,dxy-1);
      nav_msgs::Path path_in_grid = get_path_aroundpnt(path_full,path_cand.poses[i].pose.position,dxy);
      std::vector<float> vz_grid = get_vec_attribute(path_in_grid,"z");
      float rel_score  = (vec_to_max(vz_grid) - vz_min) / vz_rng;
      if(rel_score > 0.5)
        draw_path_by_score(path_in_grid,vz_grid,2,2,2,rel_score);
      else
        draw_path_by_score(path_in_grid,vz_grid,1,1,1,rel_score);
      nav_msgs::Path visited_in_grid = get_path_aroundpnt(path_visited,path_cand.poses[i].pose.position,dxy);
      float density_in_grid = float(path_in_grid.poses.size()) / grid_area;
      float zmax_grid = vec_to_max(vz_grid);
      float zmin_grid = vec_to_min(vz_grid);
      float zrng_grid = zmax_grid-zmin_grid;
      path_cand.poses[i].pose.position.z = zmax_grid;
      if(visited_in_grid.poses.size() == 0)
        path_cand_out.poses.push_back(path_cand.poses[i]);
      ROS_INFO("DENSITY: %.2f size: %i,zmax_grid: %.0f zmin_grid: %.0f zrng_grid: %.0f",density_in_grid,path_in_grid.poses.size(),zmax_grid,zmin_grid,zrng_grid);
    }
  }
  std::vector<float> vz_out = get_vec_attribute(path_cand_out,"z");
  draw_path_by_score(path_cand_out,vz_out,0,2,1,1);

  return path_cand_out;
}
std::vector<float> get_path_score(nav_msgs::Path pathin,float weight_hdng,float weight_z, float weight_dst2d, float weight_inclination, float weight_startdst){
  std::vector<float> vec_z = get_vec_attribute(pathin,"z");
  std::vector<float> vec_d = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> vec_0 = get_vec_attribute(pathin,"dst_2d0");
  std::vector<float> vec_h = get_vec_attribute(pathin,"hdng_abs");
  std::vector<float> vec_i = get_vec_attribute(pathin,"inclination");
  std::vector<float> mma_v = vec_to_min_max_ave(vec_z);
  std::vector<float> mma_d = vec_to_min_max_ave(vec_d);
  std::vector<float> mma_h = vec_to_min_max_ave(vec_h);
  std::vector<float> mma_i = vec_to_min_max_ave(vec_h);
  std::vector<float> mma_0 = vec_to_min_max_ave(vec_0);

  std::vector<float> scores;
  std::vector<float> scores_rel;
  for(int i = 0; i < pathin.poses.size(); i++){
    float rel_z  = (vec_z[i] - mma_v[0]) / (mma_v[1]-mma_v[0]);
    float rel_d  = (vec_d[i] - mma_d[0]) / (mma_d[1]-mma_d[0]);
    float rel_h  = (vec_h[i] - mma_h[0]) / (mma_h[1]-mma_h[0]);
    float rel_i  = (vec_i[i] - mma_i[0]) / (mma_i[1]-mma_i[0]);
    float rel_0  = (vec_0[i] - mma_0[0]) / (mma_0[1]-mma_0[0]);
    float score = weight_z * rel_z + weight_dst2d * rel_d + weight_hdng * rel_h + rel_i * weight_inclination + rel_0 * weight_startdst;
    scores.push_back(score);
  }
  scores_rel.resize(scores.size());
  std::vector<float> mma_s = vec_to_min_max_ave(scores);
  for(int i = 0; i < scores.size(); i++){
    scores_rel[i]  = (scores[i] - mma_v[0]) / (mma_v[1]-mma_v[0]);
  }
  return scores;
}

sensor_msgs::LaserScan get_frontier(float num_is,float stride_length){
  geometry_msgs::PolygonStamped poly;
  geometry_msgs::PoseStamped ps;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  poly.header = hdr();
  ps.header = hdr();
  float rads_pr_i = 2*M_PI / num_is;
  pathout.poses.resize(num_is);
  poly.polygon.points.resize(num_is);
  sensor_msgs::LaserScan scanout;
  scanout.ranges.resize(num_is);
  scanout.angle_min = -M_PI;
  scanout.angle_max = M_PI;
  scanout.angle_increment = rads_pr_i;
  scanout.header = hdr();
  scanout.range_min = 1.0;
  scanout.range_max = 500;

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
    scanout.ranges[i] = cur_ray_len;
    poly.polygon.points[i].z = pathout.poses[i].pose.position.z = get_zmax(last_segment_hit);
    poly.polygon.points[i].x = pathout.poses[i].pose.position.x = midpoint.x;
    poly.polygon.points[i].y = pathout.poses[i].pose.position.y = midpoint.y;
  }
  std::vector<float> vz_grid = get_vec_attribute(pathout,"z");
  draw_poly(poly,get_color(0,0,200));
  draw_path_by_score(pathout,vz_grid,0,2,1,1);
  drawimg(M_PI,tf::getYaw(ps.pose.orientation),"closest_lowest_straightest"+ std::to_string(count_target_paths) );
  return scanout;
}

geometry_msgs::PoseStamped test_scoring(nav_msgs::Path pathin,std::string scoretype){
  geometry_msgs::PoseStamped ps;
  ps.header = hdr();

  float weight_hdng = 0.0;
  float weight_z = 0.0;
  float weight_dst2d = 0.0;
  float weight_inclination = 0.0;
  float weight_startdst = 0.0;
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
  std::vector<float> scores = get_path_score(pathin,weight_hdng,weight_z,weight_dst2d,weight_inclination,weight_startdst);
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/2);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/2);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);
  private_nh.param("setpoint_scanrange", par_scanrange, 25.0);//*2.0);
  private_nh.param("setpoint_zclearing", par_zclearing, 5.0);//*2.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
  pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  pub_cmdpose       	= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/posemb",10);
  ros::Publisher pub_target   	= nh.advertise<geometry_msgs::PoseStamped>("/tb_target",10);
  ros::Publisher pub_forward    = nh.advertise<geometry_msgs::PointStamped>("/tb_pnt/forward",10);
  ros::Publisher pub_scanpoint  = nh.advertise<geometry_msgs::PointStamped>("/tb_pnt/scanpoint",10);
  ros::Publisher pub_scanpoint2	= nh.advertise<geometry_msgs::PointStamped>("/tb_pnt/scanpoint2",10);
  pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);

  ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
  ros::Subscriber ss5 = nh.subscribe("/tb_path/hdng_clear",10,pathhdngcleared_cb);
  ros::Subscriber s0 = nh.subscribe("/tb_path/cleared_poly",1,poylcleared_cb);
  ros::Subscriber s01 = nh.subscribe("/tb_path/cleared",1,pathcleared_cb);

  ros::Subscriber oc       = nh.subscribe("/override",10,override_cb);
  ros::Subscriber as1       = nh.subscribe("/velodyne_scan",10,rayranges_cb);
  ros::Subscriber a1        = nh.subscribe("/scan_down",10,scan_dn_cb);
  ros::Subscriber a3        = nh.subscribe("/scan_up",10,scan_up_cb);
  ros::Subscriber a6        = nh.subscribe("/scan_stabilized",10,scan_stab_cb);
  ros::Subscriber a2        = nh.subscribe("/scan_tilt_up",10,rayranges_hi_cb);
  ros::Subscriber as4       = nh.subscribe("/scan_tilt_down",10,rayranges_lo_cb);
  ros::Subscriber z1        = nh.subscribe("/tb_path/altmax",10,altmax_cb);
  ros::Subscriber ods        = nh.subscribe("/odom_global",10,odom_cb);

  ros::Subscriber asb4 = nh.subscribe("/tb_obs/closest_pos",10,closest_pos_cb);
  ros::Subscriber asbb4 = nh.subscribe("/tb_obs/closest_left",10,closest_left_cb);
  ros::Subscriber assaa4 = nh.subscribe("/tb_obs/closest_right",10,closest_right_cb);
  ros::Subscriber assa4 = nh.subscribe("/tb_obs/closest_mid",10,closest_mid_cb);
  ros::Subscriber avssa4 = nh.subscribe("/tb_obs/closest_blw",10,closest_blw_cb);
  ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
  ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
  ros::Publisher pub_scanzone_path = nh.advertise<nav_msgs::Path>("/tb_scanzone_path", 10);
  ros::Publisher pub__path = nh.advertise<nav_msgs::Path>("/tb_targetpath", 10);
	ros::Rate rate(5.0);
  ros::Time last_check;
  ros::Time start= ros::Time::now();

  define_colors();
  bool ros_inform = true;
  setpoint_altitude = 15;
  float dt;
  float z_err,s_err,t_err,y_err;
  float z_i,s_i,t_i,y_i;
  float z_d,s_d,t_d,y_d;
  float pz_P,ps_P,pt_P,py_P;
  float pz_D,ps_D,pt_D,py_D;
  float pz_I,ps_I,pt_I,py_I;
  pz_P = ps_P = pt_P = py_P = 1;
  pz_D = ps_D = pt_D = py_D = 0.3;
  pz_I = ps_I = pt_I = py_I = 0.01;

  float y_err_last,z_err_last,s_err_last,t_err_last;
  float current_revolution = 0;
  float current_radlength  = 10;
  float leadlength_radians = M_PI/4;

  ros::Time time_last = ros::Time::now();
  float setpoint_altitude2 = 10;
  std_msgs::Float64 arm1_tilt_msg;
  std_msgs::Float64 target_alt_msg;
  while(ros::ok()){
    bool need_to_look_down = false;
    bool need_to_look_forw = false;
    bool need_to_look_up   = false;
    dt= (ros::Time::now() - time_last).toSec();
    time_last = ros::Time::now();
    rate.sleep();
    ros::spinOnce();
    update_path_full();
    geometry_msgs::Point p0;
    //draw_path_full();

    if((ros::Time::now()-last_check).toSec() > 1.0){
      check_front();
      //draw_grids();
      geometry_msgs::PoseStamped ps;
      bool show_scoring = false;
        if(show_scoring){
        ps = test_scoring(path_full,"closeststart");
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
        drawimg(M_PI,tf::getYaw(ps.pose.orientation),"closeststart"+ std::to_string(count_target_paths) );

        ps = test_scoring(path_full,"highest");
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
        drawimg(M_PI,tf::getYaw(ps.pose.orientation),"highest"+ std::to_string(count_target_paths) );

        ps = test_scoring(path_full,"lowest");
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
        drawimg(M_PI,tf::getYaw(ps.pose.orientation),"lowest"+ std::to_string(count_target_paths) );

        ps = test_scoring(path_full,"farthest_highest_straightest");
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
        drawimg(M_PI,tf::getYaw(ps.pose.orientation),"farthest_highest_straightest"+ std::to_string(count_target_paths) );

        ps = test_scoring(path_full,"closest_lowest_straightest");
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
        drawimg(M_PI,tf::getYaw(ps.pose.orientation),"closest_lowest_straightest"+ std::to_string(count_target_paths) );

        ps = test_scoring(path_full,"farthest_closeststart");
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
        drawimg(M_PI,tf::getYaw(ps.pose.orientation),"farthest_closeststart"+ std::to_string(count_target_paths) );
      }
      nav_msgs::Path pathtarg;
      pathtarg = consider_targets();
      pathtarg.header = hdr();
      //geometry_msgs::PolygonStamped poly_frontier = get_frontier(32,10);
      current_revolution = atan2(pos.y,pos.x);
      current_radlength  = get_dst2d(pos,p0);
      float leading_revolution = constrainAngle(current_revolution + leadlength_radians);
      sensor_msgs::LaserScan scan_frontier = get_frontier(32,10);
      int current_i = (current_revolution -scan_frontier.angle_min )/scan_frontier.angle_increment;
      int leading_i = (leading_revolution -scan_frontier.angle_min )/scan_frontier.angle_increment;
      float a0 = scan_frontier.angle_min + scan_frontier.angle_increment * current_i;
      float a1 = scan_frontier.angle_min + scan_frontier.angle_increment * leading_i;
      float r0 = scan_frontier.ranges[current_i];
      float r1 = scan_frontier.ranges[leading_i];
      ROS_INFO("Current_i: %i leading_i: %i a0a1: %.2f %.2f r0: %.0f r1: %.0f ",current_i,leading_i,a0,a1,r0,r1);
      draw_line(p0,a0,r0,get_color(0,0,200));
      draw_line(p0,a1,r1,get_color(0,200,0));
      ps.pose.position.x = r1 * cos(a1);
      ps.pose.position.y = r1 * sin(a1);
      ps.header = hdr();
      set_target_pose(ps);

      drawimg(M_PI,tf::getYaw(ps.pose.orientation),"frontier"+ std::to_string(count_target_paths) );

      if(pathtarg.poses.size() == 0)
        need_to_look_down = true;
      ps = test_scoring(pathtarg,"closest_lowest_closeststart");
      ps.header = hdr();
      setpoint_altitude = ps.pose.position.z + par_zclearing;
      pub__path.publish(pathtarg);
    //  ps.pose.position = get_ave_pnt(pathtarg);
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
      drawimg(M_PI,tf::getYaw(ps.pose.orientation),"targets"+ std::to_string(count_target_paths) );
      //set_target_pose(ps);
      last_check = ros::Time::now();
    }
    checktf();

    float velxy = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
    geometry_msgs::Vector3 rpy_vel;
    tf2::Matrix3x3 q(tf2::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
    q.getRPY(rpy_vel.x,rpy_vel.y,rpy_vel.z);
    rpy_vel.y *= -1;
    float vxy = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
    float vz  = odom.twist.twist.linear.z;
    float vxyz = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2)+pow(odom.twist.twist.linear.z,2));
    float vaz = odom.twist.twist.angular.z;
    ROS_INFO("Velocity hdng: %.2f incl: %.2f speed: vxyz: %.2f vxy: %.2f vz: %.2f angular: %.2f",rpy_vel.z,rpy_vel.y,vxyz,vxy,vz,vaz);
    geometry_msgs::PointStamped forward_point;

    forward_point.point.x = odom.twist.twist.linear.x * 5;//  pos.x + 15 * cos(pos_yaw);
    forward_point.point.y = odom.twist.twist.linear.y * 5;// pos.y + 15 * sin(pos_yaw);
    nav_msgs::Path path_ahead       = get_pathfull_aroundline(forward_point.point,pos,10);
    nav_msgs::Path path_aheadtarget = get_pathfull_aroundline(target.pose.position,pos,10);
    std::vector<float> vfz = get_vec_attribute(path_ahead,"z");
    std::vector<float> vtz = get_vec_attribute(path_aheadtarget,"z");
    std::vector<float> vfi = get_vec_attribute(path_ahead,"inclination");
    std::vector<float> vti = get_vec_attribute(path_aheadtarget,"inclination");
    float vfz_max = vec_to_max(vfz);
    float vtz_max = vec_to_max(vtz);
    float vti_max = vec_to_max(vti);
    float vfi_max = vec_to_max(vfi);

    float decision_elevation = fmax(vtz_max,vfz_max);
    geometry_msgs::PointStamped closest_danger = get_danger();
    geometry_msgs::Point closest_obs           = get_closepnts();
  //float decision_elevation                   = get_elevation_below2();

    if((ros::Time::now() - start).toSec() < 5){
      target_alt_msg.data = 10;
      setpoint_altitude = 15;
    }
    else{
    //  look_in_directions(16,75,15);
    //  setpoint_altitude = decision_elevation + par_zclearing;
    }
    target_angle = get_a_from_dz_r(par_zclearing,par_scanrange);
    target_angle = set_tilt_scanpoint(scanpoint_actual);
    target_angle = get_tilt(need_to_look_down,need_to_look_up,need_to_look_forw);
    float yaw_err = get_shortest(get_hdng(target.pose.position,pos),pos_yaw);
    ROS_INFO("Velocity vfz_max: %.2f vtz_max: %.2f vti_max: %.2f vfi_max: %.2f decision_elevation_ %.2f setp_alt: %.2f ",vfz_max,vtz_max,vti_max,vfi_max,decision_elevation,setpoint_altitude);
    if(abs(arm1_tilt_msg.data) > M_PI)
      arm1_tilt_msg.data = 0;
    t_err = target_angle - vlp_rpy.y;
    z_err = setpoint_altitude - pos.z;

    s_err = par_scanrange - scanrange_actual;
    y_err = yaw_err;
    z_err = saturate(z_err,1);
    t_err = saturate(t_err,1);
    s_err = saturate(s_err,1);

    ROS_INFO("Target_angle: %.2f alt: %.0f elevation: %.0f sctual scanrange: %.0f target_pos: %.0f",target_angle,setpoint_altitude,decision_elevation,scanrange_actual,dst_target);

    y_i = saturate(y_i,1);
    z_i = saturate(z_i,1);
    s_i = saturate(s_i,1);
    t_i = saturate(t_i,1);
    y_d = saturate(y_d,1);
    z_d = saturate(z_d,1);
    s_d = saturate(s_d,1);
    t_d = saturate(t_d,1);
    z_i += z_err*dt; s_i += s_err*dt;  t_i += t_err*dt; y_i += y_err*dt;

    z_d = (z_err - z_err_last) / dt;
    s_d = (s_err - s_err_last) / dt;
    t_d = (t_err - t_err_last) / dt;
    y_d = (y_err - y_err_last) / dt;

    z_err_last = z_err;
    s_err_last = s_err;
    t_err_last = t_err;
    y_err_last = y_err;

    float cmd_z   = (z_err * pz_P      + z_d * pz_D     + z_i * pz_I);
    float cmd_y   = (y_err * py_P      + y_d * py_D     + y_i * py_I);
    float cmd_s   = (s_err * ps_P      + s_d * ps_D     + s_i * ps_I);
    float cmd_t   = (t_err * pt_P      + t_d * pt_D     + t_i * pt_I);

    arm1_tilt_msg.data  -= (cmd_t*dt);
    target_alt_msg.data += (cmd_z*dt);
//    target_yaw.data += cmd_y*dt;
    if(ros_inform){
      ROS_INFO("z_err: %.2f (%.2f - %.2f)",z_err,setpoint_altitude,pos.z);
      ROS_INFO("s_err: %.2f (%.2f - %.2f)",s_err,par_scanrange,scanrange_actual);
      ROS_INFO("t_err: %.2f (%.2f - %.2f)",t_err,target_angle,vlp_rpy.y);
      ROS_INFO("z_cmd:  %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",z_err,z_d,z_i,cmd_z,target_alt_msg.data);
      ROS_INFO("scan:   %.2f d: %.2f, i %.2f, cmd: %.2f ",s_err,s_d,s_i,cmd_s);
      ROS_INFO("tilt:   %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",t_err,t_d,t_i,cmd_t,arm1_tilt_msg.data);
    }
    if(override){
      arm1_tilt_msg.data  = override_tilt * M_PI/24;
      target_alt_msg.data = override_alt * 2.0;
      ROS_INFO("OVERRIDE: alt: %.2f tilt: %.2f",target_alt_msg.data,arm1_tilt_msg.data);
    }
    if(std::isnan(arm1_tilt_msg.data))
      arm1_tilt_msg.data = 0;
    if(std::isnan(target_alt_msg.data))
      target_alt_msg.data = setpoint_altitude;

    geometry_msgs::PointStamped scanpoint2_msg,scanpoint_msg;
    scanpoint_msg.point  = scanpoint_actual;
    scanpoint2_msg.point = scanpoint_actual2;
    scanpoint_msg.header = hdr();
    scanpoint2_msg.header = hdr();
    scanzone_path.header = hdr();
    pub_scanzone_path.publish(scanzone_path);
    pub_scanpoint.publish(scanpoint_msg);
    pub_forward.publish(forward_point);
    pub_scanpoint2.publish(scanpoint2_msg);
    pub_altcmd.publish(target_alt_msg);
    pub_tiltvlp.publish(arm1_tilt_msg);
    pub_target.publish(target);

  }
  return 0;
}
