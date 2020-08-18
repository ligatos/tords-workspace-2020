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

ros::Publisher pub_cmdexplore,pub_cmdmb,pub_get_next_path,pub_cmd;

geometry_msgs::PolygonStamped poly_cleared;
geometry_msgs::PoseStamped target,base_pose;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point poly_cleared_centroid,pos,pnt_midpoint,pnt_ref,obs_p,obs_r,obs_l,obs_m;
nav_msgs::Odometry odom;

nav_msgs::Path path_down_raw,path_side_raw,path_target_full,path_clear_vlp,path_world_visible,path_down_best,path_side_best,path_vlp,path_obs,path_side_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full;
tb_msgsrv::Paths paths_down,paths_side,paths_active_down,paths_active_side;
int mainstate,path_targets_i,count_target_paths;
float rad2deg = 180.0/M_PI;
float dst_target,poly_cleared_centroid_area;
std::string inspection_type = "idle";

cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

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
geometry_msgs::Point get_poly_centroidarea(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;
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

double get_shortest(double target_heading,double actual_hdng){
  double a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
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
    else if(sort_by == "dst_3d_ref")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pnt_ref,pathin.poses[i].pose.position)));
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
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin,int degrees){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	float base_yaw = tf::getYaw(base_pose.pose.orientation);
	for(int i = 0; i < pathin.poses.size(); i++){
		int abs_degs =abs(get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw) * rad2deg);
		if(abs_degs < degrees){
			pathout.poses.push_back(pathin.poses[i]);
		}
	}
	//ROS_INFO("degrees: %i, pose_in: %i path_out: %i",degrees,pathin.poses.size(),pathout.poses.size());
	return pathout;
}
nav_msgs::Path cutoff_percentage(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = 0; i < fmax(percent_length,fmin(pathin.poses.size(),3)); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  //ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}

int get_closest_i(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  int closest_i = 0;
  float closest_dst = 100;
  if(pathin.poses.size() == 0)
    return closest_i;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst3d(pathin.poses[i].pose.position,pin);
     if(dst < closest_dst){
       closest_dst = dst;
       closest_i = i;
     }
  }
  return closest_i;
}
int get_farthest_i(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  int closest_i = 0;
  float farthest_dst = 100;
  if(pathin.poses.size() == 0)
    return closest_i;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pin);
     if(dst > farthest_dst){
       farthest_dst = dst;
       closest_i = i;
     }
  }
  return closest_i;
}
void draw_path_at_img(nav_msgs::Path pathin,geometry_msgs::Point p0,
	 bool path_line,bool pose_yawline,bool pose_rectangle,bool pose_circle,bool pose_pnt,
  cv::Scalar color, int pose_size){
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
nav_msgs::Path offsetpath(nav_msgs::Path pathin,geometry_msgs::Point offsetpoint){
  for(int i = 0; i < pathin.poses.size(); i++){
    pathin.poses[i].pose.position.x += offsetpoint.x;
    pathin.poses[i].pose.position.y += offsetpoint.y;
  }
  return pathin;
}

nav_msgs::Path create_path(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
	float area_sidelength = 50;
	float total_sidelength = 1000;
	int num_grids = total_sidelength / area_sidelength-1;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z    = 10;
  pose.pose.orientation.w = 1;
  pose.header = hdr();
  float len = 50;
  for(int i = 0; i < 10; i++){
    for(int k = 0; k < i; k++){
			pose.pose.position.x += pow(-1,i) * len;
      //ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
    for(int l = 0; l < i; l++){
      pose.pose.position.y += pow(-1,i) * len;
    //  ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
  }
	return pathout;
}
std::vector<int> getinpath_indexes_simple(nav_msgs::Path pathin,geometry_msgs::Point pnt_to_check,float radius){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    //ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pnt_to_check) <= radius)
      vec_out.push_back(i);
  }
  return vec_out;
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
float get_path_density(nav_msgs::Path pathin){
  pathin      = sort_path(pathin,"dst_2d");
  nav_msgs::Path pathin_proj;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim2d(pathin_proj,pathin.poses[i].pose.position,2)){
      pathin_proj.poses.push_back(pathin.poses[i]);
    }
  }
  float dst_tot = 0;
  int neighbours_total = 0;
  for(int i = 0; i < pathin_proj.poses.size(); i++){
    std::vector<int> neighbours = getinpath_indexes_simple(pathin,pathin_proj.poses[i].pose.position,2);
    neighbours_total += neighbours.size();
    if(i > 0)
      dst_tot += get_dst2d(pathin_proj.poses[i].pose.position,pathin_proj.poses[i-1].pose.position);
  }
  float pathin_size = pathin.poses.size();
  float pathin_proj_size = pathin_proj.poses.size();
  float neighbours_tot = neighbours_total;
  float neighbours_ave = neighbours_tot / pathin_proj_size;
  float density_poses  = pathin_size / pathin_proj_size;
  float density_dist   = pathin_size / dst_tot;
  ROS_INFO("DENSITY EVALUATION: [3d-2d: %i -> %i], neighbours / meters: %i / %.0f",pathin.poses.size(),pathin_proj.poses.size(),neighbours_total,dst_tot);
  ROS_INFO("DENSITY EVALUATION: neighbours/poses/distance: [%.3f, %.3f, %.3f]",neighbours_ave,density_poses,density_dist);
  float density = (neighbours_ave + density_poses + density_dist) / 3;
  return density;
}
float score_candidates(nav_msgs::Path pathin){
  float yaw0,yawN,yawT,hdng0N,hdngT0,dst0N,dstT0,dhdng0NT0,dyaw0T,dzT0;
  float best_score = -100;
  float score_mod = 0;
  int best_i = 0;
  bool path_is_down,pose_is_down;
  if(pathin.poses.size() < 2)
    return -100;
  if(pathin.poses[0].pose.orientation.y != 0.7071 && target.pose.orientation.y != 0.7071){
    yaw0  = tf::getYaw(pathin.poses[0].pose.orientation);
    yawN  = tf::getYaw(pathin.poses[pathin.poses.size()-1].pose.orientation);
    yawT  = tf::getYaw(target.pose.orientation);
  }
  hdng0N = get_hdng(pathin.poses[pathin.poses.size()-1].pose.position,pathin.poses[0].pose.position);
  hdngT0 = get_hdng(pathin.poses[0].pose.position,target.pose.position);

  dst0N  = get_dst3d(pathin.poses[pathin.poses.size()-1].pose.position,pathin.poses[0].pose.position);
  dstT0  = get_dst3d(pathin.poses[0].pose.position,target.pose.position);

  dzT0   = pathin.poses[0].pose.position.z - target.pose.position.z;

  dhdng0NT0 = get_shortest(hdng0N,hdngT0);
  dyaw0T    = get_shortest(yaw0,yawT);

	if(dyaw0T < 0)
		dyaw0T *= -1;
  if(dhdng0NT0 < 0)
    dhdng0NT0 *= -1;

  float score_down = 0;
  float score_dst0N = dst0N / 10;
  float score_dstTO = dstT0 / -10;
  float score_hdng  = dhdng0NT0 * -1;
  float score_hyaw  = dyaw0T * -5;
  float score_cands = float(pathin.poses.size()) /10.0;
  float score_z     = -abs(dzT0)/10;
  float score_tot   = score_dstTO + score_hdng + score_cands + score_z + score_dst0N + score_down + score_mod + score_hyaw;
  ROS_INFO("SCORE EVALUATION dstT0 %.2f, score_hdng: %.2f, score_cands: %.2f, score_z: %.2f, score_dst0N: %.2f score_down: %.2f score_hyaw: %.2f tot: %.2f", score_dstTO,score_hdng,score_cands,score_z,score_dst0N,score_down,score_hyaw,score_tot);
  return score_tot;
}
float get_tot_len(nav_msgs::Path pathin){
  float len = 0;
  for(int i = 1; i < pathin.poses.size(); i++){
    len += get_dst3d(pathin.poses[i].pose.position,pathin.poses[i-1].pose.position);
  }
  return len;
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
float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}

nav_msgs::Path remove_nth_first(nav_msgs::Path pathin,int i0){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(i0 < 0)
    return pathout;
  for(int i = i0; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
void request_paths(){
	std_msgs::UInt8 get_path;
  pub_get_next_path.publish(get_path);
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

nav_msgs::Path get_next_candidates(tb_msgsrv::Paths pathsin,geometry_msgs::PoseStamped pose_comparison){
  float yaw0,yawN,yawT,hdng0N,hdngT0,dst0N,dstT0,dhdng0NT0,dyaw0T,dzT0;
  float best_score = -100;
  float score_mod = 0;
  int best_i = 0;
  bool path_is_down,pose_is_down;
  if(pose_comparison.pose.orientation.y == 0.7071)
    pose_is_down = true;
  std::vector<float> scores;
  for(int i = 0; i < pathsin.paths.size(); i++){
    if(pathsin.paths[i].poses.size() > 0){
      score_mod = 0;
      if(pathsin.paths[i].poses[0].pose.orientation.y == 0.7071)
        path_is_down = true;
      else
        path_is_down = false;


      yaw0  = tf::getYaw(pathsin.paths[i].poses[0].pose.orientation);
      yawN  = tf::getYaw(pathsin.paths[i].poses[pathsin.paths[i].poses.size()-1].pose.orientation);
      yawT  = tf::getYaw(pose_comparison.pose.orientation);

      hdng0N = get_hdng(pathsin.paths[i].poses[pathsin.paths[i].poses.size()-1].pose.position,pathsin.paths[i].poses[0].pose.position);
      hdngT0 = get_hdng(pathsin.paths[i].poses[0].pose.position,pose_comparison.pose.position);

      dst0N  = get_dst3d(pathsin.paths[i].poses[pathsin.paths[i].poses.size()-1].pose.position,pathsin.paths[i].poses[0].pose.position);
      dstT0  = get_dst3d(pathsin.paths[i].poses[0].pose.position,pose_comparison.pose.position);

      dzT0   = pathsin.paths[i].poses[0].pose.position.z - pose_comparison.pose.position.z;

      dhdng0NT0 = get_shortest(hdng0N,hdngT0);
      dyaw0T    = get_shortest(yaw0,yawT);
			if(dyaw0T < 0)
				dyaw0T *= -1;
	    if(dhdng0NT0 < 0)
        dhdng0NT0 *= -1;
    }
    else
      score_mod = -20;

    float score_down = 0;
    float score_dst0N = dst0N / 10;
    float score_dstTO = dstT0 / -10;
    float score_hdng  = dhdng0NT0 * -1;
    float score_hyaw  = dyaw0T * -5;
    float score_cands = float(pathsin.paths[i].poses.size()) /10.0;
    float score_z     = -abs(dzT0)/10;
    if(path_is_down)
      score_hyaw = 0;
    if(path_is_down == pose_is_down)
      score_down = 5;
    float score_tot   = score_dstTO + score_hdng + score_cands + score_z + score_dst0N + score_down + score_mod + score_hyaw;
  //  ROS_INFO("score_dstTO  %.2f, score_hdng: %.2f, score_cands: %.2f, score_z: %.2f, score_dst0N: %.2f score_down: %.2f score_hyaw: %.2f tot: %.2f", score_dstTO,        score_hdng,      score_cands,       score_z,             score_dst0N,     score_down,   score_hyaw,      score_tot);
    scores.push_back(score_tot);
    if(score_tot > best_score){
      best_score = score_tot;
      best_i     = i;
    }
  }

	nav_msgs::Path pathout;
	pathout.header = hdr();
	if(pathsin.paths.size() <= best_i)
		return pathout;
	else
		return pathsin.paths[best_i];
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
 dst_target = get_dst3d(target.pose.position,pos);

	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
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
std::vector<float> eval_one_path(nav_msgs::Path pathin){
  ROS_INFO("************************************");
  std::vector<float> scores;
  bool pose_is_down;
  if(pathin.poses.size() > 0){
    if(pathin.poses[0].pose.orientation.y == 0.7071)
      pose_is_down = true;
  }
  float s1 = get_path_density(pathin);
  float s2 = score_candidates(pathin);
  scores.push_back(s1);
  scores.push_back(s2);
  ROS_INFO("EVAL PATH %i POSES is_down: %i %.2f %.2f",pathin.poses.size(),pose_is_down,s1,s2);
  ROS_INFO("************************************");
  return scores;
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

void eval_paths(tb_msgsrv::Paths pathsin){
  std::vector<float> s1,s2;
  for(int i = 0; i < pathsin.paths.size(); i++){
     std::vector<float> out = eval_one_path(pathsin.paths[i]);
     s1.push_back(out[0]);
     s2.push_back(out[1]);
  }
  geometry_msgs::Point offsetpoint,p0;
  for(int i = 0; i < pathsin.paths.size(); i++){
    draw_path_at_img(pathsin.paths[i],p0,false,false,false,true,false,rel_color(s1,s1[i],"r"),1);
    draw_path_at_img(pathsin.paths[i],p0,false,false,true,false,false,rel_color(s2,s2[i],"b"),1);
  }
  cv::imwrite("/home/nuc/brain/"+std::to_string(count_target_paths)+"_eval.png",img);
}

int get_indexes_within_rad2(nav_msgs::Path pathin,float max_dst,geometry_msgs::Point centroid){
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst3d(pathin.poses[i].pose.position,centroid) > max_dst)
      return i;
  }
  return pathin.poses.size();
}
geometry_msgs::Point get_ave_pnt_ni(nav_msgs::Path pathin,int last_i){
  geometry_msgs::Point pnt;
  int n = fmin(last_i,pathin.poses.size());
  for(int i = 0; i < int(n); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= n;
  pnt.y /= n;
  pnt.z /= n;
  return pnt;
}
float get_zmax_ni(nav_msgs::Path pathin,int last_i){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}
nav_msgs::Path get_standardized_cluster(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pnt_ref       = base_pose.pose.position;
  geometry_msgs::Point ave_pnt;
  int ave_pnt_i,indexes_in_segment;
  while(pathin.poses.size() > 0){
    pathin             = sort_path(pathin,"dst_3d_ref");
    indexes_in_segment = get_indexes_within_rad2(pathin,5,pnt_ref);
    ave_pnt            = get_ave_pnt_ni(pathin,indexes_in_segment);
    float zmx          = get_zmax_ni(pathin,indexes_in_segment);
    ave_pnt_i          = get_closest_i(pathin,ave_pnt);
    pnt_ref            = pathin.poses[ave_pnt_i].pose.position;
    pathout.poses.push_back(pathin.poses[ave_pnt_i]);
    if(indexes_in_segment >= pathin.poses.size())
      break;
    else
      pathin = remove_nth_first(pathin,indexes_in_segment);
  }
  return pathout;
}

tb_msgsrv::Paths organize_clusters(tb_msgsrv::Paths clusters_in){
	tb_msgsrv::Paths clusters_standardized;
	clusters_standardized.paths.resize(clusters_in.paths.size());
	for(int i= 0; i < clusters_in.paths.size(); i++){
		clusters_standardized.paths[i] = get_standardized_cluster(clusters_in.paths[i]);//get_standardized_cluster(clusters_in.paths[i]);
	}
	return clusters_standardized;
}
void get_continous_path(nav_msgs::Path pathin){
  nav_msgs::Path path_targets_remaining = get_targets_remaining();
  if(path_targets_remaining.poses.size() > 0){
    int last_target_i = path_targets_remaining.poses.size()-1;
    geometry_msgs::Point last_target = path_targets_remaining.poses[last_target_i].pose.position;

    for(int i = 0; i < pathin.poses.size(); i++){
      int closest_i = get_closest_i(path_targets_remaining,pathin.poses[i].pose.position);
      if(closest_i == last_target_i){
        float closest_dst = get_dst3d(last_target,pathin.poses[i].pose.position);
        float dst_pos = get_dst3d(last_target,pos);
        if(dst_pos < 15){
          if(closest_dst > 3 && closest_dst < 10){
            float hdng_pos    = get_hdng(pathin.poses[i].pose.position,pos);
            float hdng_target = get_hdng(pathin.poses[i].pose.position,last_target);
            float dhdng = get_shortest(hdng_target,hdng_pos);
            if(dhdng < 0)
              dhdng *= -1;
            if(dhdng < M_PI/3){
              last_target = pathin.poses[i].pose.position;
              path_targets_remaining.poses.push_back(pathin.poses[i]);
              path_targets.poses.push_back(pathin.poses[i]);
              last_target_i = path_targets_remaining.poses.size()-1;
              ROS_INFO("Adding pose");
            }
          }
        }
      }
    }
  }
}
void superpathdown_cb(const tb_msgsrv::Paths::ConstPtr& msg){
	if(msg->paths.size() > 1){
    paths_active_down = organize_clusters(*msg);
    paths_active_down = get_new_world(paths_active_down,3,false);
    append_world_paths(paths_active_down);
    path_down_best    = get_next_candidates(paths_active_down,target);
    pose_down.header             = hdr();
    pose_down.pose.position      = get_ave_pnt_ni(path_down_best,path_down_best.poses.size());
    pose_down.pose.position.z    = get_zmax_ni(path_down_best,path_down_best.poses.size());
    pose_down.pose.orientation.w = pose_down.pose.orientation.y = 0.7071;
  }
}
void superpathside_cb(const tb_msgsrv::Paths::ConstPtr& msg){
	if(msg->paths.size() > 1){
    paths_active_side = organize_clusters(*msg);
    paths_active_side = get_new_world(paths_active_side,3,true);
    append_world_paths(paths_active_side);
    path_side_best    = get_next_candidates(paths_active_side,target);
    pose_side         = get_ave_pose(path_side_best);
    //ROS_INFO("CB superpathside_cb");
    if(inspection_type == "side"){
      get_continous_path(path_side_best);
    }
  }
}
void superpathdown_cb(const tb_msgsrv::Paths::ConstPtr& msg){
  paths_down = *msg;
  for(int i = 0; i< paths_down.paths.size(); i++){
    ROS_INFO("PATHS_DOWN: %i paths,%i poses",paths_down.paths.size(),paths_down.paths[i].poses.size());
  }
  paths_active_down = organize_clusters(*msg);
  eval_paths(*msg);
}

void superpathside_cb(const tb_msgsrv::Paths::ConstPtr& msg){
  paths_side = *msg;
  for(int i = 0; i< paths_side.paths.size(); i++){
    ROS_INFO("PATHS_SIDE: %i paths,%i poses",paths_side.paths.size(),paths_side.paths[i].poses.size());
  }
  paths_active_side = organize_clusters(*msg);
  eval_paths(*msg);
}
void closest_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_p = msg->point;
  if(msg->header.frame_id != "map"){
    obs_p.z = -100;
    obs_p.z = pos.z;
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
void hdngclear_cb(const nav_msgs::Path::ConstPtr& msg){
  path_clear_vlp = *msg;
}
void rawdown_cb(const nav_msgs::Path::ConstPtr& msg){
	path_down_raw = *msg;
  float cutoff = 5;
  for(int i = 0; i < msg->poses.size(); i++){
   int ci = get_closest_i(path_down_full,msg->poses[i].pose.position);
   if(get_dst2d(path_down_full.poses[ci].pose.position,msg->poses[i].pose.position) > cutoff)
     path_down_full.poses.push_back(msg->poses[i]);
   else if(msg->poses[i].pose.position.z > path_down_full.poses[ci].pose.position.z)
     path_down_full.poses[ci].pose.position.z = msg->poses[i].pose.position.z;
  }
}
void rawside_cb(const nav_msgs::Path::ConstPtr& msg){
	path_side_raw = *msg;
  float cutoff = 5;
 for(int i = 0; i < msg->poses.size(); i++){
  if(get_dst3d(path_side_full.poses[get_closest_i(path_side_full,msg->poses[i].pose.position)].pose.position,msg->poses[i].pose.position) > cutoff)
    path_side_full.poses.push_back(msg->poses[i]);
  }
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
}
void pathvlpobs_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
void poylcleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_cleared = *msg;
  poly_cleared_centroid      = get_poly_centroidarea(poly_cleared);
  poly_cleared_centroid_area = poly_cleared_centroid.z;
  poly_cleared_centroid.z    = base_pose.pose.position.z;
}
void pntmid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	pnt_midpoint = msg->point;
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	std_msgs::Float64 target_alt_msg;
  path_world_visible.header  = hdr();
	path_cleared_full.header   = hdr();
	path_obstacles_full.header = hdr();
	path_side_full.header 		 = hdr();
  path_down_full.header      = hdr();
  path_targets.header      = hdr();
	target.pose.orientation.w  = 1;
  path_targets_i = 0; count_target_paths = 0;
  path_side_full.poses.push_back(target);
  path_down_full.poses.push_back(target);
  path_targets.poses.push_back(target);
  path_targets.poses.push_back(target);
  ros::Subscriber s0 = nh.subscribe("/tb_path/cleared_poly",1,poylcleared_cb);
  ros::Subscriber s1 = nh.subscribe("/tb_path/cleared",1,pathvlp_cb);
  ros::Subscriber s2 = nh.subscribe("/tb_path/obstacles",1,pathvlpobs_cb);
  ros::Subscriber s3 = nh.subscribe("/tb_path/superpath_side",10,superpathside_cb);
  ros::Subscriber s4 = nh.subscribe("/tb_path/superpath_down",10,superpathdown_cb);
  ros::Subscriber as3 = nh.subscribe("/tb_path/raw_down",10,rawdown_cb);
  ros::Subscriber as5 = nh.subscribe("/tb_path/raw_side",10,rawside_cb);
  ros::Subscriber asb4 = nh.subscribe("/tb_obs/closest_pos",10,closest_pos_cb);
  ros::Subscriber asbb4 = nh.subscribe("/tb_obs/closest_left",10,closest_left_cb);
  ros::Subscriber assaa4 = nh.subscribe("/tb_obs/closest_right",10,closest_right_cb);
  ros::Subscriber assa4 = nh.subscribe("/tb_obs/closest_mid",10,closest_mid_cb);
  ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
  ros::Subscriber s8 = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);
  ros::Subscriber s9 = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);
  ros::Subscriber s19 = nh.subscribe("/tb_path/hdng_clear",10,hdngclear_cb);

  ros::Publisher pub_path                = nh.advertise<nav_msgs::Path>("/tb_behav/targets",10);
  ros::Publisher pub_path_down_best      = nh.advertise<nav_msgs::Path>("/tb_behav/down_best",10);
  ros::Publisher pub_path_side_best      = nh.advertise<nav_msgs::Path>("/tb_behav/side_best",10);
  ros::Publisher pub_path_down_full      = nh.advertise<nav_msgs::Path>("/tb_behav/down_full",10);
  ros::Publisher pub_path_side_full      = nh.advertise<nav_msgs::Path>("/tb_behav/side_full",10);

  ros::Publisher pub_path_target_full    = nh.advertise<nav_msgs::Path>("/tb_behav/full",10);
  ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("/cmd_pose",10);

	pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);
  pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  pub_cmdmb   				= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/posemb",10);
  pub_cmdexplore			= nh.advertise<geometry_msgs::PolygonStamped>("/tb_cmd/poly_explore",10);
	ros::Rate rate(2.0);

  path_target_full = create_path();
  path_target_full.header = hdr();

  while(ros::ok()){
    float dst_target = get_dst3d(target.pose.position,pos);
    ROS_INFO("Inspection type: %s target_dst: %.2f ",inspection_type.c_str(),dst_target);
    checktf();
    request_paths();
    count_target_paths++;
    pub_path_target_full.publish(path_target_full);
    pub_target_pose.publish(target);
    pub_path.publish(path_targets);

    path_down_best.header = hdr();
    path_side_best.header = hdr();
    pub_path_down_best.publish(path_down_best);
    pub_path_side_best.publish(path_side_best);
    pub_path_down_full.publish(path_down_full);
    pub_path_side_full.publish(path_side_full);
		pub_target_pose.publish(target);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
