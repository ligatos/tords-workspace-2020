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

tf2_ros::Buffer tfBuffer;

std::string inspection_type = "idle";

ros::Publisher pub_get_next_path,pub_cmd;
geometry_msgs::PoseStamped target,base_pose,target_last;
ros::Time activity_change,last_tilt;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point pos,pnt_midpoint,pnt_ref;
nav_msgs::Odometry odom;
nav_msgs::Path path_down_best,path_vlp,path_obs,path_sides_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_sides,path_down;
int mainstate;
int path_targets_i = 0;
int inspection_count = 0;
double par_vlpmaxtilt,par_vlpmintilt,par_vlptiltinterval,par_takeoffaltitude;
bool tiltlimit_reached,tilting,sides_empty;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
int count_target_paths = 0;
std::vector<int> targets_complete;
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
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
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
std::vector<nav_msgs::Path> sort_paths_in_vector(std::vector<nav_msgs::Path> path_vector,std::string sort_by){
  for(int i = 0; i < path_vector.size(); i++){
    path_vector[i] = sort_path(path_vector[i],sort_by);
  }
  return path_vector;
}
std::vector<nav_msgs::Path> paths_segmented_hdng(nav_msgs::Path pathin){
  std::vector<nav_msgs::Path> path_vector;
  nav_msgs::Path current_pathsegment;
  current_pathsegment.header = pathin.header;
  nav_msgs::Path path = sort_path(pathin,"hdng");
  float current_heading = get_hdng(path.poses[0].pose.position,base_pose.pose.position);
  for(int i = 1; i < path.poses.size(); i++){
    float candidate_heading = get_hdng(path.poses[i].pose.position,base_pose.pose.position);
    float delta_hdng        = get_shortest(candidate_heading,current_heading);
    int delta_hdng_degs     = abs(delta_hdng * rad2deg);
    if(delta_hdng_degs > 5){
      current_heading = candidate_heading;
      ROS_INFO("pathindex cutoff after %i indexes",current_pathsegment.poses.size());
      path_vector.push_back(current_pathsegment);
      current_pathsegment.poses.resize(0);
    }
    current_pathsegment.poses.push_back(path.poses[i]);
  }
  return path_vector;
}
std::vector<nav_msgs::Path> paths_segmented_z(nav_msgs::Path pathin){
  std::vector<nav_msgs::Path> path_vector;
  nav_msgs::Path current_pathsegment;
  current_pathsegment.header = pathin.header;
  nav_msgs::Path path = sort_path(pathin,"z");
  float  current_z = path.poses[0].pose.position.z;
  for(int i = 0; i < path.poses.size(); i++){
    float candidate_z = path.poses[i].pose.position.z;
    ROS_INFO("candidate_z: %.2f",candidate_z);
    if(abs(current_z - candidate_z) > 0){
      current_z = candidate_z;
      ROS_INFO("pathindex cutoff after %i indexes",current_pathsegment.poses.size());
      path_vector.push_back(current_pathsegment);
      current_pathsegment.poses.resize(0);
    }
    current_pathsegment.poses.push_back(path.poses[i]);
  }
  return path_vector;
}

nav_msgs::Path best_down(nav_msgs::Path pathin){
  std::vector<nav_msgs::Path> path_vector;
  nav_msgs::Path best_path;
 path_vector = sort_paths_in_vector(paths_segmented_hdng(pathin),"dst_2d");
//  path_vector = sort_paths_in_vector(paths_segmented_z(pathin),"dst_2d");
  ROS_INFO("Path vector returned (%i)",path_vector.size());
  float best_dst0 = 30;
  float best_hdng = 0;
  float best_deltadst = 0;
  float best_candidates = 0;
  float best_score = 0;
  float best_zmin = 0;
  float zmin = 100; float zmax = -100;
  std::vector<nav_msgs::Path> paths;
  std::vector<float> score;
  for(int i = 0; i < path_vector.size(); i++){
    int vec_size = path_vector[i].poses.size();
    ROS_INFO("path_vector[%i],size: %i",i,vec_size);
    if(vec_size > 0){
      float hdng = get_shortest(get_hdng(path_vector[i].poses[0].pose.position,base_pose.pose.position),tf::getYaw(base_pose.pose.orientation));
      float dst0 = get_dst2d(path_vector[i].poses[0].pose.position,base_pose.pose.position);
      float dstN = get_dst2d(path_vector[i].poses[path_vector[i].poses.size()-1].pose.position,base_pose.pose.position);
      for(int k = 0; k < path_vector[i].poses.size(); k++){
        float z = path_vector[i].poses[k].pose.position.z;
        if(z <  zmin)
          zmin = z;
        if(z > zmax)
          zmax = z;
      }
      float delta_dst = dstN - dst0;
      int candidates = 0;

      if(hdng < 0)
        hdng *= -1;
      float score_dst0 = dst0 / -10;
      float score_hdng = hdng * -1;
      float score_dst  = delta_dst / 10;
      float score_cands= float(vec_size) /10.0;
      float score_z    = zmin / 10.0;
      float score_tot = score_dst0 + score_hdng + score_dst + score_cands;
      ROS_INFO("score_dst0    %.2f, score_hdng: %.2f, score_dst: %.2f, score_cands: %.2f, score_z: %.2f tot: %.2f",score_dst0,score_hdng,score_dst,score_cands,score_z,score_tot);
      if(score_tot > best_score){
        best_dst0 = dst0;
        best_hdng = hdng;
        best_deltadst = delta_dst;
        best_path = path_vector[i];
        best_candidates = float(vec_size);
      }
    }
  }
  best_path.header.frame_id = "map";
  return best_path;
}
nav_msgs::Path create_path(geometry_msgs::Point midpoint,float area_sidelength,int num_centroids){
  nav_msgs::Path pathout;
  float centroid_sidelength = area_sidelength / num_centroids;
  for(int y = 0; y < num_centroids; y++){
    for(int x = 0; x < num_centroids; x++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id      = "map";
      pose.header.stamp         = ros::Time::now();
      pose.pose.position.x      = midpoint.x + pow(-1,y) * (x * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.y      = midpoint.y + -1 * (y * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.z      = midpoint.z;
      pose.pose.orientation.w   = 1;
      pathout.poses.push_back(pose);
    }
  }
  pathout.header.frame_id = "map";
  pathout.header.stamp = ros::Time::now();
  return pathout;
}
geometry_msgs::Point get_pose_pair(geometry_msgs::PoseStamped posein){
	geometry_msgs::Point point_out;
	float yaw   			= tf::getYaw(posein.pose.orientation);
	point_out.x = posein.pose.position.x + 5 * cos(yaw);
	point_out.y = posein.pose.position.y + 5 * sin(yaw);
	point_out.z = posein.pose.position.z;
	return point_out;
}
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}
int get_closest_pose_not_in_vec(nav_msgs::Path pathin,std::vector<int> vec,geometry_msgs::Point cur_tar, geometry_msgs::Point cur_obs, float cur_yaw, float cur_hdng){
	float best_tot = 1000;
	int best_tot_i;
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(vec,i)){
			geometry_msgs::Point cand_tar  = pathin.poses[i].pose.position;
			geometry_msgs::Point cand_obs  = get_pose_pair(pathin.poses[i]);
			float 			    		 cand_yaw  = tf::getYaw(pathin.poses[i].pose.orientation);
			float 							 cand_hdng = get_hdng(cand_tar,cur_tar);
			float dst_hdng = get_shortest(cand_hdng,cur_hdng);
			float dst_yaw  = get_shortest(cand_yaw,cur_yaw);
			float dst_pos  = get_dst3d(cur_tar,cand_tar);
			float dst_obs  = get_dst3d(cur_obs,cand_obs);
			float dst_tot  = dst_pos + dst_obs;
			if(dst_tot < best_tot && abs(dst_hdng) < 3){
				best_tot_i = i;
				best_tot = dst_tot;
			}
		}
	}
	return best_tot_i;
}

int get_best_neighbour(nav_msgs::Path pathin, geometry_msgs::PoseStamped pose_in,std::vector<int> vec_i){
  float best_score = -100;
  float best_dst   = 0;
  float best_hdng  = 0;
  int best_i = -1;
  float current_heading = tf::getYaw(pose_in.pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_i,i)){
      float hdng = get_shortest(get_hdng(pathin.poses[i].pose.position,pose_in.pose.position),current_heading);
      float dst  = get_dst2d(pathin.poses[i].pose.position,pose_in.pose.position);
      if(hdng < 0)
        hdng *= -1;
      if(hdng < M_PI/4 && dst > 0 && dst < 5){
        //float score_dst  = dst / -10;
        float score_hdng = hdng * -1;
    //    float score_tot = score_dst + score_hdng;
        ROS_INFO("pathin[%i]score_hdng: %.2f",i,score_hdng);
        if(score_hdng > best_score){
          best_i     = i;
          best_score = score_hdng;
          best_dst  = dst;
          best_hdng = hdng;
        }
      }
    }
  }
  return best_i;
}

nav_msgs::Path get_path(nav_msgs::Path pathin){
  int best_neighbour = 0;
  geometry_msgs::PoseStamped ps;
  ps = base_pose;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  std::vector<int> vec_i;
  if(pathin.poses.size() == 0)
    return pathout;
  while(true){
    best_neighbour = get_best_neighbour(pathin,ps,vec_i);
    vec_i.push_back(best_neighbour);
    if(best_neighbour == -1)
      break;
  //  float dhdng = get_shortest(get_hdng(pathin.poses[best_neighbour].pose.position,ps.pose.position),tf::getYaw(ps));
    float yaw      = get_hdng(pathin.poses[best_neighbour].pose.position,ps.pose.position);
    ps.pose.position = pathin.poses[best_neighbour].pose.position;
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  }
  ROS_INFO("pathout: %i",pathout.poses.size());
}
void request_paths(){
/*	path_sides.header.frame_id = "empty";
	path_down.poses.resize(0);
	path_sides.poses.resize(0);
  ROS_INFO("BRAIN: Requesting paths");*/
	std_msgs::UInt8 get_path;
  pub_get_next_path.publish(get_path);
}

void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
  target_last = target;
	target 	 = new_target_pose;
	path_targets_sent.poses.push_back(new_target_pose);
	pub_cmd.publish(new_target_pose.pose.position);
}
nav_msgs::Path get_targets_remaining(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = path_targets_i; i < path_targets.poses.size(); i++){
    pathout.poses.push_back(path_targets.poses[i]);
  }
  return pathout;
}

float get_dst_remaining(){
  nav_msgs::Path path_targets_remaining = get_targets_remaining();
  if(path_targets_remaining.poses.size() == 0)
    return 0;
  float dst_i0 = get_dst3d(path_targets_remaining.poses[0].pose.position,pos);
  float dst_sum = 0;
  for(int i = 1; i < path_targets_remaining.poses.size(); i++){
    float dst = get_dst3d(path_targets_remaining.poses[i].pose.position,path_targets_remaining.poses[i-1].pose.position);
    dst_sum += dst;
  }
  ROS_INFO("Dst i0: %.0f dst path: %.0f",dst_i0,dst_sum);
  return (dst_sum+dst_i0);
}
std::vector<geometry_msgs::Point> get_bbmin_bbmax(geometry_msgs::Point p0, geometry_msgs::Point p1,float margins){
  geometry_msgs::Point bbmin_pt,bbmax_pt;
  bbmin_pt.x = fmin(p0.x,p1.x)-margins;
  bbmin_pt.y = fmin(p0.y,p1.y)-margins;
  bbmin_pt.z = fmin(p0.z,p1.z)-margins;
  bbmax_pt.x = fmax(p0.x,p1.x)+margins;
  bbmax_pt.y = fmax(p0.y,p1.y)+margins;
  bbmax_pt.z = fmax(p0.z,p1.z)+margins;
  std::vector<geometry_msgs::Point> vec_out;
  vec_out.push_back(bbmin_pt);
  vec_out.push_back(bbmax_pt);
  return vec_out;
}

nav_msgs::Path remove_path_within_bb(nav_msgs::Path pathin,std::vector<geometry_msgs::Point> bbmin_bbmax, bool remove_outside){
  nav_msgs::Path pathout_outside,pathout_inside;
  pathout_outside.header = hdr();
  pathout_inside.header = hdr();
  int posecount = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    geometry_msgs::Point p = pathin.poses[i].pose.position;
    if(p.x > bbmin_bbmax[0].x && p.y > bbmin_bbmax[0].y && p.z > bbmin_bbmax[0].z
    && p.x < bbmin_bbmax[1].x && p.y < bbmin_bbmax[1].y && p.z < bbmin_bbmax[1].z)
      pathout_inside.poses.push_back(pathin.poses[i]);
    else
      pathout_outside.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("remove path within bb %i / %i",pathout_inside.poses.size(),pathout_outside.poses.size());

  if(remove_outside)
    return pathout_inside;
  else
    return pathout_outside;
}
nav_msgs::Path remove_overlap(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  nav_msgs::Path path_targets_remaining = get_targets_remaining();
  if(path_targets_remaining.poses.size() > 1){
    pnt_ref = path_targets_remaining.poses[path_targets.poses.size()-1].pose.position;
    pathout = remove_path_within_bb(pathin,get_bbmin_bbmax(path_targets_remaining.poses[0].pose.position,path_targets_remaining.poses[path_targets_remaining.poses.size()-1].pose.position,2),false);
  }
  else
    pathout = pathin;
  pathout = sort_path(pathout,"dst_3d_ref");
  return pathout;
}
nav_msgs::Path merge_paths(nav_msgs::Path path1,nav_msgs::Path path0){
  if(path1.poses.size() > 0 && path0.poses.size() == 0)
    return path1;
  if(path1.poses.size() == 0 && path0.poses.size() > 0)
    return path0;
  if(path1.poses.size() == 0 && path0.poses.size() == 0)
    return path0;
  for(int i = 0; i < path1.poses.size(); i++){
    path0.poses.push_back(path1.poses[i]);
  }
  return path0;
}
nav_msgs::Path analyse_path(nav_msgs::Path path_targets_new){
  count_target_paths++;
  img_blank.copyTo(img);
  nav_msgs::Path pathout;
  pathout.header = hdr();
  cv::Scalar color     = get_color(0,200,0);
  cv::Scalar color_pos = get_color(200,0,0);
  cv::Scalar color_new = get_color(0,0,200);
  cv::Scalar color_nr = get_color(0,200,200);
  nav_msgs::Path path_targets_remaining   = get_targets_remaining();
  nav_msgs::Path path_targets_non_overlap = remove_overlap(path_targets_new);
  geometry_msgs::Point p0;

  draw_path_at_img(path_targets_remaining,p0,false,false,false,false,true,color_new,1);
  draw_path_at_img(path_targets,p0,false,false,false,false,true,color,1);
  draw_path_at_img(path_targets_new,p0,false,false,false,true,false,color_new,1);
  draw_path_at_img(path_targets_non_overlap,p0,false,true,false,true,false,color_new,1);
  cv::circle(img,pnt2cv(pos),2,color_pos,1);
  ROS_INFO("drawn");
  cv::imwrite("/home/nuc/brain/overlap/targets_"+std::to_string(count_target_paths)+".png",img);
  pathout = merge_paths(path_targets_remaining,path_targets_non_overlap);
  ROS_INFO("path_targets(%i) %i/%i,path_targets_new %i, non-overlap: %i,pathout: %i",path_targets_remaining.poses.size(),path_targets_i,path_targets.poses.size(),path_targets_new.poses.size(),path_targets_non_overlap.poses.size(),pathout.poses.size());

  for(int i = 0; i < pathout.poses.size(); i++){
    float dst_pos          = get_dst3d(pathout.poses[i].pose.position,pos);
    float hdng_pos         = get_hdng(pathout.poses[i].pose.position,pos);
    bool is_down;
    if(std::isinf(dst_pos)){
      ROS_INFO("Dst_pos: %.0f xyz: %.0f %.0f %.0f",dst_pos,pathout.poses[i].pose.position.x,pathout.poses[i].pose.position.y,pathout.poses[i].pose.position.z);
    }
    if(is_down)
      ROS_INFO("New pathout targets down[%i], dst_pos. %.0f, hdng_pos: %.2f",i,dst_pos,hdng_pos);
    else
      ROS_INFO("New pathout targets side[%i], dst_pos. %.0f, hdng_pos: %.2f",i,dst_pos,hdng_pos);
  }

  for(int i = 0; i < path_targets_non_overlap.poses.size(); i++){
    float dst_pos          = get_dst3d(path_targets_non_overlap.poses[i].pose.position,pos);
    float hdng_pos         = get_hdng(path_targets_non_overlap.poses[i].pose.position,pos);
    float hdng_targets_end,dst_targets_end;
    if(path_targets.poses.size() > 1){
       hdng_targets_end = get_hdng(path_targets_non_overlap.poses[i].pose.position,path_targets.poses[path_targets.poses.size()-1].pose.position);
       dst_targets_end  = get_dst3d(path_targets_non_overlap.poses[i].pose.position,path_targets.poses[path_targets.poses.size()-1].pose.position);
    }
    bool is_down;
    if(path_targets_non_overlap.poses[i].pose.orientation.y == 0.7071)
      is_down = true;
    if(is_down)
      ROS_INFO("New path targets down[%i], dst_pos. %.0f dst_end: %.0f, hdng_pos: %.2f, hdng_end: %.2f",i,dst_pos,dst_targets_end,hdng_pos,hdng_targets_end);
    else
      ROS_INFO("New path targets side[%i], dst_pos. %.0f dst_end: %.0f, hdng_pos: %.2f, hdng_end: %.2f",i,dst_pos,dst_targets_end,hdng_pos,hdng_targets_end);
  }
  return pathout;
}


void set_target_path(nav_msgs::Path pathin){
  if(inspection_type == "down"){
    for(int i = 0; i < pathin.poses.size(); i++){
      pathin.poses[i].pose.position.z += 4;
    }
  }
  float dst_remaining = get_dst_remaining();
//  if(dst_remaining < 15){
  //  ROS_INFO("pathin: %i ",pathin.poses.size());
  //  if(path_targets.poses.size() > 0){
    //  pathin = analyse_path(pathin);
  //  }
  //  if(pathin.poses.size() > 0){
      ROS_INFO("pathin: %i ",pathin.poses.size());
    	path_targets       = pathin;
    	path_targets_i     = 0;
  //    premature_targets.resize(0);
    	set_target_pose(path_targets.poses[path_targets_i]);
//    }
//  }
}
int get_next_target(std::vector<int> vec_blacklist,nav_msgs::Path pathin,geometry_msgs::Point current_pos,float current_yaw){
  int best_i = -1;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_blacklist,i)){
      float dyaw = get_shortest(get_hdng(pathin.poses[i].pose.position,current_pos),current_yaw);
      if(dyaw < 0)
        dyaw *= -1;
      if(dyaw < M_PI/2){
        float dst = get_dst3d(current_pos,pathin.poses[i].pose.position);
        if(dst < lowest_dist){
          lowest_dist = dst;
          best_i = i;
        }
      }
    }
  }
  return best_i;
}
int get_next_target_down(std::vector<int> vec_blacklist,nav_msgs::Path pathin,geometry_msgs::Point current_pos,float current_yaw){
  int best_i = -1;
  float lowest_dyaw = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_blacklist,i)){
      float dyaw  = get_shortest(get_hdng(pathin.poses[i].pose.position,current_pos),current_yaw);
      float dyaw0 = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
      float dst  = get_dst3d(current_pos,pathin.poses[i].pose.position);
      if(dyaw < 0)
        dyaw *= -1;
      if(dyaw < M_PI/2 && dst < 5){
        if(dyaw+dyaw0 < lowest_dyaw){
          lowest_dyaw = dyaw+dyaw0;
          best_i = i;
        }
      }
    }
  }
  return best_i;
}

nav_msgs::Path create_ordered_path(nav_msgs::Path pathin){
  std::vector<int> vec_blacklist;
  geometry_msgs::Point current_pos;
  float current_yaw = vlp_rpy.z;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() < 2){
    return pathout;
  }
  current_yaw = get_hdng(pathin.poses[1].pose.position,pathin.poses[0].pose.position);
  current_pos = pathin.poses[0].pose.position;
  int best_i  = get_next_target(vec_blacklist,pathin,current_pos,current_yaw);
  while(best_i >= 0){
    current_yaw = get_hdng(pathin.poses[best_i].pose.position,current_pos);
    current_pos = pathin.poses[best_i].pose.position;
    vec_blacklist.push_back(best_i);
    best_i = get_next_target(vec_blacklist,pathin,current_pos,current_yaw);
  }
  for(int i = 0; i < vec_blacklist.size(); i++){
    pathout.poses.push_back(pathin.poses[vec_blacklist[i]]);
  }
  ROS_INFO("poses_in: %i, poses out: %i",pathin.poses.size(),pathout.poses.size());
  return pathout;
}/*
void find_premature_targets(){
  for(int i = 0; i < path_targets.poses.size(); i++){
    if(!in_vec(targets_complete,i)){
      if(get_dst3d(path_targets.poses[i].pose.position,pos) < 5)
        targets_complete.push_back(i);
    }
  }
}*/
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
void check_path_progress(){
//  find_premature_targets();
  //int next_target = get_next_target();

  if(path_targets_i < path_targets.poses.size()){
    float dst_to_target = get_dst3d(path_targets.poses[path_targets_i].pose.position,pos);
    ROS_INFO("BRAIN: Dst to target[%i of %i]: %.0f",path_targets_i,path_targets.poses.size(),dst_to_target);
    if(dst_to_target < 5){
      if(path_targets_i+3 == path_targets.poses.size()){
        request_paths();
      }
      else if(path_targets_i+1 < path_targets.poses.size()){
        path_targets_i++;
        set_target_pose(path_targets.poses[path_targets_i]);
      }
      else
        request_paths();
      /*targets_complete.push_back(path_targets_i);
      int best_i = get_next_target(targets_complete,path_targets,pos,vlp_rpy.z);
      ROS_INFO("best i %i",best_i);
      if(best_i > 0 && best_i < path_targets.poses.size()){
        path_targets_i = best_i;
        request_paths();
        set_target_pose(path_targets.poses[path_targets_i]);
      }*/
    }
  }
  else
  {
		inspection_type == "idle";
		request_paths();
  }
}



void append_world_poses(nav_msgs::Path pathin,std::string type){
	for(int i = 0; i < pathin.poses.size(); i++){
		if(type == "cleared"){
			if(dst_point_in_path_lim(path_cleared_full,pathin.poses[i].pose.position,5))
				path_cleared_full.poses.push_back(pathin.poses[i]);
		}
		if(type == "obstacles"){
			if(dst_point_in_path_lim(path_obstacles_full,pathin.poses[i].pose.position,5))
				path_obstacles_full.poses.push_back(pathin.poses[i]);
		}
		if(type == "sides"){
			if(dst_point_in_path_lim(path_sides_full,pathin.poses[i].pose.position,3))
				path_sides_full.poses.push_back(pathin.poses[i]);
		}
		if(type == "down"){
			if(dst_point_in_path_lim(path_down_full,pathin.poses[i].pose.position,3))
				path_down_full.poses.push_back(pathin.poses[i]);
		}
	}
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
	float delta_vlp = get_shortest(arm1_tilt_msg.data,vlp_rpy.y);
	if(delta_vlp < 0.05 && delta_vlp > -0.05)
		tilting = false;
//	else
//		tilting = true;
}
void extend_target_path(nav_msgs::Path pathin){
  int start_size = path_targets.poses.size();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(path_targets,pathin.poses[i].pose.position,4)
    && dst_point_in_path_lim(path_visited,pathin.poses[i].pose.position,4))
    {
      path_targets.poses.push_back(pathin.poses[i]);
    }
  }
  ROS_INFO("Path targets: %i added: %i -> %i",start_size,pathin.poses.size(),path_targets.poses.size());
}
void pathdown_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		path_down = *msg;
    path_down_best = best_down(path_down);
		append_world_poses(*msg,"down");
	}
	if(inspection_type == "down"){
		if(msg->poses.size() > 0){
      set_target_path(create_ordered_path(*msg));
/*
    	ROS_INFO("BEHAV: Continuing inspection with new path (%i poses)",msg->poses.size());
      if(path_targets.poses.size() == 0)
        set_target_path(create_ordered_path(*msg));
      else
        extend_target_path(create_ordered_path(*msg));
  */  }
		else{
			ROS_INFO("BEHAV: Path cannot continue, poses is empty, idling");
		//	inspection_type = "idle";
		}
	}
}

void pathsides_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
    sides_empty = false;
		path_sides = *msg;
		append_world_poses(*msg,"sides");
	}
	if(inspection_type == "sides"){
		if(msg->poses.size() > 0){
      set_target_path(create_ordered_path(*msg));
/*
			ROS_INFO("BEHAV: Continuing inspection with new path (%i poses)",msg->poses.size());
			if(path_targets.poses.size() == 0)
        set_target_path(create_ordered_path(*msg));
      else
        extend_target_path(create_ordered_path(*msg));
  */
    }
		else{
			ROS_INFO("BEHAV: Path cannot continue, poses is empty, idling");
      sides_empty = true;
		}
	}
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
	append_world_poses(*msg,"cleared");
}
void pathvlpobs_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
	append_world_poses(*msg,"obstacles");
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
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
void set_tilt(float radians){
  float new_tilt = radians;
  if(new_tilt < par_vlpmintilt){
    tiltlimit_reached  = true;
    new_tilt = par_vlpmintilt;
  }
  else if(new_tilt > par_vlpmaxtilt){
    tiltlimit_reached  = true;
    new_tilt = par_vlpmaxtilt;
  }
	else
		tiltlimit_reached = false;
  if(arm1_tilt_msg.data - new_tilt > 0.1 || arm1_tilt_msg.data - new_tilt < -0.1)
    tilting = true;
  ROS_INFO("BRAIN: ARM - tilt from %.2f -> %.2f",arm1_tilt_msg.data,new_tilt);
  last_tilt = ros::Time::now();
  arm1_tilt_msg.data = new_tilt;
}

void increment_tilt(float radians){
  float new_tilt = arm1_tilt_msg.data + radians;
  set_tilt(new_tilt);
}
void set_tilt_degrees(float degrees){
  float deg2rad = M_PI/180.0;
  set_tilt(deg2rad*degrees);
}
void increment_tilt_degrees(float degrees){
  float deg2rad = M_PI/180.0;
  increment_tilt(deg2rad*degrees);
}
void eval_tilting(){
  int obs = path_obs.poses.size();
  if(inspection_type == "sides"){
    if(sides_empty && !tiltlimit_reached)
      increment_tilt_degrees(-10);
    else if(sides_empty && tiltlimit_reached)
      set_tilt_degrees(20);
    else
      set_tilt(0);
  }
  else if(inspection_type == "down"){
    set_tilt_degrees(10);
  }
  else if(obs > 0){
    path_obs       = sort_path(path_obs,"hdng_abs");
    float hdng_obs = get_hdng(path_obs.poses[0].pose.position,base_pose.pose.position);
    float dst_obs  = get_dst3d(path_obs.poses[0].pose.position,base_pose.pose.position);
    ROS_INFO("hdng path_obs sorted by hdng, pose 0: hdng_obs: %.2f dst_obs: %.2f",hdng_obs,dst_obs);
    if((ros::Time::now() - last_tilt).toSec() > 1.0){
      if(dst_obs > 20)
        increment_tilt_degrees(10);
      else
        increment_tilt_degrees(-10);
    }
  }
  else{
    increment_tilt_degrees(10);
  }
}
void set_inspection(std::string type){
	inspection_count++;
	float dt = (ros::Time::now() - activity_change).toSec();
  activity_change = ros::Time::now();
	ROS_INFO("MAIN: ACTIVITY: %s -> %s (%.3f seconds in state)",inspection_type.c_str(),type.c_str(),dt);
	inspection_type = type;
	if(type == "area")
		set_target_path(create_path(pos,30,4));
	else if(type == "sides" && path_sides.poses.size() > 0)
		set_target_path(path_sides);
	else if(type == "down"  && path_down.poses.size() > 0){
    set_target_path(path_down);
  }
	else if(type == "pnt")
		set_target_pose(target);
  else
    inspection_type = "idle";
}

void set_inspection_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  if(msg->header.frame_id == "tilt"){
    set_tilt_degrees(msg->point.x);
  }
  else
    set_inspection(msg->header.frame_id);
}
void draw_options(int count){
  geometry_msgs::Point p0;
  img_blank.copyTo(img);
  cv::Scalar color_target = get_color(25,25,200);
  cv::Scalar color_sides  = get_color(125,0,0);
  cv::Scalar color_down   = get_color(0,125,0);
  cv::Scalar color_vlp    = get_color(50,50,50);
  cv::Scalar color_area   = get_color(0,50,50);
  cv::Scalar color_clear  = get_color(100,50,0);
  cv::Scalar color_obs    = get_color(0,50,100);
  cv::Scalar color_best    = get_color(0,0,255);
  nav_msgs::Path path_area = create_path(pos,30,4);

  int pcnt_cleared_full = path_cleared_full.poses.size();
  int pcnt_obstacles_full = path_obstacles_full.poses.size();
  int pcnt_area = path_area.poses.size();
  int pcnt_vlp = path_vlp.poses.size();
  int pcnt_sides = path_sides.poses.size();
  int pcnt_down = path_down.poses.size();
  int pcnt_path_down_best = path_down_best.poses.size();
  int pcnt_targets = path_targets.poses.size();
  ROS_INFO("pcnt_cleared_full %i,pcnt_obstacles_full %i,pcnt_area %i,pcnt_vlp %i,pcnt_sides %i,pcnt_down %i,pcnt_targets %i,pcnt_down_best: %i",
            pcnt_cleared_full,   pcnt_obstacles_full,   pcnt_area,   pcnt_vlp,   pcnt_sides,   pcnt_down,   pcnt_targets,   pcnt_path_down_best);

  draw_path_at_img(path_cleared_full,p0,false,false,false,false,true,color_clear,1);
  draw_path_at_img(path_obstacles_full,p0,false,false,false,false,true,color_obs,1);
  draw_path_at_img(path_area,p0,true,false,false,false,true,color_area,1);
  draw_path_at_img(path_vlp,p0,true,false,false,false,true,color_vlp,1);

  draw_path_at_img(path_sides,pos,true,true,false,true,false,color_sides,2);
  draw_path_at_img(path_down,pos,false,false,false,false,true,color_down,2);
  draw_path_at_img(path_targets,pos,true,true,true,true,false,color_target,2);
  draw_path_at_img(path_down_best,pos,true,true,true,true,false,color_best,1);
  cv::imwrite("/home/nuc/brain/draw_options_"+std::to_string(count)+".png",img);
}
void get_next_inspect(){
  int n_sides = path_sides.poses.size();
  int n_down  = path_down.poses.size();
  float yaw0,yaw1;
  geometry_msgs::Point p0_s,p1_s,p0_d,p1_d;
  if(n_sides == 0 && n_down > 0){
    ROS_INFO("Eval next inspect: Sides is empty, use DOWN");
    set_inspection("down");
  }
  else if(n_sides > 0 && n_down == 0){
    ROS_INFO("Eval next inspect: Down is empty, use SIDES");
    set_inspection("sides");
  }
  else if(n_sides > 1 && n_down > 1){
    yaw0 = tf::getYaw(path_sides.poses[0].pose.orientation);
    yaw1 = tf::getYaw(path_sides.poses[n_sides-1].pose.orientation);
    p0_s = path_sides.poses[0].pose.position;
    p1_s = path_sides.poses[n_sides-1].pose.position;
    p0_d = path_down.poses[0].pose.position;
    p1_d = path_down.poses[n_down-1].pose.position;
    float dst_d = get_dst2d(p0_d,p1_d);
    float dst_s = get_dst3d(p0_s,p1_s);
    float dst0_d = get_dst2d(p0_d,base_pose.pose.position);
    float dst0_s = get_dst3d(p0_s,base_pose.pose.position);
    float z_s = fmin(p0_s.z,p1_s.z);
    float z_d = fmin(p0_d.z,p1_d.z);
    float hdng_s0 = get_hdng(p1_s,p0_s);
    float hdng_d0 = get_hdng(p1_d,p0_d);
    float hdng_s  = get_hdng(p0_s,base_pose.pose.position);
    float hdng_d  = get_hdng(p0_d,base_pose.pose.position);

    ROS_INFO("Eval next inspect sides(%i):   dst: %.0f dst0: %.0f z: %.0f, hdng: %.2f hdng0: %.2f",n_sides,dst_s,dst0_s,z_s,hdng_s,hdng_s0);
    ROS_INFO("Eval next inspect  down(%i):   dst: %.0f dst0: %.0f z: %.0f, hdng: %.2f hdng0: %.2f",n_down,dst_d,dst0_d,z_d,hdng_d,hdng_d0);
    if(hdng_s < 0)
      hdng_s *= -1;
    if(hdng_d < 0)
      hdng_d *= -1;
    if(hdng_s0 < 0)
      hdng_s0 *= -1;
    if(hdng_d0 < 0)
      hdng_d0 *= -1;

    float s_score_hdng = (hdng_s + hdng_s0)*-1;
    float s_score_dst0 = dst0_s / -10;
    float s_score_dst  = dst_s / 10;
    float s_score_z    = z_s - z_d;
    float s_score_cand = float(n_sides);
    float s_score_tot = s_score_hdng + s_score_dst0 + s_score_dst + s_score_z + s_score_cand;

    float d_score_hdng = (hdng_d + hdng_d0)*-1;
    float d_score_dst0 = dst0_d / -10;
    float d_score_dst  = dst_d / 10;
    float d_score_z    = z_d - z_s;
    float d_score_cand = float(n_down);
    float d_score_tot = d_score_hdng + d_score_dst0 + d_score_dst + d_score_z + d_score_cand;

    ROS_INFO("Eval next inspect ScoreS: %.2f n: %.2f d0: %.2f d: %.2f z: %.2f n: %.2f",s_score_tot,s_score_hdng,s_score_dst0,s_score_dst,s_score_z,s_score_cand);
    ROS_INFO("Eval next inspect ScoreD: %.2f n: %.2f d0: %.2f d: %.2f z: %.2f n: %.2f",d_score_tot,d_score_hdng,d_score_dst0,d_score_dst,d_score_z,d_score_cand);
    if(s_score_tot > d_score_tot){
      ROS_INFO("Eval SIDES WIN: SCORE SIDES: %.2f SCORE DOWN: %.2f",s_score_tot,d_score_tot);
      set_inspection("sides");
    }
    else{
      ROS_INFO("Eval DOWN WIN: SCORE SIDES: %.2f SCORE DOWN: %.2f",s_score_tot,d_score_tot);
      set_inspection("down");
    }
  }
  else{
    set_inspection("down");
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/10);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/10);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	std_msgs::Float64 target_alt_msg;
	path_cleared_full.header   = hdr();
	path_obstacles_full.header = hdr();
	path_sides_full.header 		 = hdr();
	path_down_full.header      = hdr();
	target.pose.position.z 	   = par_takeoffaltitude;
	target.pose.orientation.w  = 1;
	ros::Subscriber s1 = nh.subscribe("/tb_path/cleared",1,pathvlp_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_path/obstacles",1,pathvlpobs_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_clusters/down",10,pathdown_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_clusters/sides",10,pathsides_cb);
	ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);

  ros::Subscriber s7 = nh.subscribe("/tb_set_inspect",100,&set_inspection_cb);
  ros::Subscriber s8 = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);
	ros::Subscriber s9 = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);

	ros::Publisher pub_path_cleared_full   = nh.advertise<nav_msgs::Path>("/tb_behav/cleared_full",100);
	ros::Publisher pub_path_obstacles_full = nh.advertise<nav_msgs::Path>("/tb_behav/obstacles_full",100);
  ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/tb_behav/targets",10);
  ros::Publisher pub_path_best = nh.advertise<nav_msgs::Path>("/tb_behav/down_best",10);
  ros::Publisher pub_path_down = nh.advertise<nav_msgs::Path>("/tb_behav/down_ordered",10);
  ros::Publisher pub_path_sides = nh.advertise<nav_msgs::Path>("/tb_behav/sides_ordered",10);

	ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
	ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
	ros::Publisher pub_tilt  	 = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_tilt", 10);
	ros::Publisher pub_pan	   = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_pan", 10);

	ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("/cmd_pose",10);

	pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);
	pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
	ros::Rate rate(2.0);
	int count = 0;
	int idle_count = 0;
	std::string last_type;
  while(ros::ok()){
		checktf();
    eval_tilting();
    nav_msgs::Path path_sides_ordered = create_ordered_path(path_sides);
    nav_msgs::Path path_down_ordered = create_ordered_path(path_down);
    pub_path_sides.publish(path_sides_ordered);
    pub_path_down.publish(path_down_ordered);
		if(mainstate == 1)
			check_path_progress();
    if(inspection_type == "idle"){
      count++;
      ROS_INFO("Get next inspection");
      draw_options(count);
      get_next_inspect();
      ROS_INFO("inspection_type: %s",inspection_type.c_str());

    }
		target_alt_msg.data = target.pose.position.z;
    pub_target_pose.publish(target);
    pub_tiltvlp.publish(arm1_tilt_msg);
    pub_altcmd.publish(target_alt_msg);
    pub_path.publish(path_targets);
    pub_path_best.publish(path_down_best);
    pub_path_cleared_full.publish(path_cleared_full);
    pub_path_obstacles_full.publish(path_obstacles_full);
		pub_target_pose.publish(target);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
