#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <chrono>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
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

using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
tf2_ros::Buffer tfBuffer;
bool got_map;
double par_maprad;
std::string par_workdir;
int elevation;

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
int main(int argc,char** argv){
  ros::init(argc,argv,"tb_mbmapper_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("map_sidelength",      par_maprad, 50.0);
  private_nh.getParam("workdir_path",     par_workdir);

  tf2_ros::TransformListener tf2_listener(tfBuffer);


  ros::Subscriber s5  = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Publisher pub_map_updates = nh.advertise<map_msgs::OccupancyGridUpdate>("/map_updates",100);
  //ros::Publisher pub_map  = nh.advertise<nav_msgs::OccupancyGrid>("/map_static",100);
  ros::Rate rate(1.0);

  geometry_msgs::TransformStamped transformStamped;
  float zmin_rel = 10;
  float zmax_rel = 4;
  int count = 0;
  while(ros::ok()){

    count++;

    try{
      transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    int xpos = int(round(transformStamped.transform.translation.x));
    int ypos = int(round(transformStamped.transform.translation.y));
    int zpos = int(round(transformStamped.transform.translation.z));
    if(got_map){
      geometry_msgs::Point bbmin_octree,bbmax_octree,bbmin_custom,bbmax_custom;
      octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
      octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
      ros::Time t0=ros::Time::now();

      bbmin_custom.x = xpos-par_maprad;
      bbmin_custom.y = ypos-par_maprad;

      bbmax_custom.x = xpos+par_maprad;
      bbmax_custom.y = ypos+par_maprad;
      bool outside_octree = false;
      float z0,z1,z;
      if(zpos > bbmax_octree.z+2)
        outside_octree = true;
      else if(zpos >= bbmax_octree.z-2)
        z = int(round(bbmax_octree.z - 1));
      else
        z = zpos;
      z0 = zpos - 2;
      z1 = zpos + 1;
      int xmin = -400;
      int ymin = -400;
      int xmax = +400;
      int ymax = +400;
      int range_x = 800;
      int range_y = 800;
      if(!outside_octree){
        octomap::point3d boundary_min(fmax(bbmin_custom.x,bbmin_octree.x),fmax(bbmin_custom.y,bbmin_octree.y),z0);
        octomap::point3d boundary_max(fmin(bbmax_custom.x,bbmax_octree.x),fmin(bbmax_custom.y,bbmax_octree.y),z1);
  //      ROS_INFO("ProxyPath Updating EDTO from min %.0f %.0f %.0f to max %.0f %.0f %.0f",boundary_min.x(),boundary_min.y(),boundary_min.z(),boundary_max.x(),boundary_max.y(),boundary_max.z());
        octomap::OcTreeKey minKey, maxKey, curKey;
        if (!octree.get()->coordToKeyChecked(boundary_min, minKey))
        {
            ROS_ERROR("Could not create OcTree key at %f %f %f", boundary_min.x(), boundary_min.y(), boundary_min.z());
        }
        if (!octree.get()->coordToKeyChecked(boundary_max, maxKey))
        {
            ROS_ERROR("Could not create OcTree key at %f %f %f", boundary_max.x(), boundary_max.y(), boundary_max.z());
        }
        ROS_INFO("MIN OCTree key at %.0f %.0f %.0f", boundary_min.x(), boundary_min.y(), boundary_min.z());
        ROS_INFO("MAX OCTree key at %.0f %.0f %.0f", boundary_max.x(), boundary_max.y(), boundary_max.z());
        ROS_INFO("DELTA             %.0f %.0f %.0f", boundary_max.x()-boundary_min.x(), boundary_max.y()-boundary_min.y(), boundary_max.z()-boundary_min.z());


        xmin    = int(round(boundary_min.x()));  ymin    = int(round(boundary_min.y()));
        xmax    = int(round(boundary_max.x()));  ymax    = int(round(boundary_max.y()));
       range_x = xmax - xmin;                   range_y = ymax - ymin;

        edf_ptr.reset (new DynamicEDTOctomap(3,octree.get(),
            boundary_min,
            boundary_max,false));
        edf_ptr.get()->update();
      }
      map_msgs::OccupancyGridUpdate update;
      update.header.stamp = ros::Time::now();
      update.header.frame_id = "map";
      update.x = 500+xmin;
      update.y = 500+ymin;
      update.width = range_x;
      update.height =range_y;
      update.data.resize(update.width * update.height);

        unsigned int i = 0;
      if(!outside_octree){
        for (int y = ymin; y < ymax; y++){
          for (int x = xmin; x < xmax; x++){
            octomap::point3d p(x,y,z);
            if(edf_ptr.get()->getDistance(p) < 3)
                update.data[i++] = 100;
              else
                update.data[i++] = 0;
          }
        }
      }
      else{
        for (int y = ymin; y < ymax; y++){
          for (int x = xmin; x < xmax; x++){
            update.data[i++] = 0;
          }
        }
      }
      float dt = (ros::Time::now()-t0).toSec();
      ROS_INFO("MBMAPupdate: %.4f",dt);
      pub_map_updates.publish(update);
    /*     map_msgs::OccupancyGridUpdate update;
      update.header.stamp = ros::Time::now();
      update.header.frame_id = "map";
      update.x = 500;
      update.y = 500;
      update.width = 200 - 0;
      update.height = 200 - 0;
      update.data.resize(update.width * update.height);

      for (unsigned int y = 0; y < 200; y++)
      {
        for (unsigned int x = 0; x < 200; x++)
        {
          update.data[i++] = 100;
        }
      }

      pub_map_updates.publish(update);


      nav_msgs::OccupancyGrid map_static,map;

      map_msgs::OccupancyGridUpdate map_update;
      unsigned i, j;

      map.info.width = 1000;
      map.info.height = 1000;
    //  map_update.width  = maxKey[0] - minKey[0] + 1;
      //map_update.height = maxKey[1] - minKey[1] + 1;
      map_update.width = 1000;
      map_update.height = 1000;

      map.info.resolution = octree.get()->getResolution();

      octomap::point3d octoorigin =   octree.get()->keyToCoord(minKey, octree.get()->getTreeDepth());
      map.info.origin.position.x = octoorigin.x() - octree.get()->getResolution() * 0.5;
      map.info.origin.position.y = octoorigin.y() - octree.get()->getResolution() * 0.5;

      map.info.origin.position.x = -500;
      map.info.origin.position.y = -500;
      map_update.x = 0;
      map_update.y = 0;

      map.info.origin.orientation.x = 0.;
      map.info.origin.orientation.y = 0.;
      map.info.origin.orientation.z = 0.;
      map.info.origin.orientation.w = 1.;

      map_update.header.frame_id = "map";
      map.header.frame_id = "map";

      map.data.resize(map.info.width * map.info.height, 0);
      map_update.data.resize(map_update.width * map_update.height, 0);

      //init with unknown
      for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
        *it = 0;
      }


      int missing_x = octoorigin.x() - octree.get()->getResolution() * 0.5 + 500;
      int missing_y = octoorigin.y() - octree.get()->getResolution() * 0.5 + 500;


      for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
      {
          for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
          {
              for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2])
              { //iterate over height
                  octomap::OcTreeNode* node = octree.get()->search(curKey);
                  if (node)
                  {
                      octomap::point3d pnt = octree.get()->keyToCoord(curKey);
                      bool occupied = octree.get()->isNodeOccupied(node);
                      //ROS_INFO("j: %i, i: %i, map[%i] xyz %.0f %.0f %.0f",
                      //j,i,(map.info.width *j + i),pnt.x(),pnt.y(),pnt.z());

                      if(occupied) {
                        map.data[map.info.width * (j+missing_y) + missing_x + i] = 100;
                    //   map_update.data[map_update.width * j + i] = 100;
                        map_update.data[map_update.width * (j+missing_y) + missing_x + i] = 100;
                        break;
                      }
                      else {
                        map.data[map.info.width * (j+missing_y) + missing_x + i] = 0;
                        map_update.data[map_update.width * (j+missing_y) + missing_x + i] = 0;
                      //  map_update.data[map_update.width * j + i] = 0;

                      //  map.data[map.info.width * j + i] = 0;
                        //map_static.data[map_static.info.width * j + i] = 0;
                      //  map_update.data[map_update.width * j + i] = 0;
                        //map_update_static.data[map_update_static.width * j + i] = 0;
                      }
                  }
              }
          }
      }
      pub_map.publish(map);
    pub_map_updates.publish(map_update);

    */

    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
