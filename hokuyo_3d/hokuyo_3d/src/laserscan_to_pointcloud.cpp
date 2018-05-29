#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_assembler/AssembleScans.h>

using namespace std;
// using namespace laser_assembler;

laser_geometry::LaserProjection projector;
tf2_ros::Buffer tfBuffer;

ros::Publisher pcl_from_scan;  //initialize a publisher for resultant pointcloud


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

  sensor_msgs::PointCloud2 cloud;

  if(tfBuffer.canTransform("laser", "servo", ros::Time::now(), ros::Duration(1))){
    projector.transformLaserScanToPointCloud("servo", *scan_in, cloud, tfBuffer);  //project scan onto transform frame
    // cloud.header.frame_id = "servo";
    cloud.header.stamp = scan_in->header.stamp;  //set stamp from laserscan stamp

    //Publish pointcloud
    pcl_from_scan.publish(cloud);
  }

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "laserscan_to_pointcloud");
  ros::NodeHandle node;

  ros::Subscriber sub_scan;  //subscriber for the scan
  tf2_ros::TransformListener tfListener(tfBuffer);
/*
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("tfscan", "world", ros::Time(0));
  }
  catch(tf2::TransformException &ex){
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
*/
  pcl_from_scan = node.advertise<sensor_msgs::PointCloud2>("points_from_scan", 1);
  sub_scan = node.subscribe<sensor_msgs::LaserScan>("filtered_scan", 1, scanCallback);

  while(ros::ok()){
    ros::spin();
  }
  node.shutdown();
  return 0;

}
