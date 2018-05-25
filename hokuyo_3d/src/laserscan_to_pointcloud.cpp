#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

laser_geometry::LaserProjection projector;
tf2_ros::Buffer tfBuffer;

ros::Publisher pcl_from_scan;  //initialize a publisher for resultant pointcloud

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

  sensor_msgs::PointCloud2 cloud;

  projector.transformLaserScanToPointCloud("tfscan", *scan_in, cloud, tfBuffer);  //project scan onto transform frame
  cloud.header.frame_id = "laser";  //set id
  cloud.header.stamp = scan_in->header.stamp;  //set stamp from laserscan stamp

  //Publish pointcloud
  pcl_from_scan.publish(cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserscan_to_pointcloud");
  ros::NodeHandle node;
  ros::Subscriber sub_scan;  //subscriber for the scan
  tf2_ros::TransformListener tfListener(tfBuffer);

  sub_scan = node.subscribe<sensor_msgs::LaserScan>("filtered_scan", 1, scanCallback);
  pcl_from_scan = node.advertise<sensor_msgs::PointCloud2>("points_from_scan", 1);

  while(ros::ok()){
    ros::spin();
  }
  node.shutdown();
  return 0;

}
