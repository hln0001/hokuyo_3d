#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/PointCloud2.h>

using namespace laser_assembler;

ros::Publisher pub_cloud;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_assembler_client");
  ros::NodeHandle node;
  pub_cloud = node.advertise<sensor_msgs::PointCloud2>("assembled_cloud",1);
  ros::service::waitForService("assemble_scans");
  ros::ServiceClient client = node.serviceClient<AssembleScans>("assemble_scans");
  AssembleScans srv;
  srv.request.begin = ros::Time(0,0);
  srv.request.end = ros::Time::now();

  if (client.call(srv))
  {
    pub_cloud.publish(srv.response.cloud);
  }
}
