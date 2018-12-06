#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Time.h"
#include <math.h>
#include <hokuyo_msgs/ServiceTimes.h>

ros::Time start_time;
ros::Time end_time;
hokuyo_msgs::ServiceTimes service_times;


void timesCallback(const hokuyo_msgs::ServiceTimes &msg)
{
  start_time = msg.start_time.data;
  end_time = msg.end_time.data;
}

namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  //subscribers, nodehandle, client, time objects
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::ServiceClient client;

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub = n.advertise<sensor_msgs::PointCloud2> ("hokuyo_points", 1); 


    // Create the service client for calling the assembler
    client = n.serviceClient<AssembleScans2>("assemble_scans2");
  }

private:

};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  float buffer_duration;
  int dxl_goal_velocity;
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("service_times", 1, timesCallback);
  ros::service::waitForService("build_cloud");
  PeriodicSnapshotter snapshotter;
  n.param("goal_speed", dxl_goal_velocity, 131);

/*  
  if (dxl_goal_velocity != 0)
  {
    buffer_duration = 1/((double)dxl_goal_velocity/131); //131 ~= 1hz
    //ROS_INFO("set duration %u", dxl_goal_velocity);
  }
  else
  {
    buffer_duration = 2;
  }
*/
  
  ros::Time last_start;
  AssembleScans2 srv;
  while(ros::ok())
  {
    ros::spinOnce();

    // Populate our service request based on our timer callback times
    if(fabs(last_start.toSec() - start_time.toSec())  > 0.25)
    {
      //srv.request.begin = start_time - ros::Duration(buffer_duration);
      srv.request.begin = start_time;
      srv.request.end = end_time;
      last_start = start_time;
      // Make the service call
      if (snapshotter.client.call(srv))
      {
	//ROS_INFO("Time difference: %f start: %f end: %f", fabs(end_time.toSec() - start_time.toSec()), start_time.toSec(), end_time.toSec());
        snapshotter.pub.publish(srv.response.cloud);
      }      

    }
  }
  return 0;
}
