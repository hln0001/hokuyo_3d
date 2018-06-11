// #include <cstdio>
#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"

namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 1); //must be "velodyne_points" for blam_slam node

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(2.5,0), &PeriodicSnapshotter::timerCallback, this); //change this line to change frequency
//                                        ^this value
    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void timerCallback(const ros::TimerEvent& e)
  {

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    AssembleScans2 srv;
    srv.request.begin = e.last_real;
    srv.request.end   = e.current_real;

    // Make the service call
    if (client_.call(srv))
    {
      // ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
      pub_.publish(srv.response.cloud);
    }
    else
    {
      // ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  // ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  // ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
