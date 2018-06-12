#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/UInt16.h"

int rotNum;

void rotNumCallback(std_msgs::UInt16 rotation_number)
{
  rotNum = rotation_number.data;
}

namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 1); //must be "velodyne_points" for blam_slam node

    sub_ = n_.subscribe<std_msgs::UInt16>("rotation_number", 1, rotNumCallback);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

    last_time = ros::Time(0,0);

    // Start the timer that will trigger the processing loop (timerCallback)
    PeriodicSnapshotter::timerCallback();

    pubNum = 0;
  }

  void timerCallback()
  {

    if(rotNum != pubNum)
    {
      // Populate our service request based on our timer callback times
      AssembleScans2 srv;
      srv.request.begin = last_time;
      srv.request.end   = ros::Time::now();

      // Make the service call
      if (client_.call(srv))
      {
        // ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
        pub_.publish(srv.response.cloud);
        pubNum++;
        last_time = ros::Time::now();

      }
      else
      {
        // ROS_ERROR("Error making service call\n") ;
      }
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
  ros::Time last_time;
  ros::Timer timer_;
  uint16_t pubNum;

};
}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ros::service::waitForService("build_cloud");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
