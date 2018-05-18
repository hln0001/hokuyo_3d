#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//Removes close points from the laserscan to filter out the ugv parts

using namespace std;

//define infinity
float inf = numeric_limits<float>::infinity();

float ugv_threshold = 0.5; //distance threshold for laserscan range

//create a publisher
ros::Publisher pub;

void filter(const sensor_msgs::LaserScan old_scan){
  sensor_msgs::LaserScan new_scan;  //create new message for new new_scan

  //transfer data from old_scan to new_scan
  new_scan.header = old_scan.header;
  new_scan.angle_min = old_scan.angle_min;
  new_scan.angle_max = old_scan.angle_max;
  new_scan.angle_increment = old_scan.angle_increment;
  new_scan.scan_time = old_scan.scan_time;
  new_scan.range_min = 0;
  new_scan.range_max = inf;
  new_scan.intensities[920];

  //defines ranges array length to 920
  new_scan.ranges.resize(920);

  //check each range for <0.5m and remove those that angle_increment
  for(int i = 0; i < 921; i++){
    if (new_scan.ranges[i] > 0.5){
      new_scan.ranges[i] = old_scan.ranges[i]; //transfer original range if valid
    }

    else{
      new_scan.ranges[i] = inf; //set to infinity if not valid
    }
  }

  pub.publish(new_scan);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserscan_ugv_filter", 1);
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, filter);

  pub = node.advertise<sensor_msgs::LaserScan>("filtered_scan", 1);

  ros::spin();
}
