#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//Removes close points from the laserscan to filter out the ugv parts

using namespace std;

//define infinity
float inf = numeric_limits<float>::infinity();

double ugv_threshold; //distance threshold variable for laserscan range

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
//  new_scan.intensities = old_scan.intensities;

  //defines ranges array length to 920
  new_scan.ranges.resize(1081);
  new_scan.intensities.resize(1081);
  //check each range for <0.5m and remove those that aren't
  for(int i = 0; i < 1082; i++){
    if (old_scan.ranges[i] > ugv_threshold){
      new_scan.ranges[i] = old_scan.ranges[i]; //transfer original range if valid
       new_scan.intensities[i] = old_scan.intensities[i]; //also transfer intensity data
    }

    else{
      new_scan.ranges[i] = inf; //set to infinity if within the ugv_threshold
    }
  }

  pub.publish(new_scan);  //publish the filtered scan
}

int main(int argc, char **argv) {
  //initialize ROS
  ros::init(argc, argv, "laserscan_ugv_filter", 1);
  ros::NodeHandle node;

  node.param("filter_threshold", ugv_threshold, 1.3); //accept param for ugv_threshold, default 1.3m

  //create publisher and subscribers
  ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("scan", 1, filter);
  pub = node.advertise<sensor_msgs::LaserScan>("filtered_scan", 1);

  ros::spin();
}
