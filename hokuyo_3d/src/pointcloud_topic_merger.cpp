#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//takes in pointclouds from several topics and republishes them on the /cloud_in topic for use with octomap

using namespace std;

ros::Publisher pub;

void hokuyoCallback(const sensor_msgs::PointCloud2 msg)
{
    sensor_msgs::PointCloud2 hokuyo_cloud;
    hokuyo_cloud = msg;
    pub.publish(hokuyo_cloud); 
}

void velodyneCallback(const sensor_msgs::PointCloud2 msg)
{
    sensor_msgs::PointCloud2 velodyne_cloud;
    velodyne_cloud = msg;
    pub.publish(velodyne_cloud); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_topic_merger", 1);
    ros::NodeHandle n;

    pub = n.advertise<sensor_msgs::PointCloud2>("cloud_in", 1);

    ros::Subscriber hokuyo_sub = n.subscribe<sensor_msgs::PointCloud2>("hokuyo_points", 1, hokuyoCallback);
    ros::Subscriber velodyne_sub = n.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, velodyneCallback);

    ros::spin();
}
