#include<tf2_ros/transform_broadcaster.h>
#include<tf2/LinearMath/Quaternion.h>
#include<ros/ros.h>
#include<std_msgs/UInt16.h>
#include<geometry_msgs/TransformStamped.h>

/* This node publishes the tf between the laser scan and the servo.  This is based on the angle published by the servo. */

using namespace std;

//global variables
float pos;

//Recieves position values from dynamixel servo and uses angle to apply transform to laser scan
void obtainValues(const std_msgs::UInt16 &msg)
{
    //gets position from message
    pos = msg.data;
    double pos_rad = pos/4096*2*3.1416;

    //create transform object
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //set transform information
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "servo";
    transformStamped.child_frame_id = "laser";
    tf2::Quaternion q;

    //perform transform
    q.setRPY(pos_rad, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    //publish transform to the buffer
    br.sendTransform(transformStamped);
}

//main
int main(int argc, char **argv)
{
    //initialize
    ros::init(argc, argv, "laser_transform_broadcaster");
    ros::NodeHandle node;

    //subscirber to current position
    ros::Subscriber position_sub = node.subscribe("dxl_pos", 5, &obtainValues);

    //wait for updates in position
    ros::spin();
}
