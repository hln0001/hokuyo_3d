#include<tf2_ros/transform_broadcaster.h>
#include<tf2/LinearMath/Quaternion.h>
#include<ros/ros.h>
#include<dynamixel_workbench_msgs/msg/DynamixelState.h>

/* This node publishes the tf between the laser scan and the servo.  This is based on the angle published by the servo. */

//Module that applies transform to laser scan of tilting hokuyo laser
using namespace std;

//global variables
float pos;

//Recieves position values from dynamixel servo and uses angle to apply transform to laser scan
void obtainValues(const dynamixel_workbench_msgs::DynamixelState &msg) 
{
    //gets position from message
    pos = msg.present_position;
    
    //perform transform
    static tf2_ros::TransformBroadcaster br;
    tf2::Transform transform;
    transform.setOrigin( tf2::Vector3(0.0, 0.0, 0.0) );
    tf2::Quaternion q;
    q.setRPY(pos, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf2::StampedTransform(transform, ros::Time::now(), "servo", "laser"));
}

//main
int main(int argc, char **argv) 
{
    //initialize
    ros::init(argc, argv, "laser_transform");
    ros::NodeHandle node;
  
    //subscirber to current position
    ros::Subscriber position_sub = node.subscribe("present_position", 5, &obtainValues);

    //wait for updates in position
    ros::spin();
}
