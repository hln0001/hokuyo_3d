#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/static_transform_broadcaster.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Vector3.h>
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
    static tf2_ros::StaticTransformBroadcaster sbr;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped staticTransformStamped;

    //set transform between base_link and servo
    staticTransformStamped.header.stamp = ros::Time::now();
    staticTransformStamped.header.frame_id = "base_link";
    staticTransformStamped.child_frame_id = "servo";
    tf2::tf2Vector4 v;
    tf2::Quaternion sq;

    //set transform between servo and laser
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "servo";
    transformStamped.child_frame_id = "laser";
    tf2::Quaternion q;

//0.207087
    //perform transforms
    //q.setRPY(pos_rad-0.207087+3.14159, 0, 0);  //forward facing, nodding 180deg
    q.setRPY(0, 3 * 3.14159 / 2, pos_rad - 0.207087 - 3.14159/2);  //facing up, full rotation
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    //sq.setRPY(0, 0, 3.14159);  //forward facing, nodding 180deg
    //sq.setRPY(0, 0, 0);  //facing up, full rotation
    sq.setRPY(0, 3.14159/4, 0);  //facing up, 45deg mount angle forward.
    v.setValue(-0.381, 0, 0.6604, 0);
    staticTransformStamped.transform.translation.x = v.x();
    staticTransformStamped.transform.translation.y = v.y();
    staticTransformStamped.transform.translation.z = v.z();
    //transformStamped.transform.translation.w = v.w();
    staticTransformStamped.transform.rotation.x = sq.x();
    staticTransformStamped.transform.rotation.y = sq.y();
    staticTransformStamped.transform.rotation.z = sq.z();
    staticTransformStamped.transform.rotation.w = sq.w();


    //publish transform to the buffer
    sbr.sendTransform(staticTransformStamped);
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
