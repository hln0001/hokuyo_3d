#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<dynamixel_workbench_msgs/DynamixelState.h>
#include<dynamixel_workbench_msgs/JointCommand.h>
#include<cmath>
#include<ros.time.h>
#include<std_msgs/Time.h>
#include<std_msgs/Empty.h>

//This code sweeps the dynamixel motor continuously between two set angles
//It also publishes start and end times for the sweep for the point cloud assembler

using namespace std;

//global vars
int min_angle;
int max_angle;
float pause_time;

//create commands for motor
class Dynamixel {
	private:
	ros::NodeHandle node;
//	ros::Publisher pub_cmd;  //motor command
	ros::Publisher pub_ts;  //start time
	ros::Publisher pub_te;  //end time
	
	
	public:
	Dynamixel();
	void moveMotor(double set_pos);
	void startTime();
	void endTime();
}

//Dynamixel class constructor creates publishers
Dynamixel::Dynamixel() {
	//create publisher for motor commands
	//pub_cmd = node.advertise<std_msgs::Float64>(DynamixelState::goal_position, 10);
	
	//create a publisher for publishing start and end times of sweeps
	pub_ts = node.advertise<std_msgs::Time>("/time/start_time", 1);
	pub_te = node.advertise<std_msgs::Time>("/time/end_time", 1);
}

/*
void Dynamixel::moveMotor(double set_pos) {
	double set_pos_rad = (set_pos * 3.14/180);
	std_msgs::Float64 msg;
	msg.data = set_pos_rad;
	pub_cmd.publish(msg);
	ROS_INFO_STREAM(msg);
}
*/

//publish start time for cloud assembly
void Dynamixel::startTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_ts.publish(msg);
}


//publish end time for cloud assembly
void Dynamixel::endTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_te.publish(msg);
}

//initialize motor to min angle
void initialize() {
	Dynamixel motor_init; //create class object only used in the function
	motor_init.moveMotor(min_angle);
	ros::Duration(pause_time).sleep();
}

/*
//perform a sweep
void sweep() {
	Dynamixel motor;
	motor.startTime();
	motor.moveMotor(max_angle):
	ros::Duration(pause_time).sleep();
	motor.moveMotor(min_angle);
	ros::Duration(pause_time).sleep();
	motor.endTime();
	ros::Duration(pause_time).sleep():
	ROS_INFO("Sweep Complete");
}
*/

int main(int argc, char **argv) {
	//initialize ros
	ros::init(argc,argv, "motor_sweep");
	dynamixel_workbench_msgs::JointCommand joint_command;
	ros::NodeHandle node;

	//init the service client
	ros::ServiceClient joint_command_client = node_handle.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");


	int max;
	int min;
	double pausetime;
	Dynamixel motor;

	node.param("maximum",max,92);
	node.param("minimum",min,-92);
	node.param("pause", pausetime, 0.1);

	//set global variables to parameter values
	max_angle = max;
	min_angle = min;
	pause_time = pausetime;

	//confirm servo initialization before starting sweep:
	ros::topic::waitForMessage<dynamixel_workbench_msgs::DynamixelState>("present_position",ros::Duration(100));

	ros::Duration(1).sleep();
	initialize();

	while(ros::ok()) {
		joint_command.request.goal_position = max_angle;
		ros::Duration(pause_time).sleep();
		joint_command.request.goal_position = min_angle; 
        	ros::Duration(pause_time).sleep();
		//sweep();
	}
}
