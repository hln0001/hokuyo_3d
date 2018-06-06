#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <std_msgs/Empty.h>

#include <stdlib.h>
#include <stdio.h>

#include <dynamixel_sdk.h>

using namespace std;

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different for each Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0

// Default setting
#define DXL_ID                          8                   // Dynamixel ID: 8
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold


//create commands for motor
class Dynamixel {
	private:
	ros::NodeHandle node;
	ros::Publisher pub_ts;  //start time
	ros::Publisher pub_te;  //end time
	ros::Publisher pub_pos; //position

	public:
	Dynamixel();
	void startTime();
	void endTime();
  void moveMotor(int index, uint8_t dxl_error, uint16_t dxl_present_position);
  void initialize(int index, uint8_t dxl_error, uint16_t dxl_present_position);
  double pause_time;
	int dxl_goal_position[2];
	int ADDR_PRO_GOAL_POSITION;
	int dxl_comm_result;
};

//Dynamixel class constructor creates publishers
Dynamixel::Dynamixel() {
	//create publisher for motor commands
	pub_pos = node.advertise<std_msgs::UInt16>("dxl_pos", 10);

	//create a publisher for publishing start and end times of sweeps
	pub_ts = node.advertise<std_msgs::Time>("start_time", 1);
	pub_te = node.advertise<std_msgs::Time>("end_time", 1);
};

//Function for moving motor
void Dynamixel::moveMotor(int index, uint8_t dxl_error, uint16_t dxl_present_position) {
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		ROS_INFO("%s/n",packetHandler->getRxPacketError(dxl_error));
	}

	do
	{
		// Read present position
		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}

		ROS_INFO("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], &dxl_present_position);

	}while(abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD);
	//change the index to turn the other way next loop
	if (index == 0) {
		index = 1;
	}
	else if (index == 1) {
		index = 0;
	}
	ros::Duration(pause_time).sleep();
	std_msgs::UInt16 msg;
	msg.data = dxl_present_position;
	pub_pos.publish(msg);
};

//publish start time for cloud assembly
void Dynamixel::startTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_ts.publish(msg);
};


//publish end time for cloud assembly
void Dynamixel::endTime() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    pub_te.publish(msg);
};

//initialize motor to min angle
void Dynamixel::initialize(int index, uint8_t dxl_error, uint16_t dxl_present_position) {
	Dynamixel motor_init; //create class object only used in the function
	motor_init.moveMotor(index, dxl_error, dxl_present_position);
	ros::Duration(pause_time).sleep();
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_sweep_half");

	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux
	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler
	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl_present_position = 0;              // Present position

	Dynamixel motor;

	// Open port
	if (portHandler->openPort())
	{
		ROS_INFO("Succeeded to open the port!\n");
	}

	else
	{
		ROS_INFO("Failed to open the port!\n");
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		ROS_INFO("Succeeded to change the baudrate!\n");
	}

	else
	{
		ROS_INFO("Failed to change the baudrate!\n");
		return 0;
	}

	// Enable Dynamixel Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_INFO("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}

	else if (dxl_error != 0)
	{
		ROS_INFO("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	else
	{
		ROS_INFO("Dynamixel has been successfully connected \n");
	}

  double pause_time = 0.1;

	ros::Duration(1).sleep();
	motor.initialize(index, dxl_error, dxl_present_position);

	//Main loop
	while(ros::ok()) {
		motor.moveMotor(index, dxl_error, dxl_present_position);
	}
}
