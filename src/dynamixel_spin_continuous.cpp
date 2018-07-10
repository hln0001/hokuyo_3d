#include "dynamixel_sdk.h"

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Time.h>

//Spins the dynamixel and publishes the raw position value (0-->2048) as well as the start and end times of each rotation

// Control table address
#define ADDR_MX_TORQUE_ENABLE           64                  // Control table address is different in Dynamixel model
#define ADDR_MX_DRIVE_MODE              11
#define ADDR_MX_GOAL_VELOCITY           104
#define ADDR_MX_PRESENT_POSITION        132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default settings
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.4:1.0-port0"      // Check which port is being used on your controller
															// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
// Packet values for control
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define VELOCITY_CTRL_MODE              1                   // Value to set drive mode to wheel mode

std_msgs::Time start_time;
std_msgs::Time end_time;

class Dynamixel {
private:
  ros::Publisher pub_pos; //position
  ros::Publisher pub_rot; //rotation
  ros::Publisher stpub;   //start time
  ros::Publisher etpub;   //end time
public:
  Dynamixel();
  void timePub();
  ros::NodeHandle node;
  void positionPub(uint16_t dxl_present_position, uint16_t rotation_count);
};

//Dynamixel class constructor creates publishers
Dynamixel::Dynamixel()
{
	//create publisher for motor commands
  pub_pos = node.advertise<std_msgs::UInt16>("dxl_pos", 10);
  pub_rot = node.advertise<std_msgs::UInt16>("rotation_count", 10);
  stpub = node.advertise<std_msgs::Time>("start_time", 1);
  etpub = node.advertise<std_msgs::Time>("end_time", 1);
  end_time.data = ros::Time::now();
};


//Publishes raw position value to ROS
void Dynamixel::positionPub(uint16_t dxl_present_position, uint16_t rotation_count)
{
  std_msgs::UInt16 msg;
  std_msgs::UInt16 rotmsg;
  msg.data = dxl_present_position;
  rotmsg.data = rotation_count;
  pub_pos.publish(msg);
  pub_rot.publish(rotmsg);
}

void Dynamixel::timePub()
{
    start_time.data = end_time.data;
    end_time.data = ros::Time::now();
    stpub.publish(start_time);
    etpub.publish(end_time);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spin_test");
  Dynamixel motor;

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_goal_velocity;
  motor.node.param("goal_speed", dxl_goal_velocity, 50);


  uint8_t dxl_error;                          // Dynamixel error
  uint16_t dxl_present_position;              // Present position
  uint16_t rotation_number;                   // Rotation number 0-15
  uint16_t last_rotation;                     // Adjusted rotation count, accounting for rollover
  uint16_t rotation_count;                    // overall count
  uint16_t half_count;                        // count of half rotations

  // Open port
  portHandler->openPort();

  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE);

  // Disable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  // Change Operating Mode
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_DRIVE_MODE, VELOCITY_CTRL_MODE, &dxl_error);

  // Enable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  while(ros::ok() && dxl_error==0)
  {
	// Write goal speed
	packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_VELOCITY, dxl_goal_velocity, &dxl_error);

	// Read present position
	packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);

	if(dxl_present_position > 4096)
	{
	  rotation_number = floor(dxl_present_position/4096);
	  if (last_rotation != rotation_number)
	  {
	      rotation_count++;
              half_count = 0;
	      last_rotation = rotation_number;
	      motor.timePub();
	  }
	  dxl_present_position -= (rotation_number*4096);
	}

  if(dxl_present_position > 2048 && half_count == 0){
    half_count = 1;
    motor.timePub();
  }

	motor.positionPub(dxl_present_position, rotation_count);


	if(!ros::ok())
	{
	  packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_VELOCITY, 0, &dxl_error);
	  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	}
  }
}
