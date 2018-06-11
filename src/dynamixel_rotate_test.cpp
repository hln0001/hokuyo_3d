#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>

//Spins rotates the dynamixel back and forth 180 deg and publishes the raw position value (0-->2048)

// Control table addresses
#define ADDR_MX_TORQUE_ENABLE           64
#define ADDR_MX_DRIVE_MODE              11
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PRESENT_POSITION        132

// Protocol version
#define PROTOCOL_VERSION                2.0

// Default settings
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"

// Default control table values
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define POSITION_CTRL_MODE              3                   // Value to set drive mode to JOINT mode
#define DXL_MINIMUM_POSITION_VALUE      0                   // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      2048                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

// Class for the node handle, publisher, constructor
class Dynamixel {
private:
  ros::NodeHandle node;
  ros::Publisher pub_pos; //position
public:
  Dynamixel();
  void positionPub(uint16_t dxl_present_position);
};

//Dynamixel class constructor creates publishers
Dynamixel::Dynamixel() {
	//create publisher for position of servo
  pub_pos = node.advertise<std_msgs::UInt16>("dxl_pos", 10);
};

//Publishes raw position value to ROS
void Dynamixel::positionPub(uint16_t dxl_present_position) {
  std_msgs::UInt16 msg;
	msg.data = dxl_present_position;
	pub_pos.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spin_test");
  Dynamixel motor;

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;                                  // used for switching positions
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result declared
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position array
  int pause_time = 1;                             //pause in between position changes

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position

  // Open port
  if (portHandler->openPort())
  {
    // printf("Succeeded to open the port!\n");
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    // printf("Succeeded to change the baudrate!\n");
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    // printf("Dynamixel has been successfully connected \n");
  }

  // Change Operating Mode to Joint
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_DRIVE_MODE, POSITION_CTRL_MODE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    // printf("In position control mode! \n");
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    // printf("Dynamixel has been successfully connected \n");
  }

  while(ros::ok())
  {
    // Write goal position
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    do
    {
      // Read present position
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);
      motor.positionPub(dxl_present_position);
    }while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
    ros::Duration(pause_time).sleep();
  }
}