/* 
* simple driver test: change the bit rate registers with ioctl()
*
* first argument can be the device name -- else it uses can0
*
* if a second arg is given, it is used as new bit rate
*
*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
extern "C"{
#include <can4linux.h>
#include <canfunc.h>
#include <robot/arm_joint.h>
#include <robot/arm_cmd.h>
#include <robot/head_joint.h>
#include <robot/head_cmd.h>
#include <robot/claw_joint.h>
#include <robot/claw_cmd.h>
}

#include "ros/ros.h"
#include "rgmp/Pos.h"
#include "rgmp/Robotcontrol.h"
#include "boost/thread.hpp"
#include "unistd.h"
#include "boost/thread.hpp"

#define ROBOT_LEFT_ARM_DOF 	6
#define ROBOT_RIGHT_ARM_DOF 6

#define ROBOT_RESET 			0x01
#define ROBOT_HEAD_1_MSG 		0x02
#define ROBOT_HEAD_2_MSG 		0x03
#define ROBOT_LEFT_ARM_MSG  	0x04
#define ROBOT_RIGHT_ARM_MSG  	0x05
#define ROBOT_LEFT_CLAW_MSG  	0x06
#define ROBOT_RIGHT_CLAW_MSG  	0x07

#define LEFT_ARM 	1
#define RIGHT_ARM 	2

#define ROBOT_SERVO_DEFAULT_SPEED     150
#define ROBOT_SERVO_DEFAULT_POS   	  511
#define ROBOT_CLAW_SERVO_DEFAULT_POS  511

#define ROBOT_HEAD_SERVO1_NAME      "head_1_servo"
#define ROBOT_HEAD_SERVO2_NAME      "head_2_servo"
#define ROBOT_LEFT_CLAW_NAME        "left_claw"
#define ROBOT_RIGHT_CLAW_NAME       "right_claw"
#define ROBOT_LEFT_ARM_NAME         "left_joint"
#define ROBOT_RIGHT_ARM_NAME        "right_joint"

using namespace boost;
using namespace std;

rgmp::Robotcontrol robot_state_msg;
boost::recursive_mutex can_msg_lock;
ros::Publisher  pub;

void robotInit();
void fill_robot_state_msg(unsigned int cmd, string name, int length, float data[]);
void read_robot_state();

void read_robot_state(){
	int i;
   	float data[ROBOT_LEFT_ARM_DOF];

	while(ros::ok()){
		//step 1 read the left and right arm msg;
		for(i=1; i<=0x06; i++){
	   		can_msg_lock.lock();	
			data[i-1] = arm_readpos_joint(i, JOINT_RADIAN, SYS_POSITION_L);
			can_msg_lock.unlock();
		}
			fill_robot_state_msg(ROBOT_LEFT_ARM_MSG, ROBOT_LEFT_ARM_NAME, ROBOT_LEFT_ARM_DOF, data);
			pub.publish(robot_state_msg);
		for(i=0x11; i<=0x16; i++){
	   		can_msg_lock.lock();	
			data[i-0x11] = arm_readpos_joint(i, JOINT_RADIAN, SYS_POSITION_L);
			can_msg_lock.unlock();
		}
			fill_robot_state_msg(ROBOT_RIGHT_ARM_MSG, ROBOT_RIGHT_ARM_NAME, ROBOT_RIGHT_ARM_DOF, data);
			pub.publish(robot_state_msg);
		//step 2 read the head 1 and 2
	   		can_msg_lock.lock();	
			data[0] = head_readpos_joint(SYS_HEAD_SERVO_1_ID);
			can_msg_lock.unlock();
			fill_robot_state_msg(ROBOT_HEAD_1_MSG, ROBOT_HEAD_SERVO1_NAME, 1, data);
			pub.publish(robot_state_msg);
	   		can_msg_lock.lock();	
			data[0] = head_readpos_joint(SYS_HEAD_SERVO_2_ID);
			can_msg_lock.unlock();
			fill_robot_state_msg(ROBOT_HEAD_2_MSG, ROBOT_HEAD_SERVO2_NAME, 1, data);
			pub.publish(robot_state_msg);
		//step 3 read the claw data
	   		can_msg_lock.lock();	
			data[0] = claw_readpos_joint(SYS_CLAW_LEFT_ID);
			can_msg_lock.unlock();
			fill_robot_state_msg(ROBOT_LEFT_CLAW_MSG, ROBOT_LEFT_CLAW_NAME, 1, data);
			pub.publish(robot_state_msg);
	   		can_msg_lock.lock();	
			data[0] = claw_readpos_joint(SYS_CLAW_RIGHT_ID);
			can_msg_lock.unlock();
			fill_robot_state_msg(ROBOT_RIGHT_CLAW_MSG, ROBOT_RIGHT_CLAW_NAME, 1, data);
			pub.publish(robot_state_msg);
			sleep(1);
	}

}

void rgmpCallback(const rgmp::Robotcontrol& msg)
{
	int i;

	ROS_INFO("david davdi");
	can_msg_lock.lock();
	switch(msg.cmd){
	case ROBOT_HEAD_1_MSG:
		head_setpos_joint_noth(SYS_HEAD_SERVO_1_ID, (int)msg.data[0]);
		break;
	case ROBOT_HEAD_2_MSG:
		head_setpos_joint_noth(SYS_HEAD_SERVO_2_ID, (int)msg.data[0]);
		break;
	case ROBOT_LEFT_CLAW_MSG:	
		claw_setpos_joint_noth(SYS_CLAW_LEFT_ID,(int)msg.data[0]);
		break;
	case ROBOT_RIGHT_CLAW_MSG:
		claw_setpos_joint_noth(SYS_CLAW_RIGHT_ID,(int)msg.data[0]);
		break;
	case ROBOT_LEFT_ARM_MSG:
		for(i=1; i<=0x06; i++){
			ROS_INFO("arm %d data is %f",i,msg.data[i-1]);
			arm_setpos_joint_noth(i, msg.data[i-1], JOINT_RADIAN);
		}
		break;
	case ROBOT_RIGHT_ARM_MSG:
		for(i=0x11; i<=0x16; i++){
			ROS_INFO("arm %d data is %f",i,msg.data[i-0x11]);
			arm_setpos_joint_noth(i, msg.data[i-0x11], JOINT_RADIAN);
		}
		break;
	default:
		break;
	}
	can_msg_lock.unlock();
	return;
}

int main(int argc,char **argv)
{
   int can_fd;
   
   ros::init(argc, argv, "pcican_main");
   ros::NodeHandle n;

   can_fd = arm_caninit();
   sleep(1);
   robotInit();

   ros::Subscriber sub = n.subscribe("robot_can_msg", 1, rgmpCallback);
   pub = n.advertise<rgmp::Robotcontrol>("robot_state_msg", 10);

   boost::thread robot_run_mode(boost::bind(&read_robot_state));
   /* Use the new CAB nit rate to send one message 
    * If no other CAN node is connected, we can see this mesage
    * using an oscilloscope and we can measure the bit rate 
    */
   while(ros::ok()){
	 /* 
	   can_msg_lock.lock();	
		//step 1 read the left and right arm msg;
		for(i=1; i<=0x06; i++) 
			data[i-1] = arm_readpos_joint(i, JOINT_RADIAN, SYS_POSITION_L);
			fill_robot_state_msg(ROBOT_LEFT_ARM_MSG, ROBOT_LEFT_ARM_NAME, ROBOT_LEFT_ARM_DOF, data);
			pub.publish(robot_state_msg);
		for(i=0x11; i<=0x16; i++) 
			data[i-0x11] = arm_readpos_joint(i, JOINT_RADIAN, SYS_POSITION_L);
			fill_robot_state_msg(ROBOT_RIGHT_ARM_MSG, ROBOT_RIGHT_ARM_NAME, ROBOT_RIGHT_ARM_DOF, data);
			pub.publish(robot_state_msg);
		//step 2 read the head 1 and 2
			data[0] = head_readpos_joint(SYS_HEAD_SERVO_1_ID);
			fill_robot_state_msg(ROBOT_HEAD_1_MSG, ROBOT_HEAD_SERVO1_NAME, 1, data);
			pub.publish(robot_state_msg);
			data[0] = head_readpos_joint(SYS_HEAD_SERVO_2_ID);
			fill_robot_state_msg(ROBOT_HEAD_2_MSG, ROBOT_HEAD_SERVO2_NAME, 1, data);
			pub.publish(robot_state_msg);
		//step 3 read the claw data
			data[0] = claw_readpos_joint(SYS_CLAW_LEFT_ID);
			fill_robot_state_msg(ROBOT_LEFT_CLAW_MSG, ROBOT_LEFT_CLAW_NAME, 1, data);
			pub.publish(robot_state_msg);
			data[0] = claw_readpos_joint(SYS_CLAW_RIGHT_ID);
			fill_robot_state_msg(ROBOT_RIGHT_CLAW_MSG, ROBOT_RIGHT_CLAW_NAME, 1, data);
			pub.publish(robot_state_msg);
		can_msg_lock.unlock();
  		ros::spinOnce(); 
		sleep(1);*/
  		ros::spinOnce(); 
   }
 //  		head_readpos_joint(1);
//      head_setpos_joint(1,1000);
 //     canmsg_t txmsg;
//	  arm_setpos_joint_noth(0x01,0.78,JOINT_RADIAN);
//	  arm_setpos_joint_noth(0x02,0.78,JOINT_RADIAN);
   close(can_fd);
   return 0;
}

void fill_robot_state_msg(unsigned int cmd, string name, int length, float data[]){
	int i;
	robot_state_msg.cmd = cmd;
	robot_state_msg.name = name;
	robot_state_msg.length = length;
	robot_state_msg.header.stamp = ros::Time().now();
	robot_state_msg.data.resize(robot_state_msg.length);
		for(i=0; i<=length; i++) 
			robot_state_msg.data[i] = data[i];
	return;
}

void robotInit(){
	int i;

	//step 1 set the speed of the head servo
	head_setspeed_joint_noth(SYS_HEAD_SERVO_1_ID, ROBOT_SERVO_DEFAULT_SPEED);
	head_setspeed_joint_noth(SYS_HEAD_SERVO_2_ID, ROBOT_SERVO_DEFAULT_SPEED);
	claw_setspeed_joint_noth(SYS_CLAW_LEFT_ID, ROBOT_SERVO_DEFAULT_SPEED);
	claw_setspeed_joint_noth(SYS_CLAW_RIGHT_ID, ROBOT_SERVO_DEFAULT_SPEED);

	//step 2 set the pos of the head servo
	head_setpos_joint_noth(SYS_HEAD_SERVO_1_ID, ROBOT_SERVO_DEFAULT_POS);
	head_setpos_joint_noth(SYS_HEAD_SERVO_2_ID, ROBOT_SERVO_DEFAULT_POS);
	claw_setpos_joint_noth(SYS_CLAW_LEFT_ID,  ROBOT_CLAW_SERVO_DEFAULT_POS);
	claw_setpos_joint_noth(SYS_CLAW_RIGHT_ID,  ROBOT_CLAW_SERVO_DEFAULT_POS);
	//wait for go to position
	sleep(2);
	
	//step 3 set the speed of the J60 AND j80 joint

	//step 4 set the pos of each joint
	for(i=0x01; i<=0x06; i++)
		arm_setpos_joint_noth(i, 0, JOINT_RADIAN);
	for(i=0x11; i<=0x16; i++)
		arm_setpos_joint_noth(i, 0, JOINT_RADIAN);

	arm_write_worddata(0x06, LIT_MAX_ACC, 3000);
	arm_write_worddata(0x16, LIT_MAX_ACC, 3000);
	arm_write_worddata(0x06, LIT_MAX_SPEED, 2000);
	arm_write_worddata(0x16, LIT_MAX_SPEED, 2000);
	/*
		arm_setpos_joint_noth(1, 1.178810, JOINT_RADIAN);
		arm_setpos_joint_noth(2, -0.656736, JOINT_RADIAN);
		arm_setpos_joint_noth(3, -0.99311, JOINT_RADIAN);
		arm_setpos_joint_noth(4, 0, JOINT_RADIAN);
		arm_setpos_joint_noth(5, -0.336787, JOINT_RADIAN);
		arm_setpos_joint_noth(6, -1.178810, JOINT_RADIAN);*/
		//arm_setpos_joint_noth(0x12, -1.390782, JOINT_RADIAN);
		//arm_setpos_joint_noth(0x13, 1.910847, JOINT_RADIAN);
	//step 5 wait for the each joint and servo go to the initial position
	sleep(4);

	return;
}

