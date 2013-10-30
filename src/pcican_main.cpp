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
}

#include "ros/ros.h"
#include "rgmp/Pos.h"
#include "unistd.h"

rgmp::Pos joint_msg;

void jointCallback(const rgmp::Pos& msg )
{
	if(joint_msg.joint1==msg.joint1&&joint_msg.joint2==msg.joint2&&joint_msg.joint3==msg.joint3&&joint_msg.joint4==msg.joint4&&joint_msg.joint5==msg.joint5&&joint_msg.joint6==msg.joint6)
		return;
	  arm_setpos_joint_noth(0x06, msg.joint6,JOINT_RADIAN);
	  arm_setpos_joint_noth(0x05, msg.joint5,JOINT_RADIAN);
	  arm_setpos_joint_noth(0x04, msg.joint4,JOINT_RADIAN);
	  arm_setpos_joint_noth(0x03, msg.joint3,JOINT_RADIAN);
	  arm_setpos_joint_noth(0x02, msg.joint2,JOINT_RADIAN);
	  arm_setpos_joint_noth(0x01, msg.joint1,JOINT_RADIAN);
    
	  joint_msg.joint1 = msg.joint1;
	  joint_msg.joint2 = msg.joint2;
	  joint_msg.joint3 = msg.joint3;
	  joint_msg.joint4 = msg.joint4;
	  joint_msg.joint5 = msg.joint5;
	  joint_msg.joint6 = msg.joint6;
	printf("recv msg %f\n",msg.joint1);
	printf("recv msg %f\n",msg.joint2);
	printf("recv msg %f\n",msg.joint3);
	printf("recv msg %f\n",msg.joint4);
	printf("recv msg %f\n",msg.joint5);
	printf("recv msg %f\n",msg.joint6);
}

int main(int argc,char **argv)
{
   int can_fd;
   
   joint_msg.joint1 = 0.01;
   ros::init(argc, argv, "pcican_main");
   ros::NodeHandle n;

   can_fd = arm_caninit();
   sleep(1);

//   ros::Subscriber sub = n.subscribe("chatter", 4, jointCallback);
   /* Use the new CAB nit rate to send one message 
    * If no other CAN node is connected, we can see this mesage
    * using an oscilloscope and we can measure the bit rate 
    */
   while(ros::ok()){
  		ros::spinOnce(); 
		usleep(50000);
   }
	  arm_setpos_joint_noth(0x01,0.78,JOINT_RADIAN);
	  arm_setpos_joint_noth(0x02,0.78,JOINT_RADIAN);
   sleep(1);
   close(can_fd);
   return 0;
}

