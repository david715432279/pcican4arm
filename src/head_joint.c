/****************************************************************************
 * user/head_joint.c
 *
 *   Copyright (C) 2013 BUAA robot label. All rights reserved.
 *   Author: David Huang <davidhuang715432279@gmail.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <can4linux.h>
#include <canfunc.h>
#include <robot/head_joint.h>
#include <robot/arm_joint.h>
#include <robot/head_cmd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: head_init(void)
 ****************************************************************************/

int head_devinit(){
	//set the dev to the 0 pos	
}

/****************************************************************************
 * Name: head_sendmsg() 
 * 
 * Description:
 *   Send the msg to the arm 
 * param:
 *   int joint (01-02) 
 *   uint8_t cmd
 *   uint8_t index
 *   uint8_t *data 
 *   int data_length
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int head_sendmsg(uint16_t joint, uint8_t cmd, uint8_t index, uint8_t *data, int data_length){
  	ssize_t nbytes;
	canmsg_t txmsg;

	if(cmd == CMDTYPE_WR){
		if(data_length > 6){
  	 		printf("arm_sendmsg error,the data_length must <= 6!\n");
        }
		// clear the txmsg
  		memset(&txmsg,0,sizeof(canmsg_t));
		// Construct the txmsg
		// 1 the STID
    		txmsg.id= joint;
    		txmsg.flags = MSG_BASE;
		// 2 the DLC/4 is the data_length + 2
    		txmsg.length   = data_length + 2;
		// 3 the data 1 Byte is the cmd
        txmsg.data[0] = cmd;
        txmsg.data[1] = index;
		memcpy(&txmsg.data[2], data, data_length);

        //Send the TX message 

    	nbytes = write(fd, &txmsg, 1);
    	if (nbytes < 0)
      	{
        	printf("ERROR: write 1 returned %d\n", nbytes);
			return -1;
      	}
		//TODO : receive the ack message
		if(head_recvack(joint, cmd, index)==0)
			return 0;
		else{
			printf("recvack is error\n");
			return -1;
		}
	}else if(cmd == CMDTYPE_WR_NR){
		if(data_length > 6){
  	 		printf("arm_sendmsg error,the data_length must <= 6!\n");
        }
		// clear the txmsg
  		memset(&txmsg,0,sizeof(canmsg_t));
		// Construct the txmsg
		// 1 the STID
    		txmsg.id    = joint;
    		txmsg.flags = MSG_BASE;
		// 2 the DLC/4 is the data_length + 2
    		txmsg.length   = data_length + 2;
		// 3 the data 1 Byte is the cmd
        txmsg.data[0] = cmd;
        txmsg.data[1] = index;
		memcpy(&txmsg.data[2], data, data_length);

        //Send the TX message 

    	nbytes = write(fd, &txmsg, 1);
    	if (nbytes < 0)
      	{
        	printf("ERROR: write returned %d\n",  nbytes);
			return -1;
      	}
		return 0;
	
	}else if(cmd == CMDTYPE_RD){
		// clear the txmsg
  		memset(&txmsg,0,sizeof(canmsg_t));
		// Construct the txmsg
		// 1 the STID
    		txmsg.id    = joint;
    		txmsg.flags = MSG_BASE;
		// 2 the DLC/4 is the data_length + 2
		// 2 the DLC/4 is 03 when read cmd
    		txmsg.length   = 0x03;
		// 3 the data 1 Byte is the cmd
        txmsg.data[0] = cmd;
        txmsg.data[1] = index;
        txmsg.data[2] = (uint8_t)data_length;

        //Send the TX message 

    	nbytes = write(fd, &txmsg, 1);
    	if (nbytes < 0)
      	{
        	printf("ERROR: sendmsg returned %d\n", nbytes);
			return -1;
      	}
		return 0;
	}else{
      		printf("cmd error,the joint must in 00-05!\n");
			return -1;
	}
        	printf("unknown error\n");
			return -1;
}

/************head_setpos_joint********************/

int head_setpos_joint(int joint, int position){
	uint8_t pos[2];   
	
	if(position < 0 || position >1023){
		printf("the position is error, the position is %d \n", position);
		return -1;
	}

	pos[1]= (uint8_t)((position&0xff00)>>8);
	pos[0]= (uint8_t)(position&0xff);

		if(joint == SYS_HEAD_SERVO_1_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR, SYS_HEAD_SERVO_1_POSITION, pos, 2);
		}
		else if(joint == SYS_HEAD_SERVO_2_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR, SYS_HEAD_SERVO_2_POSITION, pos, 2);
		}
		else{
			printf("set pos error!");
			return -1;	
		}

		printf("the head set pos is %x %x\n",pos[0],pos[1]);
	return 0;
}

/************head_setpos_joint_noth********************/

int head_setpos_joint_noth(int joint, int position){
	uint8_t pos[2];   
	
	if(position < 0 || position >1023){
		printf("the position is error, the position is %d \n", position);
		return -1;
	}

	pos[1]= (uint8_t)((position&0xff00)>>8);
	pos[0]= (uint8_t)(position&0xff);

		if(joint == SYS_HEAD_SERVO_1_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR_NR, SYS_HEAD_SERVO_1_POSITION, pos, 2);
		}
		else if(joint == SYS_HEAD_SERVO_2_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR_NR, SYS_HEAD_SERVO_2_POSITION, pos, 2);
		}
		else{
			printf("set pos error!");
			return -1;	
		}

		printf("the head set pos is %x %x\n",pos[0],pos[1]);
	return 0;
}

/************head_setspeed_joint********************/
int head_setspeed_joint(int joint, int speed){
	uint8_t pos[2];   
		
	if(speed < 1 || speed >1023){
		printf("set the speed num is error! the speed is %d \n", speed);
		return -1;
	}

	pos[1] = (uint8_t)((speed&0xff00)>>8);
	pos[0] = (uint8_t)(speed&0xff);

		if(joint == SYS_HEAD_SERVO_1_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR, SYS_HEAD_SERVO_1_SPEED, pos, 2);
		}
		else if(joint == SYS_HEAD_SERVO_2_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR, SYS_HEAD_SERVO_2_SPEED, pos, 2);
		}
		else{
			printf("set speed error!");
			return -1;	
		}

		printf("the head set speed is %x %x\n",pos[0],pos[1]);
	return 0;
}

/************head_setspeed_joint_noth********************/
int head_setspeed_joint_noth(int joint, int speed){
	uint8_t pos[2];   
		
	if(speed < 1 || speed >1023){
		printf("set the speed num is error! the speed is %d \n", speed);
		return -1;
	}

	pos[1] = (uint8_t)((speed&0xff00)>>8);
	pos[0] = (uint8_t)(speed&0xff);

		if(joint == SYS_HEAD_SERVO_1_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR_NR, SYS_HEAD_SERVO_1_SPEED, pos, 2);
		}
		else if(joint == SYS_HEAD_SERVO_2_ID){
  	    	head_sendmsg(SYS_HEAD_ID, CMDTYPE_WR_NR, SYS_HEAD_SERVO_2_SPEED, pos, 2);
		}
		else{
			printf("set speed error!");
			return -1;	
		}

		printf("the head set speed is %x %x\n",pos[0],pos[1]);
	return 0;

}

/*********************head_recvack************************/
int head_recvack(uint16_t joint, uint8_t cmd, uint8_t index){
  	ssize_t nbytes;

	canmsg_t rxmsg;

	//TODO: becareful the thread 
  		memset(&rxmsg,0,sizeof(canmsg_t));
    /* Read the RX message */
    	nbytes = read(fd, &rxmsg, 1);
    	if (nbytes < 0)
      	{
        	printf("ERROR: read ack error returned %d\n", nbytes);
			return -1;
      	}
	   	if((rxmsg.id != joint + (uint16_t)0x100)){
        	printf("ERROR: read ack ID error\n");
			return -1;
		}
	   	if(rxmsg.length != 0x03){
        	printf("ERROR: read ack DLC error\n");
			return -1;
		}
	   	if(rxmsg.data[0] != cmd){
        	printf("ERROR: read ack cmd error\n");
			return -1;
		}
	   	if(rxmsg.data[1] != index){
        	printf("ERROR: read ack index error\n");
			return -1;
		}
		return 0;
}

/**************** head_readpos_joint***********************/
int head_readpos_joint(int joint){
	
	uint8_t data[2];
	int temp;

	if(joint == SYS_HEAD_SERVO_1_ID){
		if(arm_readmsg(SYS_HEAD_ID, SYS_HEAD_SERVO_1_POSITION, 0x02, data) != 0){
			printf("ERROR: read head servo 1 position error");
		}
	}else if(joint == SYS_HEAD_SERVO_2_ID){
		if(arm_readmsg(SYS_HEAD_ID, SYS_HEAD_SERVO_2_POSITION, 0x02, data) != 0){
			printf("ERROR: read head servo 1 position error");
		}
	}

	temp = (int)(data[0] + data[1]*256);
   	printf("read the head servo %d position is %d \n",joint, temp);
	return temp;
}


/**************** head_readspeed_joint***********************/
int head_readspeed_joint(int joint){
	
	uint8_t data[2];
	int temp;

	if(joint == SYS_HEAD_SERVO_1_ID){
		if(arm_readmsg(SYS_HEAD_ID, SYS_HEAD_SERVO_1_SPEED, 0x02, data) != 0){
			printf("ERROR: read head servo 1 position error");
		}
	}else if(joint == SYS_HEAD_SERVO_2_ID){
		if(arm_readmsg(SYS_HEAD_ID, SYS_HEAD_SERVO_2_SPEED, 0x02, data) != 0){
			printf("ERROR: read head servo 2 position error");
		}
	}

	temp = (int)(data[0] + data[1]*256);
   	printf("read the head servo %d position is %d \n",joint, temp);
	return temp;
}

