/****************************************************************************
 * user/claw_joint.c
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
#include <robot/claw_joint.h>
#include <robot/arm_joint.h>
#include <robot/claw_cmd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: claw_init(void)
 ****************************************************************************/

int claw_devinit(){
	//set the dev to the 0 pos	
}

/************claw_setpos_joint********************/

int claw_setpos_joint(int joint, int position){
	uint8_t pos[2];   
	
	if(position < 0 || position >1023){
		printf("the position is error, the position is %d \n", position);
		return -1;
	}

	if(joint!= SYS_CLAW_LEFT_ID && joint != SYS_CLAW_RIGHT_ID){
		printf("the claw joint id is error\n");
		return -1;
	}

	pos[1]= (uint8_t)((position&0xff00)>>8);
	pos[0]= (uint8_t)(position&0xff);

    if(arm_sendmsg(joint, CMDTYPE_WR, SYS_CLAW_POSITION, pos, 2)<0){
		printf("set pos error!");
		return -1;	
	}

	printf("the claw set pos is %x %x\n",pos[0],pos[1]);
	return 0;
}

/************claw_setpos_joint_noth********************/

int claw_setpos_joint_noth(int joint, int position){
	uint8_t pos[2];   
	
	if(position < 0 || position >1023){
		printf("the position is error, the position is %d \n", position);
		return -1;
	}

	if(joint!= SYS_CLAW_LEFT_ID && joint != SYS_CLAW_RIGHT_ID){
		printf("the claw joint id is error\n");
		return -1;
	}

	pos[1]= (uint8_t)((position&0xff00)>>8);
	pos[0]= (uint8_t)(position&0xff);

    if(arm_sendmsg(joint, CMDTYPE_WR_NR, SYS_CLAW_POSITION, pos, 2)<0){
		printf("set pos error!");
		return -1;	
	}

	printf("the claw set pos is %x %x\n",pos[0],pos[1]);
	return 0;
}

/************claw_setspeed_joint********************/
int claw_setspeed_joint(int joint, int speed){
	uint8_t pos[2];   
		
	if(speed < 1 || speed >1023){
		printf("set the speed num is error! the speed is %d \n", speed);
		return -1;
	}

	if(joint!= SYS_CLAW_LEFT_ID && joint != SYS_CLAW_RIGHT_ID){
		printf("the claw joint id is error\n");
		return -1;
	}

	pos[1] = (uint8_t)((speed&0xff00)>>8);
	pos[0] = (uint8_t)(speed&0xff);

    if(arm_sendmsg(joint, CMDTYPE_WR, SYS_CLAW_SPEED, pos, 2)<0){
		printf("set pos error!");
		return -1;	
	}

	printf("the claw set speed is %x %x\n",pos[0],pos[1]);
	return 0;
}

/************claw_setspeed_joint_noth********************/
int claw_setspeed_joint_noth(int joint, int speed){
	uint8_t pos[2];   
		
	if(speed < 1 || speed >1023){
		printf("set the speed num is error! the speed is %d \n", speed);
		return -1;
	}

	if(joint!= SYS_CLAW_LEFT_ID && joint != SYS_CLAW_RIGHT_ID){
		printf("the claw joint id is error\n");
		return -1;
	}

	pos[1] = (uint8_t)((speed&0xff00)>>8);
	pos[0] = (uint8_t)(speed&0xff);

    if(arm_sendmsg(joint, CMDTYPE_WR_NR, SYS_CLAW_SPEED, pos, 2)<0){
		printf("set pos error!");
		return -1;	
	}

	printf("the claw set speed is %x %x\n",pos[0],pos[1]);
	return 0;

}

/**************** claw_readpos_joint***********************/
int claw_readpos_joint(int joint){
	
	uint8_t data[2];
	int temp;

	if(joint!= SYS_CLAW_LEFT_ID && joint != SYS_CLAW_RIGHT_ID){
		printf("the claw joint id is error\n");
		return -1;
	}

	if(arm_readmsg(joint, SYS_CLAW_POSITION, 0x02, data) != 0){
		printf("ERROR: read claw position error");
		return -1;
	}

	temp = (int)(data[0] + data[1]*256);
   	printf("read the claw id %d position is %d \n",joint, temp);
	return temp;
}


/**************** claw_readspeed_joint***********************/
int claw_readspeed_joint(int joint){
	
	uint8_t data[2];
	int temp;

	if(joint!= SYS_CLAW_LEFT_ID && joint != SYS_CLAW_RIGHT_ID){
		printf("the claw joint id is error\n");
		return -1;
	}

	if(arm_readmsg(joint, SYS_CLAW_POSITION, 0x02, data) != 0){
		printf("ERROR: read claw position error");
		return -1;
	}

	temp = (int)(data[0] + data[1]*256);
   	printf("read the claw servo %d position is %d \n",joint, temp);
	return temp;
}

