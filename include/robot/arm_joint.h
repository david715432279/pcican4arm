/****************************************************************************
 * roscpp/include/robot/arm_joint.h
 *
 *   Copyright (C) 2011 David Huang. All rights reserved.
 *   Author: David Huang <davidhuang715432279@gmail.com>
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
#ifndef __APPS_ROSCPP_ARM_JOINT_H_
#define __APPS_ROSCPP_ARM_JOINT_H_
#include <can4linux.h>
#include <stdint.h>

#define JOINT_ANGLE		0x01
#define JOINT_RADIAN  	0x02
#define JOINT_AMOUNT    0x06
#define JOINT_READ_TIMEDLY    10000


//typedef unsigned short int uint8_t;
//typedef unsigned int uint16_t;
/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/
//    int     fd;
	int		cur_pos[JOINT_AMOUNT+1];  //the Joint ID is 0x01-0x06
	int		tar_pos[JOINT_AMOUNT+1];  //the Joint ID is 0x01-0x06
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_caninit() 
 * 
 * Description:
 *   Init the can 1860 and read the device state
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_caninit(void);

/****************************************************************************
 * Name: arm_sendmsg() 
 * 
 * Description:
 *   Send the msg to the arm 
 * param:
 *   int joint (01-06) 
 *   uint8_t cmd
 *   uint8_t index
 *   uint8_t *data 
 *   int data_length
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_sendmsg(uint16_t joint, uint8_t cmd, uint8_t index, uint8_t *data, int data_length);

/****************************************************************************
 * Name: arm_recvack() 
 * 
 * Description:
 *   Recv the msg from the arm 
 * param:
 *   int joint (01-06) 
 *   uint8_t cmd
 *   uint8_t index
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_recvack(uint16_t joint, uint8_t cmd, uint8_t index);

/****************************************************************************
 * Name: arm_readmsg() 
 * 
 * Description:
 *   Read the msg from the arm 
 * param:
 *   int 		joint (01-06) 
 *   uint8_t	addr
 *   int8_t 	length the length want read
 *   uint8_t	return the data
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_readmsg(int joint, uint8_t addr, int8_t length, uint8_t *data);

/****************************************************************************
 * Name: arm_read_REDU_RATIO() 
 * 
 * Description:
 *   Read the ratio of the modle
 * param:
 *   int 		joint (01-06) 
 * return: >=0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_read_REDU_RATIO(int joint);

/****************************************************************************
 * Name: arm_read_worddata() 
 * 
 * Description:
 *   Read the ratio of the modle
 * param:
 *   int 		joint (01-06) 
 *   uint8_t    addr   @the addr your want to read
 *
 * return: >=0 the int data you read
 *         <=0 error
 *
 ****************************************************************************/

int arm_read_worddata(int joint, uint8_t addr);

/****************************************************************************
 * Name: arm_write_worddata() 
 * 
 * Description:
 *   Read the ratio of the modle
 * param:
 *   int 		joint (01-06) 
 *   uint8_t    addr   @the addr your want to read
 *   int        num   @the int data your want to write
 *
 * return: >=0 OK
 *         <=0 error
 *
 ****************************************************************************/

int arm_write_worddata(int joint, uint8_t addr, int num);

/****************************************************************************
 * Name: arm_setpos_joint(int joint, int angle, int cmd) 
 * 
 * Description:
 *   Set the arm in to the position
 * param:
 *   int 		joint (01-06) 
 *   int 		angle (radian or angle) 
 *   int        cmd   (1: the angle is angle 2:the angle is radian)
 * return: >=0  OK
 *         <0 ERROR
 ****************************************************************************/

int arm_setpos_joint(int joint, float angle, int cmd);

int arm_setpos_joint_noth(int joint, float angle, int cmd);

/****************************************************************************
 * Name: arm_readpos_joint(int joint,int cmd, uint8_t) 
 * 
 * Description:
 *   Set the arm in to the position
 * param:
 *   int 		joint (01-06) 
 *   int        cmd   (1: the angle is angle 2:the angle is radian)
 *   uint8_t    opt   (add to let the arm_readmsg can read the limit position)
 * return: the angle or the radian for the joint
 *
 ****************************************************************************/

float arm_readpos_joint(int joint, int cmd, uint8_t opt);

/****************************************************************************
 * Name: arm_readerror_joint(int joint) 
 * 
 * Description:
 *   read the arm error
 * param:
 *   int 		joint (01-06) 
 * return: 0 is OK
 *
 ****************************************************************************/

int arm_readerror_joint(int joint);

/****************************************************************************
 * Name: arm_readpos_thread() 
 * 
 * Description:
 *   read the pos_thread about the pos for each joint in this moment  
 * param:
 * return: 0 is OK
 *
 ****************************************************************************/

 void *arm_readcurpos_thread(void *parameter);

/****************************************************************************
 * Name: arm_init() 
 * 
 * Description:
 * 		init the arm device start the read thread
 * param:
 * return: 0 is OK
 *         <0 is error
 *
 ****************************************************************************/
int arm_init(void);

/****************************************************************************
 * Name: arm_checkpositon() 
 * 
 * Description:
 * 		 check if the current and the target position is equal
 * param:
 *       bool *opt   the check is block or not
 * return: true is OK
 *         false is error
 *
 ****************************************************************************/

 int arm_checkposition(int *opt);

/****************************************************************************
 * Name: arm_action() 
 * 
 * Description:
 * 			define the arm action in the realtime_OS
 * param:
 * return: none 
 *
 ****************************************************************************/

 void arm_action(void);
#endif /* __APPS_ROSCPP_ARM_JOINT_H_*/


