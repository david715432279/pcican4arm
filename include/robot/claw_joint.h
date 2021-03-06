/****************************************************************************
 * roscpp/include/robot/claw_joint.h
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
#ifndef __ROBOT_CLAW_JOINT_H_
#define __ROBOT_CLAW_JOINT_H_
#include <can4linux.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: claw_devinit() 
 * 
 * Description:
 *   Init the device state
 * return: 0  OK
 *         <0 ERROR
 ****************************************************************************/

int claw_devinit(void);

int claw_recvack(uint16_t joint, uint8_t cmd, uint8_t index);

/****************************************************************************
 * Name: claw_setpos_joint(int joint, int position) 
 * 
 * Description:
 *   Set the claw in to the position
 * param:
 *   int 		joint (claw ID) 
 *   int 		position (0~1023)
 * return: >=0  OK
 *         <0 ERROR
 ****************************************************************************/

int claw_setpos_joint(int joint, int position);

int claw_setpos_joint_noth(int joint, int position);

/****************************************************************************
 * Name: claw_setspeed_joint(int joint, int speed) 
 * 
 * Description:
 *   Set the claw in to the position
 * param:
 *   int 		joint (claw ID) 
 *   int 		speed (1~1023)
 * return: >=0  OK
 *         <0 ERROR
 ****************************************************************************/

int claw_setspeed_joint(int joint, int speed);

int claw_setspeed_joint_noth(int joint, int speed);

/****************************************************************************
 * Name: claw_readpos_joint(int joint) 
 * 
 * Description:
 *   Set the claw in to the position
 * param:
 *   int 		joint (claw ID) 
 * return: the pos for the claw
 *
 ****************************************************************************/

int claw_readpos_joint(int joint);

/****************************************************************************
 * Name: claw_readspeed_joint(int joint) 
 * 
 * Description:
 *   Set the claw in to the position
 * param:
 *   int 		joint (claw id) 
 * return: the pos for the claw
 *
 ****************************************************************************/

int claw_readspeed_joint(int joint);

/****************************************************************************
 * Name: claw_readerror_joint(int joint) 
 * 
 * Description:
 *   read the claw error
 * param:
 *   int 		joint (01-02) 
 * return: 0 is OK
 *
 ****************************************************************************/

int claw_readerror_joint(int joint);

#endif //__ROBOT_CLAW_JOINT_H_

