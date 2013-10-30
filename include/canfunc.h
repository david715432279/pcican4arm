/**
 * \file canfun.h
 * \author David Huang
 *
 * pci can function
 */

#ifndef __CAN_FUNC_H
#define __CAN_FUNC_H

/***********************************************************************
 * *
 * * set_bitrate - sets the CAN bit rate
 * *
 * *
 * * Changing these registers only possible in Reset mode.
 * *
 * *
 * */

int set_bitrate(int can_fd, int baud);

#endif /*__CAN_FUNC_H*/
