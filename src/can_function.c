/*
 * can_function for the pci_can drvier controller
 */


#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#include <can4linux.h>
#include <canfunc.h>

/***********************************************************************
 * *
 * * set_bitrate - sets the CAN bit rate
 * *
 * *
 * * Changing these registers only possible in Reset mode.
 * *
 * *
 * */

int set_bitrate(
			int can_fd, /* device descriptor */
			int baud    /* bit rate */
				)
{
	   Config_par_t  cfg;
	   volatile Command_par_t cmd;

	   cmd.cmd = CMD_STOP;
	   ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);

	   cfg.target = CONF_TIMING; 
	   cfg.val1   = baud;
	   ioctl(can_fd, CAN_IOCTL_CONFIG, &cfg);

	   cmd.cmd = CMD_START;
	   ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);
	   return 0;
}
