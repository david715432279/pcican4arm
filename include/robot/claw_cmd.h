#ifndef CAN_ROBOT_CLAW_CMD_H 
#define CAN_ROBOT_CLAW_CMD_H

#define BROADCAST_ID		0xFF

#define SYS_CLAW_LEFT_ID    0x07
#define SYS_CLAW_RIGHT_ID   0x17


//指令类型宏定义
#define CMDTYPE_RD			0x01		//读控制表指令
#define CMDTYPE_WR			0x02		//写控制表指令
#define CMDTYPE_WR_NR		0x03		//写控制表指令（无返回）

//手爪相关参数
#define SYS_CLAW_MODEL_TYPE         0x0002      //手爪型号
#define SYS_CLAW_FW_VERSION         0x0003      //手爪固件版本
#define SYS_CLAW_ERROR              0x0004      //手爪错误代码
#define SYS_CLAW_VOLTAGE            0x0005      //手爪系统电压
#define SYS_CLAW_TEMP               0x0006      //手爪系统温度
#define SYS_CLAW_BAUDRATE_CAN       0x0009      //手爪can总线波特率
#define SYS_CLAW_ENABLE             0x000a      //手爪使能标志
#define SYS_CLAW_SAVE_TO_FLASH      0x000b      //保存数据到flash标志
#define SYS_CLAW_CLEAR_ERROR        0x000c      //保存错误标志
#define SYS_CLAW_SPEED      		0x0010      //手爪速度
#define SYS_CLAW_POSITION   		0x0011      //手爪位置
#define SYS_CLAW_LIT_MAX_LOAD 		0x001d    //手爪最大负载
#define SYS_CLAW_MIN_POSITION 		0x001e    //手爪最小位置
#define SYS_CLAW_MAX_POSITION 		0x001f    //手爪最大位置

#endif //CAN_ROBOT_CLAW_CMD_H
