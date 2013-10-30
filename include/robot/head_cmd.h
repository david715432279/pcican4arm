#ifndef CAN_ROBOT_HEAD_CMD_H 
#define CAN_ROBOT_HEAD_CMD_H

#define BROADCAST_ID		0xFF

#define SYS_HEAD_SERVO_1_ID   0x01
#define SYS_HEAD_SERVO_2_ID   0x02


//指令类型宏定义
#define CMDTYPE_RD			0x01		//读控制表指令
#define CMDTYPE_WR			0x02		//写控制表指令
#define CMDTYPE_WR_NR		0x03		//写控制表指令（无返回）

//云台相关参数
#define SYS_HEAD_ID					0x0021      //云台ID
#define SYS_HEAD_MODEL_TYPE         0x0002      //云台型号
#define SYS_HEAD_FW_VERSION         0x0003      //云台固件版本
#define SYS_HEAD_ERROR              0x0004      //云台错误代码
#define SYS_HEAD_VOLTAGE            0x0005      //云台系统电压
#define SYS_HEAD_TEMP               0x0006      //云台系统温度
#define SYS_HEAD_BAUDRATE_CAN       0x0009      //云台can总线波特率
#define SYS_HEAD_ENABLE             0x000a      //云台使能标志
#define SYS_HEAD_SAVE_TO_FLASH      0x000b      //保存数据到flash标志
#define SYS_HEAD_CLEAR_ERROR        0x000c      //保存错误标志
#define SYS_HEAD_SERVO_1_SPEED      0x0010      //舵机1速度
#define SYS_HEAD_SERVO_1_POSITION   0x0011      //舵机1位置
#define SYS_HEAD_SERVO_2_SPEED      0x0012      //舵机2速度
#define SYS_HEAD_SERVO_2_POSITION   0x0013      //舵机2位置
#define SYS_HEAD_SERVO_1_LIT_MAX_LOAD 		0x001a    //舵机1最大负载
#define SYS_HEAD_SERVO_1_LIT_MIN_POSITION 	0x001b    //舵机1最小位置
#define SYS_HEAD_SERVO_1_LIT_MAX_POSITION 	0x001c    //舵机1最大位置
#define SYS_HEAD_SERVO_2_LIT_MAX_LOAD 		0x001d    //舵机2最大负载
#define SYS_HEAD_SERVO_2_LIT_MIN_POSITION 	0x001e    //舵机2最小位置
#define SYS_HEAD_SERVO_2_LIT_MAX_POSITION 	0x001f    //舵机2最大位置

#endif //CAN_ROBOT_HEAD_CMD_H
