#ifndef CAN_ROBOT_ARM_CMD_H
#define CAN_ROBOT_ARM_CMD_H 

#define BROADCAST_ID		0xFF

//指令类型宏定义
#define CMDTYPE_RD			0x01		//读控制表指令
#define CMDTYPE_WR			0x02		//写控制表指令
#define CMDTYPE_WR_NR		0x03		//写控制表指令（无返回）
#define CMDTYPE_WR_REG		0x04		//异步写入（保留）
#define CMDTYPE_SCP			0x05		//示波器数据返回指令（保留）
#define CMDTYPE_RST			0x011		//把控制表复位成出厂值（复位）

//模块类型宏定义
#define MODEL_TYPE_J60		0x02				
#define MODEL_TYPE_J80		0x03

//模块减速比宏定义
#define GEAR_RATIO_J60		225
#define GEAR_RATIO_J80		440

//内存控制表宏定义
#define CMDMAP_LEN			128			//内存控制表总长度（半字单位）
#define CMDMAP_INDLEN		8 			//内存控制表主索引数
#define CMDMAP_SUBLEN		16 			//内存控制表子索引数

//驱动器模式定义
#define MODE_OPEN			0			//开环模式
#define MODE_CURRENT		1			//电流模式
#define MODE_SPEED			2			//速度模式
#define MODE_POSITION		3			//位置模式

//系统状态相关
#define SYS_ID					0x01		//驱动器ID
#define SYS_MODEL_TYPE			0x02		//驱动器型号
#define SYS_FW_VERSION			0x03		//固件版本
#define SYS_ERROR				0x04		//错误代码
#define SYS_VOLTAGE				0x05		//系统电压
#define SYS_TEMP				0x06		//系统温度
#define SYS_REDU_RATIO			0x07		//模块减速比
#define SYS_BAUDRATE_232		0x08		//232端口波特率
#define SYS_BAUDRATE_CAN		0x09		//CAN总线波特率
#define SYS_ENABLE_DRIVER		0x0a		//驱动器使能标志
#define SYS_ENABLE_ON_POWER		0x0b		//上电使能驱动器标志
#define SYS_SAVE_TO_FLASH		0x0c		//保存数据到Flash标志
#define SYS_DEMA_ABSPOS			0x0d		//自动标定绝对位置标志
#define SYS_SET_ZERO_POS		0x0e		//将当前位置设置为零点标志
#define SYS_CLEAR_ERROR			0x0f		//清除错误标志

#define SYS_CURRENT_L			0x10		//当前电流低16位（mA）
#define SYS_CURRENT_H			0x11		//当前电流高16位（mA）
#define SYS_SPEED_L				0x12		//当前速度低16位（units/s）
#define SYS_SPEED_H				0x13		//当前速度高16位（units/s）
#define SYS_POSITION_L			0x14		//当前位置低16位（units）
#define SYS_POSITION_H			0x15		//当前位置高16位（units）
#define SYS_POTEN_VALUE			0x16		//数字电位器值
#define SYS_ZERO_POS_OFFSET_L	0x17		//零点位置偏移量低16位（units）
#define SYS_ZERO_POS_OFFSET_H	0x18		//零点位置偏移量高16位（units）

//电机相关信息
#define MOT_RES					0x20		//电机内阻
#define MOT_INDUC				0x21		//电机电感
#define MOT_RATED_VOL			0x22		//电机额定电压
#define MOT_RATED_CUR			0x23		//电机额定电流
#define MOT_ENC_LINES			0x400		//码盘线数
#define MOT_HALL_VALUE			0x25		//当前霍尔状态

//控制目标值
#define TAG_WORK_MODE			0x30		//工作模式，0-开环，1-电流模式，2-速度模式，3-位置模式
#define TAG_OPEN_PWM			0x31		//开环模式下占空比（0~100）
#define TAG_CURRENT_L			0x32		//目标电流低16位（mA）
#define TAG_CURRENT_H			0x33		//目标电流高16位（mA）
#define TAG_SPEED_L				0x34		//目标速度低16位（units/s）
#define TAG_SPEED_H				0x35		//目标速度高16位（units/s）
#define TAG_POSITION_L			0x36		//目标位置低16位（units）
#define TAG_POSITION_H			0x37		//目标位置高16位（units）

//控制限制值
#define LIT_MAX_CURRENT			0x40		//最大电流（mA）
#define LIT_MAX_SPEED			0x41		//最大速度（rpm）
#define LIT_MAX_ACC				0x42		//最大加速度（rpm/s）
#define LIT_MIN_POSITION_L		0x43		//最小位置低16位（units）
#define LIT_MIN_POSITION_H		0x44		//最小位置高16位（units）
#define LIT_MAX_POSITION_L		0x45		//最大位置低16位（units）
#define LIT_MAX_POSITION_H		0x46		//最大位置高16位（units）

//三闭环环相关
#define SEV_PARAME_LOCKED		0x50		//三闭环参数锁定标志
#define SEV_CURRENT_P			0x51		//电流环P参数
#define SEV_CURRENT_I			0x52		//电流环I参数
#define SEV_CURRENT_D			0x53		//电流环D参数
#define SEV_SPEED_P				0x54		//速度环P参数
#define SEV_SPEED_I				0x55		//速度环I参数
#define SEV_SPEED_D				0x56		//速度环D参数
#define SEV_SPEED_DS			0x57		//速度P死区
#define SEV_POSITION_P			0x58		//位置环P参数
#define SEV_POSITION_I			0x59		//位置环I参数
#define SEV_POSITION_D			0x5a		//位置环D参数
#define SEV_POSITION_DS			0x5b		//位置P死区

//示波器模块子索引地址定义
#define SCP_MASK				0x70		//记录对象标志MASK
#define SCP_TRI_SOC				0x71		//触发源，0为开环触发，1为电流触发，2为速度触发，3为位置触发，4为用户触发
#define SCP_TRI_MOD				0x72		//触发方式，0为上升沿，1为下降沿，2为连续采样
#define SCP_TRI_FLG				0x73		//用户触发标志
#define SCP_REC_TIM				0x74		//记录时间间隔（对10kHZ的分频值）
#define SCP_REC_OFS				0x75		//记录时间偏置（默认以信号过零点时刻±60次数据）
#define SCP_TAGCUR				0x76		//目标电流数据集
#define SCP_MEACUR				0x77		//实际电流数据集
#define SCP_TAGSPD				0x78		//目标速度数据集
#define SCP_MEASPD				0x79		//实际速度数据集
#define SCP_TAGPOS				0x7A		//目标位置数据集
#define SCP_MEAPOS				0x7B		//实际位置数据集

//波特率宏定义
#define BAUD_232_19200		0x0000		//19200
#define BAUD_232_115200		0x0001		//115200
#define BAUD_232_500000		0x0002		//500k
#define BAUD_232_1000000	0x0003		//1M
#define BAUD_CAN_250000		0x0000		//250K
#define BAUD_CAN_500000		0x0001		//500K
#define BAUD_CAN_1000000	0x0002		//1M

//示波器触发模式宏定义
#define TRIGER_UP		0x0000		//上升沿
#define TRIGER_DOWN		0x0001		//下降沿
#define TRIGER_USER		0x0002		//用户
#define TRIGER_OTHER	0x0003		//连续采样

//示波器记录对象MASK定义
#define MASK_TAGCUR		0x0001		//记录目标电流MASK
#define MASK_MEACUR		0x0002		//记录实际电流MASK
#define MASK_TAGSPD		0x0004		//记录目标速度MASK
#define MASK_MEASPD		0x0008		//记录实际速度MASK
#define MASK_TAGPOS		0x0010		//记录目标位置MASK
#define MASK_MEAPOS		0x0020		//记录实际位置MASK

//错误字节MASK定义
#define ERROR_MASK_OVER_CURRENT		0x0001		//过流
#define ERROR_MASK_OVER_VOLTAGE		0x0002		//过压
#define ERROR_MASK_UNDER_VOLTAGE	0x0004		//欠压
#define ERROR_MASK_OVER_TEMP		0x0008		//过温
#define ERROR_MASK_HALL				0x0010		//霍尔错误
#define ERROR_MASK_ENCODER			0x0020		//码盘错误
#define ERROR_MASK_POTEN			0x0040		//电位器错误
#define ERROR_MASK_CURRENT_INIT		0x0080		//电流检测错误
#define ERROR_MASK_FUSE				0x0100		//保险丝断开错误

#endif //CAN_ROBOT_ARM_CMD_H

