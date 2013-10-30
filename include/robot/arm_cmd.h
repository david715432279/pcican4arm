#ifndef CAN_ROBOT_ARM_CMD_H
#define CAN_ROBOT_ARM_CMD_H 

#define BROADCAST_ID		0xFF

//ָ�����ͺ궨��
#define CMDTYPE_RD			0x01		//�����Ʊ�ָ��
#define CMDTYPE_WR			0x02		//д���Ʊ�ָ��
#define CMDTYPE_WR_NR		0x03		//д���Ʊ�ָ��޷��أ�
#define CMDTYPE_WR_REG		0x04		//�첽д�루������
#define CMDTYPE_SCP			0x05		//ʾ�������ݷ���ָ�������
#define CMDTYPE_RST			0x011		//�ѿ��Ʊ�λ�ɳ���ֵ����λ��

//ģ�����ͺ궨��
#define MODEL_TYPE_J60		0x02				
#define MODEL_TYPE_J80		0x03

//ģ����ٱȺ궨��
#define GEAR_RATIO_J60		225
#define GEAR_RATIO_J80		440

//�ڴ���Ʊ�궨��
#define CMDMAP_LEN			128			//�ڴ���Ʊ��ܳ��ȣ����ֵ�λ��
#define CMDMAP_INDLEN		8 			//�ڴ���Ʊ���������
#define CMDMAP_SUBLEN		16 			//�ڴ���Ʊ���������

//������ģʽ����
#define MODE_OPEN			0			//����ģʽ
#define MODE_CURRENT		1			//����ģʽ
#define MODE_SPEED			2			//�ٶ�ģʽ
#define MODE_POSITION		3			//λ��ģʽ

//ϵͳ״̬���
#define SYS_ID					0x01		//������ID
#define SYS_MODEL_TYPE			0x02		//�������ͺ�
#define SYS_FW_VERSION			0x03		//�̼��汾
#define SYS_ERROR				0x04		//�������
#define SYS_VOLTAGE				0x05		//ϵͳ��ѹ
#define SYS_TEMP				0x06		//ϵͳ�¶�
#define SYS_REDU_RATIO			0x07		//ģ����ٱ�
#define SYS_BAUDRATE_232		0x08		//232�˿ڲ�����
#define SYS_BAUDRATE_CAN		0x09		//CAN���߲�����
#define SYS_ENABLE_DRIVER		0x0a		//������ʹ�ܱ�־
#define SYS_ENABLE_ON_POWER		0x0b		//�ϵ�ʹ����������־
#define SYS_SAVE_TO_FLASH		0x0c		//�������ݵ�Flash��־
#define SYS_DEMA_ABSPOS			0x0d		//�Զ��궨����λ�ñ�־
#define SYS_SET_ZERO_POS		0x0e		//����ǰλ������Ϊ����־
#define SYS_CLEAR_ERROR			0x0f		//��������־

#define SYS_CURRENT_L			0x10		//��ǰ������16λ��mA��
#define SYS_CURRENT_H			0x11		//��ǰ������16λ��mA��
#define SYS_SPEED_L				0x12		//��ǰ�ٶȵ�16λ��units/s��
#define SYS_SPEED_H				0x13		//��ǰ�ٶȸ�16λ��units/s��
#define SYS_POSITION_L			0x14		//��ǰλ�õ�16λ��units��
#define SYS_POSITION_H			0x15		//��ǰλ�ø�16λ��units��
#define SYS_POTEN_VALUE			0x16		//���ֵ�λ��ֵ
#define SYS_ZERO_POS_OFFSET_L	0x17		//���λ��ƫ������16λ��units��
#define SYS_ZERO_POS_OFFSET_H	0x18		//���λ��ƫ������16λ��units��

//��������Ϣ
#define MOT_RES					0x20		//�������
#define MOT_INDUC				0x21		//������
#define MOT_RATED_VOL			0x22		//������ѹ
#define MOT_RATED_CUR			0x23		//��������
#define MOT_ENC_LINES			0x400		//��������
#define MOT_HALL_VALUE			0x25		//��ǰ����״̬

//����Ŀ��ֵ
#define TAG_WORK_MODE			0x30		//����ģʽ��0-������1-����ģʽ��2-�ٶ�ģʽ��3-λ��ģʽ
#define TAG_OPEN_PWM			0x31		//����ģʽ��ռ�ձȣ�0~100��
#define TAG_CURRENT_L			0x32		//Ŀ�������16λ��mA��
#define TAG_CURRENT_H			0x33		//Ŀ�������16λ��mA��
#define TAG_SPEED_L				0x34		//Ŀ���ٶȵ�16λ��units/s��
#define TAG_SPEED_H				0x35		//Ŀ���ٶȸ�16λ��units/s��
#define TAG_POSITION_L			0x36		//Ŀ��λ�õ�16λ��units��
#define TAG_POSITION_H			0x37		//Ŀ��λ�ø�16λ��units��

//��������ֵ
#define LIT_MAX_CURRENT			0x40		//��������mA��
#define LIT_MAX_SPEED			0x41		//����ٶȣ�rpm��
#define LIT_MAX_ACC				0x42		//�����ٶȣ�rpm/s��
#define LIT_MIN_POSITION_L		0x43		//��Сλ�õ�16λ��units��
#define LIT_MIN_POSITION_H		0x44		//��Сλ�ø�16λ��units��
#define LIT_MAX_POSITION_L		0x45		//���λ�õ�16λ��units��
#define LIT_MAX_POSITION_H		0x46		//���λ�ø�16λ��units��

//���ջ������
#define SEV_PARAME_LOCKED		0x50		//���ջ�����������־
#define SEV_CURRENT_P			0x51		//������P����
#define SEV_CURRENT_I			0x52		//������I����
#define SEV_CURRENT_D			0x53		//������D����
#define SEV_SPEED_P				0x54		//�ٶȻ�P����
#define SEV_SPEED_I				0x55		//�ٶȻ�I����
#define SEV_SPEED_D				0x56		//�ٶȻ�D����
#define SEV_SPEED_DS			0x57		//�ٶ�P����
#define SEV_POSITION_P			0x58		//λ�û�P����
#define SEV_POSITION_I			0x59		//λ�û�I����
#define SEV_POSITION_D			0x5a		//λ�û�D����
#define SEV_POSITION_DS			0x5b		//λ��P����

//ʾ����ģ����������ַ����
#define SCP_MASK				0x70		//��¼�����־MASK
#define SCP_TRI_SOC				0x71		//����Դ��0Ϊ����������1Ϊ����������2Ϊ�ٶȴ�����3Ϊλ�ô�����4Ϊ�û�����
#define SCP_TRI_MOD				0x72		//������ʽ��0Ϊ�����أ�1Ϊ�½��أ�2Ϊ��������
#define SCP_TRI_FLG				0x73		//�û�������־
#define SCP_REC_TIM				0x74		//��¼ʱ��������10kHZ�ķ�Ƶֵ��
#define SCP_REC_OFS				0x75		//��¼ʱ��ƫ�ã�Ĭ�����źŹ����ʱ�̡�60�����ݣ�
#define SCP_TAGCUR				0x76		//Ŀ��������ݼ�
#define SCP_MEACUR				0x77		//ʵ�ʵ������ݼ�
#define SCP_TAGSPD				0x78		//Ŀ���ٶ����ݼ�
#define SCP_MEASPD				0x79		//ʵ���ٶ����ݼ�
#define SCP_TAGPOS				0x7A		//Ŀ��λ�����ݼ�
#define SCP_MEAPOS				0x7B		//ʵ��λ�����ݼ�

//�����ʺ궨��
#define BAUD_232_19200		0x0000		//19200
#define BAUD_232_115200		0x0001		//115200
#define BAUD_232_500000		0x0002		//500k
#define BAUD_232_1000000	0x0003		//1M
#define BAUD_CAN_250000		0x0000		//250K
#define BAUD_CAN_500000		0x0001		//500K
#define BAUD_CAN_1000000	0x0002		//1M

//ʾ��������ģʽ�궨��
#define TRIGER_UP		0x0000		//������
#define TRIGER_DOWN		0x0001		//�½���
#define TRIGER_USER		0x0002		//�û�
#define TRIGER_OTHER	0x0003		//��������

//ʾ������¼����MASK����
#define MASK_TAGCUR		0x0001		//��¼Ŀ�����MASK
#define MASK_MEACUR		0x0002		//��¼ʵ�ʵ���MASK
#define MASK_TAGSPD		0x0004		//��¼Ŀ���ٶ�MASK
#define MASK_MEASPD		0x0008		//��¼ʵ���ٶ�MASK
#define MASK_TAGPOS		0x0010		//��¼Ŀ��λ��MASK
#define MASK_MEAPOS		0x0020		//��¼ʵ��λ��MASK

//�����ֽ�MASK����
#define ERROR_MASK_OVER_CURRENT		0x0001		//����
#define ERROR_MASK_OVER_VOLTAGE		0x0002		//��ѹ
#define ERROR_MASK_UNDER_VOLTAGE	0x0004		//Ƿѹ
#define ERROR_MASK_OVER_TEMP		0x0008		//����
#define ERROR_MASK_HALL				0x0010		//��������
#define ERROR_MASK_ENCODER			0x0020		//���̴���
#define ERROR_MASK_POTEN			0x0040		//��λ������
#define ERROR_MASK_CURRENT_INIT		0x0080		//����������
#define ERROR_MASK_FUSE				0x0100		//����˿�Ͽ�����

#endif //CAN_ROBOT_ARM_CMD_H

