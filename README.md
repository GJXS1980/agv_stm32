一、通信协议

发送报文：帧头、帧长度、数据段、校验位


1.1 帧头  	三个字节
	Send_Data.buffer[0] = 0xcd; 
	Send_Data.buffer[1] = 0xeb; 
	Send_Data.buffer[2] = 0xd7;

1.2 帧长度 	一个字节


1.3 数据帧
typedef struct _SEND_DATA_  
{
		unsigned char buffer[SEND_DATA_SIZE];

		int status;							//小车状态，0表示未初始化，1表示正常，-1表示error
		float power;             					//电源电压【9 13】v
		float theta_imu;         					//方位角，【0 360）°
		int encoder_ppr_imu;        				//车轮1转对应的编码器个数
		int encoder_delta_r;     					//右轮编码器增量， 个为单位
		int encoder_delta_l;     					//左轮编码器增量， 个为单位
		unsigned int upwoard;    				//1表示正向安装,0表示反向安装
		unsigned int hbz_status; 					//急停按钮
		float sonar_distance[4]; 					//超声模块距离值 单位m
		float quat[4];          					//IMU四元数
		float IMU[9];           					//IMU 9轴数据
		unsigned int time_stamp_imu;				//时间戳
				
		float current;						//电流		
		unsigned int left_sensor2;				//红外									
	  	unsigned int right_sensor2;       			//红外
	  	float distance1;       					//后置超声波
	
}SEND_DATA;														


1.3 校验位计算
	每一个数据段追加字节32的字节

二、接收报文 （同上）
	报文：帧头+长度+数据段+校验位


三、超声波读取
	公用IIC总线，两个超声波地址相同，使用前需要修改其中的超声波地址，有专用的修改IIC地址程序
