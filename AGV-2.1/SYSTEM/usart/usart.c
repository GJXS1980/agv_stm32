#include "sys.h"
#include "usart.h"
#include "ARV_System.h"
#include "FreeRTOS.h"					//FreeRTOS使用
#include "task.h"
#include "semphr.h"
#include "mpu9250.h"
#include "infrared.h"

extern SemaphoreHandle_t uartDMATCSemaphore;//DMA发送完成二值信号量句柄
extern SemaphoreHandle_t uartRxIDLESemaphore;

extern u8 SendBuff[BUFF_SIZE];	//发送数据缓冲区
extern u8 ReceiveBuff[BUFF_SIZE];
extern u8 rxbuf[200];
extern u8 rx_cnt;//接收数据个数计数变量
u8 imu_reset=0; 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 

/**************************************************************************
声明外部变量
入口参数：无
返回  值：无
**************************************************************************/
extern SEND_DATA Send_Data;//发送数据的
extern RECEIVE_DATA Receive_Data;//接收数据的
extern Sensor_Switch Switch_Enable;
extern Error Error_code;
extern volatile uint16_t ultrasonic[3];									
extern volatile u8 Power_State;										
																					     
//=======================================
//串口DMA发送配置
//=======================================
void dma_uart_tx_init()
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	
	DMA_DeInit(Uart_Tx_DMAStream);//使用----->DMA2_Stream7
	while (DMA_GetCmdStatus(Uart_Tx_DMAStream) != DISABLE){}//等待DMA可配置 

	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel            = DMA_Channel_4;              //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;           //目的：DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)SendBuff;              //源：DMA存储器0地址
	DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral; //方向：存储器到外设模式
	//DMA_InitStructure.DMA_BufferSize       = BUFF_SIZE;                   //长度：数据传输量(先不配置)
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  //外设非增量模式
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;       //存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;    //存储器数据长度:8位
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;            //使用普通模式 
	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;        //DMA优先级：中等优先级
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;       //FIFO模式 
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;     //FIFO大小
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;     //存储器单次传输
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single; //外设单次传输
	DMA_Init(Uart_Tx_DMAStream, &DMA_InitStructure);//初始化DMA Stream
	
	//中断配置
	DMA_ITConfig(Uart_Tx_DMAStream,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;//抢占优先级8
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送
	DMA_Cmd (Uart_Tx_DMAStream,DISABLE);//先不要使能DMA！           
}

//=======================================
//串口DMA接收配置
//=======================================
void dma_uart_rx_init()
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	
	DMA_DeInit(Uart_Rx_DMAStream);//使用----->DMA2_Stream5
	while (DMA_GetCmdStatus(Uart_Rx_DMAStream) != DISABLE){}//等待DMA可配置 

	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel            = DMA_Channel_4;              //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;           //源：DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)ReceiveBuff;           //目的：DMA存储器0地址
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory; //方向：外设到存储器模式
	DMA_InitStructure.DMA_BufferSize         = BUFF_SIZE;                   //长度：数据传输量
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  //外设非增量模式
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;       //存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;    //存储器数据长度:8位
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;            //使用普通模式 
	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;        //DMA优先级：中等优先级
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;       //FIFO模式 
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;     //FIFO大小
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;     //存储器单次传输
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single; //外设单次传输
	DMA_Init(Uart_Rx_DMAStream, &DMA_InitStructure);//初始化DMA Stream
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //使能串口1的DMA接收
	DMA_Cmd (Uart_Rx_DMAStream,ENABLE);//使能          
}

//=======================================
//串口配置
//=======================================
void uart_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟

	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1

	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;             //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;             //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

	//USART1 初始化设置
	USART_InitStructure.USART_BaudRate            = bound;                         //波特率设置
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;           //字长为8位数据格式
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;              //一个停止位
	USART_InitStructure.USART_Parity              = USART_Parity_No;               //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1

    //DMA Config
  dma_uart_tx_init();//串口DMA发送配置
	dma_uart_rx_init();//串口DMA接收配置
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启串口空闲中断
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7; //抢占优先级8
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
}

//=======================================
//串口DMA发送函数
//======================================= 
void uart_DMA_send(u8 *str,u16 ndtr)
{
	u8 i;
	u8 *p=str;
	
	while(xSemaphoreTake(uartDMATCSemaphore,2)!=pdTRUE);//获取信号量，等待DMA发送可用
	
	DMA_Cmd(Uart_Tx_DMAStream, DISABLE);                      //关闭DMA传输 
	while (DMA_GetCmdStatus(Uart_Tx_DMAStream) != DISABLE){}	//确保DMA可以被设置  
	DMA_SetCurrDataCounter(Uart_Tx_DMAStream,ndtr);          //数据传输量 
	for(i=0;i<ndtr;i++)
	{
		SendBuff[i]=*p++;
	}
	DMA_Cmd(Uart_Tx_DMAStream, ENABLE);                      //开启DMA传输 
}

//=======================================
//串口1空闲中断服务程序，用于DMA接收
//=======================================
void USART1_IRQHandler(void)                	
{
	uint8_t data;//接收数据暂存变量
	BaseType_t xHigherPriorityTaskWoken;
	
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//空闲中断
	{
		data = USART1->SR;
		data = USART1->DR;
		
		DMA_Cmd(Uart_Rx_DMAStream,DISABLE);//关闭DMA接收
		while (DMA_GetCmdStatus(Uart_Rx_DMAStream) != DISABLE){}	//确保DMA可以被设置 
		rx_cnt = BUFF_SIZE - DMA_GetCurrDataCounter(Uart_Rx_DMAStream);//得到真正接收数据个数  
		DMA_SetCurrDataCounter(Uart_Rx_DMAStream,BUFF_SIZE);//重新设置接收数据个数    
	    //printf("rx_cnt:%d\r\n",rx_cnt);
		memcpy(rxbuf,ReceiveBuff,rx_cnt);//先复制出来，防止下次的数据来了之后将其覆盖
	    DMA_ClearFlag(Uart_Rx_DMAStream,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);//这里的各种标志还没搞懂
		DMA_Cmd(Uart_Rx_DMAStream,ENABLE); //开启DMA接收
			
		if(uartRxIDLESemaphore!=NULL)
		{
			//printf("nnnnnnn\r\n");
			//释放二值信号量
			xSemaphoreGiveFromISR(uartRxIDLESemaphore,&xHigherPriorityTaskWoken);//释放串口空闲中断二值信号量
		}
		if( xHigherPriorityTaskWoken == pdTRUE )
		{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
			//portSWITCH_CONTEXT();
		}
		
	}
} 

//=======================================
//DMA发送完成中断服务程序
//=======================================
void DMA2_Stream7_IRQHandler(void)
{
		BaseType_t xHigherPriorityTaskWoken;
    if(DMA_GetITStatus(Uart_Tx_DMAStream,DMA_IT_TCIF7)!= RESET) //检查DMA传输完成中断 DMA_IT_TCIF7
    {
        DMA_ClearITPendingBit(Uart_Tx_DMAStream,DMA_IT_TCIF7); 
				//printf("dma tx ok\r\n");
				if(uartDMATCSemaphore!=NULL)
				{
					//释放二值信号量
					xSemaphoreGiveFromISR(uartDMATCSemaphore,&xHigherPriorityTaskWoken);	//释放DMA传输完成二值信号量
				}
				
				if( xHigherPriorityTaskWoken == pdTRUE )
				{
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
						//portSWITCH_CONTEXT();
				}
    }
}
 
void usart1_send(u8 data)
{
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕 
	USART1->DR = data;
}

/**************************************************************************
函数功能：串口发送数据
入口参数：无
返回  值：无
**************************************************************************/

void USART1_SEND(int len)
{
  unsigned char i = 0;	
	
	for(i=0; i<len; i++)
	{
		usart1_send(Send_Data.buffer[i]);
		//printf("%02X ",Send_Data.buffer[i]);
	}	 
	//printf("\r\n");
}

/**************************************************************************
*  函数功能：串口1接收中断
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/

int USART_DATA_Handler(u8 *data,u8 cnt)
{	
		memcpy(Receive_Data.buffer,data,RECEIVE_DATA_SIZE);

		if ((Receive_Data.buffer[0]==HEADER_ONE)&&(Receive_Data.buffer[1]==HEADER_TWO)&&(Receive_Data.buffer[2]==HEADER_THREE))	//验证数据包的长度
		{  
				if(Receive_Data.buffer[3]==0x01)
				{
					switch(Receive_Data.buffer[4])
					{
						case 0x31:	NVIC_SystemReset();	break;
						case 0x29:  break;
						case 0x43:  imu_reset = 1; break;				
						default : break;			
					}
				
				}else if(Receive_Data.buffer[3]==0x06)
				{
					
				}	
		 }
		
		return 0;	
}

/**************************************************************************
函数功能：串口发送的错误码帧进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void error_transition(void)
{
	
}

/*

  int status;										//小车状态，0表示未初始化，1表示正常，-1表示error              1 
  float power;             			//电源电压【9 13】v                           								51.2
  float theta_imu;         			//方位角，【0 360）°                         								yaw
  int encoder_ppr_imu;     			//车轮1转对应的编码器个数                      								4095  12
  int encoder_delta_r;     			//右轮编码器增量， 个为单位                     							0
  int encoder_delta_l;     			//左轮编码器增量， 个为单位                     							0
  unsigned int upwoard;    			//1表示正向安装,0表示反向安装                  								1
  unsigned int hbz_status; 			//急停                            						 							1
  float sonar_distance[4]; 			//超声模块距离值 单位m                         							4m
  float quat[4];          			//IMU四元数                                    							四元数 
  float IMU[9];           			//IMU 9轴数据                                 								 9 
  unsigned int time_stamp_imu;	//时间戳                                  			 								0 

*/

void data_transition(void)
{
	int i,j;
	u8 *p=NULL;
	
//帧头
	Send_Data.buffer[0] = 0xcd; 
	Send_Data.buffer[1] = 0xeb; 
	Send_Data.buffer[2] = 0xd7;
//长度
	Send_Data.buffer[3] = 0x96;
	
//int status 5-8  --> 小端模式	
	 p=(u8*)(&Send_Data.status);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[4+i]=*p++;

//float power 10-13 
	 p=(u8*)(&Send_Data.power);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[9+i]=*p++;
	
//float theta_imu 	
	 p=(u8*)(&Send_Data.theta_imu);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[14+i]=*p++;	
	
//int encoder_ppr_imu
	 p=(u8*)(&Send_Data.encoder_ppr_imu);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[19+i]=*p++;		 
	
//int encoder_delta_r
	 p=(u8*)(&Send_Data.encoder_delta_r);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[24+i]=*p++;	 
	 
//int encoder_delta_l;  
	 p=(u8*)(&Send_Data.encoder_delta_l);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[29+i]=*p++;			 

//unsigned int upwoard; 
	 p=(u8*)(&Send_Data.upwoard);	
	 for(i=0;i<4;i++)
	 Send_Data.buffer[34+i]=*p++;	 	
	 
//unsigned int hbz_status}
	 p=(u8*)(&Send_Data.hbz_status);	
	 for(i=0;i<4;i++)
	 Send_Data.buffer[39+i]=*p++;	
	 
//float sonar_distance[4]
	 p=(u8*)(&Send_Data.sonar_distance[0]);	
	 for(i=0;i<4;i++)
	 Send_Data.buffer[44+i]=*p++;

	 p=(u8*)(&Send_Data.sonar_distance[1]);	
	 for(i=0;i<4;i++)
	 Send_Data.buffer[49+i]=*p++;
	 
	 p=(u8*)(&Send_Data.sonar_distance[2]);	
	 for(i=0;i<4;i++)
	 Send_Data.buffer[54+i]=*p++;

	 p=(u8*)(&Send_Data.sonar_distance[3]);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[59+i]=*p++;
	 
//float quat[4];
	 p=(u8*)(&Send_Data.quat[0]);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[64+i]=*p++;

	 p=(u8*)(&Send_Data.quat[1]);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[69+i]=*p++;
	 
	 p=(u8*)(&Send_Data.quat[2]);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[74+i]=*p++;
	 
	 p=(u8*)(&Send_Data.quat[3]);
	 for(i=0;i<4;i++)
	 Send_Data.buffer[79+i]=*p++;
	 
	for(j=0;j<9;j++)
	{	
		p=(u8*)(&Send_Data.IMU[j]);	
		for(i=0;i<4;i++)
			Send_Data.buffer[5*(17+j)+i -1]=*p++;
	}
	
	
#if 0
	printf("--------------------------------\r\n");
	printf("accel:%f %f %f \r\n",Send_Data.IMU[0],Send_Data.IMU[1],Send_Data.IMU[2]);
	printf("gyro:%f %f %f \r\n",Send_Data.IMU[3],Send_Data.IMU[4],Send_Data.IMU[5]);
	printf("magnet:%f %f %f \r\n",Send_Data.IMU[6],Send_Data.IMU[7],Send_Data.IMU[8]);
	printf("quat:%f %f %f %f\r\n",Send_Data.quat[0],Send_Data.quat[1],Send_Data.quat[2],Send_Data.quat[3]);	
	printf("theta_imu:%f\r\n",Send_Data.theta_imu);
#endif	

//时间戳	
	p=(u8*)(&Send_Data.time_stamp_imu);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*26+i-1]=*p++;
	Send_Data.time_stamp_imu+=10;
	
	Send_Data.left_sensor2  =  Infrared[0] ;
	Send_Data.right_sensor2 =  Infrared[1];
	
//	printf("Send_Data.current:%f\r\n",Send_Data.current);
//电流	
	p=(u8*)(&Send_Data.current);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*27+i-1]=*p++;	
	
//红外
	p=(u8*)(&Send_Data.left_sensor2);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*28+i-1]=*p++;

	p=(u8*)(&Send_Data.right_sensor2);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*29+i-1]=*p++;


	p=(u8*)(&Send_Data.distance1);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*30+i-1]=*p++;
	
	
//校验位  
	 for(j=1;j<31;j++)
   {
       Send_Data.buffer[5*j+3]=32;
	 }
}

/**************************************************************************
函数功能：计算发送的数据校验位
入口参数：
返回  值：检验位
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	//发送数据的校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
		check_sum=check_sum^Send_Data.buffer[k];
	}
	//接收数据的校验
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
		check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}











