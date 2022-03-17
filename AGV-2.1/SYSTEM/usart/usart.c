#include "sys.h"
#include "usart.h"
#include "ARV_System.h"
#include "FreeRTOS.h"					//FreeRTOSʹ��
#include "task.h"
#include "semphr.h"
#include "mpu9250.h"
#include "infrared.h"

extern SemaphoreHandle_t uartDMATCSemaphore;//DMA������ɶ�ֵ�ź������
extern SemaphoreHandle_t uartRxIDLESemaphore;

extern u8 SendBuff[BUFF_SIZE];	//�������ݻ�����
extern u8 ReceiveBuff[BUFF_SIZE];
extern u8 rxbuf[200];
extern u8 rx_cnt;//�������ݸ�����������
u8 imu_reset=0; 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 

/**************************************************************************
�����ⲿ����
��ڲ�������
����  ֵ����
**************************************************************************/
extern SEND_DATA Send_Data;//�������ݵ�
extern RECEIVE_DATA Receive_Data;//�������ݵ�
extern Sensor_Switch Switch_Enable;
extern Error Error_code;
extern volatile uint16_t ultrasonic[3];									
extern volatile u8 Power_State;										
																					     
//=======================================
//����DMA��������
//=======================================
void dma_uart_tx_init()
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
	
	DMA_DeInit(Uart_Tx_DMAStream);//ʹ��----->DMA2_Stream7
	while (DMA_GetCmdStatus(Uart_Tx_DMAStream) != DISABLE){}//�ȴ�DMA������ 

	/* ���� DMA Stream */
	DMA_InitStructure.DMA_Channel            = DMA_Channel_4;              //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;           //Ŀ�ģ�DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)SendBuff;              //Դ��DMA�洢��0��ַ
	DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral; //���򣺴洢��������ģʽ
	//DMA_InitStructure.DMA_BufferSize       = BUFF_SIZE;                   //���ȣ����ݴ�����(�Ȳ�����)
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  //���������ģʽ
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;       //�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;    //�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;            //ʹ����ͨģʽ 
	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;        //DMA���ȼ����е����ȼ�
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;       //FIFOģʽ 
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;     //FIFO��С
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;     //�洢�����δ���
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single; //���赥�δ���
	DMA_Init(Uart_Tx_DMAStream, &DMA_InitStructure);//��ʼ��DMA Stream
	
	//�ж�����
	DMA_ITConfig(Uart_Tx_DMAStream,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;//��ռ���ȼ�8
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����
	DMA_Cmd (Uart_Tx_DMAStream,DISABLE);//�Ȳ�Ҫʹ��DMA��           
}

//=======================================
//����DMA��������
//=======================================
void dma_uart_rx_init()
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
	
	DMA_DeInit(Uart_Rx_DMAStream);//ʹ��----->DMA2_Stream5
	while (DMA_GetCmdStatus(Uart_Rx_DMAStream) != DISABLE){}//�ȴ�DMA������ 

	/* ���� DMA Stream */
	DMA_InitStructure.DMA_Channel            = DMA_Channel_4;              //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;           //Դ��DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)ReceiveBuff;           //Ŀ�ģ�DMA�洢��0��ַ
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory; //�������赽�洢��ģʽ
	DMA_InitStructure.DMA_BufferSize         = BUFF_SIZE;                   //���ȣ����ݴ�����
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  //���������ģʽ
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;       //�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;    //�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;            //ʹ����ͨģʽ 
	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;        //DMA���ȼ����е����ȼ�
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;       //FIFOģʽ 
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;     //FIFO��С
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;     //�洢�����δ���
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single; //���赥�δ���
	DMA_Init(Uart_Rx_DMAStream, &DMA_InitStructure);//��ʼ��DMA Stream
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //ʹ�ܴ���1��DMA����
	DMA_Cmd (Uart_Rx_DMAStream,ENABLE);//ʹ��          
}

//=======================================
//��������
//=======================================
void uart_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��

	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1

	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;             //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	      //�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //���츴�����
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;             //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate            = bound;                         //����������
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;           //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;              //һ��ֹͣλ
	USART_InitStructure.USART_Parity              = USART_Parity_No;               //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1

    //DMA Config
  dma_uart_tx_init();//����DMA��������
	dma_uart_rx_init();//����DMA��������
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7; //��ռ���ȼ�8
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
}

//=======================================
//����DMA���ͺ���
//======================================= 
void uart_DMA_send(u8 *str,u16 ndtr)
{
	u8 i;
	u8 *p=str;
	
	while(xSemaphoreTake(uartDMATCSemaphore,2)!=pdTRUE);//��ȡ�ź������ȴ�DMA���Ϳ���
	
	DMA_Cmd(Uart_Tx_DMAStream, DISABLE);                      //�ر�DMA���� 
	while (DMA_GetCmdStatus(Uart_Tx_DMAStream) != DISABLE){}	//ȷ��DMA���Ա�����  
	DMA_SetCurrDataCounter(Uart_Tx_DMAStream,ndtr);          //���ݴ����� 
	for(i=0;i<ndtr;i++)
	{
		SendBuff[i]=*p++;
	}
	DMA_Cmd(Uart_Tx_DMAStream, ENABLE);                      //����DMA���� 
}

//=======================================
//����1�����жϷ����������DMA����
//=======================================
void USART1_IRQHandler(void)                	
{
	uint8_t data;//���������ݴ����
	BaseType_t xHigherPriorityTaskWoken;
	
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//�����ж�
	{
		data = USART1->SR;
		data = USART1->DR;
		
		DMA_Cmd(Uart_Rx_DMAStream,DISABLE);//�ر�DMA����
		while (DMA_GetCmdStatus(Uart_Rx_DMAStream) != DISABLE){}	//ȷ��DMA���Ա����� 
		rx_cnt = BUFF_SIZE - DMA_GetCurrDataCounter(Uart_Rx_DMAStream);//�õ������������ݸ���  
		DMA_SetCurrDataCounter(Uart_Rx_DMAStream,BUFF_SIZE);//�������ý������ݸ���    
	    //printf("rx_cnt:%d\r\n",rx_cnt);
		memcpy(rxbuf,ReceiveBuff,rx_cnt);//�ȸ��Ƴ�������ֹ�´ε���������֮���串��
	    DMA_ClearFlag(Uart_Rx_DMAStream,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);//����ĸ��ֱ�־��û�㶮
		DMA_Cmd(Uart_Rx_DMAStream,ENABLE); //����DMA����
			
		if(uartRxIDLESemaphore!=NULL)
		{
			//printf("nnnnnnn\r\n");
			//�ͷŶ�ֵ�ź���
			xSemaphoreGiveFromISR(uartRxIDLESemaphore,&xHigherPriorityTaskWoken);//�ͷŴ��ڿ����ж϶�ֵ�ź���
		}
		if( xHigherPriorityTaskWoken == pdTRUE )
		{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//�����Ҫ�Ļ�����һ�������л�
			//portSWITCH_CONTEXT();
		}
		
	}
} 

//=======================================
//DMA��������жϷ������
//=======================================
void DMA2_Stream7_IRQHandler(void)
{
		BaseType_t xHigherPriorityTaskWoken;
    if(DMA_GetITStatus(Uart_Tx_DMAStream,DMA_IT_TCIF7)!= RESET) //���DMA��������ж� DMA_IT_TCIF7
    {
        DMA_ClearITPendingBit(Uart_Tx_DMAStream,DMA_IT_TCIF7); 
				//printf("dma tx ok\r\n");
				if(uartDMATCSemaphore!=NULL)
				{
					//�ͷŶ�ֵ�ź���
					xSemaphoreGiveFromISR(uartDMATCSemaphore,&xHigherPriorityTaskWoken);	//�ͷ�DMA������ɶ�ֵ�ź���
				}
				
				if( xHigherPriorityTaskWoken == pdTRUE )
				{
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//�����Ҫ�Ļ�����һ�������л�
						//portSWITCH_CONTEXT();
				}
    }
}
 
void usart1_send(u8 data)
{
	while((USART1->SR&0X40)==0);//ѭ������,ֱ��������� 
	USART1->DR = data;
}

/**************************************************************************
�������ܣ����ڷ�������
��ڲ�������
����  ֵ����
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
*  �������ܣ�����1�����ж�
*
*  ��ڲ�������
*
*  �� �� ֵ����
**************************************************************************/

int USART_DATA_Handler(u8 *data,u8 cnt)
{	
		memcpy(Receive_Data.buffer,data,RECEIVE_DATA_SIZE);

		if ((Receive_Data.buffer[0]==HEADER_ONE)&&(Receive_Data.buffer[1]==HEADER_TWO)&&(Receive_Data.buffer[2]==HEADER_THREE))	//��֤���ݰ��ĳ���
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
�������ܣ����ڷ��͵Ĵ�����֡���и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void error_transition(void)
{
	
}

/*

  int status;										//С��״̬��0��ʾδ��ʼ����1��ʾ������-1��ʾerror              1 
  float power;             			//��Դ��ѹ��9 13��v                           								51.2
  float theta_imu;         			//��λ�ǣ���0 360����                         								yaw
  int encoder_ppr_imu;     			//����1ת��Ӧ�ı���������                      								4095  12
  int encoder_delta_r;     			//���ֱ����������� ��Ϊ��λ                     							0
  int encoder_delta_l;     			//���ֱ����������� ��Ϊ��λ                     							0
  unsigned int upwoard;    			//1��ʾ����װ,0��ʾ����װ                  								1
  unsigned int hbz_status; 			//��ͣ                            						 							1
  float sonar_distance[4]; 			//����ģ�����ֵ ��λm                         							4m
  float quat[4];          			//IMU��Ԫ��                                    							��Ԫ�� 
  float IMU[9];           			//IMU 9������                                 								 9 
  unsigned int time_stamp_imu;	//ʱ���                                  			 								0 

*/

void data_transition(void)
{
	int i,j;
	u8 *p=NULL;
	
//֡ͷ
	Send_Data.buffer[0] = 0xcd; 
	Send_Data.buffer[1] = 0xeb; 
	Send_Data.buffer[2] = 0xd7;
//����
	Send_Data.buffer[3] = 0x96;
	
//int status 5-8  --> С��ģʽ	
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

//ʱ���	
	p=(u8*)(&Send_Data.time_stamp_imu);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*26+i-1]=*p++;
	Send_Data.time_stamp_imu+=10;
	
	Send_Data.left_sensor2  =  Infrared[0] ;
	Send_Data.right_sensor2 =  Infrared[1];
	
//	printf("Send_Data.current:%f\r\n",Send_Data.current);
//����	
	p=(u8*)(&Send_Data.current);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*27+i-1]=*p++;	
	
//����
	p=(u8*)(&Send_Data.left_sensor2);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*28+i-1]=*p++;

	p=(u8*)(&Send_Data.right_sensor2);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*29+i-1]=*p++;


	p=(u8*)(&Send_Data.distance1);
	for(i=0;i<4;i++)
	Send_Data.buffer[5*30+i-1]=*p++;
	
	
//У��λ  
	 for(j=1;j<31;j++)
   {
       Send_Data.buffer[5*j+3]=32;
	 }
}

/**************************************************************************
�������ܣ����㷢�͵�����У��λ
��ڲ�����
����  ֵ������λ
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	//�������ݵ�У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
		check_sum=check_sum^Send_Data.buffer[k];
	}
	//�������ݵ�У��
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
		check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}











