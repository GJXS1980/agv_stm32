#include "sys.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "string.h"
#include "malloc.h"
#include "usart2.h"
#include "imu.h"
#include "task.h"
#include "semphr.h"
#include "ARV_System.h"
#include "TIM_6_7.h"
#include "infrared.h"
#include "urf02.h"
#include "dht11.h"
#include "IIC.h"
#include "infrared.h"
#include "gpio.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

#define 	RTOS_DEBUG  0
volatile  u8 Debug = 1;

#define START_TASK_PRIO		2																										//�������ȼ�
#define START_STK_SIZE 		64  																								//�����ջ��С	
TaskHandle_t StartTask_Handler;																								//������
void start_task(void *pvParameters);																					//������

#define USART_TASK_PRIO 5																											//�������ȼ�
#define USART_STK_SIZE  512 																									//�����ջ��С	
TaskHandle_t USART_Handler;																										//������
void USART_task(void *pvParameters);																					//������

#define MPU_TASK_PRIO		6																											//�������ȼ�
#define MPU_STK_SIZE 		512  																								//�����ջ��С	
TaskHandle_t MPU_Task_Handler;																								//������
void mpu9250_task(void *pvParameters);																				//������


#define ULT_TASK_PRIO 3																												//�������ȼ�
#define ULT_STK_SIZE  150 																										//�����ջ��С	
TaskHandle_t ULT_Handler;																											//������
void ULT_task(void *pvParameters);																						//������


#define POWER_TASK_PRIO 4																											//�������ȼ�
#define POWER_STK_SIZE  256 																									//�����ջ��С	
TaskHandle_t POWER_Handler;																										//������
void POWER_task(void *pvParameters);																					//������

xSemaphoreHandle xMutex;
SemaphoreHandle_t uartDMATCSemaphore;																					//DMA������ɶ�ֵ�ź������
SemaphoreHandle_t uartRxIDLESemaphore;																				//���ڿ����ж϶�ֵ�ź������

Sensor_Switch Switch_Enable;
Error Error_code;
MPU_Zero 	mpu_zero;

SEND_DATA Send_Data;																														//����֡
RECEIVE_DATA Receive_Data;																											//����֡

u8 SendBuff[BUFF_SIZE];	//DMA�������ݻ�����
u8 ReceiveBuff[BUFF_SIZE];//DMA�������ݻ�����
u8 rxbuf[200];
u8 rx_cnt=0;//�������ݸ�����������

volatile uint16_t ultrasonic[3];	// ������     		
volatile u8 Infrared[2]; 	// ����
float pitch,roll,yaw; 	//	������ŷ����
      		
void Parameter_Init(void)
{	
	Send_Data.upwoard = 1;
	Send_Data.encoder_delta_l = 0;	//	����������
	Send_Data.encoder_delta_r = 0;	//	�ҵ��������
	Send_Data.time_stamp_imu = 0;
	Send_Data.encoder_ppr_imu = 4096;	//	ÿȦ�ı���ֵ
	
}	

void System_Init(void)
{
		int i=0;
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);															//����ϵͳ�ж����ȼ�����4
		delay_init(168);																														//��ʼ����ʱ����
		uart_init(115200);     		//	����																									//��ʼ������
		GPIO_control();				//	GPIO����
		ULT_IIC_Init();				//	������IIC
		delay_ms(100);
		Beep_Init();				// 
		Adc_Init();					//	ADCģ��
		URF_Init();					// 
		Parameter_Init();
		delay_ms(100);
		if(MPU_Init())
		{
			Error_code.MPU_9250 = 0x01;	//	IMU����id����
		}
		delay_ms(20);						//�ȴ�ģ���ʼ�����
		GPIOE->BSRRL = GPIO_Pin_4;
		delay_ms(800);
		GPIOE->BSRRH = GPIO_Pin_4;
		TIM_6_INIT(50000-1, 84-1);
		TIM_7_INIT(50000-1, 84-1);
		IR_Init(20000-1,84-1); //���⣬10ms
		while(mpu_dmp_init())
		{
			i++;
			delay_ms(10);
			if(i>50) break;
			
		}
//		INT_IO_T_Init();
		Send_Data.status = 1;
		IWDG_Init(4,2000);
}	

/**************************************************************************
�������ܣ��������������
��ڲ�������
����  ֵ����
**************************************************************************/
void claer_zero(void)		
{
		volatile u8 i,count = 0;
		for(i=50;i>0;i--)
		{
			if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
			{
				count++;	
				mpu_zero.roll_z		+= roll;
				mpu_zero.pitch_z	+= pitch;
				mpu_zero.yaw_z		+= yaw;
				delay_ms(1);
			}	
		}
		mpu_zero.roll_z 	/=count; 
		mpu_zero.pitch_z	/=count;
		mpu_zero.yaw_z		/=count;
}	

// ջ������Ӻ���������xTask��pcTaskNameΪ��ջ�������ľ��������
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{

    printf("����%s ����ջ���\r\n", pcTaskName);

}

int main(void)
{ 
	//��ʼ����������	
	System_Init();

	//������ʼ�����������ȼ�Ϊ2��
    xTaskCreate((TaskFunction_t )start_task,            											//������
                (const char*    )"start_task",          											//��������
                (uint16_t       )START_STK_SIZE,        											//�����ջ��С
                (void*          )NULL,                  											//���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       											//�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   											//������              
    vTaskStartScheduler();          //����������ȣ�����ʵʱ�ں˴���
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	
    //����TASK1 IMU��MPU_9250�������������ȼ�Ϊ6��
    xTaskCreate((TaskFunction_t )mpu9250_task,             
                (const char*    )"mpu9250_task",           
                (uint16_t       )MPU_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )MPU_TASK_PRIO,        
                (TaskHandle_t*  )&MPU_Task_Handler);   

    //����TASK2 ���ڴ��������������ȼ�Ϊ5��
    xTaskCreate((TaskFunction_t )USART_task,     
                (const char*    )"USART_task",   
                (uint16_t       )USART_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )USART_TASK_PRIO,
                (TaskHandle_t*  )&USART_Handler); 

	//����TASK3 ��������������������ȼ�Ϊ3��
    xTaskCreate((TaskFunction_t )ULT_task,     
                (const char*    )"ULT_task",   
                (uint16_t       )ULT_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )ULT_TASK_PRIO,
                (TaskHandle_t*  )&ULT_Handler); 	
						
	// ����TASK4 ��Դ��������ռ�������������ȼ�Ϊ4��
    xTaskCreate((TaskFunction_t )POWER_task,     
                (const char*    )"POWER_task",   
                (uint16_t       )POWER_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )POWER_TASK_PRIO,
                (TaskHandle_t*  )&POWER_Handler); 		
							
		xMutex = xSemaphoreCreateMutex(); // ���ش����õĻ�����
		uartDMATCSemaphore = xSemaphoreCreateBinary();  //������Ԫ�ź���
		uartRxIDLESemaphore = xSemaphoreCreateBinary();
		xSemaphoreGive(uartDMATCSemaphore);	// �ͷ�uartDMATCSemaphore��Ԫ�ź���
								
    vTaskDelete(StartTask_Handler); 								//ɾ����ʼ����
    taskEXIT_CRITICAL();   //	�˳��ٽ��      									//�˳��ٽ���
}

//	IMU��MPU_9250������
void mpu9250_task(void *pvParameters)
{
	
#if  RTOS_DEBUG		
	uint8_t pcWriteBuffer[500];

#endif	
	while(1)
	{
		u32 lastWakeTime = getSysTickCnt();	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_500_HZ));	//��������500Hz��				

		IWDG_ReloadCounter();
		Send_Data.hbz_status = STOP;	
			
//				if(MPU9250_IsDataReady())
//				{
//						MPU_Get_Accelerometer(accel);			//�õ����ٶȴ���������
//						MPU_Get_Gyroscope(gyro);					//�õ�����������
//						MPU_Get_Magnetometer(magnet);			//��ȡ����������			

		if(imu_reset==1)
		{
			Send_Data.status = 0;
			if(mpu_dmp_init()==0)
			{	
				imu_reset = 0;
				Send_Data.status = 1;
			}
			delay_ms(50);
		}
		else
		{	
			if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
			{
				if(yaw<0) 
					yaw = 360.0f + yaw;
				Send_Data.theta_imu = yaw;				
							
				Send_Data.IMU[0] = my_accel[0]/ACCEl_RATIO; 						//x 
				Send_Data.IMU[1] = my_accel[1]/ACCEl_RATIO; 						//y	
				Send_Data.IMU[2] = my_accel[2]*10.0f/ACCEl_RATIO; 			//z
				
				Send_Data.IMU[3] = my_gyro[0]*GYROSCOPE_RATIO;
				Send_Data.IMU[4] = my_gyro[1]*GYROSCOPE_RATIO;
				Send_Data.IMU[5] = my_gyro[2]*GYROSCOPE_RATIO;		
									
				Send_Data.IMU[6] = my_magnet[0]*1.0f;
				Send_Data.IMU[7] = my_magnet[1]*1.0f;
				Send_Data.IMU[8] = my_magnet[2]*1.0f;	
			}			

		}
						
						
#if  RTOS_DEBUG							
        printf("=================================================\r\n");
        printf("������ 	\t\t����״̬ ���ȼ�   ʣ��ջ �������\r\n");
        vTaskList((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
            					//printf("\r\n������      ���м���        ʹ����\r\n");
											//vTaskGetRunTimeStats((char *)&pcWriteBuffer);
											//printf("%s\r\n", pcWriteBuffer);

//											printf("=================================================\r\n");
//											printf("roll %.3f pitch %.3f yaw %.3f\r\n",mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
//                      printf("=================================================\r\n");

#endif	
				
	}	
}

//	�������������
void ULT_task(void *pvParameters)
{
	u8 time = 1;

	while(1)
	{
		u32 lastWakeTime = getSysTickCnt();	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_25_HZ));	//��������25Hz��

		Send_Data.current = Get_Power_State(ADC_Channel_15,5)/100.0f;	
			 										//���״̬���		
		Read_URF();
		Send_Data.distance1 = URF_DIST*10.f; //��λ mm											
		if(time)
		{	
			requestRange(0X41,(uint16_t*)&ultrasonic[0]);
			takeRangeReading(0X40);
		}
		else
		{
			requestRange(0XE1,(uint16_t*)&ultrasonic[1]);
			takeRangeReading(0XE0);	
		}	
				
		time = !time;
		Send_Data.sonar_distance[0] =  ultrasonic[0]/100.0f;	//��λ m
		Send_Data.sonar_distance[1] =  ultrasonic[1]/100.0f;
		Send_Data.sonar_distance[2] =  4.0;		
		Send_Data.sonar_distance[3] =  4.0;	
				
	}	
}	

//	��Դ��������ռ������
void POWER_task(void *pvParameters)
{
	while(1)
	{
		u32 lastWakeTime = getSysTickCnt();	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_2_HZ));																				//RATE_20_HZ
		
		Send_Data.power=Get_Adc_Average(ADC_Channel_13,10);	// ��ʱһ������ָ��ʱ��																	//����	
			  
		Infrared[0] = get_ir_data(0);
		Infrared[1] = get_ir_data(1);
		//printf("Infrared[0]:%d Infrared[1]:%d \r\n",Infrared[0],	Infrared[1]	);
			
		if(Send_Data.current>0.025f)
		{
			GPIOC->BSRRL = GPIO_Pin_6;
			GPIOC->BSRRH = GPIO_Pin_4;				
		}
		else
		{
			GPIOC->BSRRL = GPIO_Pin_4;
			GPIOC->BSRRH = GPIO_Pin_6;			
		}						
	}	
}	

//	���ڴ�������
void USART_task(void *pvParameters)
{
	BaseType_t err = pdFALSE;
	while(1)
	{	
		u32 lastWakeTime = getSysTickCnt();
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));	//��������50Hz��
		
		// portMAX_DELAY�ᵼ����������ʱ�䲻ȷ�������ᳬʱ��
		xSemaphoreTake(xMutex, portMAX_DELAY); //	���ڻ�ȡ��Ԫ�ź����ĺ꣬�жϻ���
		err = xSemaphoreTake(uartRxIDLESemaphore,5);	//	����ɹ���ȡ�ź����򷵻�pdTRUE�����xBlockTime��ʱ���ź�����δ�����򷵻�pdFALSE				//��ȡ�ź���
		if(err==pdTRUE)			//��ȡ�ź����ɹ�
		{  
			USART_DATA_Handler(rxbuf,rx_cnt);
			rx_cnt = 0;
		}
		//printf("Send_Data.distance1:%f\r\n",Send_Data.distance1);	
		//printf("Infrared[0]:%d Infrared[1]:%d \r\n",Infrared[0],	Infrared[1]	);
		data_transition();
		uart_DMA_send(Send_Data.buffer,SEND_DATA_SIZE);				
		xSemaphoreGive(xMutex);	// �ͷŻ�����
	}
	
}

