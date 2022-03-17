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

#define START_TASK_PRIO		2																										//任务优先级
#define START_STK_SIZE 		64  																								//任务堆栈大小	
TaskHandle_t StartTask_Handler;																								//任务句柄
void start_task(void *pvParameters);																					//任务函数

#define USART_TASK_PRIO 5																											//任务优先级
#define USART_STK_SIZE  512 																									//任务堆栈大小	
TaskHandle_t USART_Handler;																										//任务句柄
void USART_task(void *pvParameters);																					//任务函数

#define MPU_TASK_PRIO		6																											//任务优先级
#define MPU_STK_SIZE 		512  																								//任务堆栈大小	
TaskHandle_t MPU_Task_Handler;																								//任务句柄
void mpu9250_task(void *pvParameters);																				//任务函数


#define ULT_TASK_PRIO 3																												//任务优先级
#define ULT_STK_SIZE  150 																										//任务堆栈大小	
TaskHandle_t ULT_Handler;																											//任务句柄
void ULT_task(void *pvParameters);																						//任务函数


#define POWER_TASK_PRIO 4																											//任务优先级
#define POWER_STK_SIZE  256 																									//任务堆栈大小	
TaskHandle_t POWER_Handler;																										//任务句柄
void POWER_task(void *pvParameters);																					//任务函数

xSemaphoreHandle xMutex;
SemaphoreHandle_t uartDMATCSemaphore;																					//DMA发送完成二值信号量句柄
SemaphoreHandle_t uartRxIDLESemaphore;																				//串口空闲中断二值信号量句柄

Sensor_Switch Switch_Enable;
Error Error_code;
MPU_Zero 	mpu_zero;

SEND_DATA Send_Data;																														//发送帧
RECEIVE_DATA Receive_Data;																											//接收帧

u8 SendBuff[BUFF_SIZE];	//DMA发送数据缓冲区
u8 ReceiveBuff[BUFF_SIZE];//DMA接收数据缓冲区
u8 rxbuf[200];
u8 rx_cnt=0;//接收数据个数计数变量

volatile uint16_t ultrasonic[3];	// 超声波     		
volatile u8 Infrared[2]; 	// 红外
float pitch,roll,yaw; 	//	陀螺仪欧拉角
      		
void Parameter_Init(void)
{	
	Send_Data.upwoard = 1;
	Send_Data.encoder_delta_l = 0;	//	左电机编码器
	Send_Data.encoder_delta_r = 0;	//	右电机编码器
	Send_Data.time_stamp_imu = 0;
	Send_Data.encoder_ppr_imu = 4096;	//	每圈的编码值
	
}	

void System_Init(void)
{
		int i=0;
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);															//设置系统中断优先级分组4
		delay_init(168);																														//初始化延时函数
		uart_init(115200);     		//	串口																									//初始化串口
		GPIO_control();				//	GPIO控制
		ULT_IIC_Init();				//	超声波IIC
		delay_ms(100);
		Beep_Init();				// 
		Adc_Init();					//	ADC模块
		URF_Init();					// 
		Parameter_Init();
		delay_ms(100);
		if(MPU_Init())
		{
			Error_code.MPU_9250 = 0x01;	//	IMU器件id错误
		}
		delay_ms(20);						//等待模块初始化完成
		GPIOE->BSRRL = GPIO_Pin_4;
		delay_ms(800);
		GPIOE->BSRRH = GPIO_Pin_4;
		TIM_6_INIT(50000-1, 84-1);
		TIM_7_INIT(50000-1, 84-1);
		IR_Init(20000-1,84-1); //红外，10ms
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
函数功能：陀螺仪零点消除
入口参数：无
返回  值：无
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

// 栈溢出钩子函数，参数xTask和pcTaskName为堆栈溢出任务的句柄和名字
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{

    printf("任务：%s 发现栈溢出\r\n", pcTaskName);

}

int main(void)
{ 
	//初始化各传感器	
	System_Init();

	//创建开始任务（任务优先级为2）
    xTaskCreate((TaskFunction_t )start_task,            											//任务函数
                (const char*    )"start_task",          											//任务名称
                (uint16_t       )START_STK_SIZE,        											//任务堆栈大小
                (void*          )NULL,                  											//传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       											//任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   											//任务句柄              
    vTaskStartScheduler();          //开启任务调度，启动实时内核处理
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
    //创建TASK1 IMU（MPU_9250）任务（任务优先级为6）
    xTaskCreate((TaskFunction_t )mpu9250_task,             
                (const char*    )"mpu9250_task",           
                (uint16_t       )MPU_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )MPU_TASK_PRIO,        
                (TaskHandle_t*  )&MPU_Task_Handler);   

    //创建TASK2 串口传输任务（任务优先级为5）
    xTaskCreate((TaskFunction_t )USART_task,     
                (const char*    )"USART_task",   
                (uint16_t       )USART_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )USART_TASK_PRIO,
                (TaskHandle_t*  )&USART_Handler); 

	//创建TASK3 超声波测距任务（任务优先级为3）
    xTaskCreate((TaskFunction_t )ULT_task,     
                (const char*    )"ULT_task",   
                (uint16_t       )ULT_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )ULT_TASK_PRIO,
                (TaskHandle_t*  )&ULT_Handler); 	
						
	// 创建TASK4 电源、红外接收检测任务（任务优先级为4）
    xTaskCreate((TaskFunction_t )POWER_task,     
                (const char*    )"POWER_task",   
                (uint16_t       )POWER_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )POWER_TASK_PRIO,
                (TaskHandle_t*  )&POWER_Handler); 		
							
		xMutex = xSemaphoreCreateMutex(); // 返回创建好的互斥锁
		uartDMATCSemaphore = xSemaphoreCreateBinary();  //创建二元信号量
		uartRxIDLESemaphore = xSemaphoreCreateBinary();
		xSemaphoreGive(uartDMATCSemaphore);	// 释放uartDMATCSemaphore二元信号量
								
    vTaskDelete(StartTask_Handler); 								//删除开始任务
    taskEXIT_CRITICAL();   //	退出临界段      									//退出临界区
}

//	IMU（MPU_9250）任务
void mpu9250_task(void *pvParameters)
{
	
#if  RTOS_DEBUG		
	uint8_t pcWriteBuffer[500];

#endif	
	while(1)
	{
		u32 lastWakeTime = getSysTickCnt();	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_500_HZ));	//此任务以500Hz的				

		IWDG_ReloadCounter();
		Send_Data.hbz_status = STOP;	
			
//				if(MPU9250_IsDataReady())
//				{
//						MPU_Get_Accelerometer(accel);			//得到加速度传感器数据
//						MPU_Get_Gyroscope(gyro);					//得到陀螺仪数据
//						MPU_Get_Magnetometer(magnet);			//获取磁力计数据			

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
        printf("任务名 	\t\t任务状态 优先级   剩余栈 任务序号\r\n");
        vTaskList((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
            					//printf("\r\n任务名      运行计数        使用率\r\n");
											//vTaskGetRunTimeStats((char *)&pcWriteBuffer);
											//printf("%s\r\n", pcWriteBuffer);

//											printf("=================================================\r\n");
//											printf("roll %.3f pitch %.3f yaw %.3f\r\n",mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
//                      printf("=================================================\r\n");

#endif	
				
	}	
}

//	超声波测距任务
void ULT_task(void *pvParameters)
{
	u8 time = 1;

	while(1)
	{
		u32 lastWakeTime = getSysTickCnt();	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_25_HZ));	//此任务以25Hz的

		Send_Data.current = Get_Power_State(ADC_Channel_15,5)/100.0f;	
			 										//充电状态检测		
		Read_URF();
		Send_Data.distance1 = URF_DIST*10.f; //单位 mm											
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
		Send_Data.sonar_distance[0] =  ultrasonic[0]/100.0f;	//单位 m
		Send_Data.sonar_distance[1] =  ultrasonic[1]/100.0f;
		Send_Data.sonar_distance[2] =  4.0;		
		Send_Data.sonar_distance[3] =  4.0;	
				
	}	
}	

//	电源、红外接收检测任务
void POWER_task(void *pvParameters)
{
	while(1)
	{
		u32 lastWakeTime = getSysTickCnt();	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_2_HZ));																				//RATE_20_HZ
		
		Send_Data.power=Get_Adc_Average(ADC_Channel_13,10);	// 延时一个任务到指定时间																	//电量	
			  
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

//	串口传输任务
void USART_task(void *pvParameters)
{
	BaseType_t err = pdFALSE;
	while(1)
	{	
		u32 lastWakeTime = getSysTickCnt();
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));	//此任务以50Hz的
		
		// portMAX_DELAY会导致任务阻塞时间不确定（不会超时）
		xSemaphoreTake(xMutex, portMAX_DELAY); //	用于获取二元信号量的宏，中断会打断
		err = xSemaphoreTake(uartRxIDLESemaphore,5);	//	如果成功获取信号量则返回pdTRUE，如果xBlockTime超时而信号量还未可用则返回pdFALSE				//获取信号量
		if(err==pdTRUE)			//获取信号量成功
		{  
			USART_DATA_Handler(rxbuf,rx_cnt);
			rx_cnt = 0;
		}
		//printf("Send_Data.distance1:%f\r\n",Send_Data.distance1);	
		//printf("Infrared[0]:%d Infrared[1]:%d \r\n",Infrared[0],	Infrared[1]	);
		data_transition();
		uart_DMA_send(Send_Data.buffer,SEND_DATA_SIZE);				
		xSemaphoreGive(xMutex);	// 释放互斥锁
	}
	
}

