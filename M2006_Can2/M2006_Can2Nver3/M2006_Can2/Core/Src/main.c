/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "remote_control.h"
#include "stdarg.h"
#include "bsp_rc.h"
#include "bsp_usart.h"
#include "pid.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//获取底盘电机的数据函数
#define get_motor_measure(ptr, data)                                    \
{                                                                   \
		(ptr)->last_ecd = (ptr)->ecd;                                   \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
		(ptr)->temperate = (data)[6];                                   \
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
const RC_ctrl_t *local_rc_ctrl;
static CAN_TxHeaderTypeDef  Tx_message;
 uint8_t Send_data[8];

pid_t pid_speed[4];		   //电机速度PID环
 
/*底盘电机id的结构体*/
 typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//底盘电机返回数据的结构体
typedef struct 
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
}motor_measure_t;

motor_measure_t motor_chassis[7];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//拨弹电机设置电流的函数
void CAN_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
		uint32_t Send_mail_box;
		Tx_message.StdId = CAN_GIMBAL_ALL_ID;
    Tx_message.IDE = CAN_ID_STD;
    Tx_message.RTR = CAN_RTR_DATA;
    Tx_message.DLC = 0x08;
		Send_data[0] = motor1 >> 8;
		Send_data[1] = motor1;
		Send_data[2] = motor2 >> 8;
		Send_data[3] = motor2;
		Send_data[4] = motor3 >> 8;
		Send_data[5] = motor3;
		Send_data[6] = motor4 >> 8;
		Send_data[7] = motor4;
	
	
		HAL_CAN_AddTxMessage(&hcan1, &Tx_message, Send_data, &Send_mail_box);
}

//地盘电机设置电流的函数
void CAN_chassis_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
		uint32_t Send_mail_box;
		Tx_message.StdId = CAN_CHASSIS_ALL_ID;
    Tx_message.IDE = CAN_ID_STD;
    Tx_message.RTR = CAN_RTR_DATA;
    Tx_message.DLC = 0x08;
		Send_data[0] = motor1 >> 8;
		Send_data[1] = motor1;
		Send_data[2] = motor2 >> 8;
		Send_data[3] = motor2;
		Send_data[4] = motor3 >> 8;
		Send_data[5] = motor3;
		Send_data[6] = motor4 >> 8;
		Send_data[7] = motor4;
	
	
		HAL_CAN_AddTxMessage(&hcan1, &Tx_message, Send_data, &Send_mail_box);
}


//接收底盘电机实时数据的函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
	
	switch(rx_header.StdId)
	{
		case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
		case CAN_YAW_MOTOR_ID:
    case CAN_PIT_MOTOR_ID:
    case CAN_TRIGGER_MOTOR_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_3508_M1_ID;
			get_motor_measure(&motor_chassis[i],rx_data);
			break;
		}
		
		default:
		{
			break;
		}
	}
	
	CAN_chassis_current((int16_t)(pid_speed[0].pos_out),
							 (int16_t)(pid_speed[1].pos_out),
							 (int16_t)(pid_speed[2].pos_out),
							 (int16_t)(pid_speed[3].pos_out));
	CAN_current(0, 0, (int16_t)local_rc_ctrl->rc.ch[4] * 1.5, 0);	//将ch[1]换绑为ch[4]
}

//电机滤波器设置
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
   
}

motor_measure_t* get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}



float dma_control(uint16_t i)
{
	float speed;
	speed = (float)local_rc_ctrl->rc.ch[i] / 2;
	return speed;
	
}

float dma_data(uint16_t i)
{
	float speed;
	speed = (float)local_rc_ctrl->rc.ch[i] * 4;
	return speed;
	
}

void fric_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,1500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1500);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
    //__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1000);
}
void fric_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, cmd);
}
//似乎没有使用
double pwm_control_data1()
{
	double date;
	date = 1400 + (double)local_rc_ctrl->rc.ch[2] * 26.0 / 33;
	if(date < 1080)
	{
		date = 1080;
	}
	return date;
	
}
double pwm_control_data2()
{
	double date;
	date = 1450 + (double)local_rc_ctrl->rc.ch[3] * 4.0 / 11;

	return date;
	
}




uint8_t RxData[30]; 
uint8_t TxData[30] = "no picture";
uint8_t rx_up[3];
uint8_t rx_left[3];
uint8_t rx_down[3];
uint8_t rx_right[3];
uint16_t temp_rx[4];


double auto_control_y()
{
	double date;
	date = 1500 + (240 - (double)(temp_rx[0] + temp_rx[2]) / 2) / 2;
	return date;
}

double auto_control_x()
{
	double date;
	date = 1400 + (320 - (double)(temp_rx[1] + temp_rx[3]) / 2) / 2;
	return date;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		temp_rx[0] = 0;
		temp_rx[1] = 0;
		temp_rx[2] = 0;
		temp_rx[3] = 0;

		if(strcmp(RxData, "  0   0   0   0") == 0)
		{
				HAL_UART_Transmit(&huart1, TxData, sizeof(TxData), 10000);
		}
		else{
			HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
			
				HAL_UART_Transmit(&huart1, RxData, sizeof(RxData), 10000);
			for(uint16_t i = 0, j = 100; i < 3; i++)
					{
						rx_up[i] = RxData[i];
						if(rx_up[i] <= '9' && rx_up[i] >= '0')
						{
							temp_rx[0] += (rx_up[i] - 48) * j;
							j /= 10;
						}
						else{
							j /= 10;
							continue;
						}
					}
					for(uint16_t i = 4, j = 100; i < 7; i++)
					{
						rx_left[i - 4] = RxData[i];
						if(rx_left[i - 4] <= '9' && rx_left[i - 4] >= '0')
						{
							temp_rx[1] += (rx_left[i - 4] - 48) * j;
							j /= 10;
						}
						else{
							j /= 10;
							continue;
						}
					}
					for(uint16_t i = 8, j = 100; i < 11; i++)
					{
						rx_down[i - 8] = RxData[i];
						if(rx_down[i - 8] <= '9' && rx_down[i - 8] >= '0')
						{
							temp_rx[2] += (rx_down[i - 8] - 48) * j;
							j /= 10;
						}
						else{
							j /= 10;
							continue;
						}
					}
					for(uint16_t i = 12, j = 100; i < 15; i++)
					{
						rx_right[i - 12] = RxData[i];
						if(rx_right[i - 12] <= '9' && rx_right[i - 12] >= '0')
						{
							temp_rx[3] += (rx_right[i - 12] - 48) * j;
							j /= 10;
						}
						else{
							j /= 10;
							continue;
						}
					}
//					if(temp_rx[0] == 110 && temp_rx[1] == 222)
//					{
//						HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
//						HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
//						HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
//					}
//					else
//					{
//						HAL_UART_Transmit(&huart1, "arror\n", sizeof("arror\n"), 10000);
//					}
					
					
					/*
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1, auto_control_y());
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2, auto_control_y());
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3, auto_control_y());
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4, auto_control_y());


				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, auto_control_x());
					*/
		}
	}
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//pid_t pid_speed[4];		   //电机速度PID环
	float set_speed_temp;			   //加减速时的临时设定速度
	int16_t delta;					   //设定速度与实际速度的差值
	int16_t max_speed_change = 1000;   //电机单次最大变化速度，加减速用
									   // 500经测试差不多是最大加速区间，即从零打到最大速度不异常的最大值
	static float set_speed[4]; //电机速度全局变量
	
	int positionPitchNum = 1460;//云台pitch遥控器解算得值	//第一次定义为初始值searchme
	int statical_positionPitchNum = positionPitchNum;//云台pitch最终赋值
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

	
	HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	fric_off();
	//HAL_Delay(30);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, positionPitchNum);
	HAL_Delay(300);
	
	can_filter_init();
	HAL_CAN_Start(&hcan1);
		remote_control_init();
    usart1_tx_dma_init();
    local_rc_ctrl = get_remote_control_point();
		//set_speed[0] = 	set_speed[1] = 	set_speed[2] = 	set_speed[3] = 	-200;
		
//		while(switch_is_up(local_rc_ctrl->rc.s[0]))
//    {
//        fric_on(2000);
//        if(local_rc_ctrl->rc.ch[3] > 600)
//        {
//            fric_on(1000);
//            break;
//        }
//    }
		
		for (uint16_t i = 0; i < 4; i++)
	{
		PID_struct_init(&pid_speed[i], POSITION_PID, 2000, 2000, 5.0f, 0.1f, 0.0f); //4 motos angular rate closeloop.
	}
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int tempPWMvar1 = 1200;
	int tempPWMvar2 = 1500;
	int tempPWMFlagVar = 1;
	//int positionPitchNum = 1500;
	//int statical_positionPitchNum = 1500;
	
	int basicCountRE = 0;
	int basicCountFlag = 0;
	uint8_t UART1TransmitNum[20] = {64};
	uint8_t UART1TransmitNumTemp[20] = {0};
	float positionPitchNumF = 0;
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
	
	HAL_Delay(200);

  while (1)
  {
	  /*
	  HAL_Delay(1000);
	  tempPWMFlagVar++;
	  if(tempPWMFlagVar >= 3){
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, tempPWMvar1);
		  tempPWMFlagVar = 1;
	  }else{
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, tempPWMvar2);
		  
	  }
	  */
	  
		  
		if(switch_is_down(local_rc_ctrl->rc.s[1]))
		{
			if(local_rc_ctrl->rc.s[0] == 1)
			{
				set_speed[0] = 	set_speed[1] = 	set_speed[2] = 	set_speed[3] = 	dma_data(0);
				set_speed[0] += dma_control(3);
				set_speed[2] += dma_control(3);
			}
			else if(local_rc_ctrl->rc.s[0] == 2)
			{
				//set_speed[0] = 	set_speed[1] = 	set_speed[2] = 	set_speed[3] = dma_data(0);
			set_speed[0] = 	set_speed[3] = dma_data(1);
				set_speed[1] = 	set_speed[2] = -dma_data(3);
			}
			else if(local_rc_ctrl->rc.s[0] == 3)
			{
				set_speed[0] = 	-dma_data(3) + dma_data(2) + dma_data(0);
				set_speed[1] = 	dma_data(3) + dma_data(2) + dma_data(0);
				set_speed[2] = 	dma_data(3) - dma_data(2) + dma_data(0);
				set_speed[3] = 	-dma_data(3) - dma_data(2) + dma_data(0);
			}
				// 无加减速
				//pid_calc(&pid_speed[i], (float)moto_chassis[i].speed_rpm, set_speed[i]);
					
				//加减速
			for (uint16_t i = 0; i < 5; i++)
			{
				delta = (int16_t)set_speed[i] - motor_chassis[i].speed_rpm;
				if (delta > max_speed_change)
					set_speed_temp = (float)(motor_chassis[i].speed_rpm + max_speed_change);
				else if (delta < -max_speed_change)
					set_speed_temp = (float)(motor_chassis[i].speed_rpm - max_speed_change);
				else
					set_speed_temp = set_speed[i];
				pid_calc(&pid_speed[i], (float)motor_chassis[i].speed_rpm, set_speed_temp);
			}
			
			/*	
			//CAN底盘
			CAN_chassis_current((int16_t)(pid_speed[0].pos_out),
							 (int16_t)(pid_speed[1].pos_out),
							 (int16_t)(pid_speed[2].pos_out),
							 (int16_t)(pid_speed[3].pos_out));
				HAL_Delay(2);
			*/
				/*snail电机*/
				fp32 pwm;
				pwm = 1000.0f + local_rc_ctrl->rc.ch[1];
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)(pwm));
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint16_t)(pwm));
			
			
				//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1240);
				//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1240);
//				fric_on((uint16_t)(pwm));
//				
				/*拨弹电机*/
				/*
				
 				CAN_current(0, 0, (int16_t)local_rc_ctrl->rc.ch[4] * 1.5, 0);	//将ch[1]换绑为ch[4]
				HAL_Delay(2);
				*/
				
				/*云台*/
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1, pwm_control_data2());
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2, pwm_control_data2());
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3, pwm_control_data2());
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4, pwm_control_data2());
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1, 1400);
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2, 1400);
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3, 1400);
//				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4, 1400);
//				HAL_Delay(2);


//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1500);
//				HAL_Delay(2);
			//positionPitchNumF = (float)local_rc_ctrl->rc.ch[4];	//遥控器拨盘
			positionPitchNumF = (float)local_rc_ctrl->rc.ch[1];	
			positionPitchNum = positionPitchNumF;
			UART1TransmitNum[1] = 61;//=
			
			
			
			if (positionPitchNum < 0){
				if (positionPitchNum < -600){
					positionPitchNum = -600;
				}
				positionPitchNum = positionPitchNum/1.2;
				//positionPitchNum = 600 + positionPitchNum;
				//positionPitchNum = positionPitchNum * (-1);
				
				positionPitchNum = positionPitchNum + 1500;
				UART1TransmitNum[1] = 45;//-
				
				positionPitchNum = abs(positionPitchNum);
			}else{				
				if (positionPitchNum > 600){
					positionPitchNum = 600;
				}
				positionPitchNum = positionPitchNum/1.2;
				
				positionPitchNum = positionPitchNum + 1500;
				UART1TransmitNum[1] = 43;//+
				positionPitchNum = abs(positionPitchNum) - 40;
			}
			
			if (positionPitchNum > 1900){
				positionPitchNum = 1900;
			}
			else if(positionPitchNum <1100){
				positionPitchNum = 1100;
			}
			
			positionPitchNum = abs(positionPitchNum) ;//云台pitch倒数第二步得数
			statical_positionPitchNum = statical_positionPitchNum + (positionPitchNum-1460) * 0.01 ;//云台最终输出
			
			if (statical_positionPitchNum > 1800){
				statical_positionPitchNum = 1800;
			}
			else if(statical_positionPitchNum <1200){
				statical_positionPitchNum = 1200;
			}

			statical_positionPitchNum = statical_positionPitchNum ;
			
			
			basicCountRE = positionPitchNum;
			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
			for (basicCountFlag = 10;basicCountRE != 0;basicCountFlag--){
				UART1TransmitNum[basicCountFlag] = basicCountRE%10 + 48;
		        basicCountRE = basicCountRE/10 ;
				
			}
			
			if ((float)local_rc_ctrl->rc.ch[4] > 150){
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1240);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1240);
			}else if((float)local_rc_ctrl->rc.ch[4] < -150){
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1240);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1240);
			}else{
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1240);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1240);
			}
			
			UART1TransmitNum[19] = 10;//\n
			//HAL_UART_Transmit(&huart1,UART1TransmitNum,20, HAL_MAX_DELAY);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, statical_positionPitchNum);
			HAL_Delay(5);
			/*
			if (positionPitchNum > 1000){
			}
			else if(positionPitchNum){
			}
			else if(positionPitchNum){
			}
			*/

			}
			else if(switch_is_mid(local_rc_ctrl->rc.s[1]))
			{
				HAL_UART_Receive_IT(&huart1, RxData, 15);
			}
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
