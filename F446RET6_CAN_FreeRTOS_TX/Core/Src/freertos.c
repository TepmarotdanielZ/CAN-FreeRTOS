/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bno055_stm32.h"
#include "bno055.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


	/* IMU BNO 055 */

		/* EULER */

			bno055_vector_t E;

		/* QUATERNION */

			bno055_vector_t Q;

			uint16_t heading = 0;
			uint16_t pitch   = 0;
			uint16_t roll    = 0;
			uint16_t y =0;
			uint16_t QuaX = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for IMU_BNO_055 */
osThreadId_t IMU_BNO_055Handle;
const osThreadAttr_t IMU_BNO_055_attributes = {
  .name = "IMU_BNO_055",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for RotaryEncoder */
osThreadId_t RotaryEncoderHandle;
const osThreadAttr_t RotaryEncoder_attributes = {
  .name = "RotaryEncoder",
  .stack_size = 521 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for LASER */
osThreadId_t LASERHandle;
const osThreadAttr_t LASER_attributes = {
  .name = "LASER",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* MAP */

	float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output) {
		return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
	}

/* USER CODE END FunctionPrototypes */

void Task_IMU_BNO_055(void *argument);
void Task_RotaryEncoder(void *argument);
void Task_Laser(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of IMU_BNO_055 */
  IMU_BNO_055Handle = osThreadNew(Task_IMU_BNO_055, NULL, &IMU_BNO_055_attributes);

  /* creation of RotaryEncoder */
  RotaryEncoderHandle = osThreadNew(Task_RotaryEncoder, NULL, &RotaryEncoder_attributes);

  /* creation of LASER */
  LASERHandle = osThreadNew(Task_Laser, NULL, &LASER_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task_IMU_BNO_055 */
/**
  * @brief  Function implementing the IMU_BNO_055 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_IMU_BNO_055 */
void Task_IMU_BNO_055(void *argument)
{
  /* USER CODE BEGIN Task_IMU_BNO_055 */
  /* Infinite loop */

	/* EULER */

//		uint16_t heading = 0;
//		uint16_t pitch   = 0;
//		uint16_t roll    = 0;
//		uint16_t y =0;
//		uint16_t QuaX = 0;


   	/* IMU BNO 055 */

		  bno055_assignI2C(&hi2c1);
		  bno055_setup();
		  bno055_setOperationModeNDOF();

  for(;;)
  {
	  /* IMU BNO 055 */



			  if (E.y>0)
			  {
				  y = E.y;
			  }
			  else if (E.y<0)
			  {
				  y = 360-(-1*E.y);
			  }

		  /* EULER */

			  heading = map (E.x, 0, 360, 0, 65535);
			  pitch   = map (E.y, 0, 360, 0, 65535);
			  roll    = map (E.z, 0, 360, 0, 65535);

		  /* QUATERNION */

			  QuaX    = map (Q.x, 0, 360, 0, 65535);


		   /* EULER */

			  E	  = bno055_getVectorEuler();
					HAL_Delay(5);

		    /* QUATERNION */

			  Q   = bno055_getVectorQuaternion();
				    HAL_Delay(5);

    osDelay(1);
  }
  /* USER CODE END Task_IMU_BNO_055 */
}

/* USER CODE BEGIN Header_Task_RotaryEncoder */
/**
* @brief Function implementing the RotaryEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_RotaryEncoder */
void Task_RotaryEncoder(void *argument)
{
  /* USER CODE BEGIN Task_RotaryEncoder */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_RotaryEncoder */
}

/* USER CODE BEGIN Header_Task_Laser */
/**
* @brief Function implementing the LASER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Laser */
void Task_Laser(void *argument)
{
  /* USER CODE BEGIN Task_Laser */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Laser */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

