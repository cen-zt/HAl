/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
 /**  
  *首先core是cubemx生成的HAL代码，包括所处的main，USER目录下也有一个main，不过未作使用也不会影响功能。
  * USER是上层的应用层
  * HAL则是涉及到飞控的主要文件，包括PID控制，对应遥控信号的解析，上位机的通信
  * MATH则是涉及到算法方面的东西，比如姿态解算，PID，滤波，和一些数学工具
  * HARDWARE就是一些硬件的驱动方面，比如MPU6050（初始化+数据处理），LED和NRF24L01等
  * config即使对于外设的配置（I2C,SPI,TIM，一些冲突的就在core里面生成了，比如GPIO）
  * 
  * 
  * 上电初始化：HAL_Init → SystemClock_Config → 外设HAL初始化→ USB_HID → I2C → PID参数整定 → MPU6050
  → NRF24L01 → TIM2/3 PWM → LED(闪烁=未解锁)→ 最后启动TIM1 3ms中断
  *
  *
  * 对于时间有着严格要求的闭环任务，比如姿态结算和飞行控制，放在TIM1的3ms中断回调里，确保定时执行
  * 而对于普通的信号收发和LED状态更新等不太严格的任务，放在主循环里轮询执行，避免中断过多导致的卡顿，大约100ms执行一次
  * 
  * 
  * 飞控的核心任务在main的RUN函数里（详见下方HAL_TIM_PeriodElapsedCallback回调），3ms执行一次，
  * 包含：读取MPU6050数据 → 姿态解算 → 遥控信号解析 → PID计算 → 输出PWM控制电机
  * 数据流向：MPU6050六轴 → I2C → 主控 → 互补滤波/四元数 → 串级PID → 混控矩阵 → 电机PWM
  * 具体的（MPU6050 → GetAngle → Angle结构体 → FlightPidControl → MotorControl → PWM输出）
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "ALL_DEFINE.h"
#include "delay.h"
#include "TIM.h"
#include "ADC.h"
#include "Spi.h"
#include "I2C.h"
#include "USB_HID.h"
#include "INIT.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// 声明全局变量（定义在USER/INIT.c）
extern _st_Mpu MPU6050;
extern _st_AngE Angle;

// 提前声明RUN函数
void RUN(void);//飞控主循环，每3ms调用一次，飞控的核心任务

/* USER CODE END PV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // 设置NVIC优先级分组为2bit抢占 + 2bit子优先级，与标准库一致
  // HAL_Init内部已经用默认分组4初始化了SysTick，改分组后需要重新设置SysTick优先级
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();//PWM控制的LED灯
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // 飞控模块初始化
  cycleCounterInit();
  SPI_Config();
  ADC_DMA_Init();
  ALL_Init();  // ALL_Init 内部已经完成: IIC/I2C, TIM, USB_HID 初始化

  // 最后启动TIM1中断，确保所有初始化完成才开始飞控计算
  // 避免中断提前启动访问未初始化的数据导致卡死
  TIM1_Config();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		   ANTO_polling();//数据传输轮询，用于与地面站和上位机通信，发送飞行状态和接收PID参数等
		   PilotLED();//更新LED状态
       Usb_Hid_Send();//与ANTO配套的通信协议包括下面那个
       Usb_Hid_Receive();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// TIM1更新中断回调 - 3ms中断，飞控主处理在这里
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    // 飞控定时处理
    RUN();
  }
}

// 飞控主循环，每3ms调用一次，飞控的核心任务
void RUN(void)
{
    MpuGetData();//读取MPU6050数据
    GetAngle(&MPU6050, &Angle, 0.003f);//陀螺仪+加速度计数据融合计算姿态角，参数0.003f是周期3ms
    RC_Analy();//解析遥控器数据
    FlightPidControl(0.003f);//飞控串级PID计算，参数0.003f是周期3ms
    MotorControl();//输出PWM控制电机
}

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
#ifdef USE_FULL_ASSERT
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
