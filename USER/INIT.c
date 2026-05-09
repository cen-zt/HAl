#include "ALL_DEFINE.h"

// SysTick_count 现在由HAL的uwTick提供，在sys.h中通过宏定义映射
_st_Mpu MPU6050;   //MPU6050原始数据
_st_AngE Angle;    //当前角度状态
_st_Remote Remote = {1000,1000,1000,1000,1000,1000,1000,1000}; //遥控器通道值，初始中位

_st_ALL_flag ALL_flag; //系统标志位，各种状态标志

PidObject pidRateX; //角速度PID控制
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //角度PID控制
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

PidObject pidPositionX;
PidObject pidPositionY;

PidObject pidPosRateX;
PidObject pidPosRateY;

void pid_param_Init(void); //PID参数初始化，若要修改PID参数，可以在这里修改，因为已经放到这里了，方便直接改飞控

int16_t motor_PWM_Value[4];

void ALL_Init(void)
{
	USB_HID_Init();   		//USB初始化

	IIC_Init();             //I2C初始化

	pid_param_Init();       //PID参数初始化

	delay_ms(200);
	MpuInit();              //MPU6050初始化
//----------------------------------------
// 水平校准就是掉电保存，只需要校准一次，需要每次开机之前已经校准一次了，校准值自动保存到MCU的FLASH中
// 校准，水平校准 打开飞机，静止5秒作为静止参考，等待时间到就自动写入水平校准
//	delay_ms(5000);MpuGetOffset();
//----------------------------------------
//	USART1_Config(); //串口调试

	NRF24L01_init();				//2.4G遥控器通信初始化

	TIM2_PWM_Config();			//4路PWM初始化
	TIM3_PWM_Config();      //LED PWM初始化

	// 强制初始化为闪烁状态（未解锁）
	LED.status = AllFlashLight;
	LED.FlashTime = 300;

	// TIM1_Config 移到 main.c 最后，所有初始化完成后才启动中断
	// TIM1_Config();					//系统中断调度初始化

}

////PID可以在这里改
void pid_param_Init(void)//PID参数初始化
{
	pidRateX.kp = 3.f;
	pidRateY.kp = 3.f;
	pidRateZ.kp = 6.0f;

//	pidRateX.ki = 0.05f;
//	pidRateY.ki = 0.05f;
//	pidRateZ.ki = 0.02f;

	pidRateX.kd = 0.24f;
	pidRateY.kd = 0.24f;
	pidRateZ.kd = 0.3f;

	// 角速度PID输出限制
	pidRateX.IntegLimitLow = -100.0f;
	pidRateX.IntegLimitHigh = 100.0f;
	pidRateX.OutLimitLow = -300.0f;
	pidRateX.OutLimitHigh = 300.0f;
	pidRateY.IntegLimitLow = -100.0f;
	pidRateY.IntegLimitHigh = 100.0f;
	pidRateY.OutLimitLow = -300.0f;
	pidRateY.OutLimitHigh = 300.0f;
	pidRateZ.IntegLimitLow = -100.0f;
	pidRateZ.IntegLimitHigh = 100.0f;
	pidRateZ.OutLimitLow = -300.0f;
	pidRateZ.OutLimitHigh = 300.0f;

	pidPitch.kp = 10.0f;
	pidRoll.kp = 10.0f;
	pidYaw.kp = 8.0f;

	// 角度PID输出限制
	pidPitch.IntegLimitLow = -50.0f;
	pidPitch.IntegLimitHigh = 50.0f;
	pidPitch.OutLimitLow = -200.0f;
	pidPitch.OutLimitHigh = 200.0f;
	pidRoll.IntegLimitLow = -50.0f;
	pidRoll.IntegLimitHigh = 50.0f;
	pidRoll.OutLimitLow = -200.0f;
	pidRoll.OutLimitHigh = 200.0f;
	pidYaw.IntegLimitLow = -50.0f;
	pidYaw.IntegLimitHigh = 50.0f;
	pidYaw.OutLimitLow = -200.0f;
	pidYaw.OutLimitHigh = 200.0f;

	//高度PID
	//内环（速度环）
	pidHeightRate.kp = 1.2f;
	pidHeightRate.ki = 0.04f;
	pidHeightRate.kd = 0.085f;
	pidHeightRate.IntegLimitLow = -100.0f;
	pidHeightRate.IntegLimitHigh = 100.0f;
	pidHeightRate.OutLimitLow = -200.0f;
	pidHeightRate.OutLimitHigh = 200.0f;
	//外环（位置环）
	pidHeightHigh.kp = 1.2f;
	pidHeightHigh.kd = 0.085f;
	pidHeightHigh.IntegLimitLow = -50.0f;
	pidHeightHigh.IntegLimitHigh = 50.0f;
	pidHeightHigh.OutLimitLow = -300.0f;
	pidHeightHigh.OutLimitHigh = 300.0f;

	//位置PID（光流）
	pidPosRateX.kp = 0.15f;
	pidPosRateY.kp = 0.15f;
	pidPosRateX.kd = 0.035f;
	pidPosRateY.kd = 0.035f;
	pidPosRateX.ki = 0.04f;
	pidPosRateY.ki = 0.04f;

	// 光流速度PID限制
	pidPosRateX.IntegLimitLow = -50.0f;
	pidPosRateX.IntegLimitHigh = 50.0f;
	pidPosRateX.OutLimitLow = -100.0f;
	pidPosRateX.OutLimitHigh = 100.0f;
	pidPosRateY.IntegLimitLow = -50.0f;
	pidPosRateY.IntegLimitHigh = 50.0f;
	pidPosRateY.OutLimitLow = -100.0f;
	pidPosRateY.OutLimitHigh = 100.0f;

	pidPositionX.kp = 0.02f;
	pidPositionY.kp = 0.02f;

	// 光流位置PID限制
	pidPositionX.IntegLimitLow = -50.0f;
	pidPositionX.IntegLimitHigh = 50.0f;
	pidPositionX.OutLimitLow = -100.0f;
	pidPositionX.OutLimitHigh = 100.0f;
	pidPositionY.IntegLimitLow = -50.0f;
	pidPositionY.IntegLimitHigh = 50.0f;
	pidPositionY.OutLimitLow = -100.0f;
	pidPositionY.OutLimitHigh = 100.0f;
}
