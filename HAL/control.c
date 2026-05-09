/*******************************************************************
 *MPU6050
 *@brief
 *@brief
 ******************************************************************/
#include "ALL_DATA.h"
#include "ALL_DEFINE.h"
#include "control.h"
#include "pid.h"
#include "flow.h"
#include "kalman.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE
#define DISABLE 0
#undef ENABLE
#define ENABLE 1
#undef REST
#define REST 0
#undef SET
#define SET 1
#undef EMERGENT
#define EMERGENT 0
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw
			,&pidHeightRate
			,&pidHeightHigh
			,&pidPosRateX
			,&pidPosRateY
			,&pidPositionX
			,&pidPositionY
};

float sins_high = 0;
float sins_vel = 0;

uint16_t thr_hold = 0;
/**************************************************************
 *  Height control
 * @param[in]
 * @param[out]
 * @return
 ***************************************************************/
float last_high = 0;
float last_vel = 0;
uint32_t VL53L01_high = 0;
static uint8_t high_error_count;
uint8_t set_high = 0;
void HeightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;
	int16_t acc;
	int16_t acc_error;
	static int16_t acc_offset = 0;

	{
		acc = (int16_t)GetNormAccz();
		if(!ALL_flag.unlock)
		{
			acc_offset = acc;
		}
		acc_error = acc - acc_offset;

		if(VL53L01_high < 3500 && VL53L01_high > 20)  // 有效测距范围
		{
			if(VL53L01_high - sins_high > 50)
				sins_high += 50;
			else if(VL53L01_high - sins_high < -50)
				sins_high -= 50;
			else
				sins_high = VL53L01_high;

			sins_vel = (last_vel + acc_error * dt) * 0.985f + 0.015f * (sins_high - last_high) / dt;

			pidHeightRate.measured = last_vel = sins_vel;
			pidHeightHigh.measured = last_high = sins_high;
		}
		else
		{
			// 超出测距范围时，只用加速度计推算（会有漂移）
			sins_vel = last_vel + acc_error * dt;
			sins_high = last_high + sins_vel * dt;
			pidHeightRate.measured = last_vel = sins_vel;
			pidHeightHigh.measured = last_high = sins_high;
		}
	}
	//----------------------------------------------
	if(ALL_flag.unlock == EMERGENT)
		status = EXIT_255;
	//----------------------------------------------
	switch(status)
	{
		case WAITING_1:
		  if( ALL_flag.unlock)
			{
				pidHeightRate.measured=0;
				sins_vel = 0;
				last_vel = 0;
				sins_high = VL53L01_high;  // 直接用当前高度初始化
				last_high = sins_high;
				pidHeightHigh.measured = last_high;
				status = WAITING_2;
				high_error_count=0;
			}
			break;
		case WAITING_2:
			if(ALL_flag.height_lock)
			{
				set_high=0;
				LED.status = WARNING;
				thr_hold = 100 + 0.55f * (Remote.thr - 1000);  // 用当前油门值
				pidRest(&pPidObject[6], 2);  // 进入定高前先清零高度PID积分！
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			 if(Remote.thr<1750 && Remote.thr>1150)
			 {
					if(set_high == 0)
					{
						set_high = 1;
						pidHeightHigh.desired = pidHeightHigh.measured;
					}
					pidUpdate(&pidHeightHigh,dt);
					pidHeightRate.desired = pidHeightHigh.out;
			 }
			else if(Remote.thr>1750)
			{
				if(VL53L01_high<3500)
				{
					set_high = 0;
					pidHeightRate.desired = 250;
				}
			}
			else if	(Remote.thr<1150)
			{
				set_high = 0;
				pidHeightRate.desired = -350;
				if(pidHeightHigh.measured<10)
				{
					ALL_flag.unlock = 0;
				}
			}
				pidUpdate(&pidHeightRate,dt);
				if(!ALL_flag.height_lock)
			{
				LED.status = AlwaysOn ;
				status = EXIT_255;
			}
			if(VL53L01_high<50||VL53L01_high>3700)
			{
				high_error_count++;
				if(high_error_count>50)
				{
					ALL_flag.height_lock=0;
					ALL_flag.flow_control=0;
					status = EXIT_255;
				}
			}
			else
			{
				high_error_count=0;
			}
			break;
		case EXIT_255:
			pidRest(&pPidObject[6],2);
			status = WAITING_1;
			break;
		default:
			status = WAITING_1;
			break;
	}
}
/**************************************************************
 *  flow control
 * @param[in]
 * @param[out]
 * @return
 ***************************************************************/
void FlowPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;
	static uint8_t set_pos = 0;

	if(ALL_flag.unlock == EMERGENT)
		status = EXIT_255;

	switch(status)
	{
		case WAITING_1:
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
			break;
		case WAITING_2:
			if(ALL_flag.flow_control&&VL53L01_high>200)
			{
				pidRest(&pPidObject[8],4);
				status = PROCESS_31;
				set_pos = 1;
			}
			break;
		case PROCESS_31:
			{
				if(Remote.roll>1750||Remote.roll<1250||Remote.pitch>1750||Remote.pitch<1250)
				{
					set_pos = 1;
					if(Remote.roll>1750)
						pidPosRateX.desired = -15;
					else if(Remote.roll<1250)
						pidPosRateX.desired = 15;
					if(Remote.pitch>1750)
						pidPosRateY.desired = 15;
					else if(Remote.pitch<1250)
						pidPosRateY.desired = -15;
				}
				else
				{
					if(set_pos == 1)
					{
						set_pos = 0;
						pidPositionX.desired = flow_x_lpf_att_i;
						pidPositionY.desired = flow_y_lpf_att_i;
					}
					pidPositionX.measured = flow_x_lpf_att_i;
					pidUpdate(&pidPositionX,dt);
					pidPositionY.measured = flow_y_lpf_att_i;
					pidUpdate(&pidPositionY,dt);
					pidPosRateX.desired = LIMIT(pidPositionX.out,-20,20);
					pidPosRateY.desired = LIMIT(pidPositionY.out,-20,20);
				}
				pidPosRateX.measured = flow_x_vel_lpf_i;
				pidUpdate(&pidPosRateX,dt);
				pidPosRateY.measured = flow_y_vel_lpf_i;
				pidUpdate(&pidPosRateY,dt);

				pidRoll.desired = LIMIT(pidPosRateX.out,-15,15);
				pidPitch.desired = -LIMIT(pidPosRateY.out,-15,15);

				if(!ALL_flag.flow_control||!ALL_flag.height_lock)
				{
					status = EXIT_255;
				}
			}
			break;
		case EXIT_255:
			pidRest(&pPidObject[8],4);
			status = WAITING_1;
			break;
		default:
			status = WAITING_1;
			break;
	}
}
/**************************************************************
 *  flight control
 * @param[in]
 * @param[out]
 * @return
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{
		case WAITING_1:
			if(ALL_flag.unlock)
			{
				status = READY_11;
			}
			break;
		case READY_11:
			pidRest(pPidObject,6);

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;

			status = PROCESS_31;

			break;
		case PROCESS_31:
			if(Angle.pitch<-50||Angle.pitch>50||Angle.roll<-50||Angle.roll>50)
					if(Remote.thr>1200)
						ALL_flag.unlock = EMERGENT;

      pidRateX.measured = MPU6050.gyroX * Gyro_G;
			pidRateY.measured = MPU6050.gyroY * Gyro_G;
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G;

			pidPitch.measured = Angle.pitch;
		  pidRoll.measured = Angle.roll;
			pidYaw.measured = Angle.yaw;

		 	pidUpdate(&pidRoll,dt);
			pidRateX.desired = pidRoll.out;
			pidUpdate(&pidRateX,dt);

		 	pidUpdate(&pidPitch,dt);
			pidRateY.desired = pidPitch.out;
			pidUpdate(&pidRateY,dt);

			CascadePID(&pidRateZ,&pidYaw,dt);
			break;
		case EXIT_255:
			pidRest(pPidObject,6);
			status = WAITING_1;
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT)
		status = EXIT_255;
}

#define MOTOR1 motor_PWM_Value[0]
#define MOTOR2 motor_PWM_Value[1]
#define MOTOR3 motor_PWM_Value[2]
#define MOTOR4 motor_PWM_Value[3]
uint16_t low_thr_cnt_quiet;
uint16_t low_thr_cnt;
void MotorControl(void)
{
	volatile static uint8_t status=WAITING_1;


	if(ALL_flag.unlock == EMERGENT)
		status = EXIT_255;
	switch(status)
	{
		case WAITING_1:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2:
			if(Remote.thr>1100)
			{
				low_thr_cnt_quiet=0;
				low_thr_cnt=0;
				pidRest(pPidObject,6);
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t thr;
				if(ALL_flag.height_lock) //定高模式：油门使用高度PID输出
				{
					thr = pidHeightRate.out + thr_hold;
				}
				else //非定高状态：摇杆直接控制油门
				{
					int16_t temp;
					temp = Remote.thr -1000;
					thr = 100+0.55f * temp;
					thr_hold = thr;

					if(temp<10)
					{
						if(low_thr_cnt<1500)
								low_thr_cnt++;
						thr = thr-(low_thr_cnt*0.6);
						if(MPU6050.accZ<8500&&MPU6050.accZ>7800)
						{
							low_thr_cnt++;
							if(low_thr_cnt>600)
							{
								thr = 0;
								pidRest(pPidObject,6);
								MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 =0;
								status = WAITING_2;
								break;
							}
						}
					}
					else low_thr_cnt=0;
				}
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr, 0, 700);

				MOTOR1 += + pidRateX.out + pidRateY.out + pidRateZ.out;
				MOTOR2 += + pidRateX.out - pidRateY.out - pidRateZ.out;
				MOTOR3 += - pidRateX.out + pidRateY.out - pidRateZ.out;
				MOTOR4 += - pidRateX.out - pidRateY.out + pidRateZ.out;

				// 最终限幅！电机PWM不能是负数也不能超过700！
				MOTOR1 = LIMIT(MOTOR1, 0, 700);
				MOTOR2 = LIMIT(MOTOR2, 0, 700);
				MOTOR3 = LIMIT(MOTOR3, 0, 700);
				MOTOR4 = LIMIT(MOTOR4, 0, 700);
			}
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
			status = WAITING_1;
			break;
		default:
			break;
	}


	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);
	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);
	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);
	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);
}

/************************************END OF FILE********************************************/
