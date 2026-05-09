/*******************************************************************
 *@title Remote Control;
 *@brief remote data process on polling every 10ms.
 *@brief remote control on interrupt every 24ms
 ******************************************************************/
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "mpu6050.h"

#define SUCCESS 0
#undef FAILED
#define FAILED  1
/*****************************************************************************************
 *  ͨ�����ݴ���
 * @param[in]
 * @param[out]
 * @return
 ******************************************************************************************/
uint8_t RC_rxData[32];
void remote_unlock(void);
uint16_t last_Remote_AUX1 = 1000;
uint16_t last_Remote_AUX3 = 1000;
uint16_t last_Remote_AUX4 = 1000;
uint16_t last_Remote_AUX2 = 1000;

int16_t roll_offset;
int16_t pitch_offset;

void RC_Analy(void)
{
	static uint16_t cnt;
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{

		uint8_t i;
		uint8_t CheckSum=0;
		cnt = 0;
		for(i=0;i<31;i++)
		{
			CheckSum +=  RC_rxData[i];
		}
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)
		{
			Remote.roll = ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];
			Remote.roll = LIMIT(Remote.roll,1000,2000);
			Remote.pitch = ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];
			Remote.pitch = LIMIT(Remote.pitch,1000,2000);
			Remote.thr = 	((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];
			Remote.thr = 	LIMIT(Remote.thr,1000,2000);
			Remote.yaw =  ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];
			Remote.yaw =  LIMIT(Remote.yaw,1000,2000);
			Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];
			Remote.AUX1 =  LIMIT(Remote.AUX1,1000,2000);
			Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];
			Remote.AUX2 =  LIMIT(Remote.AUX2,1000,2000);
			Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];
			Remote.AUX3 =  LIMIT(Remote.AUX3,1000,2000);
			Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];
			Remote.AUX4 = LIMIT(Remote.AUX4,1000,2000);

			//通道5作为定高控制 每拨一次就会在1000和2000之间切换
			if(Remote.AUX1!=last_Remote_AUX1)
			{
				last_Remote_AUX1=Remote.AUX1;
				if(ALL_flag.flow_control)
				{
					ALL_flag.flow_control = 0;
					ALL_flag.height_lock = 0;
				}
				else
				{
					ALL_flag.flow_control = 1;
					ALL_flag.height_lock = 1;
				}
			}
			{
				if(!ALL_flag.flow_control)//如果没有在定点模式，则角度用摇杆控制
				{
					float roll_pitch_ratio = 0.04f;
					pidPitch.desired =(-(Remote.pitch-1500)+pitch_offset)*roll_pitch_ratio;
					pidRoll.desired = (-(Remote.roll-1500)+roll_offset)*roll_pitch_ratio;
				}
				{
					const float yaw_ratio =   0.5f;
					if(Remote.yaw>1820)
					{
						pidYaw.desired -= yaw_ratio;
					}
					else if(Remote.yaw <1180)
					{
						pidYaw.desired += yaw_ratio;
					}
				}
			}
			remote_unlock();
		}
	}
	else
	{
		cnt++;
		if(cnt>500)
		{
			cnt = 0;
			ALL_flag.unlock = 0;
			NRF24L01_init();
		}
	}
}

/*****************************************************************************************
 *  ������ж�
 * @param[in]
 * @param[out]
 * @return
 ******************************************************************************************/
#include "imu.h"
void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1050 &&Remote.yaw<1150)
	{
		status = EXIT_255;
	}

	switch(status)
	{
		case WAITING_1:
			if(Remote.thr<1150)
			{
				 status = WAITING_2;
			}
			break;
		case WAITING_2:
			if(Remote.thr>1800)
			{
				static uint8_t cnt = 0;
				cnt++;
				if(cnt>5)
				{
					cnt=0;
					status = WAITING_3;
				}
			}
			break;
		case WAITING_3:
			if(Remote.thr<1100)
			{
				 status = WAITING_4;
			}
			break;
		case WAITING_4:
			ALL_flag.unlock = 1;
			status = PROCESS_31;
			LED.status = AlwaysOn;
			last_Remote_AUX1=Remote.AUX1;
			ALL_flag.height_lock = 0;
			ALL_flag.flow_control = 0;
			pidRoll.offset =   -(Remote.roll-1500)*0.04;
			pidPitch.offset  = -(Remote.pitch-1500)*0.04;
			roll_offset = (Remote.roll-1500);
			pitch_offset = (Remote.pitch-1500);
			imu_rest();
			break;
		case PROCESS_31:
			if(Remote.thr<1020)
			{
				if(!ALL_flag.height_lock )
				{
					if(cnt++ > 3000)
					{
						status = EXIT_255;
					}
				}
			}
			else if(!ALL_flag.unlock)
			{
				status = EXIT_255;
			}
			else
				cnt = 0;
			break;
		case EXIT_255:
			LED.status = AllFlashLight;
			cnt = 0;
			LED.FlashTime = 100;
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/
