/**************************************************************
 *  ������λ��������봮�ڣ�������50��
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "USART.h"
#include "USB_HID.h"

// 兼容标准库宏定义 - HAL中不提供这些宏
#define USART_IT_RXNE ((uint32_t)0x00000020)  // RXNE bit
#define USART_IT_ORE  ((uint32_t)0x00000008)  // ORE bit
/******************************************************************/
//--------------------------
//��ʱ������λ����������PID���� ��ֹ�������������������һ�鸲��
static uint8_t RatePID[18];
static uint8_t AnglePID[18];
static uint8_t HighPID[18];
int16_t checkPID;
//���յ���λ�������������־λ
static struct{
	uint8_t PID1 :1; //���ܵ���λ��PID��1
	uint8_t PID2 :1; //���ܵ���λ��PID��2
	uint8_t PID3 :1; //���ܵ���λ��PID��3
	uint8_t PID4 :1; //���ܵ���λ��PID��4
	uint8_t PID5 :1; //���ܵ���λ��PID��5
	uint8_t PID6 :1; //���ܵ���λ��PID��6	
	uint8_t CMD2_READ_PID:1; //���ܵ���λ����ȡPID������
}ANTO_Recived_flag;


/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/	
void ANO_Recive(int8_t *pt)                   //���յ���λ��������
{
	switch(pt[2])
	{
		case ANTO_RATE_PID:
			ANTO_Recived_flag.PID1 = 1;             //���յ���λ��������PID����
			memcpy(RatePID,&pt[4],18);              //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�
			break;
		case ANTO_ANGLE_PID:                      //�����PID�Ǹ��ǶȻ��õ�
			memcpy(AnglePID,&pt[4],18);
			ANTO_Recived_flag.PID2 = 1;
			break;
		case ANTO_HEIGHT_PID:                     //�����PID�Ǹ��߶Ȼ��õ�
			memcpy(HighPID,&pt[4],18);
			ANTO_Recived_flag.PID3 = 1;
			break;
		case ANTO_PID4:
			break;
		case ANTO_PID5:   
			break;
		case ANTO_PID6:
			break;
		case 0x01:                                //��λ��������CMD1 ��������У׼

			break;
		case 0x02:                                //��λ��������CMD2 ���������ȡPID��
			{
			   enum                                  //��λ����ɿ�����
				{
					READ_PID = 0X01,                    //��ȡ�ɿص�PID����
					READ_MODE = 0x02,                   //��ȡ����ģʽ
					READ_ROUTE = 0x21,                  //��ȡ������Ϣ
					READ_VERSION = 0XA0,                //��ȡ�ɿذ汾
					RETURN_DEFAULT_PID = 0xA1           //�ָ�Ĭ��PID
				 };

				switch(*(uint8_t*)&pt[4])             //�ж���λ������CMD������
				{
					case READ_PID:                      //��λ�������ȡ�ɿ�PID����
						ANTO_Recived_flag.CMD2_READ_PID = 1;
						break;
					case READ_MODE: 
						break;
					case READ_ROUTE: 
						break;					
					case READ_VERSION:  
						break;
					case RETURN_DEFAULT_PID:  
						break;					
					default: 
						break;					
				}
			
			}
			break;
		case ANTO_RCDATA: //Immediately deal with 
			break;

		default:
			break;			
	}
	return;
}
/***********************************************************************
 * //�������ݵ���λ��
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
	int16_t Anto[15];

 void ANTO_Send(const enum ANTO_SEND FUNCTION) 
{
	uint8_t i;
	uint8_t len=2;

	int8_t *pt = (int8_t*)(Anto);
	PidObject *pidX=0;
	PidObject *pidY=0;
	PidObject *pidZ=0;

	switch(FUNCTION)
	{
		case ANTO_RATE_PID:      //����PID1����λ��
				 pidX = &pidRateX;   //ָ����������Ϊ���ٶ��ڻ�
				 pidY = &pidRateY;
				 pidZ = &pidRateZ;
         goto send_pid;		
		case ANTO_ANGLE_PID:       //����PID2����λ��
				 pidX = &pidRoll;
				 pidY = &pidPitch;
				 pidZ = &pidYaw;
				 goto send_pid;				
		case ANTO_HEIGHT_PID:     //����PID3����λ��
				 goto send_pid;							
		case ANTO_PID4:	  //PID4
		case ANTO_PID5:	         //PID5
    case ANTO_PID6:
send_pid:
			if(pidX!=NULL)
			{
				Anto[2] = (int16_t)(pidX->kp *1000);
				Anto[3] = (int16_t)(pidX->ki *1000);
				Anto[4] = (int16_t)(pidX->kd *1000);
			}
			if(pidY!=NULL)
			{
				Anto[5] = (int16_t)(pidY->kp *1000);
				Anto[6] = (int16_t)(pidY->ki *1000);
				Anto[7] = (int16_t)(pidY->kd *1000);
			}
			if(pidZ!=NULL)
			{
				Anto[8] = (int16_t)(pidZ->kp *1000);
				Anto[9] = (int16_t)(pidZ->ki *1000);
				Anto[10] = (int16_t)(pidZ->kd *1000);
			}
			len = 18;
			break;
		case ANTO_MOTOR:    //send motor
				Anto[2] = motor_PWM_Value[0];
				Anto[3] = motor_PWM_Value[1];
				Anto[4] = motor_PWM_Value[2];
				Anto[5] = motor_PWM_Value[3];
				Anto[6] = 0;//����֧��8��PWM�ϴ���λ��
				Anto[7] = 0;
				Anto[8] = 0;
				Anto[9] = 0;		
				len = 16;
			break;	
		case ANTO_RCDATA: //send RC data
			Anto[2] = Remote.thr;
			Anto[3] = Remote.yaw;
			Anto[4] = Remote.roll;
			Anto[5] = Remote.pitch;
			Anto[6] = Remote.AUX1;
			Anto[7] = Remote.AUX2;
			Anto[8] = Remote.AUX3;
			Anto[9] = Remote.AUX4;
			Anto[10] = Remote.AUX3;
			Anto[11] = Remote.AUX4;		
			len = 20;
			break;
		case ANTO_MPU_MAGIC:     //����MPU6050�ʹ����Ƶ�����
			memcpy(&Anto[2],(int8_t*)&MPU6050,sizeof(_st_Mpu));
			Anto[8] = 0;//û�д����ƴ�����
			Anto[9] = 0;
			Anto[10] = 0;
			len = 18;
			break;
		case ANTO_SENSER2:

			break;
		case ANTO_STATUS:     //send angle
			
				Anto[2] = -(int16_t)(Angle.roll*100);
				Anto[3] = (int16_t)(Angle.pitch*100);
				Anto[4] = -(int16_t)(Angle.yaw*100);
				Anto[5] = 0;//û�и߶�����
				Anto[6] = 0;
				Anto[7] = ALL_flag.unlock<<8;//������Ϣ
				len = 12;
			break;
		case ANTO_POWER:

				break;
		case ANTO_CHECK:
				Anto[2] = checkPID;
				len = 2;
				break;		
		default:
			break;			
	}
	
	Anto[0] = 0XAAAA;
	Anto[1] = len | FUNCTION<<8;
	pt[len+4] = (int8_t)(0xAA+0xAA);
	for(i=2;i<len+4;i+=2)    //a swap with b;
	{
		pt[i] ^= pt[i+1];
		pt[i+1] ^= pt[i];
		pt[i] ^= pt[i+1];
		pt[len+4] += pt[i] + pt[i+1];
	}
	Usb_Hid_Adddata((u8*)pt , len+5);//����USB HID���͵���λ��
	Usb_Hid_Send();

//	Uart_Start_DMA_Tx(pt,len+5);//���ô��ڷ��͵�������λ��
}
/***********************************************************************
 * ��ѯɨ����λ���˿�.
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void ANTO_polling(void) //
{
	volatile static uint8_t status = 0;
	switch(status)
	{
		case 0:
			
			status = 1;
			break;
		case 1:
			ANTO_Send(ANTO_MPU_MAGIC);//����MPU6050
			delay_ms(30);
	  	ANTO_Send(ANTO_STATUS);//���ͽǶ�����
			delay_ms(30);
			ANTO_Send(ANTO_RCDATA);//����ң������
			delay_ms(30);//���������������ʱһ���������������
	
		
			if(*(uint8_t*)&ANTO_Recived_flag != 0) //һ�����յ���λ�������ݣ�����ͣ�������ݵ���λ����ת��ȥ�ж���λ��Ҫ��ɿ���ʲô��
			{
				status = 2;
			}
		 	break;
		case 2:
			if(*(uint8_t*)&ANTO_Recived_flag == 0)//��λ���ķ����������ݶ��������ˣ��򷵻�ר�ĵķ������ݵ���λ��
			{
				status = 1;
			}
	
			if(ANTO_Recived_flag.CMD2_READ_PID) //�ж���λ���Ƿ����󷢷��ͷɿ�PID���ݵ���λ��
			{		
					ANTO_Send(ANTO_RATE_PID);
					delay_ms(15);
					ANTO_Send(ANTO_ANGLE_PID);
					delay_ms(15);
					ANTO_Recived_flag.CMD2_READ_PID = 0;
			}
			
			if(*(uint8_t*)&ANTO_Recived_flag & 0x3f) //���յ���λ��������PID����
			{
					PidObject *pidX=0;
					PidObject *pidY=0;
					PidObject *pidZ=0;
				  uint8_t *P;
				
					if(ANTO_Recived_flag.PID1)
					{
						 pidX = &pidRateX;
						 pidY = &pidRateY;
						 pidZ = &pidRateZ;
						 P = RatePID;
						 ANTO_Recived_flag.PID1 = 0;
					}
					else if(ANTO_Recived_flag.PID2)
					{
						 pidX = &pidRoll;
						 pidY = &pidPitch;
						 pidZ = &pidYaw;
						 P = AnglePID;	
						 ANTO_Recived_flag.PID2 = 0;                             
					}
					else
					{
						status = 1;
						*(uint8_t*)&ANTO_Recived_flag = 0;
						break;
					}
					{
							union {
								uint16_t _16;
								uint8_t _u8[2];
							}data;
							
							if(pidX!=NULL)
							{
								data._u8[1] = P[0]; 
								data._u8[0] = P[1];
								pidX->kp =  data._16 /1000.0f;
								data._u8[1] = P[2]; 
								data._u8[0] = P[3];
								pidX->ki =  data._16 /1000.0f;
								data._u8[1] = P[4]; 
								data._u8[0] = P[5];
								pidX->kd =  data._16 /1000.0f;								
							}
							if(pidY!=NULL)
							{
								data._u8[1] = P[6]; 
								data._u8[0] = P[7];
								pidY->kp =  data._16 /1000.0f;
								data._u8[1] = P[8]; 
								data._u8[0] = P[9];
								pidY->ki =  data._16 /1000.0f;
								data._u8[1] = P[10]; 
								data._u8[0] = P[11];
								pidY->kd =  data._16 /1000.0f;		
							}
							if(pidZ!=NULL)
							{
								data._u8[1] = P[12]; 
								data._u8[0] = P[13];
								pidZ->kp =  data._16 /1000.0f;
								data._u8[1] = P[14]; 
								data._u8[0] = P[15];
								pidZ->ki =  data._16 /1000.0f;
								data._u8[1] = P[16]; 
								data._u8[0] = P[17];
								pidZ->kd =  data._16 /1000.0f;		
							}				
					}				
			}
			break;
		default:
			break;
	}

}
/***********************************************************************
 * ���մ��ڷ�����������
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
//void USART1_IRQHandler(void) //���ڽ���
//{ 	
//  static  int8_t ReciveBuffer[25];
//  static uint8_t count;
//	if ((USART1->SR & USART_IT_ORE))//�Ƿ���ռĴ������
//	{
//	
//	}	
//  if((USART1->SR & USART_IT_RXNE))
//  {
//		ReciveBuffer[count] = USART1->DR;	
//		switch(count)
//		{
//		case 0:
//			if(ReciveBuffer[0]==(int8_t)0xAA)
//				count++;
//				break;
//		case 1:	
//			if(ReciveBuffer[1]==(int8_t)0xAF)
//					count++;
//			else 
//					count = 0;
//				break;
//		default:
//			if(count < ReciveBuffer[3]+4)
//			{
//				count++;
//				break;
//			}
//			else
//			{
//				uint8_t i;
//				int8_t CheckSum=0;
//							
//				for(i=0;i<count;i++)
//				{
//					CheckSum += ReciveBuffer[i];			
//				}		
//				if(CheckSum == ReciveBuffer[count])  //if the data calculate sum equal to the  final data from PC.
//				{
//						static int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};	
//						
//						CheckSend[4] = ReciveBuffer[2];
//						CheckSend[5] = CheckSum;
//						CheckSend[6] = 0;
//						for(i=0;i<6;i++)
//						{
//							CheckSend[6] += CheckSend[i]; 					
//						}
//						USART1_SendByte(CheckSend,7);
//						ANO_Recive(ReciveBuffer);			//To arrange the data	and give the result to control argument.	
//				}			
//				count = 0;                  //return to the first data point,and retore from the head buffer next time.
//				ReciveBuffer[0] = 0;  //reset the data buffer.
//				ReciveBuffer[1] = 0;
//			}
//			break;							
//		}
//	}
//}

/************************END OF FILE********************/
