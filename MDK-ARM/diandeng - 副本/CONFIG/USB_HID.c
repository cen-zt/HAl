/**************************************************************
 *
 * @brief
	   ZIN-7标准飞控
		 黑马技术群551883670
		 店铺地址https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "stm32f1xx_hal.h"
#include "USB_HID.h"
#include "usbd_core.h"
#include "usbd_hid.h"
#include "delay.h"
#include "ANO_DT.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

u8 USB_ReceiveFlg = 0;
u8 Hid_RxData[64];
u8 HID_SEND_TIMEOUT = 5;			//hid打包一次帧等待HID_SEND_TIMEOUT帧才发送
u8 hid_datatemp[256];					//hid缓存数组
u8 hid_datatemp_begin = 0;		//缓存数组开头指针，指向应该发送的位置
u8 hid_datatemp_end = 0;			//缓存数组数据末尾

void USB_HID_Init(void)
{
	MX_USB_DEVICE_Init();
	USB_ReceiveFlg = 0;
}

void Usb_Hid_Adddata(u8 *dataToSend , u8 length)
{
	u8 i;
	for(i=0; i<length; i++)
	{
		hid_datatemp[hid_datatemp_end++] = dataToSend[i];
	}
}

void Usb_Hid_Receive()//轮询接收
{
	if (USB_ReceiveFlg)
	{
		if(Hid_RxData[0] < 33 && Hid_RxData[1]==0xaa)
		{
				//Hid_RxData[0]是长度，Hid_RxData[1]是帧头开头就是正确
				   if(Hid_RxData[1] == 0xaa && Hid_RxData[2] == 0xaF ) //帧头正确
					 {
											u8 i;
											u8 check_sum = 0;
											for(i=1;i<Hid_RxData[0];i++)  //buf[0] Ϊ��������PC��������ܳ���
											{
												check_sum+= Hid_RxData[i];
											}
											if(check_sum == Hid_RxData[Hid_RxData[0]]) //buf[0] Ϊ��������PC��������ܳ��� ���һ��bufΪPC���������ı�������У��ͣ��뱾���յ������ݣ��ҪȽ�
											{
													if(Hid_RxData[4] >= 0x10 && Hid_RxData[5]<=0x15) //����յ�����PID������Ҫ����У��ֵ
													{
															checkPID = Hid_RxData[4]<<8 | check_sum;  //����PIDУ������
															ANTO_Send(ANTO_CHECK); //�յ�HID������PID��Ҫ���Ϸ���У��ֵ����λ��
													}
													ANO_Recive((int8_t*)(Hid_RxData+1));		//hid�յ����ݻ���ô˺���
											}
											Hid_RxData[1] = 0;//帧头清0
							 }
		}
		USB_ReceiveFlg = 0;
	}

}

void Usb_Hid_Send (void)
{
	static u8 notfull_timeout=0;
	u8 i;
	if(hid_datatemp_end > hid_datatemp_begin)
	{
		if((hid_datatemp_end - hid_datatemp_begin) >= 63)
		{
			notfull_timeout = 0;
			hid_datatemp[0] = 63;
			for( i=0; i<63; i++)
			{
				hid_datatemp[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			USBD_HID_SendReport(&hUsbDeviceFS, hid_datatemp, 64);
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				hid_datatemp[0] = hid_datatemp_end - hid_datatemp_begin;
				for( i=0; i<63; i++)
				{
					if(i<hid_datatemp_end - hid_datatemp_begin)
						hid_datatemp[i+1] = hid_datatemp[hid_datatemp_begin+i];
					else
						hid_datatemp[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				USBD_HID_SendReport(&hUsbDeviceFS, hid_datatemp, 64);
			}
		}
	}
	else if(hid_datatemp_end < hid_datatemp_begin)
	{
		if((256 - hid_datatemp_begin + hid_datatemp_end) >= 63)
		{
			notfull_timeout = 0;
			hid_datatemp[0] = 63;
			for( i=0; i<63; i++)
			{
				hid_datatemp[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			USBD_HID_SendReport(&hUsbDeviceFS, hid_datatemp, 64);
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				hid_datatemp[0] = 256 - hid_datatemp_begin + hid_datatemp_end;
				for( i=0; i<63; i++)
				{
					if(i<256 - hid_datatemp_begin + hid_datatemp_end)
						hid_datatemp[i+1] = hid_datatemp[(u8)(hid_datatemp_begin+i)];
					else
						hid_datatemp[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				USBD_HID_SendReport(&hUsbDeviceFS, hid_datatemp, 64);
			}
		}
	}
}

/*********************END OF FILE******************************************************/
