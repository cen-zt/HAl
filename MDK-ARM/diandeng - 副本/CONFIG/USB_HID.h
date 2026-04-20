/***************************************************************/
#include "stm32f1xx_hal.h"
/***************************************************************/
#ifndef __USB_HID_H
#define __USB_HID_H
/***************************************************************/

typedef uint8_t u8;

extern u8 Hid_RxData[64];
extern u8 USB_ReceiveFlg;
void USB_HID_Init(void);
void Usb_Hid_Adddata(u8 *dataToSend , u8 length);
void Usb_Hid_Send (void);
void Usb_Hid_Receive(void);//轮询接收

#endif
