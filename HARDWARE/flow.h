#ifndef __FLOW_H_
#define __FLOW_H_
#include "stm32f1xx_hal.h"

extern float flow_x_lpf_att_i;
extern float flow_y_lpf_att_i;
extern float flow_x_vel_lpf_i;
extern float flow_y_vel_lpf_i;
extern void flow_data_sins(void);
extern void flow_get_data(void);
extern uint32_t VL53L01_high;

typedef struct
{
	int16_t vel_x;
	int16_t vel_y;
	float pos_x_i;
	float pos_y_i;
	uint8_t squal;
	uint8_t end_flag;
}FLow;
extern FLow flow_data;

#endif
