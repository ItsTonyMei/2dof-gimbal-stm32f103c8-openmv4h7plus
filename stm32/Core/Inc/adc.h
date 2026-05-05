#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sys.h"

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);

#define Battery_Ch 1
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);

#ifdef __cplusplus
}
#endif

#endif
