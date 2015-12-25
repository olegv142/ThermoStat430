#pragma once

/*
 * Internal temperature sensor support routines
 */

#define CAL_ADC_ADDR (0x10da)
#define ADC_TAG 0x10
#define CAL_ADC_TAG_    (CAL_ADC_ADDR)
#define CAL_ADC_15T30_  (CAL_ADC_ADDR + 8)
#define CAL_ADC_15T85_  (CAL_ADC_ADDR + 10)

READ_ONLY DEFC(CAL_ADC_TAG, CAL_ADC_TAG_)
READ_ONLY DEFW(CAL_ADC_15T30, CAL_ADC_15T30_)
READ_ONLY DEFW(CAL_ADC_15T85, CAL_ADC_15T85_)

#define TSCALE 278
static inline int adc2Temp(int v)
{
	int c30 = CAL_ADC_15T30;
	return (long long)(TSCALE + 30) * v / c30 - TSCALE;
}

static inline int adcTempCalibValid(void)
{
	return CAL_ADC_TAG == ADC_TAG;
}
