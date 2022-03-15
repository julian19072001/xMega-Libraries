/**************************************************************************/
/*!
@file       mq135.c
@author     Julian Della Guardia
@date		14-3-2022
@link		https://github.com/julian19072001/xMega-Libraries/tree/main

Rewritten version of the arduino library to use on the HVA xMega
Calculating parameters are directly taken from the original library.
@orginal    Georg Krocker
@link       https://github.com/GeorgK/MQ135
*/
/**************************************************************************/

#include "mq135.h"

// set up pins for the sensor
void init_MQ135(uint16_t pin)
{
	PORTA.DIRCLR     = pin;																//configure PA0 as input for ADCA
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc | ADC_CH_MUXNEG_GND_MODE3_gc;				//PA0 to channel 0
	ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_DIFF_gc;										//differential, no gain
	
	ADCA.CTRLB       =	ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm | ADC_FREERUN_bm; 		//free running mode

	ADCA.REFCTRL     =  ADC_REFSEL_INTVCC_gc;											//internal vcc/1.6 refernce
	ADCA.PRESCALER   =  ADC_PRESCALER_DIV16_gc;											//prescaling 
	ADCA.CTRLA       =  ADC_ENABLE_bm;													//enable ADC

	_rzero = RZERO;
	_rload = RLOAD;
}

//get correctiong factor through tempature and humidity
float getCorrectionFactor(float t, float h) {
	if(t < 20){
		return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
		} else {
		return CORE * t + CORF * h + CORG;
	}
}

float getResistance()
{
	int val;
	ADCA.CH0.CTRL |= ADC_CH_START_bm;                  						//start ADC conversion
	while ( !(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm) ) ;   					//wait until it's ready
	val = ADCA.CH0.RES;														//store the ADC value
	ADCA.CH0.INTFLAGS |= ADC_CH_CHIF_bm;                					//reset interrupt flag

	return ((4095./(float)val) - 1.)*_rload;								//return a resistance value
}

float getCorrectedResistance(float t, float h)
{
	return getResistance()/getCorrectionFactor(t, h);						//return the resistance value with temprature and humidity correction 
}

float getPPM()
{
	return  PARA * pow((getResistance()/_rzero), -PARB);					//return the CO2 value in ppm 
}

float getCorrectedPPM(float t, float h)
{
	return PARA * pow((getCorrectedResistance(t, h)/_rzero), -PARB);		//return the CO2 value in ppm with temprature and humidity correction
}

float getRZero() 
{
	return getResistance() * pow((ATMOCO2/PARA), (1./PARB));				//return the calculated RZero value
}

float getCorrectedRZero(float t, float h) 
{
	return getCorrectedResistance(t, h) * pow((ATMOCO2/PARA), (1./PARB));	//return the calculated RZero value with temprature and humidity correction
}
