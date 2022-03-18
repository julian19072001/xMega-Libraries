/**************************************************************************/
/*!
@file       mq135.c
@author     Julian Della Guardia
@date		18-3-2022
@link		https://github.com/julian19072001/xMega-Libraries/tree/main
@version 	1.1

Rewritten version of the arduino library to use on the HVA xMega
Calculating parameters are directly taken from the original library.
@orginal    Georg Krocker
@link       https://github.com/GeorgK/MQ135
*/
/**************************************************************************/

#include "mq135.h"

// set up pins for the sensor
void init_MQ135(int32_t port, int16_t pin)
{
	//setup for choosen port
	if(port == PORTA)
	{
		adc = ADCA;
	}
	else if(port == PORTB)
	{
		adc = ADCB;
	}
	else
	{
		adc = ADCA;
	}

	//setup for choosen pin
	if(pin == PIN0)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN0_gc;
	}
	else if(pin == PIN1)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN1_gc;
	}
	else if(pin == PIN2)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN2_gc;
	}
	else if(pin == PIN3)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN3_gc;
	}
	else if(pin == PIN4)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN4_gc;
	}	
	else if(pin == PIN5)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN5_gc;
	}	
	else if(pin == PIN6)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN6_gc;
	}	
	else if(pin == PIN7)
	{
		MUXPOS = ADC_CH_MUXPOS_PIN7_gc;
	}
	else 
	{
		MUXPOS = ADC_CH_MUXPOS_PIN0_gc;
	}

	port.DIRCLR     = pin;																//configure pin as input for ADCA
	adc.CH0.MUXCTRL = MUXPOS | ADC_CH_MUXNEG_GND_MODE3_gc;								//pin to channel 0
	adc.CH0.CTRL    = ADC_CH_INPUTMODE_DIFF_gc;											//differential, no gain
	
	adc.CTRLB       =	ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm | ADC_FREERUN_bm; 		//free running mode

	adc.REFCTRL     =  ADC_REFSEL_INTVCC_gc;											//internal vcc/1.6 refernce
	adc.PRESCALER   =  ADC_PRESCALER_DIV16_gc;											//prescaling 
	adc.CTRLA       =  ADC_ENABLE_bm;													//enable ADC

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
	adc.CH0.CTRL |= ADC_CH_START_bm;                  						//start ADC conversion
	while ( !(adc.CH0.INTFLAGS & ADC_CH_CHIF_bm) ) ;   						//wait until it's ready
	val = adc.CH0.RES;														//store the ADC value
	adc.CH0.INTFLAGS |= ADC_CH_CHIF_bm;                						//reset interrupt flag

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
