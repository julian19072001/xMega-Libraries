/**************************************************************************/
/*!
@file       mq135.h
@author     Julian Della Guardia
@date		18-3-2022
@link		https://github.com/julian19072001/xMega-Libraries/tree/main
@version    1.1

Rewritten version of the arduino library to use on the HVA xMega
Calculating parameters are directly taken from the original library.
@orginal    Georg Krocker
@link       https://github.com/GeorgK/MQ135
*/
/**************************************************************************/

#include <avr/io.h>
#include <stdio.h>
#include <math.h>
///Parameters for calculating ppm of CO2 from sensor resistance
#define PARA 116.6020682
#define PARB 2.769034857

///Parameters to model temperature and humidity dependence
#define CORA .00035
#define CORB .02718
#define CORC 1.39538
#define CORD .0018
#define CORE -.003333333
#define CORF -.001923077
#define CORG 1.130128205

///Atmospheric CO2 level for calibration purposes
#define ATMOCO2 414.47  //Global CO2 Aug 2021

#define RLOAD 10      //The load resistance on the board in kOhm
#define RZERO 27.1    //Calibration resistance at atmospheric CO2 level

float _rload;   //The load resistance on the board in kOhm
float _rzero;   //Calibration resistance at atmospheric CO2 level

int32_t adc;
int32_t MUXPOS;

void init_MQ135(int32_t port, int16_t pin);
float getCorrectionFactor(float t, float h);
float getResistance();
float getCorrectedResistance(float t, float h);
float getPPM();
float getCorrectedPPM(float t, float h);
float getRZero();
float getCorrectedRZero(float t, float h);
