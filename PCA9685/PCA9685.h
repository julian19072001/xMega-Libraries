/*  ATTiny/xMega Library for the PCA9685 16-Channel PWM Driver Module.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    Created by Kasper Skårhøj, August 3rd, 2012.
    Forked by Vitska, June 18th, 2016.
    Forked by NachtRaveVL, July 29th, 2016.
    Forked by Julian Della Guardia, October 30th, 2023.

    PCA9685-ATTiny/xMega - Version 1.0.0
*/

#ifndef F_CPU
#define F_CPU 32000000UL
#endif

#ifndef PCA9685_H
#define PCA9685_H

// Uncomment or -D this define to swap PWM low(begin)/high(end) phase values in register reads/writes (needed for some chip manufacturers).
#define PCA9685_SWAP_PWM_BEG_END_REGS


#include "i2c.h"
#include <assert.h>
#include <stdbool.h>
#include <util/delay.h> 
#include <string.h>
#include <math.h>

// Default proxy addresser i2c addresses
#define PCA9685_I2C_DEF_ALLCALL_PROXYADR    (uint8_t)0xE0      // Default AllCall i2c proxy address
#define PCA9685_I2C_DEF_SUB1_PROXYADR       (uint8_t)0xE2      // Default Sub1 i2c proxy address
#define PCA9685_I2C_DEF_SUB2_PROXYADR       (uint8_t)0xE4      // Default Sub2 i2c proxy address
#define PCA9685_I2C_DEF_SUB3_PROXYADR       (uint8_t)0xE8      // Default Sub3 i2c proxy address


// Output driver control mode (see datasheet Table 12 and Fig 13, 14, and 15 concerning correct
// usage of OUTDRV).
typedef enum PCA9685_OutputDriverMode {
    PCA9685_OutputDriverMode_OpenDrain,         // Module outputs in an open-drain (aka direct connection) style structure with 400mA @5v total sink current, useful for LEDs and low-power Servos
    PCA9685_OutputDriverMode_TotemPole,         // Module outputs in a totem-pole (aka push-pull) style structure with 400mA @5v total sink current and 160mA total source current, useful for external drivers (default)

    PCA9685_OutputDriverMode_Count,             // Internal use only
    PCA9685_OutputDriverMode_Undefined = -1     // Internal use only
} PCA9685_OutputDriverMode_t;
// NOTE: Totem-pole mode should be used when an external N-type or P-type driver is in
// use, which provides actual sourcing current while open-drain mode doesn't. At max
// channel capacity, the sink current limit is 25mA@5v per channel while the source
// current limit, in totem-pole mode, is 10mA@5v per channel. However, from datasheet
// Table 6. subnote [1]: "Some newer LEDs include integrated Zener diodes to limit
// voltage transients, reduce EMI, and protect the LEDs, and these -MUST- be driven only
// in the open-drain mode to prevent over-heating the IC." Also from datasheet, Section
// 10. question 5: "in the push-pull architecture there is a low resistance path to GND
// through the Zener and this [causes] the IC to overheat."

// Output-enabled/active-low-OE-pin=LOW driver output mode (see datasheet Table 12 and
// Fig 13, 14, and 15 concerning correct usage of INVRT).
typedef enum PCA9685_OutputEnabledMode {
    PCA9685_OutputEnabledMode_Normal,           // When OE is enabled/LOW, channels output a normal signal, useful for N-type external drivers (default)
    PCA9685_OutputEnabledMode_Inverted,         // When OE is enabled/LOW, channels output an inverted signal, useful for P-type external drivers or direct connection

    PCA9685_OutputEnabledMode_Count,            // Internal use only
    PCA9685_OutputEnabledMode_Undefined = -1    // Internal use only
} PCA9685_OutputEnabledMode_t;
// NOTE: Polarity inversion is often set according to if an external N-type driver
// (should not use INVRT) or external P-type driver/direct connection (should use INVRT)
// is used. Most breakouts have just a 220Ω resistor between the individual channel
// outputs of the IC and PWM output pins, which is useful when powering LEDs. The V+ rail
// of most breakouts can connect through a 10v 1000μF decoupling capacitor, typically
// already installed on most breakouts, which can reduce voltage spikes and ground bounce
// during phase shifts at the start/end of the PWM high phase when many channel devices
// are connected together. See https://forums.adafruit.com/viewtopic.php?f=8&t=127421 and
// https://forums.adafruit.com/viewtopic.php?f=8&t=162688 for information on installing
// a decoupling capacitor if need arises.

// Output-not-enabled/active-low-OE-pin=HIGH driver output mode (see datasheet Section
// 7.4 concerning correct usage of OUTNE).
typedef enum PCA9685_OutputDisabledMode {
    PCA9685_OutputDisabledMode_Low,             // When OE is disabled/HIGH, channels output a LOW signal (default)
    PCA9685_OutputDisabledMode_High,            // When OE is disabled/HIGH, channels output a HIGH signal (only available in totem-pole mode)
    PCA9685_OutputDisabledMode_Floating,        // When OE is disabled/HIGH, channel outputs go into a floating (aka high-impedance/high-Z) state, which may be further refined via external pull-up/pull-down resistors

    PCA9685_OutputDisabledMode_Count,           // Internal use only
    PCA9685_OutputDisabledMode_Undefined = -1   // Internal use only
} PCA9685_OutputDisabledMode_t;
// NOTE: Active-low-OE pin is typically used to synchronize multiple PCA9685 devices
// together, but can also be used as an external dimming control signal.

// Channel update strategy used when multiple channels are being updated in batch.
typedef enum PCA9685_ChannelUpdateMode {
    PCA9685_ChannelUpdateMode_AfterStop,        // Channel updates commit after full-transmission STOP signal (default)
    PCA9685_ChannelUpdateMode_AfterAck,         // Channel updates commit after individual channel update ACK signal

    PCA9685_ChannelUpdateMode_Count,            // Internal use only
    PCA9685_ChannelUpdateMode_Undefined = -1    // Internal use only
} PCA9685_ChannelUpdateMode_t;

// Software-based phase balancing scheme.
typedef enum PCA9685_PhaseBalancer {
    PCA9685_PhaseBalancer_None,                 // Disables software-based phase balancing, relying on installed hardware to handle current sinkage (default)
    PCA9685_PhaseBalancer_Linear,               // Uses linear software-based phase balancing, with each channel being a preset 16 steps (out of the 4096/12-bit value range) away from previous channel (may cause LED flickering/skipped-cycles on PWM changes)

    PCA9685_PhaseBalancer_Count,                // Internal use only
    PCA9685_PhaseBalancer_Undefined = -1        // Internal use only
} PCA9685_PhaseBalancer_t;
// NOTE: Software-based phase balancing attempts to further mitigate ground bounce and
// voltage spikes during phase shifts at the start/end of the PWM high phase by shifting
// the leading edge of each successive PWM high phase by some amount. This helps make
// the current sinks occur over the entire duty cycle range instead of all together at
// once. Software-based phase balancing can be useful in certain situations, but in
// practice has been the source of many problems, including the case whereby the PCA9685
// will skip a cycle between PWM changes when the leading/trailing edge is shifted past a
// certain point. While we may revisit this idea in the future, for now we're content on
// leaving None as the default, and limiting the shift that Linear applies.

typedef struct PCA9685
{
    uint8_t _i2cAddress;                                        // Module's i2c address (default: B000000)
    TWI_t* _twi; 
    uint64_t _f_sys;
                                                    // TWI pins
    PCA9685_OutputDriverMode_t _driverMode;                       // Output driver mode
    PCA9685_OutputEnabledMode_t _enabledMode;                     // OE enabled output mode
    PCA9685_OutputDisabledMode_t _disabledMode;                   // OE disabled output mode
    PCA9685_ChannelUpdateMode_t _updateMode;                      // Channel update mode
    PCA9685_PhaseBalancer_t _phaseBalancer;                       // Phase balancer scheme
    bool _isProxyAddresser;                                     // Proxy addresser flag (disables certain functionality)
    uint8_t _lastI2CError;                                      // Last module i2c error

    float *_coeff;      // a,b,c,d coefficient values
    bool _isCSpline;    // Cubic spline tracking, for _coeff length

    uint8_t _readBytes;
} PCA9685_t;

    // Resets modules. Typically called in setup(), before any init()'s. Calling will
    // perform a software reset on all PCA9685 devices on the Wire instance, ensuring
    // that all PCA9685 devices on that line are properly reset.
    void resetDevices(PCA9685_t device_Info);

    // Initializes module. Typically called in setup().
    void initPCA9685(PCA9685_t device_Info);

    // Initializes module as a proxy addresser. Typically called in setup(). Used when
    // instance talks through to AllCall/Sub1-Sub3 instances as a proxy object. Using
    // this method will disable any method that performs a read or conflicts with certain
    // states. Proxy addresser i2c addresses must be >= 0xE0, with defaults provided via
    // PCA9685_I2C_DEF_[ALLCALL|SUB[1-3]]_PROXYADR defines.
    void initAsProxyAddresser(PCA9685_t device_Info);



    // Min: 24Hz, Max: 1526Hz, Default: 200Hz. As Hz increases channel resolution
    // diminishes, as raw pre-scaler value, computed per datasheet, starts to require
    // much larger frequency increases for single-digit increases of the raw pre-scaler
    // value that ultimately controls the PWM frequency produced.
    void setPWMFrequency(PCA9685_t device_Info, float pwmFrequency);
    // Sets standard servo frequency of 50Hz.
    void setPWMFreqServo(PCA9685_t device_Info);

    // Turns channel either full on or full off
    void setChannelOn(PCA9685_t device_Info, int channel);
    void setChannelOff(PCA9685_t device_Info, int channel);

    // PWM amounts 0 - 4096, 0 full off, 4096 full on
    void setChannelPWM(PCA9685_t device_Info, int channel, uint16_t pwmAmount);
    void setChannelsPWM(PCA9685_t device_Info, int begChannel, int numChannels, const uint16_t *pwmAmounts);

    // Sets all channels, but won't distribute phases
    void setAllChannelsPWM(PCA9685_t device_Info, uint16_t pwmAmount);

    // Returns PWM amounts 0 - 4096, 0 full off, 4096 full on
    uint16_t getChannelPWM(PCA9685_t device_Info, int channel);

    // Enables multiple talk-through paths via i2c bus (lsb/bit0 must stay 0). To use,
    // create a new proxy instance using initAsProxyAddresser() with proper proxy i2c
    // address >= 0xE0, and pass that instance's i2c address into desired method below.
    void enableAllCallAddress(PCA9685_t device_Info, uint8_t i2cAddressAllCall);
    void enableSub1Address(PCA9685_t device_Info, uint8_t i2cAddressSub1);
    void enableSub2Address(PCA9685_t device_Info, uint8_t i2cAddressSub2);
    void enableSub3Address(PCA9685_t device_Info, uint8_t i2cAddressSub3);
    void disableAllCallAddress(PCA9685_t device_Info);
    void disableSub1Address(PCA9685_t device_Info);
    void disableSub2Address(PCA9685_t device_Info);
    void disableSub3Address(PCA9685_t device_Info);

    // Allows external clock line to be utilized (power reset required to disable)
    void enableExtClockLine(PCA9685_t device_Info);

    uint8_t getLastI2CError();

    uint8_t getMode2Value(PCA9685_t device_Info);
    void getPhaseCycle(PCA9685_t device_Info, int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);

    void writeChannelBegin(PCA9685_t device_Info, int channel);
    void writeChannelPWM(PCA9685_t device_Info, uint16_t phaseBegin, uint16_t phaseEnd);
    void writeChannelEnd(PCA9685_t device_Info);

    void writeRegister(PCA9685_t device_Info, uint8_t regAddress, uint8_t value);
    uint8_t readRegister(PCA9685_t device_Info, uint8_t regAddress);

    void i2cWire_begin(PCA9685_t device_Info);
    void i2cWire_beginTransmission(PCA9685_t device_Info, uint8_t);
    uint8_t i2cWire_endTransmission(PCA9685_t device_Info);
    uint8_t i2cWire_requestFrom(PCA9685_t device_Info, uint8_t, uint8_t);
    void i2cWire_write(PCA9685_t device_Info, uint8_t);
    uint8_t i2cWire_read(PCA9685_t device_Info);

    // Uses a cubic spline to interpolate due to an offsetted zero point that isn't
    // exactly between -90/+90 (or -1x/+1x). This takes more time to compute, but gives a
    // smoother PWM output value along the entire range.
    void PCA9685_ServoEval(PCA9685_t device_Info, uint16_t minPWMAmount, uint16_t midPWMAmount, uint16_t maxPWMAmount);

    // Returns the PWM value to use given the angle offset (-90 to +90)
    uint16_t pwmForAngle(PCA9685_t device_Info, float angle);

    // Returns the PWM value to use given the speed multiplier (-1 to +1)
    uint16_t pwmForSpeed(PCA9685_t device_Info, float speed);

#endif // /ifndef PCA9685_H