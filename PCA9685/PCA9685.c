/*  Arduino Library for the PCA9685 16-Channel PWM Driver Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    Copyright (C) 2012 Kasper Skårhøj       <kasperskaarhoj@gmail.com>
    PCA9685 Main
*/

#include "PCA9685.h"

#define PCA9685_I2C_BASE_MODULE_ADDRESS (uint8_t)0x40
#define PCA9685_I2C_BASE_MODULE_ADRMASK (uint8_t)0x3F
#define PCA9685_I2C_BASE_PROXY_ADDRESS  (uint8_t)0xE0
#define PCA9685_I2C_BASE_PROXY_ADRMASK  (uint8_t)0xFE

// Register addresses from data sheet
#define PCA9685_MODE1_REG               (uint8_t)0x00
#define PCA9685_MODE2_REG               (uint8_t)0x01
#define PCA9685_SUBADR1_REG             (uint8_t)0x02
#define PCA9685_SUBADR2_REG             (uint8_t)0x03
#define PCA9685_SUBADR3_REG             (uint8_t)0x04
#define PCA9685_ALLCALL_REG             (uint8_t)0x05
#define PCA9685_LED0_REG                (uint8_t)0x06          // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG            (uint8_t)0xFE
#define PCA9685_ALLLED_REG              (uint8_t)0xFA

// Mode1 register values
#define PCA9685_MODE1_RESTART           (uint8_t)0x80
#define PCA9685_MODE1_EXTCLK            (uint8_t)0x40
#define PCA9685_MODE1_AUTOINC           (uint8_t)0x20
#define PCA9685_MODE1_SLEEP             (uint8_t)0x10
#define PCA9685_MODE1_SUBADR1           (uint8_t)0x08
#define PCA9685_MODE1_SUBADR2           (uint8_t)0x04
#define PCA9685_MODE1_SUBADR3           (uint8_t)0x02
#define PCA9685_MODE1_ALLCALL           (uint8_t)0x01

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE      (uint8_t)0x04
#define PCA9685_MODE2_INVRT             (uint8_t)0x10
#define PCA9685_MODE2_OUTNE_TPHIGH      (uint8_t)0x01
#define PCA9685_MODE2_OUTNE_HIGHZ       (uint8_t)0x02
#define PCA9685_MODE2_OCH_ONACK         (uint8_t)0x08

#define PCA9685_SW_RESET                (uint8_t)0x06          // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL                (uint16_t)0x1000    // Special value for full on/full off LEDx modes
#define PCA9685_PWM_MASK                (uint16_t)0x0FFF    // Mask for 12-bit/4096 possible phase positions

#define PCA9685_CHANNEL_COUNT           16
#define PCA9685_MIN_CHANNEL             0
#define PCA9685_MAX_CHANNEL             (PCA9685_CHANNEL_COUNT - 1)
#define PCA9685_ALLLED_CHANNEL          -1                  // Special value for ALLLED registers

uint16_t constrain(uint16_t x, uint16_t a, uint16_t b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

void resetDevices(PCA9685_t device_Info) {

    i2cWire_begin(device_Info);

    i2cWire_beginTransmission(device_Info, 0x00);
    i2cWire_write(device_Info, PCA9685_SW_RESET);
    i2cWire_endTransmission(device_Info);

    _delay_ms(10);
}

void initPCA9685(PCA9685_t device_Info) {

    if (device_Info._isProxyAddresser) return;

    device_Info._driverMode = PCA9685_OutputDriverMode_TotemPole;
    device_Info._enabledMode = PCA9685_OutputEnabledMode_Normal;
    device_Info._disabledMode = PCA9685_OutputDisabledMode_Low;
    device_Info._updateMode = PCA9685_ChannelUpdateMode_AfterStop;
    device_Info._phaseBalancer = PCA9685_PhaseBalancer_None;

    device_Info._readBytes = 0;

    assert(!(device_Info._driverMode == PCA9685_OutputDriverMode_OpenDrain && device_Info._disabledMode == PCA9685_OutputDisabledMode_High && "Unsupported combination"));

    uint8_t mode2Val = getMode2Value(device_Info);

    i2cWire_begin(device_Info);

    writeRegister(device_Info, PCA9685_MODE1_REG, PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);
    writeRegister(device_Info, PCA9685_MODE2_REG, mode2Val);
}

void initAsProxyAddresser(PCA9685_t device_Info) {
    if (device_Info._driverMode != PCA9685_OutputDriverMode_Undefined) return;

    device_Info._i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (device_Info._i2cAddress & PCA9685_I2C_BASE_PROXY_ADRMASK);
    device_Info._isProxyAddresser = true;

    i2cWire_begin(device_Info);

}

uint8_t getMode2Value(PCA9685_t device_Info) {
    uint8_t mode2Val = (uint8_t)0x00;

    if (device_Info._driverMode == PCA9685_OutputDriverMode_TotemPole) {
        mode2Val |= PCA9685_MODE2_OUTDRV_TPOLE;
    }

    if (device_Info._enabledMode == PCA9685_OutputEnabledMode_Inverted) {
        mode2Val |= PCA9685_MODE2_INVRT;
    }

    if (device_Info._disabledMode == PCA9685_OutputDisabledMode_High) {
        mode2Val |= PCA9685_MODE2_OUTNE_TPHIGH;
    }
    else if (device_Info._disabledMode == PCA9685_OutputDisabledMode_Floating) {
        mode2Val |= PCA9685_MODE2_OUTNE_HIGHZ;
    }

    if (device_Info._updateMode == PCA9685_ChannelUpdateMode_AfterAck) {
        mode2Val |= PCA9685_MODE2_OCH_ONACK;
    }

    return mode2Val;
}

void setPWMFrequency(PCA9685_t device_Info, float pwmFrequency) {
    if (pwmFrequency < 0 || device_Info._isProxyAddresser) return;

    // This equation comes from section 7.3.5 of the datasheet, but the rounding has been
    // removed because it isn't needed. Lowest freq is 23.84, highest is 1525.88.
    int preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(device_Info, PCA9685_PRESCALE_REG, (uint8_t)preScalerVal);

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    _delay_ms(500);
}

void setPWMFreqServo(PCA9685_t device_Info) {
    setPWMFrequency(device_Info, 50);
}

void setChannelOn(PCA9685_t device_Info, int channel) {
    if (channel < 0 || channel > 15) return;

    writeChannelBegin(device_Info, channel);
    writeChannelPWM(device_Info, PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    writeChannelEnd(device_Info);
}

void setChannelOff(PCA9685_t device_Info, int channel) {
    if (channel < 0 || channel > 15) return;

    writeChannelBegin(device_Info, channel);
    writeChannelPWM(device_Info, 0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    writeChannelEnd(device_Info);
}

void setChannelPWM(PCA9685_t device_Info, int channel, uint16_t pwmAmount) {
    if (channel < 0 || channel > 15) return;

    writeChannelBegin(device_Info, channel);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(device_Info, channel, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(device_Info, phaseBegin, phaseEnd);

    writeChannelEnd(device_Info);
}

void setChannelsPWM(PCA9685_t device_Info, int begChannel, int numChannels, const uint16_t *pwmAmounts) {
    if (begChannel < 0 || begChannel > 15 || numChannels < 0) return;
    if (begChannel + numChannels > 16) numChannels -= (begChannel + numChannels) - 16;

    // From avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers. I2C_BUFFER_LENGTH is used in
    // other architectures, so we rely on PCA9685_I2C_BUFFER_LENGTH logic to sort it out.

    while (numChannels > 0) {
        writeChannelBegin(device_Info, begChannel);

        int maxChannels = numChannels;

        while (maxChannels-- > 0) {
            uint16_t phaseBegin, phaseEnd;
            getPhaseCycle(device_Info, begChannel++, *pwmAmounts++, &phaseBegin, &phaseEnd);

            writeChannelPWM(device_Info, phaseBegin, phaseEnd);
            --numChannels;
        }

        writeChannelEnd(device_Info);
        if (device_Info._lastI2CError) return;
    }
}

void setAllChannelsPWM(PCA9685_t device_Info, uint16_t pwmAmount) {

    writeChannelBegin(device_Info, PCA9685_ALLLED_CHANNEL);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(device_Info, PCA9685_ALLLED_CHANNEL, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(device_Info, phaseBegin, phaseEnd);

    writeChannelEnd(device_Info);
}

uint16_t getChannelPWM(PCA9685_t device_Info, int channel) {
    if (channel < 0 || channel > 15 || device_Info._isProxyAddresser) return 0;

    uint8_t regAddress = PCA9685_LED0_REG + (channel << 2);

    i2cWire_beginTransmission(device_Info, device_Info._i2cAddress);
    i2cWire_write(device_Info, regAddress);
    if (i2cWire_endTransmission(device_Info)) {
        return 0;
    }

    int bytesRead = i2cWire_requestFrom(device_Info, (uint8_t)device_Info._i2cAddress, 4);
    if (bytesRead != 4) {
        while (bytesRead-- > 0)
            i2cWire_read(device_Info);
        device_Info._lastI2CError = 4;
        return 0;
    }

#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
    uint16_t phaseBegin = (uint16_t)i2cWire_read(device_Info);
    phaseBegin |= (uint16_t)i2cWire_read(device_Info) << 8;
    uint16_t phaseEnd = (uint16_t)i2cWire_read(device_Info);
    phaseEnd |= (uint16_t)i2cWire_read(device_Info) << 8;
#else
    uint16_t phaseEnd = (uint16_t)i2cWire_read(device_Info);
    phaseEnd |= (uint16_t)i2cWire_read(device_Info) << 8;
    uint16_t phaseBegin = (uint16_t)i2cWire_read(device_Info);
    phaseBegin |= (uint16_t)i2cWire_read(device_Info) << 8;
#endif

    // See datasheet section 7.3.3
    uint16_t retVal;
    if (phaseEnd >= PCA9685_PWM_FULL)
        // Full OFF
        // Figure 11 Example 4: full OFF takes precedence over full ON
        // See also remark after Table 7
        retVal = 0;
    else if (phaseBegin >= PCA9685_PWM_FULL)
        // Full ON
        // Figure 9 Example 3
        retVal = PCA9685_PWM_FULL;
    else if (phaseBegin <= phaseEnd)
        // start and finish in same cycle
        // Section 7.3.3 example 1
        retVal = phaseEnd - phaseBegin;
    else
        // span cycles
        // Section 7.3.3 example 2
        retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin;

    return retVal;
}

void enableAllCallAddress(PCA9685_t device_Info, uint8_t i2cAddressAllCall) {
    if (device_Info._isProxyAddresser) return;

    uint8_t i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressAllCall & PCA9685_I2C_BASE_PROXY_ADRMASK);

    writeRegister(device_Info, PCA9685_ALLCALL_REG, i2cAddress);

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_ALLCALL));
}

void enableSub1Address(PCA9685_t device_Info, uint8_t i2cAddressSub1) {
    if (device_Info._isProxyAddresser) return;

    uint8_t i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub1 & PCA9685_I2C_BASE_PROXY_ADRMASK);

    writeRegister(device_Info, PCA9685_SUBADR1_REG, i2cAddress);

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR1));
}

void enableSub2Address(PCA9685_t device_Info, uint8_t i2cAddressSub2) {
    if (device_Info._isProxyAddresser) return;

    uint8_t i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub2 & PCA9685_I2C_BASE_PROXY_ADRMASK);

    writeRegister(device_Info, PCA9685_SUBADR2_REG, i2cAddress);

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR2));
}

void enableSub3Address(PCA9685_t device_Info, uint8_t i2cAddressSub3) {
    if (device_Info._isProxyAddresser) return;

    uint8_t i2cAddress = PCA9685_I2C_BASE_PROXY_ADDRESS | (i2cAddressSub3 & PCA9685_I2C_BASE_PROXY_ADRMASK);

    writeRegister(device_Info, PCA9685_SUBADR3_REG, i2cAddress);

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_SUBADR3));
}

void disableAllCallAddress(PCA9685_t device_Info) {
    if (device_Info._isProxyAddresser) return;

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_ALLCALL));
}

void disableSub1Address(PCA9685_t device_Info) {
    if (device_Info._isProxyAddresser) return;

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR1));
}

void disableSub2Address(PCA9685_t device_Info) {
    if (device_Info._isProxyAddresser) return;

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR2));
}

void disableSub3Address(PCA9685_t device_Info) {
    if (device_Info._isProxyAddresser) return;

    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE1_SUBADR3));
}

void enableExtClockLine(PCA9685_t device_Info) {

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    uint8_t mode1Reg = readRegister(device_Info, PCA9685_MODE1_REG);
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE1_EXTCLK));

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(device_Info, PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    _delay_ms(500);
}

uint8_t getLastI2CError(PCA9685_t device_Info) {
    return device_Info._lastI2CError;
}

void getPhaseCycle(PCA9685_t device_Info, int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
    if (channel == PCA9685_ALLLED_CHANNEL) {
        *phaseBegin = 0; // ALLLED should not receive a phase shifted begin value
    } else {
        // Get phase delay begin
        switch(device_Info._phaseBalancer) {
            case PCA9685_PhaseBalancer_None:
            case PCA9685_PhaseBalancer_Count:
            case PCA9685_PhaseBalancer_Undefined:
                *phaseBegin = 0;
                break;

            case PCA9685_PhaseBalancer_Linear:
                // Distribute high phase area over more of the duty cycle range to balance load
                *phaseBegin = (channel * ((4096 / 16) / 16)) & PCA9685_PWM_MASK;
                break;
        }
    }

    // See datasheet section 7.3.3
    if (pwmAmount == 0) {
        // Full OFF -> time_end[bit12] = 1
        *phaseEnd = PCA9685_PWM_FULL;
    }
    else if (pwmAmount >= PCA9685_PWM_FULL) {
        // Full ON -> time_beg[bit12] = 1, time_end[bit12] = <ignored>
        *phaseBegin |= PCA9685_PWM_FULL;
        *phaseEnd = 0;
    }
    else {
        *phaseEnd = (*phaseBegin + pwmAmount) & PCA9685_PWM_MASK;
    }
}

void writeChannelBegin(PCA9685_t device_Info, int channel) {
    uint8_t regAddress;

    if (channel != PCA9685_ALLLED_CHANNEL)
        regAddress = PCA9685_LED0_REG + (channel * 0x04);
    else
        regAddress = PCA9685_ALLLED_REG;

    i2cWire_beginTransmission(device_Info, device_Info._i2cAddress);
    i2cWire_write(device_Info, regAddress);
}

void writeChannelPWM(PCA9685_t device_Info, uint16_t phaseBegin, uint16_t phaseEnd) {

#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
    i2cWire_write(device_Info, (uint8_t)phaseBegin);
    i2cWire_write(device_Info, (uint8_t)((uint16_t)phaseBegin >> 8));
    i2cWire_write(device_Info, (uint8_t)phaseEnd);
    i2cWire_write(device_Info, (uint8_t)((uint16_t)phaseEnd >> 8));
#else
    i2cWire_write(device_Info, (uint8_t)phaseEnd);
    i2cWire_write(device_Info, (uint8_t)((uint16_t)phaseEnd >> 8));
    i2cWire_write(device_Info, (uint8_t)phaseBegin);
    i2cWire_write(device_Info, (uint8_t)((uint16_t)phaseBegin >> 8));
#endif
}

void writeChannelEnd(PCA9685_t device_Info) {
    i2cWire_endTransmission(device_Info);
}

void writeRegister(PCA9685_t device_Info, uint8_t regAddress, uint8_t value) {
    i2cWire_beginTransmission(device_Info, device_Info._i2cAddress);
    i2cWire_write(device_Info, regAddress);
    i2cWire_write(device_Info, value);
    i2cWire_endTransmission(device_Info);

}

uint8_t readRegister(PCA9685_t device_Info, uint8_t regAddress) {

    i2cWire_beginTransmission(device_Info, device_Info._i2cAddress);
    i2cWire_write(device_Info, regAddress);
    if (i2cWire_endTransmission(device_Info)) {
        return 0;
    }

    int bytesRead = i2cWire_requestFrom(device_Info, (uint8_t)device_Info._i2cAddress, 1);
    if (bytesRead != 1) {
        while (bytesRead-- > 0)
            i2cWire_read(device_Info);
        device_Info._lastI2CError = 4;
        return 0;
    }

    uint8_t retVal = i2cWire_read(device_Info);

    i2cWire_endTransmission(device_Info);

    return retVal;
}

void i2cWire_begin(PCA9685_t device_Info)
{
    i2c_init(device_Info._twi, TWI_BAUD(device_Info._f_sys,BAUD_400K));
}

void i2cWire_beginTransmission(PCA9685_t device_Info, uint8_t addr) {
    i2c_start(device_Info._twi, addr, I2C_WRITE);
}

uint8_t i2cWire_endTransmission(PCA9685_t device_Info) {
    i2c_stop(device_Info._twi); // Manually have to send stop bit in software i2c mode
    device_Info._lastI2CError = 0;
    return device_Info._lastI2CError;
}

uint8_t i2cWire_requestFrom(PCA9685_t device_Info, uint8_t addr, uint8_t len) {
    i2c_start(device_Info._twi, addr, I2C_READ);
    device_Info._readBytes = len;
    return device_Info._readBytes;
}

void i2cWire_write(PCA9685_t device_Info, uint8_t data) {
    i2c_write(device_Info._twi, data);
}

uint8_t i2cWire_read(PCA9685_t device_Info) {
    if (device_Info._readBytes > 1) {
        device_Info._readBytes -= 1;
        return (uint8_t)(i2c_read(device_Info._twi, I2C_NACK) & 0xFF);
    }
    else {
        device_Info._readBytes = 0;
        return (uint8_t)(i2c_read(device_Info._twi, I2C_ACK) & 0xFF);
    }
}

void PCA9685_ServoEval(PCA9685_t device_Info, uint16_t minPWMAmount, uint16_t midPWMAmount, uint16_t maxPWMAmount)
{
    minPWMAmount = fmin(minPWMAmount, PCA9685_PWM_FULL);
    midPWMAmount = constrain(midPWMAmount, minPWMAmount, PCA9685_PWM_FULL);
    maxPWMAmount = constrain(maxPWMAmount, midPWMAmount, PCA9685_PWM_FULL);

    if (maxPWMAmount - midPWMAmount != midPWMAmount - minPWMAmount) {
        memset(device_Info._coeff, 0, sizeof(*device_Info._coeff));
        device_Info._isCSpline = true;

        // Cubic spline code adapted from: https://shiftedbits.org/2011/01/30/cubic-spline-interpolation/
        /* "THE BEER-WARE LICENSE" (Revision 42): Devin Lane wrote this [part]. As long as you retain
        * this notice you can do whatever you want with this stuff. If we meet some day, and you
        * think this stuff is worth it, you can buy me a beer in return. */
       // TODO: Looks like I owe Devin Lane a beer. -NR

        float x[3] = { 0, 90, 180 };
        float y[3] = { (float)minPWMAmount, (float)midPWMAmount, (float)maxPWMAmount };
        float c[3], b[2], d[2], h[2], l[1], u[2], a[1], z[2]; // n = 3

        h[0] = x[1] - x[0];
        u[0] = z[0] = 0;
        c[2] = 0;

        for (int i = 1; i < 2; ++i) {
            h[i] = x[i + 1] - x[i];
            l[i - 1] = (2 * (x[i + 1] - x[i - 1])) - h[i - 1] * u[i - 1];
            u[i] = h[i] / l[i - 1];
            a[i - 1] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
            z[i] = (a[i - 1] - h[i - 1] * z[i - 1]) / l[i - 1];
        }

        for (int i = 1; i >= 0; --i) {
            c[i] = z[i] - u[i] * c[i + 1];
            b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] * (c[i + 1] + 2 * c[i])) / 3;
            d[i] = (c[i + 1] - c[i]) / (3 * h[i]);

            device_Info._coeff[4 * i + 0] = y[i]; // a
            device_Info._coeff[4 * i + 1] = b[i]; // b
            device_Info._coeff[4 * i + 2] = c[i]; // c
            device_Info._coeff[4 * i + 3] = d[i]; // d
        }
    }
    else {
        memset(device_Info._coeff, 0, sizeof(*device_Info._coeff));
        device_Info._isCSpline = false;

        device_Info._coeff[0] = minPWMAmount;
        device_Info._coeff[1] = (maxPWMAmount - minPWMAmount) / 180.0f;
    }
}

uint16_t pwmForAngle(PCA9685_t device_Info, float angle) {
    float retVal;
    angle = constrain(angle + 90, 0, 180);

    if (!device_Info._isCSpline) {
        retVal = device_Info._coeff[0] + (device_Info._coeff[1] * angle);
    }
    else {
        if (angle <= 90) {
            retVal = device_Info._coeff[0] + (device_Info._coeff[1] * angle) + (device_Info._coeff[2] * angle * angle) + (device_Info._coeff[3] * angle * angle * angle);
        }
        else {
            angle -= 90;
            retVal = device_Info._coeff[4] + (device_Info._coeff[5] * angle) + (device_Info._coeff[6] * angle * angle) + (device_Info._coeff[7] * angle * angle * angle);
        }
    }

    return (uint16_t)fmin((uint16_t)roundf(retVal), PCA9685_PWM_FULL);
};

uint16_t pwmForSpeed(PCA9685_t device_Info, float speed) {
    return pwmForAngle(device_Info, speed * 90.0f);
}