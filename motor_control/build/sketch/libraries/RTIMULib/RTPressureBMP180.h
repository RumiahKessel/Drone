#line 1 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\libraries\\RTIMULib\\RTPressureBMP180.h"
////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTPRESSUREBMP180_H_
#define _RTPRESSUREBMP180_H_

#include "RTPressure.h"

//  State definitions

#define BMP180_STATE_IDLE               0
#define BMP180_STATE_TEMPERATURE        1
#define BMP180_STATE_PRESSURE           2

//  Conversion reg defs

#define BMP180_SCO_TEMPCONV             0x2e                // temperature conversion
#define BMP180_SCO_PRESSURECONV_ULP     0                   // ultra low power pressure conversion
#define BMP180_SCO_PRESSURECONV_STD     1                   // standard pressure conversion
#define BMP180_SCO_PRESSURECONV_HR      2                   // high res pressure conversion
#define BMP180_SCO_PRESSURECONV_UHR     3                   // ultra high res pressure conversion

class RTIMUSettings;

class RTPressureBMP180 : public RTPressure
{
public:
    RTPressureBMP180(RTIMUSettings *settings);
    ~RTPressureBMP180();

    virtual const char *pressureName() { return "BMP180"; }
    virtual int pressureType() { return RTPRESSURE_TYPE_BMP180; }
    virtual bool pressureInit();
    virtual bool pressureRead(float &latestPressure, float &latestTemperature);

private:
    void pressureBackground();
    void setTestData();

    unsigned char m_pressureAddr;                           // I2C address
    RTFLOAT m_pressure;                                     // the current pressure
    RTFLOAT m_temperature;                                  // the current temperature

    // This is the calibration data read from the sensor

    int16_t m_AC1;
    int16_t m_AC2;
    int16_t m_AC3;
    uint16_t m_AC4;
    uint16_t m_AC5;
    uint16_t m_AC6;
    int16_t m_B1;
    int16_t m_B2;
    int16_t m_MB;
    int16_t m_MC;
    int16_t m_MD;

    int m_state;
    int m_oss;

    int32_t m_rawPressure;
    int32_t m_rawTemperature;

    bool m_validReadings;
};

#endif // _RTPRESSUREBMP180_H_

