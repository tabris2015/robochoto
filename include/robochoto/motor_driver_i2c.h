
#ifndef MOTOR_DRIVER_I2C_H
#define MOTOR_DRIVER_I2C_H

#include "RTIMUSettings.h"

#define REQ_WRITE_BYTE   0x34

#define MAX_LINEAR 0.75

#define MAX_ANGULAR 0.75

class RTIMUSettings;


class MotorDriverI2c
{
private:
    /* data */
    RTIMUSettings * m_settings;
    unsigned char address_;
    char error_buffer_[50];
    unsigned char linear_index_;
    unsigned char angular_index_;
    unsigned char req_byte_;
    float linear_;
    float angular_;
    int linear_bytes_;
    int angular_bytes_;
    
    bool WriteToDriver(unsigned char * data, unsigned char size);
public:
    MotorDriverI2c(RTIMUSettings *settings, unsigned char address);

    bool SetUnicycleVelocities(float linear, float angular);
    
    ~MotorDriverI2c();
};



/*
class RTHumidityHTS221 : public RTHumidity
{
public:
    RTHumidityHTS221(RTIMUSettings *settings);
    ~RTHumidityHTS221();

    virtual const char *humidityName() { return "HTS221"; }
    virtual int humidityType() { return RTHUMIDITY_TYPE_HTS221; }
    virtual bool humidityInit();
    virtual bool humidityRead(RTIMU_DATA& data);

private:
    unsigned char m_humidityAddr;                           // I2C address

    RTFLOAT m_humidity;                                     // the current humidity
    RTFLOAT m_temperature;                                  // the current temperature
    RTFLOAT m_temperature_m;                                // temperature calibration slope
    RTFLOAT m_temperature_c;                                // temperature calibration y intercept
    RTFLOAT m_humidity_m;                                   // humidity calibration slope
    RTFLOAT m_humidity_c;                                   // humidity calibration y intercept
    bool m_humidityValid;
    bool m_temperatureValid;

};

*/

#endif // MOTOR_DRIVER_I2C_H

