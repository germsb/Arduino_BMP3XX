#ifndef _bmp_i2c_h
#define _bmp_i2c_h

#include "bmp3.h"
#include "Arduino.h"
#include <Wire.h>
//#define BMP3XX_DEBUG

struct bmp_data
{
    double seaLevelPressure = 101325.0;
    double temperature = -99999;
    double pressure = -99999;
    double altitude = -99999;
};

class BMP3_I2C
{
public:
    BMP3_I2C(uint8_t addr = 0x77, TwoWire *theWire = &Wire);

    bool init();

    /**************************************************************************/
    /*!
        @brief Put the sensor in forced mode with desired settings. Call performReading() or readAltitude() to get datas.

        @param TemperatureOversampling        default BMP3_NO_OVERSAMPLING
        @param PressureOversampling           default BMP3_NO_OVERSAMPLING
        @param IIRFilter                      default BMP3_IIR_FILTER_DISABLE

        @return True on success, False on failure
    */
    /**************************************************************************/
    inline bool setSensorInForcedMode(uint8_t TemperatureOversampling = BMP3_NO_OVERSAMPLING, uint8_t PressureOversampling = BMP3_NO_OVERSAMPLING, uint8_t IIRFilter = BMP3_IIR_FILTER_DISABLE)
    {
        return setConfig(TemperatureOversampling, PressureOversampling, IIRFilter);
    };

    /**************************************************************************/
    /*!
        @brief Put the sensor in normal mode with desired settings. if success, sensor start measurement at defined rate.
        Call performReading() or readAltitude() to get datas.
        Set DataReadyInterrupt to true to listen when data ready (INT pin)

        @param TemperatureOversampling        default BMP3_NO_OVERSAMPLING
        @param PressureOversampling           default BMP3_NO_OVERSAMPLING
        @param IIRFilter                      default BMP3_IIR_FILTER_DISABLE
        @param OutputDataRate                 default BMP3_ODR_200_HZ
        @param DataReadyInterrupt             default false

        @return True on success, False on failure
    */
    /**************************************************************************/
    inline bool setSensorInNormalMode(uint8_t TemperatureOversampling = BMP3_NO_OVERSAMPLING, uint8_t PressureOversampling = BMP3_NO_OVERSAMPLING, uint8_t IIRFilter = BMP3_IIR_FILTER_DISABLE, uint8_t OutputDataRate = BMP3_ODR_200_HZ, bool DataReadyInterrupt = false)
    {
        return setConfig(TemperatureOversampling, PressureOversampling, IIRFilter, BMP3_NORMAL_MODE, OutputDataRate, DataReadyInterrupt);
    };
    /// put sensor in sleep mode
    bool setSensorInSleepMode(void);

    /// Perform a reading in blocking mode
    bool getSensorData(bmp_data *sensorData, bool computeAltitude = false);

    bool calcSeaLevelPressure(bmp_data *sensorData, double YourActualAltitude);

    double calcSeaLevelPressure(double atmosphericPressure, double YourActualAltitude);

    double calcAltitude(double atmosphericPressure, double seaLevelPressure);

protected:
    struct bmp3_dev the_sensor;
    bool initSensor(void);

private:
    bool _forcedModeEnabled;
    uint8_t i2c_addr;
    bool setConfig(
        uint8_t TemperatureOversampling = BMP3_NO_OVERSAMPLING,
        uint8_t PressureOversampling = BMP3_NO_OVERSAMPLING,
        uint8_t IIRFilter = BMP3_IIR_FILTER_DISABLE,
        uint8_t PowerMode = BMP3_FORCED_MODE,
        uint8_t OutputDataRate = BMP3_ODR_200_HZ,
        bool DataReadyInterrupt = false);
};
#endif