#include "../bosch_api/bmp3.h"
#include "Arduino.h"
//#define BMP3XX_DEBUG

struct bmp_data
{
    double seaLevelPressure = 101325.0;
    double temperature;
    double pressure;
    double altitude;
};

class BMP3
{
public:
    BMP3();

    bool init(void);

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

    bool getSeaLevelPressure(bmp_data *sensorData, double YourActualAltitude);

protected:
    bmp3_dev the_sensor;

private:
    bool _forcedModeEnabled;
    bool setConfig(
        uint8_t TemperatureOversampling = BMP3_NO_OVERSAMPLING,
        uint8_t PressureOversampling = BMP3_NO_OVERSAMPLING,
        uint8_t IIRFilter = BMP3_IIR_FILTER_DISABLE,
        uint8_t PowerMode = BMP3_FORCED_MODE,
        uint8_t OutputDataRate = BMP3_ODR_200_HZ,
        bool DataReadyInterrupt = false);
    double getAltitude(double atmospheriquePressure, double seaLevelPressure = 101325);
};
