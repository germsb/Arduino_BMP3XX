# Arduino_BMP3XX

### Bosch BMP3XX sensors series library wrapper for Arduino

You can find more infos [here](https://github.com/BoschSensortec/BMP3-Sensor-API) and [here](https://www.bosch-sensortec.com/bst/products/all_products/bmp388).

#

This libary can be used with any bmp3XX module that support I2C or SPI communication like [adafruit BMP388](https://www.adafruit.com/product/3966), [bluedot BMP388](https://www.bluedot.space/sensor-boards/bmp388/) and probably many others.

Right now, power modes, oversampling, iir filter, output data rate and data ready interrupt (INT pin) are supported. Soon, the fifo and related interrupts will be also.

### Getting started

```cpp
...
/******* I2C ******************************/
#include <Wire.h>
#include "bmp_i2c.h"
BMP3_I2C bmp(0x76 /* default 0x77 */);
/******************************************/
/* OR */
/******* SPI ******************************/
#include <SPI.h>
#include "bmp_spi.h"
// Hardware SPI
BMP3_SPI bmp(CS_PIN);
// or software SPI
BMP3_SPI bmp(CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);
/******************************************/
// struct to store sensor datas (temp, press, alti, sea level press)
struct bmp_data sensorData;
...

void setup() {
    Serial.begin(115200);
    // init sensor and get compensated data
    bmp.init();
    // set sensor in forced mode with desired settings
    bmp.setSensorInForcedMode(BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_2X, BMP3_IIR_FILTER_COEFF_3);
}

void loop() {
    // read the sensor data, compute the altitude and store them in the structure.
    if(bmp.getSensorData(&sensorData, true)){
        // et voila
        Serial.printf("temp_C: %0.2f, press_HPA: %0.2f, alti_M: %0.2f \n",
        sensorData.temperature, sensorData.pressure / 100., sensorData.altitude);
    }else {
        Serial.println("Something wrong!");
    }
    delay(200);
}
```

### API:

```cpp
bool setSensorInForcedMode(uint8_t TemperatureOversampling = BMP3_NO_OVERSAMPLING, uint8_t PressureOversampling = BMP3_NO_OVERSAMPLING, uint8_t IIRFilter = BMP3_IIR_FILTER_DISABLE);
```

```cpp
bool setSensorInNormalMode(uint8_t TemperatureOversampling = BMP3_NO_OVERSAMPLING, uint8_t PressureOversampling = BMP3_NO_OVERSAMPLING, uint8_t IIRFilter = BMP3_IIR_FILTER_DISABLE, uint8_t OutputDataRate = BMP3_ODR_200_HZ, bool DataReadyInterrupt = false);
```

```cpp
bool setSensorInSleepMode(void);
```

- Temperature and Pressure oversampling: [available settings](https://github.com/germsb/Arduino_BMP3XX/blob/4a495248ebf08482895c5d1506509d28537e38be/bmp3_defs.h#L208-L213)

- IIR filter: [available settings](https://github.com/germsb/Arduino_BMP3XX/blob/4a495248ebf08482895c5d1506509d28537e38be/bmp3_defs.h#L216-L223)

- Output data rate: [available settings](https://github.com/germsb/Arduino_BMP3XX/blob/4a495248ebf08482895c5d1506509d28537e38be/bmp3_defs.h#L226-L243)

- DataReadyInterrupt: `true` to get notified when data are ready ([how to](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)), `false` otherwise.
  > Note: Some combinations between oversampling and outputDataRate are not possible and will return an error. This is due to the time needed to convert the data according to the oversampling setting. See [bmp388 datasheet 3.9.1](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP388-DS001.pdf)

```cpp
bool getSensorData(bmp_data *sensorData, bool computeAltitude = false);
```

```cpp
bool calcSeaLevelPressure(bmp_data *sensorData, double referenceAltitude);
```

```cpp
double calcSeaLevelPressure(double atmosphericPressure, double referenceAltitude);
```

```cpp
double calcAltitude(double atmosphericPressure, double seaLevelPressure);
```

- sensorData: Pointer to the bmp_data struct, temperature, pressure, sea level pressure and altitude are stored in.
