# Arduino_BMP3XX

### Bosch BMP3XX sensors series library wrapper for Arduino

You can find more infos about these sensors [here](https://github.com/BoschSensortec/BMP3-Sensor-API) and [here](https://www.bosch-sensortec.com/bst/products/all_products/bmp388).

## Getting started

```cpp
#include <Arduino.h>
#include <.h>
/******* I2C *********/
#include <Wire.h>
#include "bmp_i2c"
BMP3_I2C bmp(0x76 /* default 0x77 */);
/*********************/
/*       OR          */
/******* SPI *********/
#include <SPI.h>
#include "bmp_spi"
BMP3_SPI bmp(CS_PIN); //Hardware SPI
BMP3_SPI bmp(CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN); // software SPI
/*********************/
struct bmp_data sensorData;
...
void setup() {
    Serial.begin(115200);
    bmp.init();
    bmp.setSensorInForcedMode(BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_2X, BMP3_IIR_FILTER_COEFF_7);
}

void loop() {
    // get pressure, temperature and altitude
    if(bmp.getSensorData(&sensorData, true)){
        Serial.printf("temp_C: %0.2f, press_HPA: %0.2f, alti_M: %0.2f \n",
        sensorData.temperature, sensorData.pressure / 100., sensorData.altitude);
    }else {
        Serial.println("Something wrong");
    }
    delay(200);
}
```
