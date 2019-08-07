#ifndef _bmp_spi_h
#define _bmp_spi_h

#include "bmp_i2c.h"
#include <SPI.h>

#define BMP3XX_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

class BMP3_SPI : public BMP3_I2C
{
private:
    int8_t _cs;

public:
    BMP3_SPI(int8_t cspin, int8_t mosipin = -1, int8_t misopin = -1, int8_t sckpin = -1);
    bool init();
};
#endif