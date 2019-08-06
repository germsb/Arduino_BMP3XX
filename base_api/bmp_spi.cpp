#include "../bmp_spi.h"

int8_t _BMP3_SoftwareSPI_MOSI; ///< Global SPI MOSI pin
int8_t _BMP3_SoftwareSPI_MISO; ///< Global SPI MISO pin
int8_t _BMP3_SoftwareSPI_SCK;  ///< Global SPI Clock pin

static int8_t spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int8_t spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static uint8_t spi_transfer(uint8_t x);

BMP3_SPI::BMP3_SPI(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
{
  _cs = cspin;
  _BMP3_SoftwareSPI_MOSI = mosipin;
  _BMP3_SoftwareSPI_MISO = misopin;
  _BMP3_SoftwareSPI_SCK = sckpin;
}

bool BMP3_SPI::init()
{
  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);

  if (_BMP3_SoftwareSPI_SCK == -1)
  {
    // hardware SPI
    SPI.begin();
  }
  else
  {
    // software SPI
    pinMode(_BMP3_SoftwareSPI_SCK, OUTPUT);
    pinMode(_BMP3_SoftwareSPI_MOSI, OUTPUT);
    pinMode(_BMP3_SoftwareSPI_MISO, INPUT);
  }

  the_sensor.dev_id = _cs;
  the_sensor.intf = BMP3_SPI_INTF;
  the_sensor.read = &spi_read;
  the_sensor.write = &spi_write;
  return BMP3_I2C::initSensor();
}

/**************************************************************************/
/*!
    @brief  Reads 8 bit values over SPI
*/
/**************************************************************************/
static int8_t spi_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
#ifdef BMP3XX_DEBUG
  Serial.print("\tSPI $");
  Serial.print(reg_addr, HEX);
  Serial.print(" => ");
#endif

  // If hardware SPI we should use transactions!
  if (_BMP3_SoftwareSPI_SCK == -1)
  {
    SPI.beginTransaction(SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr | 0x80);

  while (len--)
  {
    *reg_data = spi_transfer(0x00);
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }

  digitalWrite(cspin, HIGH);

  if (_BMP3_SoftwareSPI_SCK == -1)
  {
    SPI.endTransaction();
  }

#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over SPI
*/
/**************************************************************************/
static int8_t spi_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
#ifdef BMP3XX_DEBUG
  Serial.print("\tSPI $");
  Serial.print(reg_addr, HEX);
  Serial.print(" <= ");
#endif

  // If hardware SPI we should use transactions!
  if (_BMP3_SoftwareSPI_SCK == -1)
  {
    SPI.beginTransaction(SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr);
  while (len--)
  {
    spi_transfer(*reg_data);
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }

  digitalWrite(cspin, HIGH);

  if (_BMP3_SoftwareSPI_SCK == -1)
  {
    SPI.endTransaction();
  }

#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

static uint8_t spi_transfer(uint8_t x)
{
  if (_BMP3_SoftwareSPI_SCK == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--)
  {
    reply <<= 1;
    digitalWrite(_BMP3_SoftwareSPI_SCK, LOW);
    digitalWrite(_BMP3_SoftwareSPI_MOSI, x & (1 << i));
    digitalWrite(_BMP3_SoftwareSPI_SCK, HIGH);
    if (digitalRead(_BMP3_SoftwareSPI_MISO))
      reply |= 1;
  }
  return reply;
}