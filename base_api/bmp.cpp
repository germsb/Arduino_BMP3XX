#include "bmp.h"

static void delay_msec(uint32_t ms);
static void delay_msec(uint32_t ms)
{
    delay(ms);
}

BMP3::BMP3()
{
    _forcedModeEnabled = false;
}
bool BMP3::init()
{
    the_sensor.delay_ms = delay_msec;
    int8_t rslt = BMP3_OK;
    rslt = bmp3_init(&the_sensor);
#ifdef BMP3XX_DEBUG
    Serial.print("Result: ");
    Serial.println(rslt);
#endif

    if (rslt != BMP3_OK)
        return false;

#ifdef BMP3XX_DEBUG
    Serial.print("T1 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_t1);
    Serial.print("T2 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_t2);
    Serial.print("T3 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_t3);
    Serial.print("P1 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p1);
    Serial.print("P2 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p2);
    Serial.print("P3 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p3);
    Serial.print("P4 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p4);
    Serial.print("P5 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p5);
    Serial.print("P6 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p6);
    Serial.print("P7 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p7);
    Serial.print("P8 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p8);
    Serial.print("P9 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p9);
    Serial.print("P10 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p10);
    Serial.print("P11 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p11);
    //Serial.print("T lin = "); Serial.println(the_sensor.calib_data.reg_calib_data.t_lin);
#endif

    return true;
}
