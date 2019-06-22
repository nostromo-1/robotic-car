/*************************************************************************

Control code for the BMP085/BMP180 sensor from Bosch
It reads temperature and atmosferic pressure

*****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <pigpio.h>

#include "bmp085.h"



#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         fprintf(stderr, "%s: " format "\n" , __func__ , ## arg);      \
         return ret;                                                   \
   }
   
  
/* i2c bus where PCF8591 is connected. 3 is a bit-bang gpio-driver.
It must be activated by adding this line to config.txt: 
dtoverlay=i2c-gpio,i2c_gpio_sda=14,i2c_gpio_scl=15,i2c_gpio_delay_us=3
*/
#define I2C_BUS 3   


static int i2c_handle = -1;

/*
Oversampling Setting
0: ultra low power, 1: standard, 2: high resolution, 3: ultra high resolution
*/
static enum {ULP=0,STD,HR,UHR} oss = UHR;

// Calibration values read from the chip
static int16_t ac1, ac2, ac3; 
static uint16_t ac4, ac5, ac6;
static int16_t b1, b2;
static int16_t mb, mc, md;

static int16_t b5; 

static int temp, press;
static const double p0 = 1013.25;     // Pressure at sea level (hPa)


static uint16_t bmp085ReadUint(int reg);
static void readSensor(void);


/*
bmp085ReadUint: it reads 2 bytes from the given register address. MSB is expected first, LSB next.
The BMP085 calibration data 0xFFFF and 0x0000 are invalid values, 
so 0xFFFF is used to signal an error read
*/
static uint16_t bmp085ReadUint(int reg)
{
int rc, msb, lsb;
   
   rc = i2cReadWordData(i2c_handle, reg);
   if (rc < 0) goto rw_error;  
   msb = rc&0x00FF;
   lsb = (rc&0xFF00) >> 8;
   
   return (msb<<8) | lsb;
   
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(0xFFFF, "Cannot read data from BMP085");      
}



int setupBMP085(int addr)
{
int rc;

   i2c_handle = i2cOpen(I2C_BUS, addr, 0);
   if (i2c_handle < 0) ERR(-1, "Cannot open BMP085 pressure sensor");  

   /* Read calibration data from sensor */ 
   ac1 = bmp085ReadUint(0xAA);
   if (ac1 == -1) goto rw_error;  
   ac2 = bmp085ReadUint(0xAC);
   if (ac2 == -1) goto rw_error; 
   ac3 = bmp085ReadUint(0xAE);
   if (ac3 == -1) goto rw_error; 
   ac4 = bmp085ReadUint(0xB0);
   if (ac4 == 0xFFFF) goto rw_error; 
   ac5 = bmp085ReadUint(0xB2);
   if (ac5 == 0xFFFF) goto rw_error; 
   ac6 = bmp085ReadUint(0xB4);
   if (ac6 == 0xFFFF) goto rw_error; 
   b1 = bmp085ReadUint(0xB6);
   if (b1 == -1) goto rw_error; 
   b2 = bmp085ReadUint(0xB8);
   if (b2 == -1) goto rw_error; 
   mb = bmp085ReadUint(0xBA);
   if (mb == -1) goto rw_error; 
   mc = bmp085ReadUint(0xBC);
   if (mc == -1) goto rw_error; 
   md = bmp085ReadUint(0xBE);
   if (md == -1) goto rw_error;  
   
   rc = gpioSetTimerFunc(4, 500, readSensor);  // read data every 500ms, timer#4
   if (rc<0) ERR(-1, "Cannot set timer for BMP085");
   
   return 0;
 
   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   return -1;
}


void closeBMP085(void)
{
   if (i2c_handle>=0) i2cClose(i2c_handle);   
   i2c_handle = -1;    
}



// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
static int16_t bmp085GetTemperature(uint16_t ut)
{
int32_t x1, x2, xx;
int16_t tmp;
  
   x1 = ((uint32_t)(ut-ac6)*(uint32_t)ac5) >> 15;
   xx = x1 + (int32_t)md;
   if (xx == 0) return -100;
   x2 = ((int32_t)mc << 11) / xx;
   b5 = x1 + x2;
   tmp = (b5 + 8)>>4;
   
   return tmp;  
}


// Calculate pressure given value up (uncalibrated pressure)
// calibration values must be known
// b5 is also required so bmp085GetTemperature must be called first.
// Value returned will be pressure in units of Pa.
static int32_t bmp085GetPressure(uint32_t up)
{
int32_t x1, x2, x3, b3, b6, p;
uint32_t b4, b7;
  
   b6 = b5 - 4000;
   // Calculate B3
   x1 = (b2 * (b6 * b6)>>12)>>11;
   x2 = (ac2 * b6)>>11;
   x3 = x1 + x2;
   b3 = (((((int32_t)ac1)*4 + x3)<<oss) + 2)>>2;
  
   x1 = ((int32_t)ac3 * b6)>>13;
   x2 = ((int32_t)b1 * ((b6 * b6)>>12))>>16;
   x3 = ((x1 + x2) + 2)>>2;
   b4 = (ac4 * (uint32_t)(x3 + 32768))>>15;
   if (b4 == 0) return -1;
  
   b7 = (up - b3) * (50000>>oss);
   if (b7 < 0x80000000) p = (b7<<1)/b4;
   else p = (b7/b4)<<1;
    
   x1 = (p>>8) * (p>>8);
   x1 = (x1 * 3038)>>16;
   x2 = (-7357 * p)>>16;
   p += (x1 + x2 + 3791)>>4;
  
   return p;
}



/*
Returns -1 in caso of error, 0 otherwise. 
Temperature: an integer value representing 10 times the temperature (e.g.: for a temperature of 15.2, 
it returns 152)
Pressure: an integer value in Pa (for mbar or hPa, divide by 100; 
you get 2 significant figures after the comma)

It takes from 11 to 32 ms to complete, depending on the value of oss (I2C bus at 100 kHz)

300 to 1100 hPa operational range
Pressure typical absolute accuracy +-1.0 hPa, maximum absolute accuracy +-2.5 hPa 
Pressure relative accuracy +-0.2 hPa at 25°C
-40 to +85°C operational range, +-2°C temperature accuracy between 0 and 65 °C
Temperature typical accuracy +-0.5°C at 25°C
*/
static void readSensor(void)
{
int rc;
uint16_t ut, xlsb=0;
uint32_t up;   
  
   rc = i2cWriteByteData(i2c_handle, 0xF4, 0x2E);  // Tell sensor to read temperature
   if (rc < 0) goto rw_error; 
   gpioDelay(4500);  // Wait 4.5 ms for the process to complete
   ut = bmp085ReadUint(0xF6);
   
   rc = i2cWriteByteData(i2c_handle, 0xF4, 0x34 + (oss<<6));  // Tell sensor to read pressure
   if (rc < 0) goto rw_error; 

   // Wait for the process to complete
   switch(oss) {
      case 0: gpioDelay(4500);
              break;
      case 1: gpioDelay(7500);
              break;
      case 2: gpioDelay(13500);
              break;
      case 3: gpioDelay(25500);
              break;     
      default: ERR(, "Invalid value (%d) for sensor mode (0..3)", oss); 
   }
   
   // Read raw result
   up = bmp085ReadUint(0xF6); 
   if (oss > 0) {
      rc = i2cReadByteData(i2c_handle, 0xF8);
      if (rc < 0) goto rw_error; 
      xlsb = rc;
      up = ((up<<8) + xlsb) >> (8-oss);
   }

   temp = bmp085GetTemperature(ut);      
   press = bmp085GetPressure(up);   

   return;
   
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(, "Cannot read/write data from BMP085");     
}



int getAtmosfericData(double *temperature, double *pressure, double *altitude)
{
   *temperature = temp/10.0;  // In degrees centigrade
   *pressure = press/100.0;   // In hPa or mbar
   *altitude = 44330.0*(1 - pow((double)press/p0, 1/5.255));  // In meters
   
   if (i2c_handle>=0) return 0;
   else return -1;
}


