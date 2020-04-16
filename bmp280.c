/*************************************************************************

Control code for the BMP280 sensor from Bosch
It reads temperature and atmosferic pressure

300 to 1100 hPa operational range
Pressure typical absolute accuracy +-1.0 hPa, maximum absolute accuracy +-2.5 hPa 
Pressure typical relative accuracy +-0.12 hPa
-40 to +85°C operational range, +-1°C temperature accuracy between 0 and 65 °C
Temperature typical accuracy +-0.5°C at 25°C

*****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <pigpio.h>

#include "oled96.h"
#include "bmp280.h"



#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         fprintf(stderr, "%s: " format "\n" , __func__ , ## arg);      \
         return ret;                                                   \
   }
   
  

#define I2C_BUS 1   


static int i2c_handle = -1;
static unsigned timerNumber; // The timer used to periodically read the sensor

/************
Define workig mode, oversampling and filter. 
Settings are done for the "Handheld device" application, described in Table 7 of the data sheet
Oversampling Setting
0: Skip, 1: ultra low power, 2: low power, 3: standard, 4: high resolution, 5: ultra high resolution
*************/

static const enum {SLEEP,FORCED,FORCED2,NORMAL} mode = NORMAL;
static const enum {SKIP,ULP,LP,STD,HR,UHR} osrs_t = LP, osrs_p = UHR;
static const enum {F_OFF,F_2,F_4,F_8,F_16} filter = F_OFF;

/* Define list of standby times for normal mode, rounded up, in milliseconds */
static const int t_standby[] = {1, 63, 125, 250, 500, 1000, 2000, 4000};  // 1 ms will not work
static const int t_sb = 3;  // The value with this index in t_standby is used


/* Compensation parameters read from the chip */
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static double pressure, temperature, p0, altitude;
static bool notFirstTime;


/* Kalman filter  variables and types */
typedef struct {
  const double err_measure;   // Measurement Uncertainty: How much do we expect to our measurement vary
  const double q;             // Process Variance: usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01
  double err_estimate;        // Estimation Uncertainty: Can be initilized with the same value as e_mea since the kalman filter will adjust its value
  double kalman_gain;
  double last_estimate, current_estimate;
} KalmanFilter_t;


static KalmanFilter_t pressureFilter = {
    .err_measure = 2.0,  
    .err_estimate = 2.0,
    .q = 0.6
};



/* Function prototypes */
static void readSensor(void);
static double updateEstimate(KalmanFilter_t *filter, double mea);



int setupBMP280(int addr, unsigned timer)
{
int rc;
char buf[24];

   i2c_handle = i2cOpen(I2C_BUS, addr, 0);
   if (i2c_handle < 0) ERR(-1, "Cannot open BMP280 sensor");  

   /* Check device ID */
   rc = i2cReadByteData(i2c_handle, 0xD0);
   if (rc < 0) ERR(-1, "Cannot read ID from BMP280 sensor");
   if (rc != 0x58) ERR(-1, "Temperature/pressure sensor is not a BMP280");
   
   /* Reset device */
   rc = i2cWriteByteData(i2c_handle, 0xE0, 0xB6);
   if (rc < 0) ERR(-1, "Cannot reset BMP280 sensor");   
   gpioDelay(2000);  // 2ms delay
   
   /* Read compensation parameters from sensor */ 
   rc = i2cReadI2CBlockData(i2c_handle, 0x88, buf, sizeof(buf));  // Burst read all parameters
   if (rc < 0) goto rw_error;

   dig_T1 = (uint16_t)buf[0] | ((uint16_t)buf[1])<<8;
   dig_T2 = (uint16_t)buf[2] | ((uint16_t)buf[3])<<8;   
   dig_T3 = (uint16_t)buf[4] | ((uint16_t)buf[5])<<8;  
   
   dig_P1 = (uint16_t)buf[6] | ((uint16_t)buf[7])<<8;
   dig_P2 = (uint16_t)buf[8] | ((uint16_t)buf[9])<<8;
   dig_P3 = (uint16_t)buf[10] | ((uint16_t)buf[11])<<8;
   dig_P4 = (uint16_t)buf[12] | ((uint16_t)buf[13])<<8;
   dig_P5 = (uint16_t)buf[14] | ((uint16_t)buf[15])<<8;
   dig_P6 = (uint16_t)buf[16] | ((uint16_t)buf[17])<<8;
   dig_P7 = (uint16_t)buf[18] | ((uint16_t)buf[19])<<8;
   dig_P8 = (uint16_t)buf[20] | ((uint16_t)buf[21])<<8;
   dig_P9 = (uint16_t)buf[22] | ((uint16_t)buf[23])<<8;
      
   /* Set operation mode */
   rc = i2cWriteByteData(i2c_handle, 0xF5, t_sb<<5 | filter<<2); // SPI bit is ignored, we work in I2C mode
   if (rc < 0) ERR(-1, "Cannot initialize BMP280 sensor");    
   
   rc = i2cWriteByteData(i2c_handle, 0xF4, osrs_t<<5 | osrs_p<<2 | mode);
   if (rc < 0) ERR(-1, "Cannot initialize BMP280 sensor");     
   
   /* Start reading thread */
   timerNumber = timer;
   rc = gpioSetTimerFunc(timerNumber, t_standby[t_sb], readSensor);  // read data every xxx ms, timer#4
   if (rc<0) ERR(-1, "Cannot set timer for BMP280");
   
   return 0;
 
   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   return -1;
}


void closeBMP280(void)
{
   gpioSetTimerFunc(timerNumber, 500, NULL);  // Stop reading thread
   i2cWriteByteData(i2c_handle, 0xF4, 0);  // Set sensor to sleep mode
   if (i2c_handle>=0) i2cClose(i2c_handle);   
   i2c_handle = -1;    
}



/*****************
The next 2 functions are the compensation formulas for temperature and pressure.
They are copied from the data sheet, chapter 3.11.3, page 22
Both pressure and temperature values are expected to be received in 20 bit format, 
positive, stored in a 32 bit signed integer.
*****************/
typedef int32_t BMP280_S32_t;
typedef uint32_t BMP280_U32_t;
typedef int64_t BMP280_S64_t;

// Return temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equal 51.23 DegC.
// t_fine carries fine temperature as global value
static BMP280_S32_t t_fine;
static BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
BMP280_S32_t var1, var2, T;
   
   var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
   var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) * ((BMP280_S32_t)dig_T3)) >> 14;
   t_fine = var1 + var2;
   T = (t_fine * 5 + 128)>>8;
   return T; 
}


// Return pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
BMP280_S64_t var1, var2, p;
   
   var1 = ((BMP280_S64_t)t_fine) - 128000;
   var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
   var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
   var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
   var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
   var1 = ((((BMP280_S64_t)1<<47)+var1)*((BMP280_S64_t)dig_P1))>>33;  
   if (var1 == 0) return 0;  // Avoid division by zero
   p = 1048576 - adc_P;
   p = (((p<<31)-var2)*3125)/var1;
   var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
   var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
   p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
   return (BMP280_U32_t)p;
}




static void readSensor(void)
{
int rc;
uint32_t raw_p, raw_t;
char buf[6], str[17];   
double p;
static char str_old[17];

   rc = i2cReadI2CBlockData(i2c_handle, 0xF7, buf, sizeof(buf));  // Burst read temperature and pressure values
   if (rc < 0) ERR(, "Cannot read data from BMP280"); 
   raw_p = ((uint32_t)buf[0]<<12) | ((uint32_t)buf[1]<<4) | ((uint32_t)buf[2]>>4);
   raw_t = ((uint32_t)buf[3]<<12) | ((uint32_t)buf[4]<<4) | ((uint32_t)buf[5]>>4);

   // Calculate compensated values. Temperature must be calculated before pressure
   temperature = bmp280_compensate_T_int32((int32_t)raw_t)/100.0;  // In degrees centigrade  
   p = bmp280_compensate_P_int64((int32_t)raw_p)/25600.0;   // In hPa or mbar
   if (p == 0) ERR(, "Invalid data read from BMP280"); 
   //printf("P:%.2f\n", p);
   
   if (!notFirstTime) {  
      pressure = p0 = p;   // The first time we read the sensor, we save the pressure as reference in p0
      pressureFilter.last_estimate = p;  // Initialize Kalman filter
      notFirstTime = true;
   }
   else pressure = updateEstimate(&pressureFilter, p);  // Kalman filter
      
   altitude = round(10*8.43 * (p0 - pressure))/10;  // Valid near sea level
   
   //printf("T:%.2f P:%.2f A:%.1f\n", temperature, pressure, altitude);
   
   // Update display only if values changed (it is a slow operation)
   snprintf(str, sizeof(str), "%.1fC %.1fhPa", temperature, pressure);
   if (strcmp(str, str_old)) {
      oledWriteString(0, 7, str, false);
      strcpy(str_old, str);
   }  
      
   return;
}



int getAtmosfericData(double *t, double *p, double *a)
{
   *t = temperature;
   *p = pressure;
   *a = altitude;
   
   return 0;
}



/********
Kalman filter for single variable model
Based on the following software

Copyright (c) 2017 Denys Sene

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
**********/



static double updateEstimate(KalmanFilter_t *filter, double mea)
{
  filter->kalman_gain = filter->err_estimate/(filter->err_estimate + filter->err_measure);
  filter->current_estimate = filter->last_estimate + filter->kalman_gain * (mea - filter->last_estimate);
  filter->err_estimate =  (1.0 - filter->kalman_gain)*filter->err_estimate + filter->q * fabs(filter->last_estimate - filter->current_estimate);
  filter->last_estimate = filter->current_estimate;

  return filter->current_estimate;
}





