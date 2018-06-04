/*************************************************************************

NXP PCF8591 control code.
This chip is an A/D converter with 4 channels and D/A converter with 1 channel.

*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <pigpio.h>


#include "pcf8591.h"
#include "oled96.h"


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
static double voltage, current;


// 3.3 is the voltage reference, 255 are the steps (8 bits ADC resolution)
// 22000 and 12100 are the precision (1%) resistors in series connected to ADC#1 for battery voltage
static const double factor_v = 3.3/255*(22000+12100)/12100;  
// 1100 and 100 are the precision (1%) resistors in the current sensing circuit connected to ADC#2
// 0.1 is the sensing resistor (1%)
static const double factor_i = 3.3/255*(0.1*1100/100);  


int setupPCF8591(int addr)
{
int rc, byte;

   i2c_handle = i2cOpen(I2C_BUS, addr, 0);
   if (i2c_handle < 0) ERR(-1, "Cannot open PCF8591 ADC. No power supply checks.");   
  
   /*  We have to set the chip to reading channel 0 and not increment, so that we have
   a defined state to begin with */
   byte = 0; 
   rc = i2cWriteByte(i2c_handle, byte);  
   if (rc < 0) goto rw_error; 
   gpioDelay(100000);  // After writing a control byte, wait for 0.1 sec 
   rc = i2cReadByte(i2c_handle);  // Read previous conversion and ignore it
   if (rc < 0) goto rw_error;  
   gpioDelay(10000);

   /* Now, set increment flag and start reading with channel 1, 
   as channel 0 was triggered before and will be read by next bus read */
   byte = 5;
   rc = i2cWriteByte(i2c_handle, byte);  
   if (rc < 0) goto rw_error;  
   gpioDelay(100000);   

   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot read/write data from PCF8591");   
}



double getMainPowerValue(void)
{
   return voltage;
}



/* 
This function gets called at fixed intervals. It reads the ADC#1, connected to the main power supply.
It displays a symbol in the display according to the battery status.
*/
void checkPowerPCF(void)
{
uint8_t emptybatt_glyph[] = {0, 254, 130, 131, 131, 130, 254, 0};
int rc, step;
char adc[4];  // Read all 4 channels of ADC
char str[17];
static int n, old_step = -1;
static double old_v=-1, old_i=-1;

   rc = i2cReadDevice(i2c_handle, adc, sizeof(adc));
   if (rc < 0) goto rw_error;

   voltage = factor_v*adc[0];
   //printf("voltage=%.1f V\n", voltage);
   
   current = factor_i*adc[1];  
   //printf("current=%.0f mA\n", 1000*current); 
   
   if (voltage < 6.2) step = 0;        // Battery at 0%
   else if (voltage < 6.6) step = 64;  // Battery at 20%
   else if (voltage < 7.0) step = 64+32;      // Battery at 40%  
   else if (voltage < 7.4) step = 64+32+16;   // Battery at 60%
   else if (voltage < 7.8) step = 64+32+16+8; // Battery at 80%
   else step = 64+32+16+8+4;  // Battery at 100%
    
   // If battery state changed, update battery symbol on display
   if (step!=old_step) {
      emptybatt_glyph[2] += step;
      emptybatt_glyph[3] += step;
      emptybatt_glyph[4] = emptybatt_glyph[3];
      emptybatt_glyph[5] = emptybatt_glyph[2];      
      oledSetBitmap8x8(14*8, 0, emptybatt_glyph);  // ca. 0.8 ms 
      old_step = step;
   }
   
   // Symbol blinks when battery low
   if (step<=64) { 
      if (n++&1) oledSetBitmap8x8(14*8, 0, NULL);
      else oledSetBitmap8x8(14*8, 0, emptybatt_glyph);
   }
    
   if (voltage!=old_v || current!=old_i) {
      snprintf(str, sizeof(str), "%.1fV %4.0fmA", voltage, current*1000);
      oledWriteString(0, 7, str, false);
      old_v = voltage;
      old_i = current;
   }
               
   return;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(, "Cannot read data from PCF8591");       
}




