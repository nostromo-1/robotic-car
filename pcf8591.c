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

// 3.3 is the voltage reference, 255 are the steps (8 bits ADC resolution)
// 22000 and 12000 are the resistors in series connected to ADC#1
static const double factor = 3.3/255*(22000+12000)/12000;  



int setupPCF8591(int addr)
{
int rc, byte;

   i2c_handle = i2cOpen(I2C_BUS, addr, 0);
   if (i2c_handle < 0) ERR(-1, "Cannot open PCF8591 ADC. No power supply checks.");   
   
   byte = 0;
   rc = i2cWriteByteData(i2c_handle, byte, 0);  // write control byte and a 0 as digital value for DAC
   if (rc < 0) goto rw_error;   
   gpioDelay(100000);  // delay of 0.1 sec, so that control byte works and ADC works correctly
   
   // Read and discard first byte read: after reset, first byte is always 0x80
   // Read several bytes, as sometimes the first results are invalid
   for (byte=0; byte<5; byte++) {
      rc = i2cReadByte(i2c_handle);  
      if (rc < 0) goto rw_error;
      gpioDelay(1000);  // delay of 1 ms
   }
   
   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot read/write data from PCF8591");   
}




double getMainPowerValue(void)
{
int rc;
   
   if (i2c_handle==-1) return -1;   
   rc = i2cReadByte(i2c_handle);  
   if (rc < 0) goto rw_error;
   return factor*rc;   

   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot read data from PCF8591");          
}



// This function gets called at fixed intervals
void checkPowerPCF(void)
{
uint8_t emptybatt_glyph[] = {0, 254, 130, 131, 131, 130, 254, 0};
int rc, step;
double volts;
static int n, old_step = -1;
    
   if (i2c_handle==-1) return;
   rc = i2cReadByte(i2c_handle);  
   if (rc < 0) goto rw_error;
   volts = factor*rc;
   //printf("volts=%.1f\n", volts);

   if (volts < 6.2) step = 0;        // Battery at 0%
   else if (volts < 6.6) step = 64;  // Battery at 20%
   else if (volts < 7.0) step = 64+32;      // Battery at 40%  
   else if (volts < 7.4) step = 64+32+16;   // Battery at 60%
   else if (volts < 7.8) step = 64+32+16+8; // Battery at 80%
   else step = 64+32+16+8+4;  // Battery at 100%
    
   // If battery state changed, update battery symbol on display
   if (step!=old_step) {
      emptybatt_glyph[2] += step;
      emptybatt_glyph[3] += step;
      emptybatt_glyph[4] = emptybatt_glyph[3];
      emptybatt_glyph[5] = emptybatt_glyph[2];
      oledSetBitmap8x8(14*8, 0, emptybatt_glyph);
      old_step = step;
   }
   
   // Symbol blinks when battery low
   if (step<=64) {
      if (n++&1) oledSetBitmap8x8(14*8, 0, NULL);
      else oledSetBitmap8x8(14*8, 0, emptybatt_glyph);
   }
     
   return;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(, "Cannot read data from PCF8591");       
}




