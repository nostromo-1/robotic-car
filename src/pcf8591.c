/*************************************************************************

NXP PCF8591 control code.
This chip is an 8 bit A/D converter with 4 channels and D/A converter with 1 channel.
It is read via I2C bus (max freq is 100 kHz)
We use it in single-ended mode (4 input channels). Reference and power voltage
are both the 3.3V output of the Raspberry Pi (it is very stable).

*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <pigpio.h>

#include "pcf8591.h"
#include "oled96.h"
#include "robot.h"


#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         fprintf(stderr, "%s: " format "\n" , __func__ , ## arg);      \
         return ret;                                                   \
   }
   
  
/* i2c bus where PCF8591 is connected. 3 is a bit-bang gpio-driver.
It must be activated by adding this line to config.txt: 
dtoverlay=i2c-gpio,i2c_gpio_sda=9,i2c_gpio_scl=11,i2c_gpio_delay_us=3
*/
#define I2C_BUS 3   



static int i2c_handle = -1;
static double voltage, current;
static const unsigned millis = 200;  // Time between calls to checkPower in milliseconds
static unsigned timerNumber;         // The timer used to periodically read the ADC

// 3.3 is the voltage reference (+-1%), 255 are the steps (8 bits ADC resolution)
// 22000 and 12100 are the precision (1%) resistors in series connected to ADC#0 for battery voltage
// Accuracy: about 13 mV (3.3/255) quantisation error due to ADC, times 2.82 (resistors), 
// which is a total error of about +-18 mV (+-13/2*2.82)
// Max. allowed voltage value: 9.3 V
static const double factor_v = 3.3/255*(22000+12100)/12100; 

// ADC#3 is connected to the middle point of the battery pack, via 2 22k precision (1%) resistors
static const double factor_v2 = 3.3/255*2; 

// 1100 and 100 are the precision (1%) resistors in the current sensing circuit connected to ADC#1
// 0.1 is the sensing resistor (1%)
// current = voltage measured / 1.1
// Accuracy: 6 mA due to offset voltage in opamp (600 uV in NPN stage, thus 0.6 mV/0.1) 
// plus 12 mA due to ADC error (13 mV/1.1), which is a total error of about +-9 mA
// Max. allowed current value: 3 A
static const double factor_i = 3.3/255/(0.1*1100/100);  


// Function prototypes
static void checkPower(void);


int setupPCF8591(int addr, unsigned timer)
{
int rc, byte;

   i2c_handle = i2cOpen(I2C_BUS, addr, 0);
   if (i2c_handle < 0) ERR(-1, "Cannot open PCF8591 ADC. No power supply checks.");   

   byte = 6+64;  // Set autoincrement flag, start reading channel 2, single ended inputs, enable DAC 
   rc = i2cWriteByteData(i2c_handle, byte, 0);  // Write control byte, set DAC output to zero
   if (rc < 0) goto rw_error;  
   gpioSleep(PI_TIME_RELATIVE, 0, 5000);  // Does not read correctly without delay
   rc = i2cReadByte(i2c_handle);  // Read previous conversion and ignore it
   if (rc < 0) goto rw_error;   
   
   /* Call checkPower ciclycally, every millis miliseconds, using the given pigpio timer, timer#1 */
   timerNumber = timer;
   rc = gpioSetTimerFunc(timerNumber, millis, checkPower); 
   if (rc < 0) goto rw_error;  
   
   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot read/write data from PCF8591");   
}


void closePCF8591(void)
{
   printf("Closing power meter...\n");
   gpioSetTimerFunc(timerNumber, millis, NULL);
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
}


double getMainVoltageValue(void)
{
   if (i2c_handle < 0) return -1;
   else return voltage;
}



/* 
This function gets called at fixed intervals, every millis milliseconds
Voltage: It reads the ADC#0, connected to the main power supply (max voltage is 9.3V).
Current: It reads the ADC#1, connected to a current sensing circuit (max current is 3A). 
Low currents (in tens of mA) are overestimated.
It displays a symbol in the display according to the battery status.
*/
static void checkPower(void)
{
uint8_t battery_glyph[] = {0, 254, 130, 131, 131, 130, 254, 0};  // glyph for empty battery
int rc, step;
char adc[4];  // Store ADC values
char str[17];
double bat1, bat2;
static char str_old[17];
static int n, old_step = -1;
static const unsigned maxUndervoltageTime = 4000;  // Milliseconds with undervoltage before shutdown is triggered
static unsigned underVoltageTime = 0;

   /* 
      Read all 4 ADC channels: ch2, ch3, ch0, ch1 into adc array
      ch2 was triggered in the previous read, so the value we get 
      is a past value (by the period of the call to this function)
      Reading in this order is done so that ch0 (V) and ch1 (I) 
      are triggered and read in this function call, no delay between both
   */
   rc = i2cReadDevice(i2c_handle, adc, sizeof(adc));  
   if (rc < 0) goto rw_error;
   
   /* If DAC is activated, we can write value to it like this:
   static int t;
   rc = i2cWriteByteData(i2c_handle, 64+7, 100+100*sin(6.28*10*millis*t++));
   if (rc < 0) goto rw_error;
   */

   voltage = factor_v*adc[2];  // Battery voltage level, channel 0
   current = factor_i*adc[3];  // Current draw, channel 1
   if (adc[1]>10)   // channel 3, if there is a non-zero reading, a cable is connected at the mid-battery point
      bat1 = factor_v2*adc[1];  // Voltage level at the middle of the battey pack (1 18650 if 2 in series are used)
   else  // if voltage is too low, it means cable is not connected
      bat1 = voltage/2;  // if mid-point cable is not connected, assume this is half the battery voltage
   bat2 = voltage - bat1;
   
   if (voltage < 6.2) step = 0;        // Battery at 0%
   else if (voltage < 6.6) step = 64;  // Battery at 20%
   else if (voltage < 7.0) step = 64+32;      // Battery at 40%  
   else if (voltage < 7.4) step = 64+32+16;   // Battery at 60%
   else if (voltage < 7.8) step = 64+32+16+8; // Battery at 80%
   else step = 64+32+16+8+4;  // Battery at 100%
   
   battery_glyph[2] += step;
   battery_glyph[3] += step;
   battery_glyph[4] = battery_glyph[3];
   battery_glyph[5] = battery_glyph[2];     
    
   // If battery state changed, update battery symbol on display
   if (step!=old_step) {    
      oledSetBitmap8x8(14*8, 0, battery_glyph);  
      old_step = step;
   }
   
   // Symbol blinks when battery low
   if (step<=64) { 
      if (n++&1) oledSetBitmap8x8(14*8, 0, NULL);
      else oledSetBitmap8x8(14*8, 0, battery_glyph);
   }
    
   // Update display only if values changed (it is a slow operation)
   snprintf(str, sizeof(str), "%.1fV %.2fA", voltage, current);
   if (strcmp(str, str_old)) {
      oledWriteString(0, 1, str, false);
      strcpy(str_old, str);
   }

   // Shutdown if voltage is too low for a long period
   if (bat1<2.9 || bat2<2.9) underVoltageTime += millis;
   else underVoltageTime = 0;
   if (underVoltageTime >= maxUndervoltageTime) {  
      oledBigMessage(0, "Battery!");   
      oledBigMessage(1, "SHUTDOWN");
      closedown();
      execlp("sudo", "sudo", "poweroff", NULL);   // should never return
      exit(1);
   }
   
   return;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   ERR(, "Cannot read data from PCF8591");       
}




