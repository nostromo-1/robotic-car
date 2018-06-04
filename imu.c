/*************************************************************************

IMU control code, for the LSM9DS1 9DoF MARG sensor (Magnetic, Angular Rate and Gravity)
3D accelerometer, 3D gyroscope, 3D magnetometer 
Controlled by I2C bus (should be 400 KHz). It shows two addresses: 
the acelerometer/gyroscope and the magnetometer.
It must be placed in the robot car such that the dot on the chip is in the right bottom corner
when looking at the car from above and from the back to the front.

*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pigpio.h>
#include <math.h>

#include "imu.h"
#include "ekf.h"
#include "oled96.h"


#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         fprintf(stderr, "%s: " format "\n" , __func__ , ## arg);      \
         return ret;                                                   \
   }
   
   
#define I2C_BUS 1   // i2c bus of IMU: Bus 0 for Rev 1 boards, bus 1 for newer boards


static void imuRead(void);
static void calibrate_accel_gyro(void);
static void calibrate_magnetometer(void);
static void calculateAttitude(void);
static void MadgwickQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, 
                                     double mx, double my, double mz);


static int i2c_accel_handle = -1;
static int i2c_mag_handle = -1;
static FILE *accel_fp;


// gRes, aRes, and mRes store the current resolution for each sensor. 
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
static double gRes, aRes, mRes;

/* Store error offset for calibration of accel/gyro and magnetometer */
static int16_t err_AL[3];  // ex,ey,ez values (error for each axis in accelerometer)
static int16_t err_GY[3];  // ex,ey,ez values (error for each axis in gyroscope)
static int16_t err_MA[3];  // ex,ey,ez values (error for each axis in magnetometer, hardiron effects)
static double scale_MA[3]; // ex,ey,ez values (error for each axis in magnetometer, softiron effects)
static const char *cal_file = "/boot/calibration.dat";
static const double declination = +51/60.0;  // Local magnetic declination as given by http://www.magnetic-declination.com/


/* Madgwick filter variables */
static double q[4] = {1.0, 0.0, 0.0, 0.0};    // vector to hold quaternion

/// Quote from kriswinner regarding beta parameter:
/* 
There is a tradeoff in the beta parameter between accuracy and response speed.
In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
In any case, this is the free parameter in the Madgwick filtering and fusion scheme. 
*/

// gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasError (M_PI * (40.0/180.0))
static const double beta = 1.73205/2 * GyroMeasError; // compute beta, sqrt(3/4)*GyroMeasError
static const double deltat = 1.0/119;  // Inverse of gyro/accel ODR


// Output variables of module
double roll, pitch, yaw;


/*****************************************************
Digital filters. LPF is used for filterig magnetic sensor data.


******************************************************/
/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 40 Hz

* 0 Hz - 5 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.5917525761070728 dB

* 10 Hz - 20 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.26645553189413 dB

*/

#define LPFFILTER_TAP_NUM 11

typedef struct {
  double history[LPFFILTER_TAP_NUM];
  unsigned int last_index;
} LPFFilter;

static LPFFilter filter_x, filter_y, filter_z;

void LPFFilter_init(LPFFilter* f);
void LPFFilter_put(LPFFilter* f, double input);
double LPFFilter_get(LPFFilter* f);

static double filter_taps[LPFFILTER_TAP_NUM] = {
  -0.018161596335410146,
  -0.022750945545202503,
  0.026960154865719534,
  0.14940224353886064,
  0.290005928547412,
  0.35294963112469024,
  0.290005928547412,
  0.14940224353886064,
  0.026960154865719534,
  -0.022750945545202503,
  -0.018161596335410146
};

void LPFFilter_init(LPFFilter* f) {
  int i;
  for(i = 0; i < LPFFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void LPFFilter_put(LPFFilter* f, double input) {
  f->history[f->last_index++] = input;
  if(f->last_index == LPFFILTER_TAP_NUM)
    f->last_index = 0;
}

double LPFFilter_get(LPFFilter* f) {
  double acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < LPFFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : LPFFILTER_TAP_NUM-1;
    acc += f->history[index] * filter_taps[i];
  };
  return acc;
}



// Function to calculate heading, using magnetometer readings.
// It only works if the sensor is lying flat (z-axis normal to Earth).
void printHeading(double mx, double my)
{
double heading;
  
   heading = -180/M_PI*atan2(my, mx);  // positive westwards
   printf("Heading: %3.0f\n", heading);
}


/*
This function prints the LSM9DS1's orientation based on the
accelerometer and magnetometer data: its roll, pitch and yaw angles.
It represents a 3D tilt-compensated compass.
Procedure according https://www.nxp.com/docs/en/application-note/AN4248.pdf and 
https://www.nxp.com/docs/en/application-note/AN4249.pdf
Angles according extrinsic rotation sequence x-y-z (https://en.wikipedia.org/wiki/Euler_angles),
which is equivalent to the intrinsic rotation z-y'-x'' (so the angles are the same).
See also https://en.wikipedia.org/wiki/Davenport_chained_rotations
*/
void printOrientation(double ax, double ay, double az, double mx, double my, double mz)
{
double yaw, pitch, roll, chi;
double sinpitch, cospitch, sinroll, cosroll, rootayaz, rootaxayaz, alpha=0.05;
  
   rootayaz = sqrt(ay*ay+az*az);
   rootaxayaz = sqrt(ax*ax+ay*ay+az*az);
   
   /*********** Calculate roll and pitch *************/
   // Original roll equation: roll = atan2(ay, az).
   // But this is unstable when ay and az tend to zero (pitch=90 degrees).
   // To avoid this, add 5% of ax in denominator
   roll = atan2(ay, az+alpha*ax);  // roll angle able to range between -180 and 180, positive clockwise
   pitch = atan(-ax/rootayaz);     // pitch angle restricted between -90 and 90, positive downwards
   
   /*********** Calculate tilt-compensated heading (yaw angle) *************/
   // intermediate results
   sinroll = ay/rootayaz;
   cosroll = az/rootayaz;
   sinpitch = -ax/rootaxayaz;
   cospitch = rootayaz/rootaxayaz;
   
   // now, calculate yaw
   yaw = atan2(mz*sinroll-my*cosroll, mx*cospitch+my*sinpitch*sinroll+mz*sinpitch*cosroll);
   
   /*********** Calculate tilt angle from vertical: cos(chi)=cos(roll)*cos(pitch) *************/ 
   chi = 180/M_PI*acos(cosroll*cospitch);
   
   yaw *= 180/M_PI;
   pitch *= 180/M_PI;
   roll *= 180/M_PI;
   yaw -= declination;
     
   printf("M/AG Yaw, Pitch, Roll: %3.0f %3.0f %3.0f   Tilt: %3.0f\n", yaw, pitch, roll, chi);
}



 
 
/* Read the calibration data for acclerometer, gyroscope and magnetometer, if file exists */
static void read_calibration_data(void)
{
FILE *fp;  
int rc;
   
   fp = fopen(cal_file, "r");
   if (!fp) {
      perror("Cannot open calibration file");
      return;
   }
   
   rc = fscanf(fp, "AL: %d, %d, %d\n", &err_AL[0], &err_AL[1], &err_AL[2]);
   if (rc==EOF || rc!=3) goto rw_error;
   rc = fscanf(fp, "GY: %d, %d, %d\n", &err_GY[0], &err_GY[1], &err_GY[2]);   
   if (rc==EOF || rc!=3) goto rw_error;
   rc = fscanf(fp, "MGH: %d, %d, %d\n", &err_MA[0], &err_MA[1], &err_MA[2]);
   if (rc==EOF || rc!=3) goto rw_error;   
   rc = fscanf(fp, "MGS: %lf, %lf, %lf\n", &scale_MA[0], &scale_MA[1], &scale_MA[2]);   
   if (rc==EOF || rc!=3) goto rw_error;
   
   fclose(fp);
   return;
   
rw_error:
   fclose(fp);
   err_AL[0] = err_AL[1] = err_AL[2] = 0;
   err_GY[0] = err_GY[1] = err_GY[2] = 0; 
   err_MA[0] = err_MA[1] = err_MA[2] = 0; 
   scale_MA[0] = scale_MA[1] = scale_MA[2] = 1.0;       
   ERR(, "Cannot read data from calibration file %s", cal_file);
}
 

/* Write the calibration data for acclerometer, gyroscope and magnetometer into a file */ 
static void write_calibration_data(void)
{
FILE *fp;    

   fp = fopen(cal_file, "w");
   if (!fp) {
      perror(__func__);
      return;
   }  
   fprintf(fp, "AL: %d, %d, %d\n", err_AL[0], err_AL[1], err_AL[2]);
   fprintf(fp, "GY: %d, %d, %d\n", err_GY[0], err_GY[1], err_GY[2]);   
   fprintf(fp, "MGH: %d, %d, %d\n", err_MA[0], err_MA[1], err_MA[2]);    
   fprintf(fp, "MGS: %f, %f, %f\n", scale_MA[0], scale_MA[1], scale_MA[2]);     
   fclose(fp);
}


 
 /* 
 Calibrate accelerometer and gyroscope. The IMU should rest horizontal, so that
 measured accel values should be (0,0,1) and measured gyro values should be (0,0,0)
 It writes the measured error in global variables err_AL and err_GY
 */
static void calibrate_accel_gyro(void)
{
int32_t gx=0, gy=0, gz=0; // x, y, and z axis readings of the gyroscope
int32_t ax=0, ay=0, az=0; // x, y, and z axis readings of the accelerometer   
char buf[12];
int rc, samples=0;   
   
   printf("Calibrating accelerometer and gyroscope. Leave robot car horizontal...\n");
   gpioDelay(1000000);
   /* Do 200 readings and average them */
   do {
      rc = i2cReadByteData(i2c_accel_handle, 0x27);  // Read STATUS_REG register
      if (rc < 0) goto rw_error;  
      if (rc&0x01) {  // Nuevo dato de acelerometro disponible
      
         // Burst read. Accelerometer and gyroscope sensors are activated at the same ODR
         // So read 12 bytes: 2x3x2
         rc = i2cReadI2CBlockData(i2c_accel_handle, 0x18, buf, 12);  
         if (rc < 0) goto rw_error;
         samples++;
         
         /* Read accel and gyro data. X and Y axis are exchanged, so that reference system is
            right handed, and filter algorithms work correctly */
         gy += buf[1]<<8 | buf[0]; gx += buf[3]<<8 | buf[2]; gz += buf[5]<<8 | buf[4];
         ay += buf[7]<<8 | buf[6]; ax += buf[9]<<8 | buf[8]; az += buf[11]<<8 | buf[10]; 
         gpioDelay(10000);  // 10 ms delay, new data arrives after 8.4 ms (119 Hz)
      }
   } while (samples<200); 
   
   err_AL[0] = ax/samples; err_AL[1] = ay/samples; err_AL[2] = az/samples-(int32_t)(1/aRes); 
   err_GY[0] = gx/samples; err_GY[1] = gy/samples; err_GY[2] = gz/samples;   
   printf("Done\n");
   return;
  
rw_error:
   ERR(, "Cannot read/write data from IMU");
} 



 /* 
 Calibrate magnetometer.
 It writes the measured error in global variable err_MA. This corresponds to the hardiron effects.
 */
static void calibrate_magnetometer(void)
{
FILE *fp;   
uint32_t start_tick;   
int rc, timediff;
char buf[12];
int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
int32_t min_x=INT16_MAX, min_y=INT16_MAX, min_z=INT16_MAX;
int32_t max_x=INT16_MIN, max_y=INT16_MIN, max_z=INT16_MIN;
int32_t rad_x, rad_y, rad_z;; 
double rad_mean;
  
   fp = fopen("mag_data.csv", "w");
   if (!fp) {
      perror(__func__);
      return;
   }
   fprintf(fp, "X;Y;Z\n");
   printf("Calibrating magnetometer...\n");
   printf("Rotate robot car slowly in all directions...\n");
   start_tick = gpioTick();     
   do {    
      gpioDelay(10000);  // 10 ms wait

      // Read status register in magnetometer
      rc = i2cReadByteData(i2c_mag_handle, 0x27);  // Read STATUS_REG register  
      if (rc < 0) goto rw_error;        
      if (rc&0x08) {  // Nuevo dato de magnetometro en XYZ disponible

         /* Read magnetometer data. X and Y axis are exchanged, so that reference system is
         right handed, and filter algorithms work correctly. Sign of Y must be exchanged,
         so that the axis coincide with accel/gyro */
         
         // Read 6 bytes (X,Y,Z axis), starting in OUT_X_L_M register
         rc = i2cReadI2CBlockData(i2c_mag_handle, 0x28, buf, 6); 
         if (rc < 0) goto rw_error;

         my = buf[1]<<8 | buf[0]; mx = buf[3]<<8 | buf[2]; mz = buf[5]<<8 | buf[4];  
         my *= -1;         
         
         if (mx>max_x) max_x = mx; if (my>max_y) max_y = my; if (mz>max_z) max_z = mz;
         if (mx<min_x) min_x = mx; if (my<min_y) min_y = my; if (mz<min_z) min_z = mz;
         
         fprintf(fp, "%d;%d;%d\n", mx, my, mz);
      }            
      timediff = gpioTick() - start_tick;
   } while (timediff<30000000);  // loop for 30 seconds
   fclose(fp);
   printf("Done\n");
   
   /* Hardiron error */
   err_MA[0] = (min_x + max_x)/2; 
   err_MA[1] = (min_y + max_y)/2;
   err_MA[2] = (min_z + max_z)/2;
       
   /* Simplified softiron error estimate (assuming axis of elipsoid are parallel to main axis */
   rad_x = (max_x - min_x)/2;
   rad_y = (max_y - min_y)/2;   
   rad_z = (max_z - min_z)/2;     
   rad_mean = (rad_x + rad_y + rad_z)/3.0;
   scale_MA[0] = rad_mean/rad_x;
   scale_MA[1] = rad_mean/rad_y;  
   scale_MA[2] = rad_mean/rad_z; 
   
   return;
   
rw_error:
   fclose(fp);
   ERR(, "Cannot read/write data from IMU");   
}







/* 
This function is called periodically, every 20 ms (50 Hz).
It reads the IMU data and feeds the Magdwick fusion filter and 3D tilt compensated compass algorithm. 
Both do the same and have the same results, but the fusion filter needs much longer to converge.
The magnetomer data is read (ODR is 40 Hz). If data was available, the function goes on to read the FIFO
of the accelerometer/gyroscope, which has a much higher ODR (119 Hz) (this is why the FIFO is used, 
to store data without having to poll so often).
For each value of the accel/gyro, the fusion filter and the 3D compensated compass are called, using
the same previously read magnetometer data. It works OK, although the sampling rates are different.
Better solution (but seems unnecessary) would be to oversample the magnetometer data.
*/
static void imuRead(void)
{
int n, rc, samples, timediff;   
char *p, str[17], buf[12*32+1], commands[] = {0x07, 0x01, 0x18, 0x01, 0x06, 0x00, 0x00, 0x00};
uint32_t start_tick, end_tick, mtick;
static uint32_t old_mtick;
bool magdata=false;

// These values are the RAW signed 16-bit readings from the sensors
int16_t gx, gy, gz; // x, y, and z axis raw readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis raw readings of the accelerometer
int16_t mx, my, mz; // x, y, and z axis raw readings of the magnetometer

// Real (scaled and compensated) readings of the sensors
double axr, ayr, azr;
double gxr, gyr, gzr;
double mxr, myr, mzr; 
double mxrf, myrf, mzrf; // filtered values

   start_tick = gpioTick(); 
   // Read status register in magnetometer, to check if new data is available
   rc = i2cReadByteData(i2c_mag_handle, 0x27);  // Read STATUS_REG register, ca. 0.25 ms   
   if (rc < 0) goto rw_error;        
   if (rc&0x08) {  // Nuevo dato de magnetometro en XYZ disponible, needs ca. 0.3 ms to read
      /* Read magnetometer data. X and Y axis are exchanged, and Y is negated, 
         so that the axis coincide with accel/gyro */
         
      // Read 6 bytes (X,Y,Z axis), starting in OUT_X_L_M register
      rc = i2cReadI2CBlockData(i2c_mag_handle, 0x28, buf, 6); 
      if (rc < 0) goto rw_error;
      magdata = true;  // mark that magnetometer was read, and then read accel/gyro
      my = buf[1]<<8 | buf[0]; mx = buf[3]<<8 | buf[2]; mz = buf[5]<<8 | buf[4];  
      my *= -1;         
 
      /* Substract measured error values (hardiron effects) */
      mx -= err_MA[0]; my -= err_MA[1]; mz -= err_MA[2];      
      /* Compensate for softiron (simplified handling) */
      mxr = mx*scale_MA[0]*mRes; myr = my*scale_MA[1]*mRes; mzr = mz*scale_MA[2]*mRes; 
      
      // Magnetometer data is rather noisy. So apply a low pass filter.
      LPFFilter_put(&filter_x, mxr); LPFFilter_put(&filter_y, myr); LPFFilter_put(&filter_z, mzr);
      mxrf = LPFFilter_get(&filter_x); myrf = LPFFilter_get(&filter_y); mzrf = LPFFilter_get(&filter_z);
      
      /*
      mtick = gpioTick();
      if (old_mtick) printf("Time between MAG samples: %.1f ms\n", (mtick-old_mtick)/1000.0);
      old_mtick = mtick;
      */
         
      //snprintf(str, sizeof(str), "M:%-6.1f mG", 1000*sqrt(mxr*mxr+myr*myr+mzr*mzr));  
      //oledWriteString(0, 5, str, false);  
      /*
      snprintf(str, sizeof(str), "MX:%-6.1f mG", mxr*1000);  
      oledWriteString(0, 5, str, false);
      snprintf(str, sizeof(str), "MY:%-6.1f mG", myr*1000);  
      oledWriteString(0, 6, str, false);        
      snprintf(str, sizeof(str), "MZ:%-6.1f mG", mzr*1000);  
      oledWriteString(0, 7, str, false); 
      */
         
      //printHeading(mxr, myr);
   }      
   
   // Read accel/gyro data only if new magnetometer data was available
   if (magdata) {
      rc = i2cReadByteData(i2c_accel_handle, 0x2F);  // Read FIFO_SRC register
      if (rc < 0) goto rw_error;
      if (rc&0x40) fprintf(stderr, "%s: FIFO overrun!\n", __func__);
      // samples in FIFO are between 2 and 5, average 3: AG ODR = 119 Hz, M ODR = 40 Hz, 119/40=3
      samples = rc&0x3F;   // samples could be zero  
      //printf("Samples: %d\n", samples);
      
      if (samples) {
         /* Burst read. Accelerometer and gyroscope sensors are activated at the same ODR.
            So read all FIFO lines (12 bytes each) in a single I2C transaction 
            using a special function in pigpio library. */
         commands[5] = (12*samples)&0xFF;  // LSb value of number of bytes to read
         commands[6] = (12*samples)>>8;    // MSb value of number of bytes to read
         rc = i2cZip(i2c_accel_handle, commands, sizeof(commands), buf, sizeof(buf));   // Up to 2.3 ms for 5 samples
         if (rc < 0) goto rw_error;         
      }
      for (n=0; n<samples; n++) { 
         p = buf + 12*n;
      
         /* Read accel and gyro data. X and Y axis are exchanged, so that reference system is
         right handed, X axis points forwards, Y to the left, and filter algorithms work correctly */
         gy = p[1]<<8 | p[0]; gx = p[3]<<8 | p[2]; gz = p[5]<<8 | p[4];
         ay = p[7]<<8 | p[6]; ax = p[9]<<8 | p[8]; az = p[11]<<8 | p[10];         

         /* Substract measured error values */
         ax -= err_AL[0]; ay -= err_AL[1]; az -= err_AL[2]; 
         gx -= err_GY[0]; gy -= err_GY[1]; gz -= err_GY[2]; 

         /* Store real values in variables */
         axr = ax*aRes; ayr = ay*aRes; azr = az*aRes;      
         gxr = gx*gRes; gyr = gy*gRes; gzr = gz*gRes;              
         
         if (accel_fp) fprintf(accel_fp, "%3.5f;%3.5f;%3.5f\n", axr, ayr, azr);
         
         /*
         snprintf(str, sizeof(str), "AX:% 7.1f mg", axr*1000);  
         if (n==0) oledWriteString(0, 2, str, false); 
         snprintf(str, sizeof(str), "AY:% 7.1f mg", ayr*1000);
         if (n==1) oledWriteString(0, 3, str, false);       
         snprintf(str, sizeof(str), "AZ:% 7.1f mg", azr*1000); 
         if (n==2) oledWriteString(0, 4, str, false);
         
         
         snprintf(str, sizeof(str), "GX:% 7.1f dps", gxr);  
         if (n==0) oledWriteString(0, 5, str, false);
         snprintf(str, sizeof(str), "GY:% 7.1f dps", gyr);  
         if (n==1) oledWriteString(0, 6, str, false);        
         snprintf(str, sizeof(str), "GZ:% 7.1f dps", gzr);  
         if (n==2) oledWriteString(0, 7, str, false);               
         */
         
         // 3D compass
         //printOrientation(axr, ayr, azr, mxrf, myrf, mzrf);
         
         // Update sensor fusion filter with the data gathered. Do not filter magnetometer data.
         //MadgwickQuaternionUpdate(axr, ayr, azr, gxr*M_PI/180, gyr*M_PI/180, gzr*M_PI/180, mxr, myr, mzr);
         //getAttitude(&yaw, &pitch, &roll);
         
         // Kalman extended filter
         //calculateEKFAttitude(gxr, gyr, gzr, axr, ayr, azr, deltat);
      }     
   }
   
   end_tick = gpioTick();
   timediff = end_tick - start_tick;
   //printf("Elapsed time in %s: %.1f ms\n", __func__, timediff/1000.0);
   return;
   
rw_error:
   ERR(, "Cannot read/write data from IMU");
}


   
/************************************************************
Function called from motor.c when initializing, setup() function

************************************************************/
int setupLSM9DS1(int accel_addr, int mag_addr, bool calibrate)
{
int rc;
int dl, dh;
int16_t temp;
uint8_t byte;

   /************************* Open connection and check state ***********************/
   i2c_accel_handle = i2cOpen(I2C_BUS, accel_addr, 0);
   if (i2c_accel_handle < 0) ERR(-1, "Cannot open accelerometer/gyroscope device");
   
   i2c_mag_handle = i2cOpen(I2C_BUS, mag_addr, 0);
   if (i2c_mag_handle < 0) ERR(-1, "Cannot open magnetometer device");

   // Check acelerometer/gyroscope   
   rc = i2cReadByteData(i2c_accel_handle, 0x0F);  // Read WHO_AM_I register
   if (rc < 0) goto rw_error;
   if (rc != 0x68) ERR(-1, "Invalid data from accelerometer/gyroscope device");
   
   // Check magnetometer  
   rc = i2cReadByteData(i2c_mag_handle, 0x0F);  // Read WHO_AM_I register
   if (rc < 0) goto rw_error;
   if (rc != 0x3D) ERR(-1, "Invalid data from magnetometer device");  
     
   /************************* Set Magnetometer ***********************/
   
   // Set magnetometer, CTRL_REG1_M
   // TEMP_COMP: No (b0), XY mode: high (b10), ODR: 40 Hz (b110), 0 (b0), ST: No (b0)
   byte = (0x0<<7) + (0x02<<5) + (0x06<<2) + 0x0; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x20, byte);
   if (rc < 0) goto rw_error;  
   
   // Set magnetometer, CTRL_REG2_M
   // 0 (b0), FS: 4 G (b00), REBOOT: 0 (b0), SOFT_RST: 0 (b0), 0 (b0), 0 (b0)
   byte = 0x0; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x21, byte);
   if (rc < 0) goto rw_error;   
   mRes = 4.0/32768;   // G/LSB
   
   // Activate magnetometer, CTRL_REG3_M
   // I2C: Yes (b0), 0 (b0), LP: No (b0), 0 (b0), 0 (b0), SPI: wo (b0), mode: Continuous (b00)
   byte = 0x0; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x22, byte);
   if (rc < 0) goto rw_error; 
      
   // Activate magnetometer, CTRL_REG4_M
   // OMZ: Medium (b01), BLE: data LSb at lower address (b0)
   byte = 0x01<<2; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x23, byte);
   if (rc < 0) goto rw_error;
   
   /************************* Set Acelerometer / gyroscope ***********************/   
   
   // Activate accelerometer, CTRL_REG6_XL
   // ODR: 50 Hz (b010), FS: 2g (b00), BW_SCAL: auto (b0), BW sel: 408 Hz (b00)
   byte = (0x02<<5) + (0x0<<3) + 0x0; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x20, byte);
   if (rc < 0) goto rw_error; 
   aRes = 2.0/32768;   // g/LSB
   
   // Activate accelerometer and gyro, CTRL_REG1_G
   // Gyro ODR: 119 Hz LPF1 31 Hz (b011), Gyro scale: 245 dps (b00), 0 (b0), Gyro BW LPF2: 31 Hz (b01)
   byte = (0x03<<5) + (0x0<<3) + 0x01; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x10, byte);
   if (rc < 0) goto rw_error; 
   gRes = 245.0/32768;   // dps/LSB
   
   // Set gyro, CTRL_REG2_G
   // INT_SEL: 0 (b00), OUT_SEL: 0 (b00) (output after LPF1)
   byte = 0x0; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x11, byte);
   if (rc < 0) goto rw_error; 
   
   // Set gyro, CTRL_REG3_G
   // LP_mode: disabled (b0), HP_EN: disabled (b0), HPCF_G: 1 Hz (b0010) 
   byte = (0x0<<6) + 0x02; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x12, byte);
   if (rc < 0) goto rw_error;  
 
   // Set FIFO, CTRL_REG9
   // 0 (b0), SLEEP_G: disabled (b0), 0 (b0), FIFO_TEMP_EN: no (b0), 
   // DRDY_mask_bit: disabled (b0), I2C_DISABLE: both (b0), FIFO_EN: yes (b1), STOP_ON_FTH: no (b0)
   byte = 0x02; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x23, byte);
   if (rc < 0) goto rw_error;   
   
   // Set FIFO, FIFO_CTRL
   // FMODE: continous mode (b110), threshold: 0 (b00000)
   byte = (0x06<<5) + 0x0; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x2E, byte);
   if (rc < 0) goto rw_error; 

   /************************** Read Termometer ***********************/
   // Read temperature  
   dl = rc = i2cReadByteData(i2c_accel_handle, 0x15);  // Read OUT_TEMP_L register   
   if (rc < 0) goto rw_error;
   dh = rc = i2cReadByteData(i2c_accel_handle, 0x16);  // Read OUT_TEMP_H register   
   if (rc < 0) goto rw_error;
   temp = dh<<8 | dl;  // Raw value read from device, it can be negative
   printf("Temperatura del LSM9DS1: %.1f grados\n", 25+(double)temp/16.0);
   
   /************************** Handle calibration ********************/
   if (calibrate) {
      calibrate_accel_gyro();
      calibrate_magnetometer();
      write_calibration_data();
   }
   else read_calibration_data();
   
   /************************* Final actions ***********************/   
   // Initialize low pass filter for magnetometer data
   LPFFilter_init(&filter_x); LPFFilter_init(&filter_y); LPFFilter_init(&filter_z);
   // Start the IMU-reading thread
   rc = gpioSetTimerFunc(3, 20, imuRead);  // 20ms clock, timer#3
   if (rc<0) ERR(-1, "Cannot set timer for IMU");
   
   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   closeLSM9DS1();
   ERR(-1, "Cannot read/write data from IMU");
}



void closeLSM9DS1(void)
{
   gpioSetTimerFunc(3, 10, NULL);
   if (i2c_accel_handle>=0) i2cClose(i2c_accel_handle);
   if (i2c_mag_handle>=0) i2cClose(i2c_mag_handle);   
   i2c_accel_handle = -1;
   i2c_mag_handle = -1;
}



// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
// In this coordinate system, the positive z-axis is down toward Earth. 
// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
// Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
// applied in the correct order which for this configuration is yaw, pitch, and then roll.
// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
// See https://en.wikipedia.org/wiki/Euler_angles for more information.
// Yaw, pitch and roll are the Tait-Bryan angles in a z-y’-x'' intrinsic rotation
int getAttitude(double *yaw, double *pitch, double *roll)
{
   
   *yaw   = atan2(2 * (q[1]*q[2] + q[0]*q[3]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));   // Positive angle westwards
   *pitch = asin(2 * (q[0]*q[2] - q[1]*q[3]));
   *roll  = atan2(2 * (q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
   *yaw   *= 180/M_PI;  // heading
   *pitch *= 180/M_PI;  // elevation
   *roll  *= 180/M_PI;  // bank
   *yaw -= declination;
   
   printf("Magd Yaw, Pitch, Roll: %3.0f %3.0f %3.0f\n", *yaw, *pitch, *roll);
   if (i2c_accel_handle>=0 && i2c_mag_handle>=0) return 0;
   else return -1;
}


void save_accel_data(void)
{
static char *template="accel_XXXXXX.csv";
char filename[30];
int fd;

   if (accel_fp) {
      fclose(accel_fp);
      accel_fp = NULL;
      printf("Close accelerometer data file\n");
   }
   else {
      strcpy(filename, template);
      fd = mkstemps(filename, 4);  // Generate unique file. Suffix length is 4: ".csv"
      if (fd == -1) perror(__func__);
      else {
         accel_fp = fdopen(fd, "w");
         if (accel_fp) printf("Saving accelerometer data in file %s\n", filename);   
         if (accel_fp) fprintf(accel_fp, "X; Y; Z\n");
      }
   }
}





// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
// Original code and explanation: https://github.com/kriswiner/LSM9DS1
void MadgwickQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz)
{
double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
double norm;
double hx, hy, _2bx, _2bz;
double s1, s2, s3, s4;
double qDot1, qDot2, qDot3, qDot4;

// Auxiliary variables to avoid repeated arithmetic
double _2q1mx, _2q1my, _2q1mz, _2q2mx;
double _4bx, _4bz;
double _2q1 = 2.0 * q1, _2q2 = 2.0 * q2, _2q3 = 2.0 * q3, _2q4 = 2.0 * q4;
double _2q1q3 = 2.0 * q1 * q3, _2q3q4 = 2.0 * q3 * q4;
double q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4;
double q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4;
double q3q3 = q3 * q3, q3q4 = q3 * q4;
double q4q4 = q4 * q4;

   // Normalise accelerometer measurement
   norm = sqrt(ax * ax + ay * ay + az * az);
   if (norm == 0.0) return; // handle NaN
   norm = 1.0/norm;
   ax *= norm;
   ay *= norm;
   az *= norm;

   // Normalise magnetometer measurement
   norm = sqrt(mx * mx + my * my + mz * mz);
   if (norm == 0.0) return; // handle NaN
   norm = 1.0/norm;
   mx *= norm;
   my *= norm;
   mz *= norm;

   // Reference direction of Earth's magnetic field
   _2q1mx = 2.0 * q1 * mx;
   _2q1my = 2.0 * q1 * my;
   _2q1mz = 2.0 * q1 * mz;
   _2q2mx = 2.0 * q2 * mx;
   hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
   hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
   _2bx = sqrt(hx * hx + hy * hy);
   _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
   _4bx = 2.0 * _2bx;
   _4bz = 2.0 * _2bz;

   // Gradient decent algorithm corrective step
   s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
   s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
   s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
   s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
   norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
   norm = 1.0/norm;
   s1 *= norm;
   s2 *= norm;
   s3 *= norm;
   s4 *= norm;

   // Compute rate of change of quaternion
   qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
   qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
   qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
   qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

   // Integrate to yield quaternion
   q1 += qDot1 * deltat;
   q2 += qDot2 * deltat;
   q3 += qDot3 * deltat;
   q4 += qDot4 * deltat;
   norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
   norm = 1.0/norm;
   q[0] = q1 * norm;
   q[1] = q2 * norm;
   q[2] = q3 * norm;
   q[3] = q4 * norm;
}
  




