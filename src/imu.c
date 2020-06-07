/*************************************************************************

IMU control code, for the LSM9DS1 9DoF MARG sensor (Magnetic, Angular Rate and Gravity)
3D accelerometer, 3D gyroscope, 3D magnetometer 
Controlled by I2C bus (should be 400 KHz). It shows two addresses: 
the acelerometer/gyroscope and the magnetometer.

It must be placed in the robot car such that the dot on the chip is in the right bottom corner
when looking at the car from above and from the back to the front. 
The reference axis in this module are the following, looking from above the car:

                          X
                          |
                          |
                          |
                          |
              Y<----------Z

              Z axis points above the car (right handed orientation)

The LSM9DS1 has a linear acceleration full scale of ±2g/±4g/±8/±16 g, 
a magnetic field full scale of ±4/±8/±12/±16 gauss and an angular rate of ±245/±500/±2000 dps

Magnetic field strength of Earth is about 0.5 gauss, 500 mGauss, 50 uTeslas or 50000 nTeslas

*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
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

#define STD_DEV(s1, s2, N) sqrt(((s2) - ((s1)*(s1))/(double)(N))/((N)-1))
   
#define I2C_BUS 1   // i2c bus of IMU: Bus 0 for Rev 1 boards, bus 1 for newer boards



typedef struct {
    int *xvalues, *yvalues, *zvalues;
    int num_elems;
} SampleList_t;


typedef struct {
  double *history, *taps;
  unsigned int last_index, taps_num;
} Filter_t;


static int i2c_accel_handle = -1;
static int i2c_mag_handle = -1;
static FILE *accel_fp;
static unsigned timerNumber; // The timer used to periodically read the sensor

/* 
Define ODR of accel/gyro and magnetometer. 
The accelerometer and gyroscope are both activated and use the same ODR.
Upsampling requires that ODR of accel/gyro is greater than ODR of magnetometer.
The IMU is read with the magnetometer ODR; the accelerometer and gyroscope data are stored
by the IMU in a FIFO (32 samples max), allowing a higher ODR in accel/gyro than in magnetometer.
In order for the algorithms to work, AG_ODR must be higher or equal than M_ODR. If it is much higher, the FIFO will overrun.
So eg AG_ODR_952 is only possible if ODR_M = M_ODR_80, otherwise the FIFO overruns.
In order for the upsampling of magnetometer data to work, ODR_AG must be an integer multiple of ODR_M (or very close).
*/
static const enum {AG_ODR_OFF,AG_ODR_14_9,AG_ODR_59_5,AG_ODR_119,AG_ODR_238,AG_ODR_476,AG_ODR_952} ODR_AG = AG_ODR_238; 
static const double odr_ag_modes[] = {0,14.9,59.5,119,238,476,952};

static const enum {M_ODR_0_625,M_ODR_1_25,M_ODR_2_5,M_ODR_5,M_ODR_10,M_ODR_20,M_ODR_40,M_ODR_80} ODR_M = M_ODR_40; 
static const double odr_m_modes[] = {0.625,1.25,2.5,5,10,20,40,80};


static int upsampling_factor;  /* upsampling_factor is the ratio between both ODRs */

static Filter_t filter_mx, filter_my, filter_mz; /* Interpolating filters for magnetometer */
static Filter_t filter_ax, filter_ay, filter_az; /* Noise reducction low pass filters for accelerometer */

// gRes, aRes, and mRes store the current resolution for each sensor. 
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15), as data resolution is 16 bits, signed.
static double gRes, aRes, mRes;

/* Store error offset for calibration of accel/gyro and magnetometer */
static int16_t err_AL[3];  // ex,ey,ez values (error for each axis in accelerometer)
static int16_t err_GY[3];  // ex,ey,ez values (error for each axis in gyroscope)
static int16_t err_MA[3];  // ex,ey,ez values (error for each axis in magnetometer, hardiron effects)
static double scale_MA[3] = {1.0, 1.0, 1.0}; // ex,ey,ez values (error for each axis in magnetometer, softiron effects)

static double deviation_AL[3];  // Measured standard deviation of x, y and z values of accelerometer
static double deviation_GY[3];  // Measured standard deviation of x, y and z values of gyroscope
static double deviation_MA[3];  // Measured standard deviation of x, y and z values of magnetometer

static const char *cal_file = "calibration.dat"; // File where calibration data is stored
static const char *dev_file = "deviation.dat";   // File where standard deviation data is stored
static const double declination = +1.266;  // Local magnetic declination as given by http://www.magnetic-declination.com/
static const double magneticField = 0.457;   // Magnitude of the local magnetic field in Gauss

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
static const double beta = 1.73205/2 * GyroMeasError;   // compute beta, sqrt(3/4)*GyroMeasError
static const double deltat = 1.0/odr_ag_modes[ODR_AG];  // Inverse of gyro/accel ODR


// Output variables of module
double roll, pitch, yaw, tilt;


/* Prototypes */
static void imuRead(void);
static void calibrate_accel_gyro(void);
static void calibrate_magnetometer(void);
static double ellipsoid_fit(SampleList_t *sample_list);
static double compute_error(double Vx, double Vy, double Vz, double A, double B, double C, 
                            double Bm, SampleList_t *sample_list);
static void MadgwickQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, 
                                     double mx, double my, double mz);
                                     
                                     
                                     
                                     
/*****************************************************

Digital FIR filter. A LPF is used for filterig magnetometer data.
It is used to interpolate after upsampling from ODR_M to ODR_AG.
It is designed to work with these combinations (upsampling x3 or x6):
ODR_M=80, ODR_AG=476; ODR_M=80, ODR_AG=238;
ODR_M=40, ODR_AG=238; ODR_M=40, ODR_AG=119; 
ODR_M=20, ODR_AG=119; ODR_M=20, ODR_AG=59.5; 
ODR_M=10, ODR_AG=59.5; 

******************************************************/
/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 240 Hz

* 0 Hz - 4 Hz
  gain = 1
  desired ripple = 2 dB
  actual ripple = 1.0141307953166252 dB

* 5 Hz - 19 Hz
  gain = 1
  desired ripple = 35 dB
  actual ripple = 33.758521533154735 dB

* 20 Hz - 120 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -43.613766226277974 dB

*/

static double LP_20_240_filter_taps[] = {
  0.0017433948030936106,
  0.009143190985861756,
  0.012133516280499421,
  0.01983655704542007,
  0.02830242809740451,
  0.03812682806131509,
  0.048540195172121076,
  0.05892094411702151,
  0.06848428950018577,
  0.07647321877548367,
  0.08222112771837956,
  0.08523022650867189,
  0.08523022650867189,
  0.08222112771837956,
  0.07647321877548367,
  0.06848428950018577,
  0.05892094411702151,
  0.048540195172121076,
  0.03812682806131509,
  0.02830242809740451,
  0.01983655704542007,
  0.012133516280499421,
  0.009143190985861756,
  0.0017433948030936106
};


/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 240 Hz

* 0 Hz - 2 Hz
  gain = 1
  desired ripple = 2 dB
  actual ripple = 0.9673152635399324 dB

* 3 Hz - 9 Hz
  gain = 1
  desired ripple = 35 dB
  actual ripple = 26.642185422323546 dB

* 10 Hz - 120 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.61351226633883 dB

*/

static double LP_10_240_filter_taps[] = {
  0.005971139238851345,
  0.004284876252226764,
  0.005706608900364033,
  0.007354427109791928,
  0.0092055716099583,
  0.01126528584840552,
  0.013511905734478582,
  0.015911620908019886,
  0.018440918457958876,
  0.021063915357805933,
  0.023731627358926564,
  0.026398828810262873,
  0.029019156588604933,
  0.03154122957077722,
  0.033914554393050085,
  0.036088808744910154,
  0.03801579844329516,
  0.039655081367493177,
  0.04097170597229545,
  0.041935178632098835,
  0.04252308474795214,
  0.042720929310042295,
  0.04252308474795214,
  0.041935178632098835,
  0.04097170597229545,
  0.039655081367493177,
  0.03801579844329516,
  0.036088808744910154,
  0.033914554393050085,
  0.03154122957077722,
  0.029019156588604933,
  0.026398828810262873,
  0.023731627358926564,
  0.021063915357805933,
  0.018440918457958876,
  0.015911620908019886,
  0.013511905734478582,
  0.01126528584840552,
  0.0092055716099583,
  0.007354427109791928,
  0.005706608900364033,
  0.004284876252226764,
  0.005971139238851345
};


static int LPFilter_init(Filter_t *f, double *tap_array, unsigned tap_list_size) 
{
int i;

   if (f == NULL) ERR(-1, "Invalid filter descriptor");
   if (tap_array == NULL || tap_list_size == 0) ERR(-1, "Invalid tap array for filter");
   f->last_index = 0;
   f->taps_num = tap_list_size;
   f->taps = tap_array;
   f->history = calloc(tap_list_size, sizeof(double));
   if (f->history == NULL) ERR(-1, "Cannot allocate memory: %s", strerror(errno));
   return 0;
}


static double LPFilter_close(Filter_t *f) 
{
   if (f->history) free(f->history);
}


static void LPFilter_DCgain(Filter_t *f, double gain_value)
{
int i;
double DC_gain = 0;  

   // Calculate current DC gain
   for (i = 0; i < f->taps_num; ++i) DC_gain += f->taps[i];
   // Now, change taps so that the new DC gain is 'gain_value'
   for (i = 0; i < f->taps_num; ++i) f->taps[i] *= gain_value/DC_gain;     
}



static void LPFilter_put(Filter_t *f, double input) 
{
  f->history[f->last_index++] = input;
  if (f->last_index == f->taps_num) f->last_index = 0;
}


static double LPFilter_get(Filter_t *f) 
{
  double acc = 0;
  int index = f->last_index, i;
  
  for (i = 0; i < f->taps_num; ++i) {
    index = index != 0 ? index-1 : f->taps_num-1;
    acc += f->history[index] * f->taps[i];
  }
  return acc;
}



/*
Function to calculate heading, using magnetometer readings.
It only works if the sensor is lying flat (z-axis normal to Earth).
It is a non tilt-compensated compass.
*/
void printHeading(double mx, double my)
{
double heading;
  
   heading = -180/M_PI*atan2(my, mx);  // positive westwards
   printf("Heading: %3.0f\n", heading);
}


/*
This function prints the LSM9DS1's orientation based on the
accelerometer and magnetometer data: its roll, pitch and yaw angles, in aerospace convention.
It represents a 3D tilt-compensated compass.
It also calculates the tilt: angle that the normal of the car forms with the vertical.

Procedure according https://www.nxp.com/docs/en/application-note/AN4248.pdf, 
https://www.nxp.com/docs/en/application-note/AN4249.pdf and https://www.nxp.com/docs/en/application-note/AN3461.pdf 
Angles according extrinsic rotation sequence x-y-z (https://en.wikipedia.org/wiki/Euler_angles),
which is equivalent to the intrinsic rotation z-y'-x'' (so the angles are the same): yaw -> pitch -> roll.
See also https://en.wikipedia.org/wiki/Davenport_chained_rotations
*/
void updateOrientation(double ax, double ay, double az, double mx, double my, double mz)
{
//double yaw, pitch, roll, tilt;
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
   // yaw angle able to range between -180 and 180, positive westwards
   yaw = atan2(mz*sinroll-my*cosroll, mx*cospitch+my*sinpitch*sinroll+mz*sinpitch*cosroll);
   
   /*********** Calculate tilt angle from vertical: cos(tilt)=cos(roll)*cos(pitch) *************/ 
   tilt = acos(cosroll*cospitch);  // tilt angle able to range between 0 and 180
   
   /*** Translate angles in radians to degrees ***/
   tilt *= 180/M_PI; 
   yaw *= 180/M_PI; yaw -= declination; if (yaw<0) yaw += 360;  // yaw (heading) must be positive
   pitch *= 180/M_PI; 
   roll *= 180/M_PI; 
     
   //printf("M/A-based Yaw, Pitch, Roll: %3.0f %3.0f %3.0f   Tilt: %3.0f\n", yaw, pitch, roll, tilt);
}



 
 
/* Read the calibration data for accelerometer, gyroscope and magnetometer, if file exists */
static void read_calibration_data(void)
{
FILE *fp;  
int rc;
   
   fp = fopen(cal_file, "r");
   if (!fp) ERR(, "Cannot open calibration file %s: %s", cal_file, strerror(errno));
   
   rc = fscanf(fp, "AL: %d, %d, %d\n", &err_AL[0], &err_AL[1], &err_AL[2]);
   if (rc==EOF || rc!=3) goto cal_error;
   rc = fscanf(fp, "GY: %d, %d, %d\n", &err_GY[0], &err_GY[1], &err_GY[2]);   
   if (rc==EOF || rc!=3) goto cal_error;
   rc = fscanf(fp, "MGH: %d, %d, %d\n", &err_MA[0], &err_MA[1], &err_MA[2]);
   if (rc==EOF || rc!=3) goto cal_error;   
   rc = fscanf(fp, "MGS: %lf, %lf, %lf\n", &scale_MA[0], &scale_MA[1], &scale_MA[2]);   
   if (rc==EOF || rc!=3) goto cal_error;  
   fclose(fp);
   
   fp = fopen(dev_file, "r");
   if (!fp) ERR(, "Cannot open deviation file %s: %s", dev_file, strerror(errno));
   
   rc = fscanf(fp, "AL: %lf, %lf, %lf\n", &deviation_AL[0], &deviation_AL[1], &deviation_AL[2]);
   if (rc==EOF || rc!=3) goto dev_error;
   rc = fscanf(fp, "GY: %lf, %lf, %lf\n", &deviation_GY[0], &deviation_GY[1], &deviation_GY[2]);
   if (rc==EOF || rc!=3) goto dev_error;  
   rc = fscanf(fp, "MG: %lf, %lf, %lf\n", &deviation_MA[0], &deviation_MA[1], &deviation_MA[2]);
   if (rc==EOF || rc!=3) goto dev_error;   
   fclose(fp);   
   
   return;
   
cal_error:
   fclose(fp);
   err_AL[0] = err_AL[1] = err_AL[2] = 0;
   err_GY[0] = err_GY[1] = err_GY[2] = 0; 
   err_MA[0] = err_MA[1] = err_MA[2] = 0; 
   scale_MA[0] = scale_MA[1] = scale_MA[2] = 1.0;       
   ERR(, "Cannot read data from calibration file %s", cal_file);
   
dev_error:
   fclose(fp);
   deviation_AL[0] = deviation_AL[1] = deviation_AL[2] = 0.0;
   deviation_GY[0] = deviation_GY[1] = deviation_GY[2] = 0.0; 
   deviation_MA[0] = deviation_MA[1] = deviation_MA[2] = 0.0;      
   ERR(, "Cannot read data from deviation file %s", dev_file);  
}
 

/* Write the calibration data for accelerometer, gyroscope and magnetometer into a file */ 
static void write_calibration_data(void)
{
FILE *fp;    

   fp = fopen(cal_file, "w");
   if (!fp) ERR(, "Cannot open calibration file %s: %s", cal_file, strerror(errno));

   fprintf(fp, "AL: %d, %d, %d\n", err_AL[0], err_AL[1], err_AL[2]);
   fprintf(fp, "GY: %d, %d, %d\n", err_GY[0], err_GY[1], err_GY[2]);   
   fprintf(fp, "MGH: %d, %d, %d\n", err_MA[0], err_MA[1], err_MA[2]);    
   fprintf(fp, "MGS: %f, %f, %f\n", scale_MA[0], scale_MA[1], scale_MA[2]);     
   fclose(fp);

   fp = fopen(dev_file, "w");
   if (!fp) ERR(, "Cannot open deviation file %s: %s", dev_file, strerror(errno));

   fprintf(fp, "AL: %f, %f, %f\n", deviation_AL[0], deviation_AL[1], deviation_AL[2]);
   fprintf(fp, "GY: %f, %f, %f\n", deviation_GY[0], deviation_GY[1], deviation_GY[2]);
   fprintf(fp, "MG: %f, %f, %f\n", deviation_MA[0], deviation_MA[1], deviation_MA[2]);  
   fclose(fp);    
}


/* Read and discard several samples of accel/gyro data, useful when initializing */
static void read_discard_samples(int samples)
{
char buf[12]; 
int i, rc;
  
   for (i=0; i<samples;i++) {
      gpioDelay(lround(1E6/odr_ag_modes[ODR_AG]));   // Wait for new data to arrive
      rc = i2cReadI2CBlockData(i2c_accel_handle, 0x18, buf, 12);  
      if (rc < 0) goto rw_error;   
   }
   return;

rw_error:
   ERR(, "Cannot read/write data from IMU");   
}



 
 /* 
 Calibrate accelerometer and gyroscope. The IMU should rest horizontal, so that
 measured accel values should be (0,0,1) and measured gyro values should be (0,0,0)
 It writes the measured error in global variables err_AL and err_GY
 */
static void calibrate_accel_gyro(void)
{
int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer   
char buf[12];
int i, rc, elapsed_useconds, samples; 
int32_t s1_ax=0, s1_ay=0, s1_az=0; 
int32_t s2_ax=0, s2_ay=0, s2_az=0;
int32_t s1_gx=0, s1_gy=0, s1_gz=0; 
int32_t s2_gx=0, s2_gy=0, s2_gz=0;
uint32_t start_tick;   
const int cal_seconds = 4; // Number of seconds to take samples
   
   printf("Calibrating accelerometer and gyroscope. Leave robot car horizontal...\n");
   gpioSleep(PI_TIME_RELATIVE, 1, 0); // 1 second delay
   
   start_tick = gpioTick();
   elapsed_useconds = 0; 
   samples = 0;
   do {
      gpioDelay(lround(1E6/odr_ag_modes[ODR_AG]));   // Wait for new data to arrive, acc. ODR selected (4.2 ms for 238 Hz)
      rc = i2cReadByteData(i2c_accel_handle, 0x27);  // Read STATUS_REG register
      if (rc < 0) goto rw_error;  
      if (rc&0x01) {  // New accelerometer data available
      
         // Burst read. Accelerometer and gyroscope sensors are activated at the same ODR
         // So read 12 bytes: 2x3x2
         rc = i2cReadI2CBlockData(i2c_accel_handle, 0x18, buf, 12);  
         if (rc < 0) goto rw_error;
 
         /* Read accel and gyro data. X and Y axis are exchanged, so that reference system is
            right handed, and filter algorithms work correctly */
         gy = buf[1]<<8 | buf[0]; gx = buf[3]<<8 | buf[2]; gz = buf[5]<<8 | buf[4];
         ay = buf[7]<<8 | buf[6]; ax = buf[9]<<8 | buf[8]; az = buf[11]<<8 | buf[10]; 
         az -= (int32_t)(1/aRes);  // Expected value for az is 1g, not 0g
         
         s1_ax += ax; s2_ax += (int32_t)ax*(int32_t)ax; 
         s1_ay += ay; s2_ay += (int32_t)ay*(int32_t)ay; 
         s1_az += az; s2_az += (int32_t)az*(int32_t)az; 
         s1_gx += gx; s2_gx += (int32_t)gx*(int32_t)gx; 
         s1_gy += gy; s2_gy += (int32_t)gy*(int32_t)gy; 
         s1_gz += gz; s2_gz += (int32_t)gz*(int32_t)gz;              
         samples++;
      }
      elapsed_useconds = gpioTick() - start_tick;   
   } while (elapsed_useconds < cal_seconds*1E6);  // loop for 4 seconds  
         
   // Calculate the mean of accelerometer biases and store it in variable err_AL
   err_AL[0] = s1_ax/samples; err_AL[1] = s1_ay/samples; err_AL[2] = s1_az/samples; 
   printf("Accelerometer bias: %d %d %d\n", err_AL[0], err_AL[1], err_AL[2]);
   
   // Calculate the mean of gyroscope biases and store it in variable err_GY   
   err_GY[0] = s1_gx/samples; err_GY[1] = s1_gy/samples; err_GY[2] = s1_gz/samples; 
   printf("Gyroscope bias: %d %d %d\n", err_GY[0], err_GY[1], err_GY[2]);   
 
   // Calculate std deviation
   deviation_AL[0] = STD_DEV(s1_ax, s2_ax, samples);
   deviation_AL[1] = STD_DEV(s1_ay, s2_ay, samples);
   deviation_AL[2] = STD_DEV(s1_az, s2_az, samples); 
   printf("Accelerometer std dev: sigma_x=%.2f, sigma_y=%.2f, sigma_z=%.2f\n", deviation_AL[0], deviation_AL[1], deviation_AL[2]);
   
   deviation_GY[0] = STD_DEV(s1_gx, s2_gx, samples); 
   deviation_GY[1] = STD_DEV(s1_gy, s2_gy, samples); 
   deviation_GY[2] = STD_DEV(s1_gz, s2_gz, samples);
   printf("Gyroscope std dev: sigma_x=%.2f, sigma_y=%.2f, sigma_z=%.2f\n", deviation_GY[0], deviation_GY[1], deviation_GY[2]);   
    
   printf("Done\n");
   return;
  
rw_error:
   err_AL[0] = err_AL[1] = err_AL[2] = 0;
   err_GY[0] = err_GY[1] = err_GY[2] = 0; 
   ERR(, "Cannot read/write data from IMU");
} 





 /* 
 Calibrate magnetometer.
 It writes the measured error in global variables err_MA and scale_MA. 
 This corresponds to the hardiron and softiron effects respectively.
 The car should be rotated in all 3 axis, and then rotated in all directions.
 In future, it would be good to make sure that roll and pitch angles are evenly distributed in the calibration samples.
 The samples can be graphically viewed from the file mag_data.csv. It should be an ellipsoid.
 */
static void calibrate_magnetometer(void)
{
FILE *fp;   
uint32_t start_tick;   
int rc, elapsed_useconds, max_num_samples;
char buf[12];
SampleList_t sample_list = {.xvalues=NULL, .yvalues=NULL, .zvalues=NULL, .num_elems=0};
int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
int min_x=INT16_MAX, min_y=INT16_MAX, min_z=INT16_MAX;
int max_x=INT16_MIN, max_y=INT16_MIN, max_z=INT16_MIN;
int64_t s1_mx=0, s1_my=0, s1_mz=0;
int64_t s2_mx=0, s2_my=0, s2_mz=0;
int rad_x, rad_y, rad_z;
bool do_first_part = true;
double rad_mean, mean_cuad_error;
const int cal_seconds = 60; // Number of seconds to take samples
const int error_meas_seconds = 2;  // Number of seconds for error measurement, must be less than 'seconds'
  
   fp = fopen("mag_data.csv", "w");  // This file is useful to see if the rotation adequately pictured the ellipsoid in all planes
   if (!fp) ERR(, "Cannot open file: %s", strerror(errno));
   fprintf(fp, "X;Y;Z\n");
   
   max_num_samples = cal_seconds * odr_m_modes[ODR_M];
   if (!(sample_list.xvalues = calloc(max_num_samples, sizeof(int))) || 
       !(sample_list.yvalues = calloc(max_num_samples, sizeof(int))) || 
       !(sample_list.zvalues = calloc(max_num_samples, sizeof(int)))) {
      fprintf(stderr, "%s: cannot allocate memory: %s\n", __func__, strerror(errno));
      goto rw_error; 
   }

   printf("Calibrating magnetometer...\n");
   start_tick = gpioTick();
   elapsed_useconds = 0; 
   do {    
      gpioDelay(1000+lround(1E6/odr_m_modes[ODR_M]));   // Wait for new data to arrive, acc. ODR selected (25 ms for 40 Hz)
      rc = i2cReadByteData(i2c_mag_handle, 0x27);  // Read STATUS_REG register  
      if (rc < 0) goto rw_error;        
      if (rc&0x08) {  // New magnetometer data in XYZ available
         /* Read magnetometer data. X and Y axis are exchanged, so that reference system is
         right handed, and filter algorithms work correctly. Sign of Y must be exchanged,
         so that the axis coincide with accel/gyro */
         
         // Read 6 bytes (X,Y,Z axis), starting in OUT_X_L_M register
         rc = i2cReadI2CBlockData(i2c_mag_handle, 0x28, buf, 6); 
         if (rc < 0) goto rw_error;

         my = buf[1]<<8 | buf[0]; mx = buf[3]<<8 | buf[2]; mz = buf[5]<<8 | buf[4];  
         my *= -1;         
         if (sample_list.num_elems < max_num_samples) {
            s1_mx += mx; s1_my += my; s1_mz += mz;
            s2_mx += (int64_t)mx*(int64_t)mx; s2_my += (int64_t)my*(int64_t)my; s2_mz += (int64_t)mz*(int64_t)mz;
            sample_list.xvalues[sample_list.num_elems] = mx; 
            sample_list.yvalues[sample_list.num_elems] = my; 
            sample_list.zvalues[sample_list.num_elems] = mz;
            sample_list.num_elems++;
         }
            
         // The first 2 seconds, the car is stationary. Take samples and then calculate std deviation of error
         if (elapsed_useconds > error_meas_seconds*1E6) {  
         // More than 2 seconds elapsed. Now, perform calibration
            if (do_first_part) {
               // Calculate std deviation with the stored samples
               deviation_MA[0] = STD_DEV(s1_mx, s2_mx, sample_list.num_elems);
               deviation_MA[1] = STD_DEV(s1_my, s2_my, sample_list.num_elems);
               deviation_MA[2] = STD_DEV(s1_mz, s2_mz, sample_list.num_elems);
               printf("Magnetometer std dev: sigma_x=%.2f, sigma_y=%.2f, sigma_z=%.2f\n\n", deviation_MA[0], deviation_MA[1], deviation_MA[2]);
               printf("Rotate robot car slowly in all directions for %d seconds...\n", cal_seconds);
               do_first_part = false;
               sample_list.num_elems = 0; // Restart writing from the beginning, so only calibration values are stored 
            }
            else {
               if (mx>max_x) max_x = mx; if (my>max_y) max_y = my; if (mz>max_z) max_z = mz;
               if (mx<min_x) min_x = mx; if (my<min_y) min_y = my; if (mz<min_z) min_z = mz;
               fprintf(fp, "%d;%d;%d\n", mx, my, mz);   
            }
         }
      }
      elapsed_useconds = gpioTick() - start_tick;   
   } while (elapsed_useconds<(error_meas_seconds+cal_seconds)*1E6);  // loop for 60+2 seconds
   fclose(fp);
   
   /****** Now estimate hardiron and softiron effects ******/
   /* Hardiron error, initial estimation */
   err_MA[0] = (min_x + max_x)/2; 
   err_MA[1] = (min_y + max_y)/2;
   err_MA[2] = (min_z + max_z)/2;
       
   /* Simplified softiron error estimate as starting point */
   rad_x = (max_x - min_x)/2;
   rad_y = (max_y - min_y)/2;   
   rad_z = (max_z - min_z)/2;     
   rad_mean = (rad_x + rad_y + rad_z)/3.0;
   scale_MA[0] = rad_mean/rad_x;
   scale_MA[1] = rad_mean/rad_y;  
   scale_MA[2] = rad_mean/rad_z; 
   
   /* Calculate a better approximation for err_MA and scale_MA, changing the global variables if succesful */
   mean_cuad_error = ellipsoid_fit(&sample_list);
   if (mean_cuad_error > 0.02)   // Limit found experimentally
      fprintf(stderr, "%s: Mean cuadratic error (%.3f) is too big, repeat calibration\n", __func__, mean_cuad_error);
   
   free(sample_list.xvalues); free(sample_list.yvalues); free(sample_list.zvalues);
   printf("Done\n");
   return;
   
rw_error:
   if (fp) fclose(fp);
   if (sample_list.xvalues) free(sample_list.xvalues); 
   if (sample_list.yvalues) free(sample_list.yvalues); 
   if (sample_list.zvalues) free(sample_list.zvalues);
   err_MA[0] = err_MA[1] = err_MA[2] = 0; 
   scale_MA[0] = scale_MA[1] = scale_MA[2] = 1.0;   
   ERR(, "Cannot calibrate magnetometer");   
}




/* 
   Calculate a better approximation for err_MA and scale_MA. Do it by fitting the measured samples into an ellipsoid. 
   It is based on ideas from https://www.nxp.com/docs/en/application-note/AN4246.pdf
   This ellipsoid is assumed to have axis parallel to X, Y and Z (ie, it is not rotated). This assumption is normally true.
   The assumption implies that the ellipsoid fit matrix A is diagonal, leaving only three unknowns, called A, B and C.
   The inverse softiron matrix W^-1 is then also diagonal, with coefficients equal to the square root of A, B and C.
   The function calculates the ellipsoid that fits best into the given samples, minimizing the sum of the square errors of all samples.
   In order to do that, it travels its path, starting from the given initial values in err_MA and scale_MA, along
   the opposite gradient of the error function until a minimum is found.
   The correct magnitude of the magnetic field cannot be calculated from the samples, 
   as all 3 axis can be subject to softiron deformation. So the approximation uses the global variable magneticField, 
   which should contain the magnitude of the magnetic field in the location. A correct value is not needed for orientation
   (ie, calculation of yaw, pitch and roll), so its accurate filling is optative.
   Non constant global variables accessed: err_MA, scale_MA, mRes. They must have correct values prior to call.
   The variables err_MA and scale_MA are modified to reflect the new fit.
*/
static double ellipsoid_fit(SampleList_t *sample_list)
{
int iter, num_excess_err; 
double Vx, Vy, Vz;
double dVx, dVy, dVz;
double A, B, C;
double dA, dB, dC; 
double Bm, err, init_err, min_err, step; 
double min_found[7];  // err, Vx, Vy, Vx, A, B, C 
const double delta = 0.001;
   
   Bm = lround(magneticField/mRes);  // Value of magnetic field in IMU units
   /* Set initial point */
   Vx = (double)err_MA[0]; Vy = (double)err_MA[1]; Vz = (double)err_MA[2];
   A = scale_MA[0]; B = scale_MA[1]; C = scale_MA[2];  
   
   /* Iterate a maximum of 50 times to find a minimum in the error function */
   for (iter=0; iter<50;iter++) {
      /* Compute error of current point */
      err = compute_error(Vx, Vy, Vz, A, B, C, Bm, sample_list);
      if (iter==0) init_err = err;  // Store error of initial point
      /* Check if we found a minimum */
      if (iter==0 || err<min_err) {
         min_err = min_found[0] = err; 
         min_found[1] = Vx; min_found[2] = Vy; min_found[3] = Vz; 
         min_found[4] = A; min_found[5] = B; min_found[6] = C; 
         num_excess_err = 0;
      }
      /* If a minimum was not found, check that we do not exceed 8 iterations with no new minimum found */
      if (err>min_err && ++num_excess_err==8) break;  
      
      /* Calculate gradient of error function */
      dA = (compute_error(Vx, Vy, Vz, A+delta, B, C, Bm, sample_list)-err)/delta;
      dB = (compute_error(Vx, Vy, Vz, A, B+delta, C, Bm, sample_list)-err)/delta;
      dC = (compute_error(Vx, Vy, Vz, A, B, C+delta, Bm, sample_list)-err)/delta;
      dVx = (compute_error(Vx+delta, Vy, Vz, A, B, C, Bm, sample_list)-err)/delta;
      dVy = (compute_error(Vx, Vy+delta, Vz, A, B, C, Bm, sample_list)-err)/delta;
      dVz = (compute_error(Vx, Vy, Vz+delta, A, B, C, Bm, sample_list)-err)/delta;
      
      /* Calculate new point in direction of negative gradient, separately for each 3 dimension space */
      step = 1.0/(2+iter);  // we use 1.0 instead of 2.0 (as is usual eg in FrankWolfe algorithm) to get smaller steps
      A -= dA*step/sqrt(dA*dA+dB*dB+dC*dC);
      B -= dB*step/sqrt(dA*dA+dB*dB+dC*dC);  
      C -= dC*step/sqrt(dA*dA+dB*dB+dC*dC);  
      // The step must be higher for the center coordinates, otherwise it hardly moves
      Vx -= dVx*100*step/sqrt(dVx*dVx+dVy*dVy+dVz*dVz);
      Vy -= dVy*100*step/sqrt(dVx*dVx+dVy*dVy+dVz*dVz);
      Vz -= dVz*100*step/sqrt(dVx*dVx+dVy*dVy+dVz*dVz);     
   }
   //printf("%d iterations\n", iter);
    
   /* A minimum was possibly found */
   if (min_err < init_err) {  // if error of new point is less than initial error, eureka
      err_MA[0] = lrint(min_found[1]); err_MA[1] = lrint(min_found[2]); err_MA[2] = lrint(min_found[3]);
      scale_MA[0] = sqrt(min_found[4]); scale_MA[1] = sqrt(min_found[5]); scale_MA[2] = sqrt(min_found[6]); 
      printf("Vx:%f Vy:%f Vz:%f\n", min_found[1],min_found[2], min_found[3]);
      printf("A:%f B:%f C:%f\n", min_found[4],min_found[5], min_found[6]);
      printf("Minimum found, initial error=%f, final error=%f\n", init_err, min_found[0]);
   }
   return min_err/sample_list->num_elems;  // Mean cuadratic error
}


/*
Compute the total error of the ellipsoid fit given by the parameters Vx, Vy, Vz (the displacement, or hardiron)
and A, B C (the axis coefficients, or softiron). It uses the given magnetometer samples in the 
variables xvalues, yvalues and zvalues. num_samples is the size of the arrays.
Bm is the local magnetic field strength (in IMU scale, between -32768 and 32767).
The error is the sum of the cuadratic errors of each sample with the given fit.
The ellipsoid equation is A(Bx-Vx)^2+B(By-Vy)^2+C(Bz-Vz)^2 - Bm^2 = 0, so the 
error of each sample measures how far from zero is each sample, squared.

*/
static double compute_error(double Vx, double Vy, double Vz, double A, double B, double C, 
                            double Bm, SampleList_t *sample_list)
{
double cuad_err_total=0.0, err, ex, ey, ez;
int i;   
   
   for (i=0; i<sample_list->num_elems; i++) {
      ex = (sample_list->xvalues[i]-Vx)/Bm; 
      ey = (sample_list->yvalues[i]-Vy)/Bm; 
      ez = (sample_list->zvalues[i]-Vz)/Bm;
      err = A*ex*ex + B*ey*ey + C*ez*ez - 1;
      cuad_err_total += err * err;
   }   
   return cuad_err_total;
}




/* 
This function is called periodically, with the rate of the magnetometer ODR.
It reads the IMU data and feeds the Magdwick fusion filter and 3D tilt compensated compass algorithm. 
Both do the same and have the same results, but the fusion filter needs much longer to converge.
The magnetomer data is read. If data was available, the function goes on to read the FIFO
of the accelerometer/gyroscope, which has a much higher ODR (this is why the FIFO is used, 
to store data without having to poll so often).
For each value of the accel/gyro, the fusion filter and the 3D compensated compass are called, using
the same previously read magnetometer data. It works OK, although the sampling rates are different.
Better solution (but seems unnecessary) would be to oversample the magnetometer data.

It takes about 5 ms to complete (mostly between 4.5 and 5.5 ms).
*/
static void imuRead(void)
{
int n, rc, samples;   
char *p, str[17], buf[12*32+1], commands[] = {0x07, 0x01, 0x18, 0x01, 0x06, 0x00, 0x00, 0x00};
uint32_t start_tick, mtick;
static uint32_t old_mtick;
static unsigned int samples_count, count;
static double v_m, e_m;

/* These values are the RAW signed 16-bit readings from the sensors */
int16_t gx, gy, gz; // x, y, and z axis raw readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis raw readings of the accelerometer
int16_t mx, my, mz; // x, y, and z axis raw readings of the magnetometer

/* Real (scaled and compensated) readings of the sensors */
double axr, ayr, azr;
double gxr, gyr, gzr;
double mxr, myr, mzr; 
double axrf, ayrf, azrf; // values after LPF
double mxrf, myrf, mzrf; // values after LPF

   start_tick = gpioTick(); 

   // Read status register in magnetometer, to check if new data is available
   rc = i2cReadByteData(i2c_mag_handle, 0x27);  // Read magnetometer STATUS_REG register, needs 0.1 ms   
   if (rc < 0) goto rw_error;  
   
   if (rc&0x08 == 0) return;  // If no new data available, return
   
   /* New magnetometer data in XYZ is available
      Read magnetometer data. X and Y axis are exchanged, and then Y is negated, 
      so align the axis with tha ones used in this module for car orientation */
         
   // Read 6 bytes (X,Y,Z axis), starting in OUT_X_L_M register
   rc = i2cReadI2CBlockData(i2c_mag_handle, 0x28, buf, 6); // Needs 0.2 ms
   if (rc < 0) goto rw_error;
   my = buf[1]<<8 | buf[0]; mx = buf[3]<<8 | buf[2]; mz = buf[5]<<8 | buf[4];  
   my *= -1;         
 
   /* Substract measured error values (hardiron effects) */
   mx -= err_MA[0]; my -= err_MA[1]; mz -= err_MA[2];      
   /* Compensate for softiron and scale result */
   mxr = mx*scale_MA[0]*mRes; myr = my*scale_MA[1]*mRes; mzr = mz*scale_MA[2]*mRes; 
           
   /*
   mtick = gpioTick();
   if (old_mtick) printf("Time between MAG samples: %.1f ms, ODR AG=%.1f\n", (mtick-old_mtick)/1000.0, (mtick-old_mtick)/1000000.0);
   old_mtick = mtick;
   */
      
   //snprintf(str, sizeof(str), "M:%-6.1f mG", 1000*sqrt(mxr*mxr+myr*myr+mzr*mzr));  
   //oledWriteString(0, 6, str, false);  
   
   /*
   snprintf(str, sizeof(str), "MX:%-6.1f mG", mxr*1000);  
   oledWriteString(0, 4, str, false);
   snprintf(str, sizeof(str), "MY:%-6.1f mG", myr*1000);  
   oledWriteString(0, 5, str, false);        
   snprintf(str, sizeof(str), "MZ:%-6.1f mG", mzr*1000);  
   oledWriteString(0, 6, str, false); 
   */
       
   /* 
   Now, read accel/gyro data. Read data from FIFO, as ODR of accel/gyro is probably higher
   than that of magnetometer. The accel/gyro stores samples in the FIFO until it is read.
   */

   rc = i2cReadByteData(i2c_accel_handle, 0x2F);  // Read FIFO_SRC register, needs 0.1 ms
   if (rc < 0) goto rw_error;
   if (rc&0x40) fprintf(stderr, "%s in %s: FIFO overrun!\n", __func__, __FILE__);  // Should not happen
   // samples in FIFO are between 3 and 4, average 3: AG ODR = 119 Hz, M ODR = 40 Hz, 119/40=3
   // if AG ODR = 238, samples is between 6 and 8
   samples = rc&0x3F;   // samples in FIFO could be zero  
   //printf("Samples: %d\n", samples);
   if (samples < upsampling_factor) {  // Should not happen. Emit a warning message.
      fprintf(stderr, "%s: Timing problem: number of samples of accelerometer (%d) is less than upsampling factor (%d)", 
            __func__, samples, upsampling_factor);    
   }   
      
   if (samples) {  // if FIFO has sth, read it
      /* Burst read. Accelerometer and gyroscope sensors are activated at the same ODR.
         So read all FIFO lines (12 bytes each) in a single I2C transaction 
         using a special function in pigpio library */
      commands[5] = (12*samples)&0xFF;  // LSb value of number of bytes to read
      commands[6] = (12*samples)>>8;    // MSb value of number of bytes to read
      rc = i2cZip(i2c_accel_handle, commands, sizeof(commands), buf, sizeof(buf));   // Needs 1.2 ms for 4 samples
      if (rc < 0) goto rw_error;         
   }

   for (n=0; n<samples; n++) { 
      p = buf + 12*n;
      
      /* Store accel and gyro data. X and Y axis are exchanged, so that reference system is
      right handed, X axis points forwards, Y to the left, and filter algorithms work correctly */
      gy = p[1]<<8 | p[0]; gx = p[3]<<8 | p[2]; gz = p[5]<<8 | p[4];
      ay = p[7]<<8 | p[6]; ax = p[9]<<8 | p[8]; az = p[11]<<8 | p[10];         

      /* Substract the measured error values obtained during calibration */
      ax -= err_AL[0]; ay -= err_AL[1]; az -= err_AL[2]; 
      gx -= err_GY[0]; gy -= err_GY[1]; gz -= err_GY[2]; 

      /* Store real values in float variables */
      axr = ax*aRes; ayr = ay*aRes; azr = az*aRes;      
      gxr = gx*gRes; gyr = gy*gRes; gzr = gz*gRes;              
         
      /* Pass accelerometer data through a low pass filter to eliminate noise */
      LPFilter_put(&filter_ax, axr); LPFilter_put(&filter_ay, ayr); LPFilter_put(&filter_az, azr); 
      axrf = LPFilter_get(&filter_ax); ayrf = LPFilter_get(&filter_ay); azrf = LPFilter_get(&filter_az);      
       
      if (accel_fp) fprintf(accel_fp, "%3.5f;%3.5f;%3.5f\n", axr, ayr, azr);
 
      /* 
      Perform upsampling of magnetometer data to the ODR of the accelerometer/gyro by a factor of N. 
      First, introduce N-1 0-valued samples to align both ODR. Then, filter with a low pass filter to
      eliminate the spectral replica of the original signal. This interpolates the values.
      */
      if (samples_count++%upsampling_factor == 0) {  // After the 0-valued samples, feed the real magnetometer value 
         LPFilter_put(&filter_mx, mxr); LPFilter_put(&filter_my, myr); LPFilter_put(&filter_mz, mzr);  
      }
      else {  // Introduce 0-valued samples to align both ODR
         LPFilter_put(&filter_mx, 0); LPFilter_put(&filter_my, 0); LPFilter_put(&filter_mz, 0);         
      }
      // Take output of the filter
      mxrf = LPFilter_get(&filter_mx); myrf = LPFilter_get(&filter_my); mzrf = LPFilter_get(&filter_mz);
      
      /***************** sensor values are calculated. Now do whatever with them ********/
      v_m += 9.81*axr * deltat;
      e_m += v_m*deltat;
      //printf("a=%f, v=%f, e=%f\n", 9.81*axr, v_m, e_m);
      
      /*
      snprintf(str, sizeof(str), "AX:% 7.1f mg", axr*1000);  
      if (n==0) oledWriteString(0, 4, str, false); 
      snprintf(str, sizeof(str), "AY:% 7.1f mg", ayr*1000);
      if (n==1) oledWriteString(0, 5, str, false);       
      snprintf(str, sizeof(str), "AZ:% 7.1f mg", azr*1000); 
      if (n==2) oledWriteString(0, 6, str, false);
      */
      /*
      snprintf(str, sizeof(str), "GX:% 7.1f dps", gxr);  
      if (n==0) oledWriteString(0, 4, str, false);
      snprintf(str, sizeof(str), "GY:% 7.1f dps", gyr);  
      if (n==1) oledWriteString(0, 5, str, false);        
      snprintf(str, sizeof(str), "GZ:% 7.1f dps", gzr);  
      if (n==2) oledWriteString(0, 6, str, false);               
      */
         
      // 3D compass; valid if car does not accelerate (ie, only acceleration is gravity)
      //updateOrientation(axrf, ayrf, azrf, mxrf, myrf, mzrf);   
      
      // Update sensor fusion filter with the data gathered
      MadgwickQuaternionUpdate(axrf, ayrf, azrf, gxr*M_PI/180, gyr*M_PI/180, gzr*M_PI/180, mxrf, myrf, mzrf);
      getAttitude(&yaw, &pitch, &roll);
         
      // Kalman extended filter
      //EKFUpdateStatus(gxr*M_PI/180, gyr*M_PI/180, gzr*M_PI/180, axrf, ayrf, azrf, deltat);
      //EKFUpdateStatus(0.01, 0.01, 0.01, 0.01, 0.01, 1.01, deltat);
   }
   
   snprintf(str, sizeof(str), "Yaw:  %- 6.1f", yaw);  
   if (count%3==0) oledWriteString(0, 4, str, false);
   snprintf(str, sizeof(str), "Pitch:%- 4.0f", pitch);  
   if (count%3==1) oledWriteString(0, 5, str, false);        
   snprintf(str, sizeof(str), "Roll: %- 4.0f", roll);  
   if (count%3==2) oledWriteString(0, 6, str, false);  
   count++;
   
   //printf("Elapsed time in %s: %.1f ms\n", __func__, (gpioTick()-start_tick)/1000.0);
   return;
   
rw_error:
   ERR(, "Cannot read/write data from IMU");
}


   
/************************************************************
Function called from motor.c when initializing, setup() function

************************************************************/
int setupLSM9DS1(int accel_addr, int mag_addr, bool calibrate, unsigned timer)
{
int rc;
int dl, dh;
int16_t temp;
uint8_t byte;

   /***** Initial checks *****/
   if (odr_ag_modes[ODR_AG] < odr_m_modes[ODR_M]) {
      fprintf(stderr, "%s: Invalid ODR values for IMU\n", __FILE__);
      return -1;
   }
   upsampling_factor = lround(odr_ag_modes[ODR_AG] / odr_m_modes[ODR_M]);

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
   // TEMP_COMP: Yes (b1), XY mode: ultra (b11), ODR: 40 Hz (b110), 0 (b0), ST: No (b0)
   byte = (0x01<<7) + (0x03<<5) + (ODR_M<<2) + 0x0; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x20, byte);
   if (rc < 0) goto rw_error;  
   
   // Set magnetometer, CTRL_REG2_M
   // 0 (b0), FS: 4 Gauss (b00), REBOOT: 0 (b0), SOFT_RST: 0 (b0), 0 (b0), 0 (b0)
   // 4000 mGauss is enough for earth magnetic field + hardiron offset
   byte = 0x0; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x21, byte);
   if (rc < 0) goto rw_error;   
   mRes = 4.0/32768;   // G/LSB
   
   // Activate magnetometer, CTRL_REG3_M
   // I2C: enabled (b0), 0 (b0), LP: No (b0), 0 (b0), 0 (b0), SPI: wo (b0), mode: Continuous (b00)
   byte = 0x0; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x22, byte);
   if (rc < 0) goto rw_error; 
      
   // Set magnetometer, CTRL_REG4_M
   // OMZ: ultra (b11), BLE: data LSb at lower address (b0)
   byte = 0x03<<2; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x23, byte);
   if (rc < 0) goto rw_error;
   
   // Set magnetometer, CTRL_REG5_M
   // BDU:  continuous update (b0)
   byte = 0x00; 
   rc = i2cWriteByteData(i2c_mag_handle, 0x24, byte);
   if (rc < 0) goto rw_error;
   
   /************************* Set Acelerometer / gyroscope ***********************/   

   // Set IMU, CTRL_REG8
   // BOOT: normal mode (b0), BDU: continuous update (b0), H_LACTIVE: active high (b0),
   // PP_OD: 0 (b0), SIM: 0 (b0), IF_ADD_INC: enabled (b1), BLE: data LSB @ lower address (b0), SW_RESET: normal mode (b0)
   byte = 0x04; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x22, byte);
   if (rc < 0) goto rw_error; 
   
   // Set accelerometer, CTRL_REG5_XL
   // DEC: no decimation (b00), Zen_XL, Yen_XL, Xen_XL: enabled (b1)
   byte = (0x00<<6) + (0x07<<3); 
   rc = i2cWriteByteData(i2c_accel_handle, 0x1F, byte);
   if (rc < 0) goto rw_error; 
 
   // Set accelerometer, CTRL_REG6_XL
   // ODR: Power down (b000), FS: 2g (b00), BW_SCAL: auto (b0), BW sel: 408 Hz (b00)
   byte = (0x00<<5) + (0x0<<3) + 0x0; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x20, byte);
   if (rc < 0) goto rw_error; 
   aRes = 2.0/32768;   // g/LSB

   // Set accelerometer, CTRL_REG7_XL
   // HR: enabled (b1), DCF: ODR/9 (b10), FDS: internal filter bypassed (b0), HPIS1:  filter bypassed (b0)
   // both LPF enabled, HPF disabled
   byte = (0x01<<7) + (0x02<<5) + 0x0; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x21, byte);
   if (rc < 0) goto rw_error; 
   
   // Set gyro, CTRL_REG4
   // Zen_G, Yen_G, Xen_G: enabled (b1), LIR_XL1: 0 (b0), 4D_XL1: 0 (b0)
   byte = 0x07<<3; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x1E, byte);
   if (rc < 0) goto rw_error; 
   
   // Set gyro, ORIENT_CFG_G
   // SignX_G, SignX_G, SignX_G: positive sign (b0), Orient: 0 (b000) (Pitch:X, Roll:Y, Yaw:Z)
   byte = 0x00; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x13, byte);
   if (rc < 0) goto rw_error; 
   
   // Activate accelerometer and gyro, CTRL_REG1_G. Both use the same ODR
   // ODR: xxx Hz LPF1 31 Hz (b011), Gyro scale: 245 dps (b00), 0 (b0), Gyro BW LPF2: 31 Hz (b01)
   // The ODR is variable, acc. ODR_AG, LPF2 is set to be always around 30 Hz (b01) (for ODR of 119 Hz and above)
   byte = (ODR_AG<<5) + (0x0<<3) + 0x01; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x10, byte);
   if (rc < 0) goto rw_error; 
   gRes = 245.0/32768;   // dps/LSB
   
   // Set gyro, CTRL_REG2_G
   // INT_SEL: 0 (b00), OUT_SEL: 0 (b10) (output after LPF2)
   byte = 0x02; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x11, byte);
   if (rc < 0) goto rw_error; 
   
   // Set gyro, CTRL_REG3_G
   // LP_mode: disabled (b0), HP_EN: disabled (b0), HPCF_G: 0.5 Hz for 119 Hz ODR (b0100) 
   // both LPF enabled, HPF disabled
   byte = (0x0<<7) + (0x0<<6) + 0x04; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x12, byte);
   if (rc < 0) goto rw_error;  

   /* Empty and reset FIFO, in case it was active, so we start afresh */
   rc = i2cWriteByteData(i2c_accel_handle, 0x2E, 0x00);  // Reset existing FIFO content by selecting FIFO Bypass mode 
   if (rc < 0) goto rw_error;  
 
   /* Initial samples after FIFO change and after power up (Table 12) have to be discarded */
   read_discard_samples(20);   
   
   /************************** Handle calibration ********************/
   if (calibrate) {
      calibrate_accel_gyro();
      calibrate_magnetometer();
      write_calibration_data();
      gpioSleep(PI_TIME_RELATIVE, 1, 0);  // Sleep 1 second, so the user can continue the start process
   }
   else read_calibration_data();   
   
   /************** Continue with accelerometer setting, activate FIFO ****************/
   // Set FIFO, FIFO_CTRL
   // FMODE: continuous mode (b110), threshold: 0 (b00000)
   byte = (0x06<<5) + 0x0; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x2E, byte);
   if (rc < 0) goto rw_error; 

   // Activate FIFO, CTRL_REG9
   // SLEEP_G: disabled (b0), FIFO_TEMP_EN: no (b0), 
   // DRDY_mask_bit: disabled (b0), I2C_DISABLE: both (b0), FIFO_EN: yes (b1), STOP_ON_FTH: no (b0)
   byte = 0x02; 
   rc = i2cWriteByteData(i2c_accel_handle, 0x23, byte);
   if (rc < 0) goto rw_error;  
   
   read_discard_samples(2);  // Discard samples after FIFO activation
   
   /************************** Read Termometer ***********************/
   /*
   // Read temperature  
   dl = rc = i2cReadByteData(i2c_accel_handle, 0x15);  // Read OUT_TEMP_L register   
   if (rc < 0) goto rw_error;
   dh = rc = i2cReadByteData(i2c_accel_handle, 0x16);  // Read OUT_TEMP_H register   
   if (rc < 0) goto rw_error;
   temp = dh<<8 | dl;  // Raw value read from device, it can be negative
   printf("Temperatura del LSM9DS1: %.1f grados\n", 25+(double)temp/16.0);
   */
   
   
   /************************* Final actions ***********************/   
   // Initialize low pass filter for interpolating magnetometer data
   rc = LPFilter_init(&filter_mx, LP_20_240_filter_taps, sizeof(LP_20_240_filter_taps)/sizeof(double));
   if (rc < 0) goto rw_error;  
   LPFilter_init(&filter_my, LP_20_240_filter_taps, sizeof(LP_20_240_filter_taps)/sizeof(double));
   if (rc < 0) goto rw_error;    
   LPFilter_init(&filter_mz, LP_20_240_filter_taps, sizeof(LP_20_240_filter_taps)/sizeof(double));
   if (rc < 0) goto rw_error; 
   // Set gain to upsampling_factor, to compensate for DC gain reduction due to interpolation
   LPFilter_DCgain(&filter_mx, upsampling_factor);  // No need for filter_my and filter_mz, as they share the LP_20_240_filter_taps
   
   // Initialize low pass filter for accelerometer data
   rc = LPFilter_init(&filter_ax, LP_10_240_filter_taps, sizeof(LP_10_240_filter_taps)/sizeof(double));
   if (rc < 0) goto rw_error;  
   LPFilter_init(&filter_ay, LP_10_240_filter_taps, sizeof(LP_10_240_filter_taps)/sizeof(double));
   if (rc < 0) goto rw_error;    
   LPFilter_init(&filter_az, LP_10_240_filter_taps, sizeof(LP_10_240_filter_taps)/sizeof(double));
   if (rc < 0) goto rw_error; 
   LPFilter_DCgain(&filter_mx, 1.0);  // No need for filter_my and filter_mz, as they share the LP_10_240_filter_taps  
   
   
   // Start the IMU reading thread
   timerNumber = timer;
   rc = gpioSetTimerFunc(timerNumber, 1+lround(1000.0/odr_m_modes[ODR_M]), imuRead);  // Read IMU with magnetometer ODR, timer#3
   if (rc<0) ERR(-1, "Cannot set timer for IMU");
   
   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   closeLSM9DS1();
   ERR(-1, "Cannot read/write data from IMU");
}



void closeLSM9DS1(void)
{
   printf("Closing IMU...\n");
   gpioSetTimerFunc(timerNumber, 20, NULL);
   if (i2c_mag_handle>=0) {
      i2cWriteByteData(i2c_mag_handle, 0x22, 0x03);   // Power down magnetometer
      i2cClose(i2c_mag_handle); 
   }
   if (i2c_accel_handle>=0) {
      i2cWriteByteData(i2c_accel_handle, 0x23, 0x00); // Disable FIFO
      i2cWriteByteData(i2c_accel_handle, 0x10, 0x00); // Power down accel/gyro
      i2cClose(i2c_accel_handle);
   }
   LPFilter_close(&filter_mx); LPFilter_close(&filter_my); LPFilter_close(&filter_mz);
   LPFilter_close(&filter_ax); LPFilter_close(&filter_ay); LPFilter_close(&filter_az);
   i2c_accel_handle = -1;
   i2c_mag_handle = -1;
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
      return;
   }

   strcpy(filename, template);
   fd = mkstemps(filename, 4);  // Generate unique file. Suffix length is 4: ".csv"
   if (fd == -1) ERR(, "Cannot create file: %s", strerror(errno))
   
   accel_fp = fdopen(fd, "w");
   if (accel_fp == NULL) ERR(, "Cannot open file %s: %s", filename, strerror(errno));
   
   printf("Saving accelerometer data in file %s\n", filename); 
   fprintf(accel_fp, "X; Y; Z\n");
}



// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
// In this coordinate system, the positive z-axis is down toward Earth. 
// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, 
// looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
// Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
// applied in the correct order which for this configuration is yaw, pitch, and then roll.
// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
// See https://en.wikipedia.org/wiki/Euler_angles for more information.
// Yaw, pitch and roll are the Tait-Bryan angles in a z-y-x'' intrinsic rotation
int getAttitude(double *yaw, double *pitch, double *roll)
{
   *yaw   = atan2(2.0 * (q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);  
   *pitch = -asin(2.0 * (q[1]*q[3] - q[0]*q[2]));
   *roll  = atan2(2.0 * (q[0]*q[1] + q[2]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
   
   *yaw   *= 180/M_PI;  // heading
   *pitch *= 180/M_PI;  // elevation
   *roll  *= 180/M_PI;  // bank
   *yaw -= declination;
   
   //printf("Magd Yaw, Pitch, Roll: %3.0f %3.0f %3.0f\n", *yaw, *pitch, *roll);
   if (i2c_accel_handle>=0 && i2c_mag_handle>=0) return 0;
   else return -1;
}



// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see https://x-io.co.uk/open-source-imu-and-ahrs-algorithms for examples and more details)
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

   // Gradient descent algorithm corrective step
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
  




