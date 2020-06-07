#ifndef IMU_H
#define IMU_H

/*************************************************************************
Control of the LSM9DS1 3D accelerometer, 3D gyroscope, 3D magnetometer

*****************************************************************************/

// Inicializa el sistema
int setupLSM9DS1(int acel_addr, int mag_addr, bool calibrate);

// Cierra ordenadamente el sistema
void closeLSM9DS1(void);

// Function to read orientation of robot car
int getAttitude(double *yaw, double *pitch, double *roll);

void save_accel_data(void);

#endif // IMU_H
