#ifndef BMP085_H
#define BMP085_H

/*************************************************************************
Control of the BMP085/BMP180 termometer and pressure sensor from Bosch

*****************************************************************************/

// Inicializa el sistema
int setupBMP085(int addr);

// Cierra ordenadamente el sistema
void closeBMP085(void);

// Function to read temperature, pressure and altitude
int getAtmosfericData(double *temperature, double *pressure, double *altitude);


#endif // BMP085_H
