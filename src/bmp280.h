#ifndef BMP280_H
#define BMP280_H

/*************************************************************************
Control of the BMP280 termometer and pressure sensor from Bosch

*****************************************************************************/

// Inicializa el sistema
int setupBMP280(int addr, unsigned timer);

// Cierra ordenadamente el sistema
void closeBMP280(void);

// Function to read temperature, pressure and altitude
int getAtmosfericData(double *temperature, double *pressure, double *altitude);


#endif // BMP280_H
