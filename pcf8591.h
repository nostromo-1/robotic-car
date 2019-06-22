#ifndef PCF8591_H
#define PCF8591_H


// Inicializa el chip
int setupPCF8591(int addr, unsigned timer, unsigned millis);
void checkPower(void);
double getMainVoltageValue(void);


#endif // PCF8591_H

