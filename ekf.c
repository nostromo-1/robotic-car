
/*******************************************************************************************
Original file: https://github.com/BBUK/Bell-Boy/blob/master/grabber/grabber.c

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.


********************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ekf.h"

#define g0 9.8189      // gravity in IS units
#define g0_2 (g0*g0)   // gravity squared
#define q_dcm2 (0.1*0.1)              // DCMVariance
#define q_gyro_bias2 (0.0001*0.0001)  // BiasVariance
#define r_acc2 (0.5*0.5)  // MeasurementVariance
#define r_a2 (10*10)      // MeasurementVarianceVariableGain

#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 


extern double roll, pitch, yaw;
static double a[3];  // Non-gravity acceleration


/* 
This function does some magic with an Extended Kalman Filter to produce roll
pitch and yaw measurements from accelerometer/gyro data.  
Github is here: https://github.com/hhyyti/dcm-imu 
MIT licence but the readme includes this line:
    If you use the algorithm in any scientific context, please cite: Heikki Hyyti and Arto Visala, 
    "A DCM Based Attitude Estimation Algorithm for Low-Cost MEMS IMUs," International Journal 
    of Navigation and Observation, vol. 2015, Article ID 503814, 18 pages, 2015. http://dx.doi.org/10.1155/2015/503814
function inputs are 
Xgyro (u0 - in degrees/sec), Ygyro (u1 - in degrees/sec), Zgyro (u2 - in degrees/sec), 
Xaccel (z0 - in g), Yaccel (z1 - in g), Zaccel (z2 - in g),
interval (h - sample period)
*/
void calculateEKFAttitude(double u0, double u1, double u2, double z0, double z1, double z2, double h)
{
static double x[] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
static double P[6][6] = {   {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}
                         };

    // convert units
    u0 = u0 * DEGREES_TO_RADIANS_MULTIPLIER;
    u1 = u1 * DEGREES_TO_RADIANS_MULTIPLIER;
    u2 = u2 * DEGREES_TO_RADIANS_MULTIPLIER;
    
    z0 = z0 * g0;
    z1 = z1 * g0;
    z2 = z2 * g0;
    
    double x_last[] = {x[0], x[1], x[2]};  // save last state to memory for rotation estimation
    
    // state prediction
    double x_0 = x[0]-h*(u1*x[2]-u2*x[1]+x[1]*x[5]-x[2]*x[4]);
    double x_1 = x[1]+h*(u0*x[2]-u2*x[0]+x[0]*x[5]-x[2]*x[3]);
    double x_2 = x[2]-h*(u0*x[1]-u1*x[0]+x[0]*x[4]-x[1]*x[3]);
    double x_3 = x[3];
    double x_4 = x[4];
    double x_5 = x[5];

    // covariance prediction
    double hh = h*h;
    double P_00 = P[0][0]-h*(P[0][5]*x[1]-P[0][4]*x[2]-P[4][0]*x[2]+P[5][0]*x[1]+P[0][2]*(u1-x[4])+P[2][0]*(u1-x[4])-P[0][1]*(u2-x[5])-P[1][0]*(u2-x[5]))+hh*(q_dcm2-x[1]*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]))+x[2]*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]))-(u1-x[4])*(P[4][2]*x[2]-P[5][2]*x[1]-P[2][2]*(u1-x[4])+P[1][2]*(u2-x[5]))+(u2-x[5])*(P[4][1]*x[2]-P[5][1]*x[1]-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5])));
    double P_01 = P[0][1]+h*(P[0][5]*x[0]-P[0][3]*x[2]+P[4][1]*x[2]-P[5][1]*x[1]+P[0][2]*(u0-x[3])-P[0][0]*(u2-x[5])-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5]))+hh*(x[0]*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]))-x[2]*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]))+(u0-x[3])*(P[4][2]*x[2]-P[5][2]*x[1]-P[2][2]*(u1-x[4])+P[1][2]*(u2-x[5]))-(u2-x[5])*(P[4][0]*x[2]-P[5][0]*x[1]-P[2][0]*(u1-x[4])+P[1][0]*(u2-x[5])));
    double P_02 = P[0][2]-h*(P[0][4]*x[0]-P[0][3]*x[1]-P[4][2]*x[2]+P[5][2]*x[1]+P[0][1]*(u0-x[3])-P[0][0]*(u1-x[4])+P[2][2]*(u1-x[4])-P[1][2]*(u2-x[5]))-hh*(x[0]*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]))-x[1]*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]))+(u0-x[3])*(P[4][1]*x[2]-P[5][1]*x[1]-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5]))-(u1-x[4])*(P[4][0]*x[2]-P[5][0]*x[1]-P[2][0]*(u1-x[4])+P[1][0]*(u2-x[5])));
    double P_03 = P[0][3]+h*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]));
    double P_04 = P[0][4]+h*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]));
    double P_05 = P[0][5]+h*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]));
    double P_10 = P[1][0]-h*(P[1][5]*x[1]-P[1][4]*x[2]+P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[1][2]*(u1-x[4])+P[0][0]*(u2-x[5])-P[1][1]*(u2-x[5]))+hh*(x[1]*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]))-x[2]*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]))+(u1-x[4])*(P[3][2]*x[2]-P[5][2]*x[0]-P[2][2]*(u0-x[3])+P[0][2]*(u2-x[5]))-(u2-x[5])*(P[3][1]*x[2]-P[5][1]*x[0]-P[2][1]*(u0-x[3])+P[0][1]*(u2-x[5])));
    double P_11 = P[1][1]+h*(P[1][5]*x[0]-P[1][3]*x[2]-P[3][1]*x[2]+P[5][1]*x[0]+P[1][2]*(u0-x[3])+P[2][1]*(u0-x[3])-P[0][1]*(u2-x[5])-P[1][0]*(u2-x[5]))+hh*(q_dcm2-x[0]*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]))+x[2]*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]))-(u0-x[3])*(P[3][2]*x[2]-P[5][2]*x[0]-P[2][2]*(u0-x[3])+P[0][2]*(u2-x[5]))+(u2-x[5])*(P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[0][0]*(u2-x[5])));
    double P_12 = P[1][2]-h*(P[1][4]*x[0]-P[1][3]*x[1]+P[3][2]*x[2]-P[5][2]*x[0]+P[1][1]*(u0-x[3])-P[2][2]*(u0-x[3])-P[1][0]*(u1-x[4])+P[0][2]*(u2-x[5]))+hh*(x[0]*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]))-x[1]*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]))+(u0-x[3])*(P[3][1]*x[2]-P[5][1]*x[0]-P[2][1]*(u0-x[3])+P[0][1]*(u2-x[5]))-(u1-x[4])*(P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[0][0]*(u2-x[5])));
    double P_13 = P[1][3]-h*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]));
    double P_14 = P[1][4]-h*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]));
    double P_15 = P[1][5]-h*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]));
    double P_20 = P[2][0]-h*(P[2][5]*x[1]-P[3][0]*x[1]+P[4][0]*x[0]-P[2][4]*x[2]+P[1][0]*(u0-x[3])-P[0][0]*(u1-x[4])+P[2][2]*(u1-x[4])-P[2][1]*(u2-x[5]))-hh*(x[1]*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]))-x[2]*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]))+(u1-x[4])*(P[3][2]*x[1]-P[4][2]*x[0]-P[1][2]*(u0-x[3])+P[0][2]*(u1-x[4]))-(u2-x[5])*(P[3][1]*x[1]-P[4][1]*x[0]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4])));
    double P_21 = P[2][1]+h*(P[2][5]*x[0]+P[3][1]*x[1]-P[4][1]*x[0]-P[2][3]*x[2]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4])+P[2][2]*(u0-x[3])-P[2][0]*(u2-x[5]))+hh*(x[0]*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]))-x[2]*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]))+(u0-x[3])*(P[3][2]*x[1]-P[4][2]*x[0]-P[1][2]*(u0-x[3])+P[0][2]*(u1-x[4]))-(u2-x[5])*(P[3][0]*x[1]-P[4][0]*x[0]-P[1][0]*(u0-x[3])+P[0][0]*(u1-x[4])));
    double P_22 = P[2][2]-h*(P[2][4]*x[0]-P[2][3]*x[1]-P[3][2]*x[1]+P[4][2]*x[0]+P[1][2]*(u0-x[3])+P[2][1]*(u0-x[3])-P[0][2]*(u1-x[4])-P[2][0]*(u1-x[4]))+hh*(q_dcm2-x[0]*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]))+x[1]*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]))-(u0-x[3])*(P[3][1]*x[1]-P[4][1]*x[0]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4]))+(u1-x[4])*(P[3][0]*x[1]-P[4][0]*x[0]-P[1][0]*(u0-x[3])+P[0][0]*(u1-x[4])));
    double P_23 = P[2][3]+h*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]));
    double P_24 = P[2][4]+h*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]));
    double P_25 = P[2][5]+h*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]));
    double P_30 = P[3][0]-h*(P[3][5]*x[1]-P[3][4]*x[2]+P[3][2]*(u1-x[4])-P[3][1]*(u2-x[5]));
    double P_31 = P[3][1]+h*(P[3][5]*x[0]-P[3][3]*x[2]+P[3][2]*(u0-x[3])-P[3][0]*(u2-x[5]));
    double P_32 = P[3][2]-h*(P[3][4]*x[0]-P[3][3]*x[1]+P[3][1]*(u0-x[3])-P[3][0]*(u1-x[4]));
    double P_33 = P[3][3]+hh*q_gyro_bias2;
    double P_34 = P[3][4];
    double P_35 = P[3][5];
    double P_40 = P[4][0]-h*(P[4][5]*x[1]-P[4][4]*x[2]+P[4][2]*(u1-x[4])-P[4][1]*(u2-x[5]));
    double P_41 = P[4][1]+h*(P[4][5]*x[0]-P[4][3]*x[2]+P[4][2]*(u0-x[3])-P[4][0]*(u2-x[5]));
    double P_42 = P[4][2]-h*(P[4][4]*x[0]-P[4][3]*x[1]+P[4][1]*(u0-x[3])-P[4][0]*(u1-x[4]));
    double P_43 = P[4][3];
    double P_44 = P[4][4]+hh*q_gyro_bias2;
    double P_45 = P[4][5];
    double P_50 = P[5][0]-h*(P[5][5]*x[1]-P[5][4]*x[2]+P[5][2]*(u1-x[4])-P[5][1]*(u2-x[5]));
    double P_51 = P[5][1]+h*(P[5][5]*x[0]-P[5][3]*x[2]+P[5][2]*(u0-x[3])-P[5][0]*(u2-x[5]));
    double P_52 = P[5][2]-h*(P[5][4]*x[0]-P[5][3]*x[1]+P[5][1]*(u0-x[3])-P[5][0]*(u1-x[4]));
    double P_53 = P[5][3];
    double P_54 = P[5][4];
    double P_55 = P[5][5]+hh*q_gyro_bias2;

    // kalman innovation
    double y0 = z0-g0*x_0;
    double y1 = z1-g0*x_1;
    double y2 = z2-g0*x_2;

    double a_len = sqrt(y0*y0+y1*y1+y2*y2);

    double S00 = r_acc2+a_len*r_a2+P_00*g0_2;
    double S01 = P_01*g0_2;
    double S02 = P_02*g0_2;
    double S10 = P_10*g0_2;
    double S11 = r_acc2+a_len*r_a2+P_11*g0_2;
    double S12 = P_12*g0_2;
    double S20 = P_20*g0_2;
    double S21 = P_21*g0_2;
    double S22 = r_acc2+a_len*r_a2+P_22*g0_2;

    // Kalman gain
    double invPart = 1.0 / (S00*S11*S22-S00*S12*S21-S01*S10*S22+S01*S12*S20+S02*S10*S21-S02*S11*S20);
    double K00 = (g0*(P_02*S10*S21-P_02*S11*S20-P_01*S10*S22+P_01*S12*S20+P_00*S11*S22-P_00*S12*S21))*invPart;
    double K01 = -(g0*(P_02*S00*S21-P_02*S01*S20-P_01*S00*S22+P_01*S02*S20+P_00*S01*S22-P_00*S02*S21))*invPart;
    double K02 = (g0*(P_02*S00*S11-P_02*S01*S10-P_01*S00*S12+P_01*S02*S10+P_00*S01*S12-P_00*S02*S11))*invPart;
    double K10 = (g0*(P_12*S10*S21-P_12*S11*S20-P_11*S10*S22+P_11*S12*S20+P_10*S11*S22-P_10*S12*S21))*invPart;
    double K11 = -(g0*(P_12*S00*S21-P_12*S01*S20-P_11*S00*S22+P_11*S02*S20+P_10*S01*S22-P_10*S02*S21))*invPart;
    double K12 = (g0*(P_12*S00*S11-P_12*S01*S10-P_11*S00*S12+P_11*S02*S10+P_10*S01*S12-P_10*S02*S11))*invPart;
    double K20 = (g0*(P_22*S10*S21-P_22*S11*S20-P_21*S10*S22+P_21*S12*S20+P_20*S11*S22-P_20*S12*S21))*invPart;
    double K21 = -(g0*(P_22*S00*S21-P_22*S01*S20-P_21*S00*S22+P_21*S02*S20+P_20*S01*S22-P_20*S02*S21))*invPart;
    double K22 = (g0*(P_22*S00*S11-P_22*S01*S10-P_21*S00*S12+P_21*S02*S10+P_20*S01*S12-P_20*S02*S11))*invPart;
    double K30 = (g0*(P_32*S10*S21-P_32*S11*S20-P_31*S10*S22+P_31*S12*S20+P_30*S11*S22-P_30*S12*S21))*invPart;
    double K31 = -(g0*(P_32*S00*S21-P_32*S01*S20-P_31*S00*S22+P_31*S02*S20+P_30*S01*S22-P_30*S02*S21))*invPart;
    double K32 = (g0*(P_32*S00*S11-P_32*S01*S10-P_31*S00*S12+P_31*S02*S10+P_30*S01*S12-P_30*S02*S11))*invPart;
    double K40 = (g0*(P_42*S10*S21-P_42*S11*S20-P_41*S10*S22+P_41*S12*S20+P_40*S11*S22-P_40*S12*S21))*invPart;
    double K41 = -(g0*(P_42*S00*S21-P_42*S01*S20-P_41*S00*S22+P_41*S02*S20+P_40*S01*S22-P_40*S02*S21))*invPart;
    double K42 = (g0*(P_42*S00*S11-P_42*S01*S10-P_41*S00*S12+P_41*S02*S10+P_40*S01*S12-P_40*S02*S11))*invPart;
    double K50 = (g0*(P_52*S10*S21-P_52*S11*S20-P_51*S10*S22+P_51*S12*S20+P_50*S11*S22-P_50*S12*S21))*invPart;
    double K51 = -(g0*(P_52*S00*S21-P_52*S01*S20-P_51*S00*S22+P_51*S02*S20+P_50*S01*S22-P_50*S02*S21))*invPart;
    double K52 = (g0*(P_52*S00*S11-P_52*S01*S10-P_51*S00*S12+P_51*S02*S10+P_50*S01*S12-P_50*S02*S11))*invPart;

    // update a posteriori
    x[0] = x_0+K00*y0+K01*y1+K02*y2;
    x[1] = x_1+K10*y0+K11*y1+K12*y2;
    x[2] = x_2+K20*y0+K21*y1+K22*y2;
    x[3] = x_3+K30*y0+K31*y1+K32*y2;
    x[4] = x_4+K40*y0+K41*y1+K42*y2;
    x[5] = x_5+K50*y0+K51*y1+K52*y2;

    //  update a posteriori covariance
    double r_adab = (r_acc2+a_len*r_a2);
    double P__00 = P_00-g0*(K00*P_00*2.0+K01*P_01+K01*P_10+K02*P_02+K02*P_20)+(K00*K00)*r_adab+(K01*K01)*r_adab+(K02*K02)*r_adab+g0_2*(K00*(K00*P_00+K01*P_10+K02*P_20)+K01*(K00*P_01+K01*P_11+K02*P_21)+K02*(K00*P_02+K01*P_12+K02*P_22));
    double P__01 = P_01-g0*(K00*P_01+K01*P_11+K02*P_21+K10*P_00+K11*P_01+K12*P_02)+g0_2*(K10*(K00*P_00+K01*P_10+K02*P_20)+K11*(K00*P_01+K01*P_11+K02*P_21)+K12*(K00*P_02+K01*P_12+K02*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
    double P__02 = P_02-g0*(K00*P_02+K01*P_12+K02*P_22+K20*P_00+K21*P_01+K22*P_02)+g0_2*(K20*(K00*P_00+K01*P_10+K02*P_20)+K21*(K00*P_01+K01*P_11+K02*P_21)+K22*(K00*P_02+K01*P_12+K02*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
    double P__03 = P_03-g0*(K00*P_03+K01*P_13+K02*P_23+K30*P_00+K31*P_01+K32*P_02)+g0_2*(K30*(K00*P_00+K01*P_10+K02*P_20)+K31*(K00*P_01+K01*P_11+K02*P_21)+K32*(K00*P_02+K01*P_12+K02*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
    double P__04 = P_04-g0*(K00*P_04+K01*P_14+K02*P_24+K40*P_00+K41*P_01+K42*P_02)+g0_2*(K40*(K00*P_00+K01*P_10+K02*P_20)+K41*(K00*P_01+K01*P_11+K02*P_21)+K42*(K00*P_02+K01*P_12+K02*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
    double P__05 = P_05-g0*(K00*P_05+K01*P_15+K02*P_25+K50*P_00+K51*P_01+K52*P_02)+g0_2*(K50*(K00*P_00+K01*P_10+K02*P_20)+K51*(K00*P_01+K01*P_11+K02*P_21)+K52*(K00*P_02+K01*P_12+K02*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
    double P__10 = P_10-g0*(K00*P_10+K01*P_11+K02*P_12+K10*P_00+K11*P_10+K12*P_20)+g0_2*(K00*(K10*P_00+K11*P_10+K12*P_20)+K01*(K10*P_01+K11*P_11+K12*P_21)+K02*(K10*P_02+K11*P_12+K12*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
    double P__11 = P_11-g0*(K10*P_01+K10*P_10+K11*P_11*2.0+K12*P_12+K12*P_21)+(K10*K10)*r_adab+(K11*K11)*r_adab+(K12*K12)*r_adab+g0_2*(K10*(K10*P_00+K11*P_10+K12*P_20)+K11*(K10*P_01+K11*P_11+K12*P_21)+K12*(K10*P_02+K11*P_12+K12*P_22));
    double P__12 = P_12-g0*(K10*P_02+K11*P_12+K12*P_22+K20*P_10+K21*P_11+K22*P_12)+g0_2*(K20*(K10*P_00+K11*P_10+K12*P_20)+K21*(K10*P_01+K11*P_11+K12*P_21)+K22*(K10*P_02+K11*P_12+K12*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
    double P__13 = P_13-g0*(K10*P_03+K11*P_13+K12*P_23+K30*P_10+K31*P_11+K32*P_12)+g0_2*(K30*(K10*P_00+K11*P_10+K12*P_20)+K31*(K10*P_01+K11*P_11+K12*P_21)+K32*(K10*P_02+K11*P_12+K12*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
    double P__14 = P_14-g0*(K10*P_04+K11*P_14+K12*P_24+K40*P_10+K41*P_11+K42*P_12)+g0_2*(K40*(K10*P_00+K11*P_10+K12*P_20)+K41*(K10*P_01+K11*P_11+K12*P_21)+K42*(K10*P_02+K11*P_12+K12*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
    double P__15 = P_15-g0*(K10*P_05+K11*P_15+K12*P_25+K50*P_10+K51*P_11+K52*P_12)+g0_2*(K50*(K10*P_00+K11*P_10+K12*P_20)+K51*(K10*P_01+K11*P_11+K12*P_21)+K52*(K10*P_02+K11*P_12+K12*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
    double P__20 = P_20-g0*(K00*P_20+K01*P_21+K02*P_22+K20*P_00+K21*P_10+K22*P_20)+g0_2*(K00*(K20*P_00+K21*P_10+K22*P_20)+K01*(K20*P_01+K21*P_11+K22*P_21)+K02*(K20*P_02+K21*P_12+K22*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
    double P__21 = P_21-g0*(K10*P_20+K11*P_21+K12*P_22+K20*P_01+K21*P_11+K22*P_21)+g0_2*(K10*(K20*P_00+K21*P_10+K22*P_20)+K11*(K20*P_01+K21*P_11+K22*P_21)+K12*(K20*P_02+K21*P_12+K22*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
    double P__22 = P_22-g0*(K20*P_02+K20*P_20+K21*P_12+K21*P_21+K22*P_22*2.0)+(K20*K20)*r_adab+(K21*K21)*r_adab+(K22*K22)*r_adab+g0_2*(K20*(K20*P_00+K21*P_10+K22*P_20)+K21*(K20*P_01+K21*P_11+K22*P_21)+K22*(K20*P_02+K21*P_12+K22*P_22));
    double P__23 = P_23-g0*(K20*P_03+K21*P_13+K22*P_23+K30*P_20+K31*P_21+K32*P_22)+g0_2*(K30*(K20*P_00+K21*P_10+K22*P_20)+K31*(K20*P_01+K21*P_11+K22*P_21)+K32*(K20*P_02+K21*P_12+K22*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
    double P__24 = P_24-g0*(K20*P_04+K21*P_14+K22*P_24+K40*P_20+K41*P_21+K42*P_22)+g0_2*(K40*(K20*P_00+K21*P_10+K22*P_20)+K41*(K20*P_01+K21*P_11+K22*P_21)+K42*(K20*P_02+K21*P_12+K22*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
    double P__25 = P_25-g0*(K20*P_05+K21*P_15+K22*P_25+K50*P_20+K51*P_21+K52*P_22)+g0_2*(K50*(K20*P_00+K21*P_10+K22*P_20)+K51*(K20*P_01+K21*P_11+K22*P_21)+K52*(K20*P_02+K21*P_12+K22*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
    double P__30 = P_30-g0*(K00*P_30+K01*P_31+K02*P_32+K30*P_00+K31*P_10+K32*P_20)+g0_2*(K00*(K30*P_00+K31*P_10+K32*P_20)+K01*(K30*P_01+K31*P_11+K32*P_21)+K02*(K30*P_02+K31*P_12+K32*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
    double P__31 = P_31-g0*(K10*P_30+K11*P_31+K12*P_32+K30*P_01+K31*P_11+K32*P_21)+g0_2*(K10*(K30*P_00+K31*P_10+K32*P_20)+K11*(K30*P_01+K31*P_11+K32*P_21)+K12*(K30*P_02+K31*P_12+K32*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
    double P__32 = P_32-g0*(K20*P_30+K21*P_31+K22*P_32+K30*P_02+K31*P_12+K32*P_22)+g0_2*(K20*(K30*P_00+K31*P_10+K32*P_20)+K21*(K30*P_01+K31*P_11+K32*P_21)+K22*(K30*P_02+K31*P_12+K32*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
    double P__33 = P_33-g0*(K30*P_03+K31*P_13+K30*P_30+K31*P_31+K32*P_23+K32*P_32)+(K30*K30)*r_adab+(K31*K31)*r_adab+(K32*K32)*r_adab+g0_2*(K30*(K30*P_00+K31*P_10+K32*P_20)+K31*(K30*P_01+K31*P_11+K32*P_21)+K32*(K30*P_02+K31*P_12+K32*P_22));
    double P__34 = P_34-g0*(K30*P_04+K31*P_14+K32*P_24+K40*P_30+K41*P_31+K42*P_32)+g0_2*(K40*(K30*P_00+K31*P_10+K32*P_20)+K41*(K30*P_01+K31*P_11+K32*P_21)+K42*(K30*P_02+K31*P_12+K32*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
    double P__35 = P_35-g0*(K30*P_05+K31*P_15+K32*P_25+K50*P_30+K51*P_31+K52*P_32)+g0_2*(K50*(K30*P_00+K31*P_10+K32*P_20)+K51*(K30*P_01+K31*P_11+K32*P_21)+K52*(K30*P_02+K31*P_12+K32*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
    double P__40 = P_40-g0*(K00*P_40+K01*P_41+K02*P_42+K40*P_00+K41*P_10+K42*P_20)+g0_2*(K00*(K40*P_00+K41*P_10+K42*P_20)+K01*(K40*P_01+K41*P_11+K42*P_21)+K02*(K40*P_02+K41*P_12+K42*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
    double P__41 = P_41-g0*(K10*P_40+K11*P_41+K12*P_42+K40*P_01+K41*P_11+K42*P_21)+g0_2*(K10*(K40*P_00+K41*P_10+K42*P_20)+K11*(K40*P_01+K41*P_11+K42*P_21)+K12*(K40*P_02+K41*P_12+K42*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
    double P__42 = P_42-g0*(K20*P_40+K21*P_41+K22*P_42+K40*P_02+K41*P_12+K42*P_22)+g0_2*(K20*(K40*P_00+K41*P_10+K42*P_20)+K21*(K40*P_01+K41*P_11+K42*P_21)+K22*(K40*P_02+K41*P_12+K42*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
    double P__43 = P_43-g0*(K30*P_40+K31*P_41+K32*P_42+K40*P_03+K41*P_13+K42*P_23)+g0_2*(K30*(K40*P_00+K41*P_10+K42*P_20)+K31*(K40*P_01+K41*P_11+K42*P_21)+K32*(K40*P_02+K41*P_12+K42*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
    double P__44 = P_44-g0*(K40*P_04+K41*P_14+K40*P_40+K42*P_24+K41*P_41+K42*P_42)+(K40*K40)*r_adab+(K41*K41)*r_adab+(K42*K42)*r_adab+g0_2*(K40*(K40*P_00+K41*P_10+K42*P_20)+K41*(K40*P_01+K41*P_11+K42*P_21)+K42*(K40*P_02+K41*P_12+K42*P_22));
    double P__45 = P_45-g0*(K40*P_05+K41*P_15+K42*P_25+K50*P_40+K51*P_41+K52*P_42)+g0_2*(K50*(K40*P_00+K41*P_10+K42*P_20)+K51*(K40*P_01+K41*P_11+K42*P_21)+K52*(K40*P_02+K41*P_12+K42*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
    double P__50 = P_50-g0*(K00*P_50+K01*P_51+K02*P_52+K50*P_00+K51*P_10+K52*P_20)+g0_2*(K00*(K50*P_00+K51*P_10+K52*P_20)+K01*(K50*P_01+K51*P_11+K52*P_21)+K02*(K50*P_02+K51*P_12+K52*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
    double P__51 = P_51-g0*(K10*P_50+K11*P_51+K12*P_52+K50*P_01+K51*P_11+K52*P_21)+g0_2*(K10*(K50*P_00+K51*P_10+K52*P_20)+K11*(K50*P_01+K51*P_11+K52*P_21)+K12*(K50*P_02+K51*P_12+K52*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
    double P__52 = P_52-g0*(K20*P_50+K21*P_51+K22*P_52+K50*P_02+K51*P_12+K52*P_22)+g0_2*(K20*(K50*P_00+K51*P_10+K52*P_20)+K21*(K50*P_01+K51*P_11+K52*P_21)+K22*(K50*P_02+K51*P_12+K52*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
    double P__53 = P_53-g0*(K30*P_50+K31*P_51+K32*P_52+K50*P_03+K51*P_13+K52*P_23)+g0_2*(K30*(K50*P_00+K51*P_10+K52*P_20)+K31*(K50*P_01+K51*P_11+K52*P_21)+K32*(K50*P_02+K51*P_12+K52*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
    double P__54 = P_54-g0*(K40*P_50+K41*P_51+K42*P_52+K50*P_04+K51*P_14+K52*P_24)+g0_2*(K40*(K50*P_00+K51*P_10+K52*P_20)+K41*(K50*P_01+K51*P_11+K52*P_21)+K42*(K50*P_02+K51*P_12+K52*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
    double P__55 = P_55-g0*(K50*P_05+K51*P_15+K52*P_25+K50*P_50+K51*P_51+K52*P_52)+(K50*K50)*r_adab+(K51*K51)*r_adab+(K52*K52)*r_adab+g0_2*(K50*(K50*P_00+K51*P_10+K52*P_20)+K51*(K50*P_01+K51*P_11+K52*P_21)+K52*(K50*P_02+K51*P_12+K52*P_22));

    double xlen = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
    double invlen3 = 1.0/(xlen*xlen*xlen);
    double invlen32 = (invlen3*invlen3);

    double x1_x2 = (x[1]*x[1]+x[2]*x[2]);
    double x0_x2 = (x[0]*x[0]+x[2]*x[2]);
    double x0_x1 = (x[0]*x[0]+x[1]*x[1]);

    // normalized a posteriori covariance
    P[0][0] = invlen32*(-x1_x2*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[0]*x[1]*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2])+x[0]*x[2]*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2]));
    P[0][1] = invlen32*(-x0_x2*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2])+x[0]*x[1]*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[1]*x[2]*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2]));
    P[0][2] = invlen32*(-x0_x1*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2])+x[0]*x[2]*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[1]*x[2]*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2]));
    P[0][3] = -invlen3*(-P__03*x1_x2+P__13*x[0]*x[1]+P__23*x[0]*x[2]);
    P[0][4] = -invlen3*(-P__04*x1_x2+P__14*x[0]*x[1]+P__24*x[0]*x[2]);
    P[0][5] = -invlen3*(-P__05*x1_x2+P__15*x[0]*x[1]+P__25*x[0]*x[2]);
    P[1][0] = invlen32*(-x1_x2*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[0]*x[1]*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2])+x[0]*x[2]*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2]));
    P[1][1] = invlen32*(-x0_x2*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2])+x[0]*x[1]*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[1]*x[2]*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2]));
    P[1][2] = invlen32*(-x0_x1*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2])+x[0]*x[2]*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[1]*x[2]*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2]));
    P[1][3] = -invlen3*(-P__13*x0_x2+P__03*x[0]*x[1]+P__23*x[1]*x[2]);
    P[1][4] = -invlen3*(-P__14*x0_x2+P__04*x[0]*x[1]+P__24*x[1]*x[2]);
    P[1][5] = -invlen3*(-P__15*x0_x2+P__05*x[0]*x[1]+P__25*x[1]*x[2]);
    P[2][0] = invlen32*(-x1_x2*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[0]*x[1]*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2])+x[0]*x[2]*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2]));
    P[2][1] = invlen32*(-x0_x2*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2])+x[0]*x[1]*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[1]*x[2]*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2]));
    P[2][2] = invlen32*(-x0_x1*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2])+x[0]*x[2]*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[1]*x[2]*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2]));
    P[2][3] = -invlen3*(-P__23*x0_x1+P__03*x[0]*x[2]+P__13*x[1]*x[2]);
    P[2][4] = -invlen3*(-P__24*x0_x1+P__04*x[0]*x[2]+P__14*x[1]*x[2]);
    P[2][5] = -invlen3*(-P__25*x0_x1+P__05*x[0]*x[2]+P__15*x[1]*x[2]);
    P[3][0] = -invlen3*(-P__30*x1_x2+P__31*x[0]*x[1]+P__32*x[0]*x[2]);
    P[3][1] = -invlen3*(-P__31*x0_x2+P__30*x[0]*x[1]+P__32*x[1]*x[2]);
    P[3][2] = -invlen3*(-P__32*x0_x1+P__30*x[0]*x[2]+P__31*x[1]*x[2]);
    P[3][3] = P__33;
    P[3][4] = P__34;
    P[3][5] = P__35;
    P[4][0] = -invlen3*(-P__40*x1_x2+P__41*x[0]*x[1]+P__42*x[0]*x[2]);
    P[4][1] = -invlen3*(-P__41*x0_x2+P__40*x[0]*x[1]+P__42*x[1]*x[2]);
    P[4][2] = -invlen3*(-P__42*x0_x1+P__40*x[0]*x[2]+P__41*x[1]*x[2]);
    P[4][3] = P__43;
    P[4][4] = P__44;
    P[4][5] = P__45;
    P[5][0] = -invlen3*(-P__50*x1_x2+P__51*x[0]*x[1]+P__52*x[0]*x[2]);
    P[5][1] = -invlen3*(-P__51*x0_x2+P__50*x[0]*x[1]+P__52*x[1]*x[2]);
    P[5][2] = -invlen3*(-P__52*x0_x1+P__50*x[0]*x[2]+P__51*x[1]*x[2]);
    P[5][3] = P__53;
    P[5][4] = P__54;
    P[5][5] = P__55;

    // normalized a posteriori state
    x[0] = x[0]/xlen;
    x[1] = x[1]/xlen;
    x[2] = x[2]/xlen;

    // compute Euler angles (not exactly a part of the extended Kalman filter)
    // yaw integration through full rotation matrix
    double u_nb1 = u1 - x[4];
    double u_nb2 = u2 - x[5];

    double cy = cos(yaw); //old angles (last state before integration)
    double sy = sin(yaw);
    double d = sqrt(x_last[1]*x_last[1] + x_last[2]*x_last[2]);
    double d_inv = 1.0 / d;

    // compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
    double R11 = cy * d;
    double R12 = -(x_last[2]*sy + x_last[0]*x_last[1]*cy) * d_inv;
    double R13 = (x_last[1]*sy - x_last[0]*x_last[2]*cy) * d_inv;
    double R21 = sy * d;
    double R22 = (x_last[2]*cy - x_last[0]*x_last[1]*sy) * d_inv;
    double R23 = -(x_last[1]*cy + x_last[0]*x_last[2]*sy) * d_inv;

    // update needed parts of R for yaw computation
    double R11_new = R11 + h*(u_nb2*R12 - u_nb1*R13);
    double R21_new = R21 + h*(u_nb2*R22 - u_nb1*R23);
    yaw = atan2(R21_new,R11_new) * RADIANS_TO_DEGREES_MULTIPLIER;

    // compute new pitch and roll angles from a posteriori states
    pitch = asin(-x[0]) * RADIANS_TO_DEGREES_MULTIPLIER;
    roll = atan2(x[1],x[2]) * RADIANS_TO_DEGREES_MULTIPLIER;

    // save the estimated non-gravitational acceleration
    a[0] = z0-x[0]*g0;
    a[1] = z1-x[1]*g0;
    a[2] = z2-x[2]*g0;
    
    printf("KEF Yaw, Pitch, Roll: %3.0f %3.0f %3.0f\n", yaw, pitch, roll);
}

