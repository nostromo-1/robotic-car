/***************

 The DCM-IMU algorithm is designed for fusing low-cost triaxial MEMS gyroscope and accelerometer measurements. 
 An extended Kalman filter is used to estimate attitude in direction cosine matrix (DCM) formation 
 and gyroscope biases online. A variable measurement covariance method is implemented for 
 acceleration measurements to ensure robustness against transient non-gravitational accelerations 
 which usually induce errors to attitude estimate in ordinary IMU-algorithms.
 If you use the algorithm in any scientific context, please cite: Heikki Hyyti and Arto Visala, 
 "A DCM Based Attitude Estimation Algorithm for Low-Cost MEMS IMUs," 
 International Journal of Navigation and Observation, vol. 2015, Article ID 503814, 18 pages, 2015. 
 http://dx.doi.org/10.1155/2015/503814
 This updated version of micro controller c code has forced symmetry of covariance matrices 
 that reduces computational complexity of the filter significantly. 
 In addition, the code divides measurement with g0 before feeding it to filter which increases 
 the stability because covariance matrix update is done with smaller weights (1 vs g0^6).
 For further safety, the variance of states is limited to a small postive value and a small 
 non-corelating noise is added to each state to keep the filter stable against rounding errors. 
 Both of these safety additions may be disabled by defining the values to 0 in this file.

Github is here: https://github.com/hhyyti/dcm-imu 
    Heikki Hyyti and Arto Visala, 
    "A DCM Based Attitude Estimation Algorithm for Low-Cost MEMS IMUs," International Journal 
    of Navigation and Observation, vol. 2015, Article ID 503814, 18 pages, 2015. http://dx.doi.org/10.1155/2015/503814
    
Copyright (C) 2020, Heikki Hyyti

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

****************/


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "ekf.h"



#define DEFAULT_g0 9.8189
#define DEFAULT_state {0,0,1,0,0,0}
#define DEFAULT_q_dcm2 (0.1*0.1)
#define DEFAULT_q_gyro_bias2 (0.0001*0.0001)
#define DEFAULT_r_acc2 (0.5*0.5)
#define DEFAULT_r_a2 (10*10)
#define DEFAULT_q_dcm2_init (1*1)
#define DEFAULT_q_gyro_bias2_init (0.1*0.1)

#define VARIANCE_MIN_LIMIT (0.1*0) //set this to a small positive number or 0 to disable the feature.
#define VARIANCE_SAFETY_INCREMENT (0.1*0) //set this to a small positive number or 0 to disable the feature.

const double Gravity = DEFAULT_g0;  // A magnitude of gravity
const double *State = NULL;  // An initial state as a array of six floats, DCM states and bias states
const double *Covariance = NULL;  // A covariance matrix (size of 6x6 floats, array of 36 floats in row-major order). 
//If a custom covariance matrix is given, parameters InitialDCMVariance and InitialBiasVariance are not used.
const double DCMVariance = DEFAULT_q_dcm2;  // a variance for DCM state update, Q(0,0), Q(1,1), and Q(2,2)
const double BiasVariance = DEFAULT_q_gyro_bias2;  // a variance for bias state update, Q(3,3), Q(4,4), and Q(5,5)
const double InitialDCMVariance = DEFAULT_q_dcm2_init;  // an initial variance for DCM state, P(0,0), P(1,1), and P(2,2). If Covariance matrix is given, this parameter is not used.
const double InitialBiasVariance = DEFAULT_q_gyro_bias2_init;  // an initial variance for bias state, P(3,3), P(4,4), and P(5,5). If Covariance matrix is given, this parameter is not used.
const double MeasurementVariance = DEFAULT_r_acc2;  // a constant part of the variance for measurement update, R(0,0), R(1,1), and R(2,2)
const double MeasurementVarianceVariableGain = DEFAULT_r_a2;  // a gain for the variable part of the variance for measurement update, R(0,0), R(1,1), and R(2,2)

double g0, inv_g0, inv_g0_2;
double x0, x1, x2, x3, x4, x5;
double q_dcm2;
double q_gyro_bias2;
double r_acc2;
double r_a2;
double a0, a1, a2;
double P00, P01, P02, P03, P04, P05;
double P11, P12, P13, P14, P15;
double P22, P23, P24, P25;
double P33, P34, P35;
double P44, P45;
double P55;
double fr0, fr1, fr2;
   
extern double roll, pitch, yaw;



/*    
Gyroscope: Xgyro (u0 - in radians/sec), Ygyro (u1 - in radians/sec), Zgyro (u2 - in radians/sec), 
Accelerometer: Xaccel (z0 - in g), Yaccel (z1 - in g), Zaccel (z2 - in g),
interval (h - sample period)
*/
void EKFUpdateStatus(double u0, double u1, double u2, double z0, double z1, double z2, double h)
{
double temp[] = DEFAULT_state;
double x_last[3];
int i;
static bool initialized;

   if (!initialized) {
      if (State != NULL) {
         for (i = 0; i < 6; ++i) temp[i] = State[i];
      }
      x0 = temp[0]; x1 = temp[1]; x2 = temp[2]; x3 = temp[3]; x4 = temp[4]; x5 = temp[5];
      if (Covariance == NULL) {
         P00 = InitialDCMVariance;
         P01 = P02 = P03 = P04 = P05 = 0;
         P11 = InitialDCMVariance;
         P12 = P13 = P14 = P15 = 0;
         P22 = InitialDCMVariance;
         P23 = P24 = P25 = 0;
         P33 = InitialBiasVariance;
         P34 = P35 = 0;
         P44 = InitialBiasVariance;
         P45 = 0;
         P55 = InitialBiasVariance;
      }
      else {
         P00 = Covariance[0];
         P01 = Covariance[1];
         P02 = Covariance[2];
         P03 = Covariance[3];
         P04 = Covariance[4];
         P05 = Covariance[5];
         P11 = Covariance[7];
         P12 = Covariance[8];
         P13 = Covariance[9];
         P14 = Covariance[10];
         P15 = Covariance[11];
         P22 = Covariance[14];
         P23 = Covariance[15];
         P24 = Covariance[16];
         P25 = Covariance[17];
         P33 = Covariance[21];
         P34 = Covariance[22];
         P35 = Covariance[23];
         P44 = Covariance[28];
         P45 = Covariance[29];
         P55 = Covariance[35];
      }
   
      g0 = Gravity;
      q_dcm2 = DCMVariance;
      q_gyro_bias2 = BiasVariance;
      r_acc2 = MeasurementVariance;
      r_a2 = MeasurementVarianceVariableGain;
      
      
      inv_g0 = 1.0 / g0;
      inv_g0_2 = inv_g0 * inv_g0;
      fr0 = 1.0; //first row for alternative rotation computation
      fr1 = 0.0; //default is yaw = 0 which happens when fr = [1, 0, 0]
      fr2 = 0.0;
      
      initialized = true;
   }

	// save last state to memory for rotation estimation
	x_last[0] = x0;
	x_last[1] = x1;
	x_last[2] = x2;

	// state prediction
	double x_0 = x0-h*(u1*x2-u2*x1+x1*x5-x2*x4);
	double x_1 = x1+h*(u0*x2-u2*x0+x0*x5-x2*x3);
	double x_2 = x2-h*(u0*x1-u1*x0+x0*x4-x1*x3);
	double x_3 = x3;
	double x_4 = x4;
	double x_5 = x5;

	// covariance prediction
	double hh = h*h;
	double P_00 = P00-h*(P05*x1*2.0-P04*x2*2.0+P02*(u1-x4)*2.0-P01*(u2-x5)*2.0)-hh*(-q_dcm2+x2*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5))+x1*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5))+(u2-x5)*(P15*x1-P14*x2+P12*(u1-x4)-P11*(u2-x5))-(u1-x4)*(P25*x1-P24*x2+P22*(u1-x4)-P12*(u2-x5)));
	double P_01 = P01+h*(P05*x0-P03*x2-P15*x1+P14*x2+P02*(u0-x3)-P12*(u1-x4)-P00*(u2-x5)+P11*(u2-x5))+hh*(x2*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5))+x0*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5))+(u2-x5)*(P05*x1-P04*x2+P02*(u1-x4)-P01*(u2-x5))-(u0-x3)*(P25*x1-P24*x2+P22*(u1-x4)-P12*(u2-x5)));
	double P_02 = P02-h*(P04*x0-P03*x1+P25*x1-P24*x2+P01*(u0-x3)-P00*(u1-x4)+P22*(u1-x4)-P12*(u2-x5))-hh*(x1*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5))-x0*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5))+(u1-x4)*(P05*x1-P04*x2+P02*(u1-x4)-P01*(u2-x5))-(u0-x3)*(P15*x1-P14*x2+P12*(u1-x4)-P11*(u2-x5)));
	double P_03 = P03-h*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5));
	double P_04 = P04-h*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5));
	double P_05 = P05+h*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5));
	double P_11 = P11+h*(P15*x0*2.0-P13*x2*2.0+P12*(u0-x3)*2.0-P01*(u2-x5)*2.0)-hh*(-q_dcm2+x2*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5))+x0*(P35*x2-P55*x0-P25*(u0-x3)+P05*(u2-x5))+(u2-x5)*(P05*x0-P03*x2+P02*(u0-x3)-P00*(u2-x5))-(u0-x3)*(P25*x0-P23*x2+P22*(u0-x3)-P02*(u2-x5)));
	double P_12 = P12-h*(P14*x0-P13*x1-P25*x0+P23*x2+P11*(u0-x3)-P01*(u1-x4)-P22*(u0-x3)+P02*(u2-x5))+hh*(x1*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5))-x0*(P45*x0-P34*x2+P24*(u0-x3)-P04*(u2-x5))+(u1-x4)*(P05*x0-P03*x2+P02*(u0-x3)-P00*(u2-x5))-(u0-x3)*(P15*x0-P13*x2+P12*(u0-x3)-P01*(u2-x5)));
	double P_13 = P13+h*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5));
	double P_14 = P14+h*(P45*x0-P34*x2+P24*(u0-x3)-P04*(u2-x5));
	double P_15 = P15-h*(P35*x2-P55*x0-P25*(u0-x3)+P05*(u2-x5));
	double P_22 = P22-h*(P24*x0*2.0-P23*x1*2.0+P12*(u0-x3)*2.0-P02*(u1-x4)*2.0)-hh*(-q_dcm2+x1*(P34*x0-P33*x1+P13*(u0-x3)-P03*(u1-x4))+x0*(P34*x1-P44*x0-P14*(u0-x3)+P04*(u1-x4))+(u1-x4)*(P04*x0-P03*x1+P01*(u0-x3)-P00*(u1-x4))-(u0-x3)*(P14*x0-P13*x1+P11*(u0-x3)-P01*(u1-x4)));
	double P_23 = P23-h*(P34*x0-P33*x1+P13*(u0-x3)-P03*(u1-x4));
	double P_24 = P24+h*(P34*x1-P44*x0-P14*(u0-x3)+P04*(u1-x4));
	double P_25 = P25+h*(P35*x1-P45*x0-P15*(u0-x3)+P05*(u1-x4));
	double P_33 = P33+hh*q_gyro_bias2;
	double P_34 = P34;
	double P_35 = P35;
	double P_44 = P44+hh*q_gyro_bias2;
	double P_45 = P45;
	double P_55 = P55+hh*q_gyro_bias2;

   // measurements (accelerometers)
	z0 *= inv_g0;
	z1 *= inv_g0;
	z2 *= inv_g0;

	// Kalman innovation
	double y0 = z0-x_0;
	double y1 = z1-x_1;
	double y2 = z2-x_2;

	double a_len = sqrt(y0*y0+y1*y1+y2*y2) * g0;
	double r_adab = (r_acc2 + a_len*r_a2) * inv_g0_2;
   
   // innovation covariance
	double S00 = P_00 + r_adab;
	double S01 = P_01;
	double S02 = P_02;
	double S11 = P_11 + r_adab;
	double S12 = P_12;
	double S22 = P_22 + r_adab;

   // verify that the innovation covariance is large enough
	if (S00 < VARIANCE_MIN_LIMIT) S00 = VARIANCE_MIN_LIMIT;
	if (S11 < VARIANCE_MIN_LIMIT) S11 = VARIANCE_MIN_LIMIT;
	if (S22 < VARIANCE_MIN_LIMIT) S22 = VARIANCE_MIN_LIMIT;

	// determinant of S
	double det_S = -S00*(S12*S12) - (S02*S02)*S11 - (S01*S01)*S22 + S01*S02*S12*2.0f + S00*S11*S22;
   
   // Kalman gain
	double invPart = 1.0 / det_S;
	double K00 = -(S02*(P_02*S11-P_01*S12)-S01*(P_02*S12-P_01*S22)+P_00*(S12*S12)-P_00*S11*S22)*invPart;
	double K01 = -(S12*(P_02*S00-P_00*S02)-S01*(P_02*S02-P_00*S22)+P_01*(S02*S02)-P_01*S00*S22)*invPart;
	double K02 = -(S12*(P_01*S00-P_00*S01)-S02*(P_01*S01-P_00*S11)+P_02*(S01*S01)-P_02*S00*S11)*invPart;
	double K10 = -(S02*(P_12*S11-P_11*S12)-S01*(P_12*S12-P_11*S22)+P_01*(S12*S12)-P_01*S11*S22)*invPart;
	double K11 = -(S12*(P_12*S00-P_01*S02)-S01*(P_12*S02-P_01*S22)+P_11*(S02*S02)-P_11*S00*S22)*invPart;
	double K12 = (S12*(P_01*S01-P_11*S00)+S02*(P_11*S01-P_01*S11)-P_12*(S01*S01)+P_12*S00*S11)*invPart;
	double K20 = (S02*(P_12*S12-P_22*S11)+S01*(P_22*S12-P_12*S22)-P_02*(S12*S12)+P_02*S11*S22)*invPart;
	double K21 = (S12*(P_02*S02-P_22*S00)+S01*(P_22*S02-P_02*S22)-P_12*(S02*S02)+P_12*S00*S22)*invPart;
	double K22 = (S12*(P_02*S01-P_12*S00)+S02*(P_12*S01-P_02*S11)-P_22*(S01*S01)+P_22*S00*S11)*invPart;
	double K30 = (S02*(P_13*S12-P_23*S11)+S01*(P_23*S12-P_13*S22)-P_03*(S12*S12)+P_03*S11*S22)*invPart;
	double K31 = (S12*(P_03*S02-P_23*S00)+S01*(P_23*S02-P_03*S22)-P_13*(S02*S02)+P_13*S00*S22)*invPart;
	double K32 = (S12*(P_03*S01-P_13*S00)+S02*(P_13*S01-P_03*S11)-P_23*(S01*S01)+P_23*S00*S11)*invPart;
	double K40 = (S02*(P_14*S12-P_24*S11)+S01*(P_24*S12-P_14*S22)-P_04*(S12*S12)+P_04*S11*S22)*invPart;
	double K41 = (S12*(P_04*S02-P_24*S00)+S01*(P_24*S02-P_04*S22)-P_14*(S02*S02)+P_14*S00*S22)*invPart;
	double K42 = (S12*(P_04*S01-P_14*S00)+S02*(P_14*S01-P_04*S11)-P_24*(S01*S01)+P_24*S00*S11)*invPart;
	double K50 = (S02*(P_15*S12-P_25*S11)+S01*(P_25*S12-P_15*S22)-P_05*(S12*S12)+P_05*S11*S22)*invPart;
	double K51 = (S12*(P_05*S02-P_25*S00)+S01*(P_25*S02-P_05*S22)-P_15*(S02*S02)+P_15*S00*S22)*invPart;
	double K52 = (S12*(P_05*S01-P_15*S00)+S02*(P_15*S01-P_05*S11)-P_25*(S01*S01)+P_25*S00*S11)*invPart;

   // update a posteriori
	x0 = x_0 + K00*y0 + K01*y1 + K02*y2;
	x1 = x_1 + K10*y0 + K11*y1 + K12*y2;
	x2 = x_2 + K20*y0 + K21*y1 + K22*y2;
	x3 = x_3 + K30*y0 + K31*y1 + K32*y2;
	x4 = x_4 + K40*y0 + K41*y1 + K42*y2;
	x5 = x_5 + K50*y0 + K51*y1 + K52*y2;

	// update a posteriori covariance
	double K00_1 = K00 - 1.0;
	double K11_1 = K11 - 1.0;
	double K22_1 = K22 - 1.0;

	double common1 = P_01*K00_1 + K01*P_11 + K02*P_12;
	double common2 = P_02*K00_1 + K01*P_12 + K02*P_22;
	double common3 = P_00*K00_1 + K01*P_01 + K02*P_02;
	double common4 = P_01*K11_1 + K10*P_00 + K12*P_02;
	double common5 = P_12*K11_1 + K10*P_02 + K12*P_22;
	double common6 = P_11*K11_1 + K10*P_01 + K12*P_12;
	double common7 = P_02*K22_1 + K20*P_00 + K21*P_01;
	double common8 = P_12*K22_1 + K20*P_01 + K21*P_11;
	double common9 = P_22*K22_1 + K20*P_02 + K21*P_12;
	double commonA = -P_03 + K30*P_00 + K31*P_01 + K32*P_02;
	double commonB = -P_13 + K30*P_01 + K31*P_11 + K32*P_12;
	double commonC = -P_23 + K30*P_02 + K31*P_12 + K32*P_22;

   double P__00 = K01*common1+K02*common2+(K00*K00)*r_adab+(K01*K01)*r_adab+(K02*K02)*r_adab+K00_1*common3;
	double P__01 = K10*common3+K12*common2+K11_1*common1+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
	double P__02 = K20*common3+K21*common1+K22_1*common2+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
	double P__03 = -P_03*K00_1+K30*common3+K31*common1+K32*common2-K01*P_13-K02*P_23+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
	double P__04 = -P_04*K00_1+K40*common3+K41*common1+K42*common2-K01*P_14-K02*P_24+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
	double P__05 = -P_05*K00_1+K50*common3+K51*common1+K52*common2-K01*P_15-K02*P_25+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
	double P__11 = K10*common4+K12*common5+(K10*K10)*r_adab+(K11*K11)*r_adab+(K12*K12)*r_adab+K11_1*common6;
	double P__12 = K20*common4+K21*common6+K22_1*common5+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
	double P__13 = -P_13*K11_1+K30*common4+K31*common6+K32*common5-K10*P_03-K12*P_23+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
	double P__14 = -P_14*K11_1+K40*common4+K41*common6+K42*common5-K10*P_04-K12*P_24+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
	double P__15 = -P_15*K11_1+K50*common4+K51*common6+K52*common5-K10*P_05-K12*P_25+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
	double P__22 = K20*common7+K21*common8+(K20*K20)*r_adab+(K21*K21)*r_adab+(K22*K22)*r_adab+K22_1*common9;
	double P__23 = -P_23*K22_1+K30*common7+K31*common8+K32*common9-K20*P_03-K21*P_13+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
	double P__24 = -P_24*K22_1+K40*common7+K41*common8+K42*common9-K20*P_04-K21*P_14+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
	double P__25 = -P_25*K22_1+K50*common7+K51*common8+K52*common9-K20*P_05-K21*P_15+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
	double P__33 = P_33+(K30*K30)*r_adab+(K31*K31)*r_adab+(K32*K32)*r_adab+K30*commonA+K31*commonB+K32*commonC-K30*P_03-K31*P_13-K32*P_23;
	double P__34 = P_34+K40*commonA+K41*commonB+K42*commonC-K30*P_04-K31*P_14-K32*P_24+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
	double P__35 = P_35+K50*commonA+K51*commonB+K52*commonC-K30*P_05-K31*P_15-K32*P_25+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
	double P__44 = P_44+(K40*K40)*r_adab+(K41*K41)*r_adab+(K42*K42)*r_adab+K40*(-P_04+K40*P_00+K41*P_01+K42*P_02)+K41*(-P_14+K40*P_01+K41*P_11+K42*P_12)+K42*(-P_24+K40*P_02+K41*P_12+K42*P_22)-K40*P_04-K41*P_14-K42*P_24;
	double P__45 = P_45+K50*(-P_04+K40*P_00+K41*P_01+K42*P_02)+K51*(-P_14+K40*P_01+K41*P_11+K42*P_12)+K52*(-P_24+K40*P_02+K41*P_12+K42*P_22)-K40*P_05-K41*P_15-K42*P_25+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
	double P__55 = P_55+(K50*K50)*r_adab+(K51*K51)*r_adab+(K52*K52)*r_adab+K50*(-P_05+K50*P_00+K51*P_01+K52*P_02)+K51*(-P_15+K50*P_01+K51*P_11+K52*P_12)+K52*(-P_25+K50*P_02+K51*P_12+K52*P_22)-K50*P_05-K51*P_15-K52*P_25;

	// Normalization of covariance
	double len = sqrt(x0*x0 + x1*x1 + x2*x2);
	double invlen3 = 1.0 / (len*len*len);
	double invlen32 = (invlen3*invlen3);

	double x1x1_x2x2 = (x1*x1 + x2*x2);
	double x0x0_x2x2 = (x0*x0 + x2*x2);
	double x0x0_x1x1 = (x0*x0 + x1*x1);
   
   P00 = invlen32*(-x1x1_x2x2*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x0*x1*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2)+x0*x2*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2));
	P01 = invlen32*(-x0x0_x2x2*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2)+x0*x1*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x1*x2*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2));
	P02 = invlen32*(-x0x0_x1x1*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2)+x0*x2*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x1*x2*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2));
	P03 = -invlen3*(-P__03*x1x1_x2x2+P__13*x0*x1+P__23*x0*x2);
	P04 = -invlen3*(-P__04*x1x1_x2x2+P__14*x0*x1+P__24*x0*x2);
	P05 = -invlen3*(-P__05*x1x1_x2x2+P__15*x0*x1+P__25*x0*x2);
	P11 = invlen32*(-x0x0_x2x2*(-P__11*x0x0_x2x2+P__01*x0*x1+P__12*x1*x2)+x0*x1*(-P__01*x0x0_x2x2+P__00*x0*x1+P__02*x1*x2)+x1*x2*(-P__12*x0x0_x2x2+P__02*x0*x1+P__22*x1*x2));
	P12 = invlen32*(-x0x0_x1x1*(-P__12*x0x0_x2x2+P__02*x0*x1+P__22*x1*x2)+x0*x2*(-P__01*x0x0_x2x2+P__00*x0*x1+P__02*x1*x2)+x1*x2*(-P__11*x0x0_x2x2+P__01*x0*x1+P__12*x1*x2));
	P13 = -invlen3*(-P__13*x0x0_x2x2+P__03*x0*x1+P__23*x1*x2);
	P14 = -invlen3*(-P__14*x0x0_x2x2+P__04*x0*x1+P__24*x1*x2);
	P15 = -invlen3*(-P__15*x0x0_x2x2+P__05*x0*x1+P__25*x1*x2);
	P22 = invlen32*(-x0x0_x1x1*(-P__22*x0x0_x1x1+P__02*x0*x2+P__12*x1*x2)+x0*x2*(-P__02*x0x0_x1x1+P__00*x0*x2+P__01*x1*x2)+x1*x2*(-P__12*x0x0_x1x1+P__01*x0*x2+P__11*x1*x2));
	P23 = -invlen3*(-P__23*x0x0_x1x1+P__03*x0*x2+P__13*x1*x2);
	P24 = -invlen3*(-P__24*x0x0_x1x1+P__04*x0*x2+P__14*x1*x2);
	P25 = -invlen3*(-P__25*x0x0_x1x1+P__05*x0*x2+P__15*x1*x2);
	P33 = P__33;
	P34 = P__34;
	P35 = P__35;
	P44 = P__44;
	P45 = P__45;
	P55 = P__55;
   
   // increment covariance slightly at each iteration (nonoptimal but keeps the filter stable against rounding errors in 32bit double computation)
	P00 += VARIANCE_SAFETY_INCREMENT;
	P11 += VARIANCE_SAFETY_INCREMENT;
	P22 += VARIANCE_SAFETY_INCREMENT;
	P33 += VARIANCE_SAFETY_INCREMENT;
	P44 += VARIANCE_SAFETY_INCREMENT;
	P55 += VARIANCE_SAFETY_INCREMENT;

	// variance is required to be always at least the minimum value
	if (P00 < VARIANCE_MIN_LIMIT) P00 = VARIANCE_MIN_LIMIT;
	if (P11 < VARIANCE_MIN_LIMIT) P11 = VARIANCE_MIN_LIMIT;
	if (P22 < VARIANCE_MIN_LIMIT) P22 = VARIANCE_MIN_LIMIT;
	if (P33 < VARIANCE_MIN_LIMIT) P33 = VARIANCE_MIN_LIMIT;
	if (P44 < VARIANCE_MIN_LIMIT) P44 = VARIANCE_MIN_LIMIT;
	if (P55 < VARIANCE_MIN_LIMIT) P55 = VARIANCE_MIN_LIMIT;

	// normalized a posteriori state
	x0 = x0/len;
	x1 = x1/len;
	x2 = x2/len;
   
   // compute Euler angles (not exactly a part of the extended Kalman filter)
	// yaw integration through full rotation matrix
	double u_nb0 = u0 - x3;
	double u_nb1 = u1 - x4;
	double u_nb2 = u2 - x5;
   
   if (false) {
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
		yaw = atan2(R21_new,R11_new);
	}
	else { //alternative method estimating the whole rotation matrix
		//integrate full rotation matrix (using first row estimate in memory)

		// calculate the second row (sr) from a rotated first row (rotation with bias corrected gyroscope measurement)
		double sr0 = -fr1*x_last[2]+fr2*x_last[1]-h*(x_last[1]*(fr1*u_nb0-fr0*u_nb1)+x_last[2]*(fr2*u_nb0-fr0*u_nb2));
		double sr1 = fr0*x_last[2]-fr2*x_last[0]+h*(x_last[0]*(fr1*u_nb0-fr0*u_nb1)-x_last[2]*(fr2*u_nb1-fr1*u_nb2));
		double sr2 = -fr0*x_last[1]+fr1*x_last[0]+h*(x_last[0]*(fr2*u_nb0-fr0*u_nb2)+x_last[1]*(fr2*u_nb1-fr1*u_nb2));

		// normalize the second row
		double invlen = 1.0 / sqrt(sr0*sr0 + sr1*sr1 + sr2*sr2);
		sr0 *= invlen;
		sr1 *= invlen;
		sr2 *= invlen;

		// recompute the first row (ensure perpendicularity)
		fr0 = sr1*x_last[2] - sr2*x_last[1];
		fr1 = -sr0*x_last[2] + sr2*x_last[0];
		fr2 = sr0*x_last[1] - sr1*x_last[0];

		// normalize the first row
		invlen = 1.0 / sqrt(fr0*fr0 + fr1*fr1 + fr2*fr2);
		fr0 *= invlen;
		fr1 *= invlen;
		fr2 *= invlen;

		// calculate yaw from first and second row
		yaw = 180/M_PI*atan2(sr0,fr0);
	}   
   
   // compute new pitch and roll angles from a posteriori states
	pitch = 180/M_PI*asin(-x0);
	roll = 180/M_PI*atan2(x1,x2);

	// save the estimated non-gravitational acceleration
	a0 = (z0-x0)*g0;
	a1 = (z1-x1)*g0;
	a2 = (z2-x2)*g0;

}

