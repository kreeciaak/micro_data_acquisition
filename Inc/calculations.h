/*
 * calculations.h
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */

#ifndef CALCULATIONS_H_
#define CALCULATIONS_H_

#include <stdlib.h>
//#include <math.h>
#include <vector.h>
#include <CalibrationConst.h>

typedef enum { false, true } bool;



extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

void XYorietationCalibration(Vector3f DataOutput);
float ZorientationCorrection(Vector3f Acc, Vector3f Gyro, Vector3f Mag);
void AccelOffsetCalculation(Vector3f DataOutput);
void GyroOffsetCalculation(Vector3f DataOutput);
void MagOffsetCalculation(Vector3f DataOutput);
void RawToResult(Vector3f Acc, Vector3f Gyro, Vector3f Mag, Vector3f MagOffset, Result vResBuff);
void RawDataOrientationCorrection(Vector3f V, const Vector3f VCorr, const Matrix3f MCorr, float *VRes);
void PitchRollYawMA(Vector3f AccCorr, Vector3f MagCorr, float *Angles, float *MY_n, int *x);
//void ComplementaryFilter(float *CFAngles, Vector3f GyroAngles, Vector3f RawAngles, float weight);
void ComplementaryFilter(float *CFAngles, Vector3f GyroN, Vector3f RawAngles, float weight, float dt);
//void KalmanFilter(Vector3f RawAngle, Vector3f NewGyro, float Timestamp, float *KalmanAngles);
void KalmanFilter(Vector3f RawAngle, Vector3f NewGyro, float dt, float *KalmanAngles, float *bias, PKalman P, bool isFirst);

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *quaternion);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *quaternion);

Vector3f *test();

#endif /* CALCULATIONS_H_ */

