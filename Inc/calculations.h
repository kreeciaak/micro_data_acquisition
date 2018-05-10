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

typedef float Result[3][7];

void RawToResult(Vector3f Acc, Vector3f Gyro, Vector3f Mag, float *vResBuff);
void RawDataOrientationCorrection(Vector3f V, Vector3f VCorr, Matrix3f MCorr, float *VRes);
void PitchRollYawMA(Vector3f AccCorr, Vector3f MagCorr, float *Angles);
void GyroIntegral(Vector3f GyroCorr, float *GyroAngles, float Timestamp);
void ComplementaryFilter(float *CFAngles, Vector3f GyroAngles, Vector3f RawAngles, float weight);
float KalmanFilter(float RawAngle, float NewGyro, float Timestamp);



#endif /* CALCULATIONS_H_ */

