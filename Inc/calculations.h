/*
 * calculations.h
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */

#ifndef CALCULATIONS_H_
#define CALCULATIONS_H_



#endif /* CALCULATIONS_H_ */

typedef float Vector3f[3];
typedef float Matrix3f[3][3];

void YawPitchRoll(Vector3f Acc, Vector3f Gyro, Vector3f Mag, Vector3f Angles);
void RawDataOrientationCorrection(Vector3f V, Vector3f VCorr, Matrix3f MCorr, Vector3f VRes);
void CorrectionValuesInit(Vector3f MagShift, Vector3f AccShift, Matrix3f MagCalib, Matrix3f AccCalib);
