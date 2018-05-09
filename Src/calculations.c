/*
 * calculations.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */


#include <stdlib.h>
#include <math.h>
#include <calculations.h>
#include <vector.h>

typedef float Vector3f[3];
typedef float Matrix3f[3][3];


void YawPitchRoll(Vector3f Acc, Vector3f Gyro, Vector3f Mag, Vector3f Angles)
{
	Matrix3f AccCorr, MagCorr, MagShift, AccShift, MagCalib, AccCalib;

	CorrectionValuesInit(MagShift,AccShift,MagCalib,AccCalib);
	RawDataOrientationCorrection(Acc,AccShift,AccCalib,Angles /*AccCorr*/);
	RawDataOrientationCorrection(Mag,MagShift,MagCalib,MagCorr);
}

void RawDataOrientationCorrection(Vector3f V, Vector3f VCorr, Matrix3f MCorr, Vector3f VRes)
{
	float VBuff[3];
	V3Subtract(V,VCorr,VBuff);
	V3fTransform(MCorr,VBuff,VRes);
}

void CorrectionValuesInit(Vector3f MagShift, Vector3f AccShift, Matrix3f MagCalib, Matrix3f AccCalib)
{
	MagShift[0] = 0.99f;
	MagShift[1] = 0.99f;
	MagShift[2] = 0.99f;

	AccShift[0] = 0.99f;
	AccShift[1] = 0.99f;
	AccShift[2] = 5000.99f;

	MagCalib[0][0] = 0.99f;
	MagCalib[0][1] = 0.99f;
	MagCalib[0][2] = 0.99f;

	MagCalib[1][0] = 0.99f;
	MagCalib[1][1] = 0.99f;
	MagCalib[1][2] = 0.99f;

	MagCalib[2][0] = 0.99f;
	MagCalib[2][1] = 0.99f;
	MagCalib[2][2] = 0.99f;

	AccCalib[0][0] = 0.99f;
	AccCalib[0][1] = 0.99f;
	AccCalib[0][2] = 0.99f;

	AccCalib[1][0] = 0.99f;
	AccCalib[1][1] = 0.99f;
	AccCalib[1][2] = 0.99f;

	AccCalib[2][0] = 0.99f;
	AccCalib[2][1] = 0.99f;
	AccCalib[2][2] = 0.99f;

}
