/*
 * vector.h
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */

#ifndef VECTOR_H_
#define VECTOR_H_

#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"


#define PI 				3.14159265358979323846f;
#define RadToDegrees 	57.29578f
#define RegisterToDPS 	250/16384.0f
#define GravityConst	9.8126f //dla Poznania
#define Gravity2GRange	16384.0f

typedef float Vector3f[3];
typedef float Matrix3f[3][3];
typedef float Result[10][3];
typedef float PKalman[3][2][2];
typedef float AvBuffer[16][3];


void M3fMultiply(Matrix3f M1, Matrix3f M2, float **MRes);
void V3fTransform(Vector3f V, Matrix3f M, float *VRes);
void V3Subtract(Vector3f V1, const Vector3f V2, float *VRes);
void RotationMatrixFromQuaternion(float *q, Matrix3f RotM);
void RotationMatrixFromAngles(Vector3f Angles, Matrix3f RotM);
float M3fDefiner(Matrix3f M1);
float M3fInvert(Matrix3f M1, Matrix3f MInv);
void MovingAverage(Vector3f DataInput, AvBuffer Buffer, Vector3f DataOutput, int numofrows, int *cnt);
void IntegrationReactangleMethod(Vector3f DataInput, Vector3f DataBuff, Vector3f DataOutput, float Timestamp);
void IntegrationTrapezoidmethod(Vector3f DataInput, Vector3f Buffer, Vector3f DataOutput, float Timestamp);
void RadiansToDegrees(Vector3f AnglesInRadians, float *AnglesInDegrees);
void DegreesToRadians(Vector3f AnglesInDegrees, float *AnglesInRadians);
void NormaliseUnits(Vector3f Acc, Vector3f Gyro, float *AccN, float *GyroN);
void V3Sum(Vector3f DataInput, Vector3f DataOutput);
void V3DivideConst(Vector3f DataInput, Vector3f DataOutput, int divideby);
float VectorTo2PowSum(Vector3f V);
float Norm(float powervalue);
float invSqrt(float x);
void QuaternionToEulerAngle(float *quaternion, Vector3f MagdwickAngles, float *siny_n, int *x);
void cpyVector3f(float *in, float *out);
void intToVector3f(int16_t *in, float*out);
void addVector3fToRes(float *in, float *out);
void addVector3fToMatrix(Vector3f V1, Result M1, int row);
int checkSignofValue(float Value);
void changeSignOfVector(Vector3f V1, Vector3f V2);
void lowPassFilter(Vector3f AccO, Vector3f LP_prev, Vector3f AccLP, float coef);

#endif /* VECTOR_H_ */
