/*
 * vector.h
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */

#ifndef VECTOR_H_
#define VECTOR_H_

#include <math.h>
#include <stdlib.h>


#define PI 3.14159265358979323846f;
#define RadToDegrees 	57.29578

typedef float Vector3f[3];
typedef float Matrix3f[3][3];


void M3fMultiply(Matrix3f M1, Matrix3f M2, float **MRes);
void V3fTransform(Vector3f V, Matrix3f M, float *VRes);
void V3Subtract(Vector3f V1, Vector3f V2, float *VRes);
void RotationMatrixFromQuaternion(float *q, Matrix3f *RotM);
float M3fDefiner(Matrix3f M1);
int8_t M3fInvert(Matrix3f M1, Matrix3f *MInv);
float Norm(Vector3f V);
float VectorTo2PowSum(Vector3f V);
float invSqrt(float x);
void QuaterniontoEulerAngle(float *quaternion, Vector3f MagdwickAngles);

#endif /* VECTOR_H_ */
