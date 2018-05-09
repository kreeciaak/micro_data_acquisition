/*
 * vector.h
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */

#ifndef VECTOR_H_
#define VECTOR_H_


#define PI 3.14159265358979323846f;


#endif /* VECTOR_H_ */

typedef float Vector3f[3];
typedef float Matrix3f[3][3];

void M3fMultiply(Matrix3f M1, Matrix3f M2, Matrix3f MRes);
void V3fTransform(Vector3f V, Matrix3f M, Vector3f VRes);
void V3Subtract(Vector3f V1, Vector3f V2, Vector3f VRes);
float Norm(Vector3f V);
