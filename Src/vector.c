/*
 * vector.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */


#include <stdlib.h>
#include <math.h>
#include <vector.h>

typedef float Vector3f[3];
typedef float Matrix3f[3][3];

void M3fMultiply(Matrix3f M1, Matrix3f M2, Matrix3f MRes)
{
	MRes[0][0]=M1[0][0]*M2[0][0] + M1[0][1]*M2[1][0] + M1[0][2]*M2[2][0];
	MRes[0][1]=M1[0][0]*M2[0][1] + M1[0][1]*M2[1][1] + M1[0][2]*M2[2][1];
	MRes[0][2]=M1[0][0]*M2[0][2] + M1[0][1]*M2[1][2] + M1[0][2]*M2[2][2];

	MRes[1][0]=M1[1][0]*M2[0][0] + M1[1][1]*M2[1][0] + M1[1][2]*M2[2][0];
	MRes[1][1]=M1[1][0]*M2[0][1] + M1[1][1]*M2[1][1] + M1[1][2]*M2[2][1];
	MRes[1][2]=M1[1][0]*M2[0][2] + M1[1][1]*M2[1][2] + M1[1][2]*M2[2][2];

	MRes[2][0]=M1[2][0]*M2[0][0] + M1[2][1]*M2[1][0] + M1[2][2]*M2[2][0];
	MRes[2][1]=M1[2][0]*M2[0][1] + M1[2][1]*M2[1][1] + M1[2][2]*M2[2][1];
	MRes[2][2]=M1[2][0]*M2[0][2] + M1[2][1]*M2[1][2] + M1[2][2]*M2[2][2];
}

void V3fTransform(Vector3f V, Matrix3f M, Vector3f VRes)
{
	VRes[0] = V[0]*M[0][0] + V[1]*M[1][0] + V[2]*M[2][0];
	VRes[1] = V[0]*M[0][1] + V[1]*M[1][1] + V[2]*M[2][1];
	VRes[2] = V[0]*M[0][2] + V[1]*M[1][2] + V[2]*M[2][2];
}

void V3Subtract(Vector3f V1, Vector3f V2, Vector3f VRes)
{
	VRes[0] = V1[0] - V2[0];
	VRes[1] = V1[1] - V2[1];
	VRes[2] = V1[2] - V2[2];
}
