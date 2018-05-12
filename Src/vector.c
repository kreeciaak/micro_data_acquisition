/*
 * vector.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */
#include <vector.h>

void M3fMultiply(Matrix3f M1, Matrix3f M2, float **MRes) //Mno¿enie macierzy - ma zwracac macierz 3x3
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

void V3fTransform(Vector3f V, Matrix3f M, float *VRes) //Mnozenie macierzy i wektora - ma zwracac wektor
{
	VRes[0] = V[0]*M[0][0] + V[1]*M[1][0] + V[2]*M[2][0];
	VRes[1] = V[0]*M[0][1] + V[1]*M[1][1] + V[2]*M[2][1];
	VRes[2] = V[0]*M[0][2] + V[1]*M[1][2] + V[2]*M[2][2];
}

void V3Subtract(Vector3f V1, Vector3f V2, float *VRes) //Odejmowanie wektorów
{
	VRes[0] = V1[0] - V2[0];
	VRes[1] = V1[1] - V2[1];
	VRes[2] = V1[2] - V2[2];
}

float VectorTo2PowSum(Vector3f V) {
	float result = 0.0;
	for(int i = 0 ; i < 3 ; i++) {
		result += powf(V[i] ,2);
	}
	return result;
}

float Norm(Vector3f V) //tu chyba ok? ma zwracac modul wetkora
{
	float norm;
	norm = 0.2;
	norm = sqrtf(norm);
	return norm; //modul wektora
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void QuaterniontoEulerAngle(float *quaternion, Vector3f MagdwickAngles)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
	double cosr = +1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
	MagdwickAngles[0] = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]);
	if (fabs(sinp) >= 1)
		MagdwickAngles[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		MagdwickAngles[1] = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]);
	double cosy = +1.0 - 2.0 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
	MagdwickAngles[2] = atan2(siny, cosy);

	for (int i=0;i<=2;i++)
	{
		MagdwickAngles[i] = MagdwickAngles[i] * RadToDegrees;
	}
}


