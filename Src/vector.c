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

void RotationMatrixFromQuaternion(float *q, Matrix3f *RotM)
{
	RotM[0][0] = 1 - 2*(q[2]^2 + q[3]^2);
	RotM[1][0] = 2 * (q[1]*q[2] + q[3]*q[0]);
	RotM[2][0] = 2 * (q[1]*q[3] - q[2]*q[0]);

	RotM[0][1] = 2 * (q[1]*q[2] - q[3]*q[0]);
	RotM[1][1] = 1 - 2*(q[1]^2 + q[3]^2);
	RotM[2][1] = 2 * (q[2]*q[3] + q[1]*q[0]);

	RotM[0][2] = 2 * (q[1]*q[3] + q[2]*q[0]);
	RotM[1][2] = 2 * (q[2]*q[3] - q[1]*q[0]);
	RotM[2][2] = 1- 2*(q[1]^2 + q[2]^2);
}

void RotationMatrixFromAngles(Vector3f Angles, Matrix3f *RotM) //Tait-Bryan angles - A3*A2*A1
{
	float yaw = 0, pitch = 0, roll = 0;
	for (int i=0;i<=2;i++)
	{
		Angles[i] = Angles[i]/RadToDegrees;
	}

	roll = Angles[0];
	pitch = Angles[1];
	yaw = Angles[2];


	RotM[0][0] = cos(yaw)*cos(pitch);
	RotM[1][0] = cos(pitch)*sin(yaw);
	RotM[2][0] = -sin(pitch);

	RotM[0][1] = cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw);
	RotM[1][1] = cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll);
	RotM[2][1] = cos(pitch)*sin(roll);

	RotM[0][2] = sin(yaw)*sin(roll)+cos(yaw)*cos(roll)*sin(pitch);
	RotM[1][2] = cos(roll)*sin(yaw)*sin(pitch)-cos(yaw)*sin(roll);
	RotM[2][2] = cos(pitch)*cos(roll);
}

float M3fDefiner(Matrix3f M1)
{
	float def;

	def =	M1[0][0] * M1[1][1] * M1[2][2]
		  + M1[1][0] * M1[0][2] * M1[2][1]
		  + M1[0][1] * M1[2][0] * M1[1][2]
		  - M1[2][0] * M1[1][1] * M1[0][2]
		  - M1[1][0] * M1[2][2] * M1[0][1]
		  - M1[2][1] * M1[0][0] * M1[1][2];

	return def;
}

int8_t M3fInvert(Matrix3f M1, Matrix3f *MInv)//do poprawki matematycznej
{
	float def = M3fDefiner(M1);
	if (def == 0)
		return 0;
	else
		def = 1.0f/def;

	MInv[0][0] = ( M1[1][1] * M1[2][2] - M1[1][2] * M1[2][1]) * def;
	MInv[1][0] = (-M1[1][0] * M1[2][2] + M1[1][2] * M1[2][0]) * def;
	MInv[2][0] = ( M1[1][0] * M1[2][1] - M1[1][1] * M1[2][0]) * def;

	MInv[0][1] = (-M1[0][1] * M1[2][2] + M1[0][2] * M1[2][1]) * def;
	MInv[1][1] = ( M1[0][0] * M1[2][2] - M1[0][2] * M1[2][0]) * def;
	MInv[2][1] = (-M1[0][0] * M1[2][1] + M1[0][1] * M1[2][0]) * def;

	MInv[0][2] = ( M1[0][1] * M1[1][2] - M1[0][2] * M1[1][1]) * def;
	MInv[1][2] = (-M1[0][0] * M1[1][2] + M1[0][2] * M1[1][0]) * def;
	MInv[2][2] = ( M1[0][0] * M1[1][1] - M1[0][1] * M1[1][0]) * def;

	return 1;
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


