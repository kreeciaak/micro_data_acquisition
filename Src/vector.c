/*
 * vector.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */
#include <vector.h>

void M3fMultiply(Matrix3f M1, Matrix3f M2, float **MRes)
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

void V3fTransform(Vector3f V, Matrix3f M, float *VRes)
{
	VRes[0] = V[0]*M[0][0] + V[1]*M[0][1] + V[2]*M[0][2];
	VRes[1] = V[0]*M[1][0] + V[1]*M[1][1] + V[2]*M[1][2];
	VRes[2] = V[0]*M[2][0] + V[1]*M[2][1] + V[2]*M[2][2];
}

void V3Subtract(Vector3f V1, Vector3f V2, float *VRes)
{
	VRes[0] = V1[0] - V2[0];
	VRes[1] = V1[1] - V2[1];
	VRes[2] = V1[2] - V2[2];
}

void RotationMatrixFromQuaternion(float *q, float **RotM)
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

void RotationMatrixFromAngles(Vector3f Angles, float **RotM) //Tait-Bryan angles - A3*A2*A1
{
	float yaw = 0, pitch = 0, roll = 0;

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

float M3fInvert(Matrix3f M1, float **MInv)
{
	float def = M3fDefiner(M1);
	if (def == 0)
		return 0;
	else
		def = 1.0f/def;

	MInv[0][0] = ( M1[1][1] * M1[2][2] - M1[1][2] * M1[2][1]) * def;
	MInv[1][0] = (-M1[0][1] * M1[2][2] + M1[0][2] * M1[2][1]) * def;
	MInv[2][0] = ( M1[0][1] * M1[1][2] - M1[0][2] * M1[1][1]) * def;

	MInv[0][1] = (-M1[1][0] * M1[2][2] + M1[1][2] * M1[2][0]) * def;
	MInv[1][1] = ( M1[0][0] * M1[2][2] - M1[0][2] * M1[2][0]) * def;
	MInv[2][1] = (-M1[0][0] * M1[1][2] + M1[0][2] * M1[1][0]) * def;

	MInv[0][2] = ( M1[1][0] * M1[2][1] - M1[1][1] * M1[2][0]) * def;
	MInv[1][2] = (-M1[0][0] * M1[2][1] + M1[0][1] * M1[2][0]) * def;
	MInv[2][2] = ( M1[0][0] * M1[1][1] - M1[0][1] * M1[1][0]) * def;

	return 1;
}

void IntegrationReactangleMethod(Vector3f DataInput, float *DataOutput, float Timestamp)
{
	for (int i=0;i<=2;i++)
	{
		DataOutput[i] += DataInput[i] * Timestamp;
	}
}

void IntegratioAdamsBashworthMethod(Vector3f DataInput, float *DataOutput, float Timestamp)
{
	static int order = 0;
	static float fv = {{0,0,0,0,0},{0,0,0}};
	for (int i=0;i<=2;i++)
	{
		if (order<=3)
		{
			fv[order][i] = DataInput[i];
			IntegrationReactangleMethod(fv[order][i], DataOutput[i], Timestamp);
			order++;
		}else{

			fv[4][i] = DataInput[i];
			DataOutput[i] += (Timestamp/1440) * (1901*fv[4][i] - 2774*fv[3][i] + 2616*fv[2][i] - 1274*fv[1][i] + 251*fv[0][i]);

			for (order=0;order<=3;order++)
			{
				fv[order][i] = fv[order+1][i];
			}
		}
	}
}

void RadiansToDegrees(Vector3f AnglesInRadians, float *AnglesInDegrees)
{
	for (int i=0;i<=2;i++)
	{
		AnglesInDegrees[i] = AnglesInRadians[i] * RadToDegrees;
	}
}

void DegreesToRadians(Vector3f AnglesInDegrees, float *AnglesInRadians)
{
	for (int i=0;i<=2;i++)
	{
		AnglesInRadians[i] = AnglesInDegrees[i] / RadToDegrees;
	}
}

void NormaliseUnits(Vector3f Acc, Vector3f Gyro, float *AccN, float *GyroN)
{
	for (int i=0;i<=2;i++)
	{
		AccN[i] = Acc[i] * GravityConst/Gravity2GRange;
		GyroN[i] = Gyro[i] * RegisterToDPS;
	}
}

float VectorTo2PowSum(Vector3f V) {
	float result = 0.0;
	for(int i=0;i<=2;i++) {
		result += powf(V[i] ,2);
	}
	return result;
}

float Norm(float powervalue)
{
	float norm;
	norm = powervalue;
	norm = sqrtf(norm);
	return norm;
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
}


