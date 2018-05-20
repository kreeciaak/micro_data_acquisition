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

void V3Subtract(Vector3f V1, const Vector3f V2, float *VRes)
{
	VRes[0] = V1[0] - V2[0];
	VRes[1] = V1[1] - V2[1];
	VRes[2] = V1[2] - V2[2];
}

void RotationMatrixFromQuaternion(float *q, Matrix3f RotM) //q bez wskaznika? , tez sie cos jebei
{
	RotM[0][0] = 1 - 2*(powf(q[2],2) + powf(q[3], 2));
	RotM[1][0] = 2 * (q[1]*q[2] + q[3]*q[0]);
	RotM[2][0] = 2 * (q[1]*q[3] - q[2]*q[0]);

	RotM[0][1] = 2 * (q[1]*q[2] - q[3]*q[0]);
	RotM[1][1] = 1 - 2*(powf(q[1],2) + powf(q[3], 2));
	RotM[2][1] = 2 * (q[2]*q[3] + q[1]*q[0]);

	RotM[0][2] = 2 * (q[1]*q[3] + q[2]*q[0]);
	RotM[1][2] = 2 * (q[2]*q[3] - q[1]*q[0]);
	RotM[2][2] = 1- 2*(powf(q[1],2) + powf(q[2],2));
}

//void RotationMatrixFromAngles(Vector3f Angles, float **RotM) //Tait-Bryan angles - A3*A2*A1
void RotationMatrixFromAngles(Vector3f Angles, Matrix3f RotM)
{
	volatile float yaw = Angles[2], pitch = Angles[1], roll = Angles[0];

	RotM[0][0] = cosf(yaw)*cosf(pitch);
	RotM[1][0] = cosf(pitch)*sinf(yaw);
	RotM[2][0] = -sinf(pitch);

	RotM[0][1] = cosf(yaw)*sinf(pitch)*sinf(roll)-cosf(roll)*sinf(yaw);
	RotM[1][1] = cosf(yaw)*cosf(roll)+sinf(yaw)*sinf(pitch)*sinf(roll);
	RotM[2][1] = cosf(pitch)*sinf(roll);

	RotM[0][2] = sinf(yaw)*sinf(roll)+cosf(yaw)*cosf(roll)*sinf(pitch);
	RotM[1][2] = cosf(roll)*sinf(yaw)*sinf(pitch)-cosf(yaw)*sinf(roll);
	RotM[2][2] = cosf(pitch)*cosf(roll);
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

float M3fInvert(Matrix3f M1, Matrix3f MInv)
{
	float def = M3fDefiner(M1);
	if (def == 0){
		return 0;
	}else{
		def = 1.0f/def;

		MInv[0][0] = ( M1[1][1] * M1[2][2] - M1[1][2] * M1[2][1]) *def;
		MInv[1][0] = (-M1[1][0] * M1[2][2] + M1[1][2] * M1[2][0]) *def;
		MInv[2][0] = ( M1[1][0] * M1[2][1] - M1[1][1] * M1[2][0]) *def;

		MInv[0][1] = (-M1[0][1] * M1[2][2] + M1[0][2] * M1[2][1]) *def;
		MInv[1][1] = ( M1[0][0] * M1[2][2] - M1[0][2] * M1[2][0]) *def;
		MInv[2][1] = (-M1[0][0] * M1[2][1] + M1[0][1] * M1[2][0]) *def;

		MInv[0][2] = ( M1[0][1] * M1[1][2] - M1[0][2] * M1[1][1]) *def;
		MInv[1][2] = (-M1[0][0] * M1[1][2] + M1[0][2] * M1[1][0]) *def;
		MInv[2][2] = ( M1[0][0] * M1[1][1] - M1[0][1] * M1[1][0]) *def;

		return 1;
	}
}

void IntegrationReactangleMethod(Vector3f DataInput, float *DataOutput, float Timestamp)
{
	for (int i=0;i<=2;i++)
	{
		DataOutput[i] += DataInput[i] * Timestamp;
	}
}

void IntegratioAdamsBashworthMethod(Vector3f DataInput,float fv[][5], float *DataOutput, float Timestamp, int order)
{
	for (int i=0;i<=2;i++)
	{
		if (order<=3)
		{
			fv[order][i] = DataInput[i];
			DataOutput[i] += fv[order][i] * Timestamp;
			//IntegrationReactangleMethod(fv[order][i], DataOutput[i], Timestamp);
			order = order + 1;
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

void QuaternionToEulerAngle(float *quaternion, Vector3f MagdwickAngles, float *siny_n, int *x)
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
	//MagdwickAngles[2] = atan2(siny, cosy);

	if ((checkSignofValue(*siny_n)!=checkSignofValue(siny)) && (cosy < 0)){
		if (siny>=0) {*x = *x +1;}
		if (siny<0) {*x = *x - 1;}
	}
	if (*x!=0){
		MagdwickAngles[2] = atan2f(siny,cosy) -2**x*PI;
		*siny_n = siny;
	}else{
		MagdwickAngles[2] = atan2f(siny,cosy);
		*siny_n = siny;
	}
}

void cpyVector3f(float *in, float *out){
	for(int i = 0 ; i < 3; i++){
		*out = *in;
		out++;
		in++;
	}
}

void intToVector3f(int16_t *in, float*out){
	for(int i = 0 ; i < 3; i++){
		*out = (float) *in;
		out++;
		in++;
	}
}

void addVector3fToRes(float *in, float *out){
	cpyVector3f(in,out);
}

void addVector3fToMatrix(Vector3f V1, Result M1, int row)
{
	M1[row][0] = V1[0];
	M1[row][1] = V1[1];
	M1[row][2] = V1[2];
}

int checkSignofValue(float Value)
{
	if (Value >= 0){
		return 1;
	}else{
		return 0;
	}
}

