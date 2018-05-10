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



