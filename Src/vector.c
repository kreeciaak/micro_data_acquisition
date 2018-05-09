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
//Po zmianach trzeba zmienic prototypy w pliku vector.h

void M3fMultiply(Matrix3f M1, Matrix3f M2) //Mno¿enie macierzy - ma zwracac macierz 3x3
{
	float MRes[3][3];

	MRes[0][0]=M1[0][0]*M2[0][0] + M1[0][1]*M2[1][0] + M1[0][2]*M2[2][0];
	MRes[0][1]=M1[0][0]*M2[0][1] + M1[0][1]*M2[1][1] + M1[0][2]*M2[2][1];
	MRes[0][2]=M1[0][0]*M2[0][2] + M1[0][1]*M2[1][2] + M1[0][2]*M2[2][2];

	MRes[1][0]=M1[1][0]*M2[0][0] + M1[1][1]*M2[1][0] + M1[1][2]*M2[2][0];
	MRes[1][1]=M1[1][0]*M2[0][1] + M1[1][1]*M2[1][1] + M1[1][2]*M2[2][1];
	MRes[1][2]=M1[1][0]*M2[0][2] + M1[1][1]*M2[1][2] + M1[1][2]*M2[2][2];

	MRes[2][0]=M1[2][0]*M2[0][0] + M1[2][1]*M2[1][0] + M1[2][2]*M2[2][0];
	MRes[2][1]=M1[2][0]*M2[0][1] + M1[2][1]*M2[1][1] + M1[2][2]*M2[2][1];
	MRes[2][2]=M1[2][0]*M2[0][2] + M1[2][1]*M2[1][2] + M1[2][2]*M2[2][2];

	return MRes; //Macierz3x3 M
}

void V3fTransform(Vector3f V, Matrix3f M) //Mnozenie macierzy i wektora - ma zwracac wektor
{
	float VRes;

	VRes[0] = V[0]*M[0][0] + V[1]*M[1][0] + V[2]*M[2][0];
	VRes[1] = V[0]*M[0][1] + V[1]*M[1][1] + V[2]*M[2][1];
	VRes[2] = V[0]*M[0][2] + V[1]*M[1][2] + V[2]*M[2][2];

	return VRes; //wektor
}

void V3Subtract(Vector3f V1, Vector3f V2) //Odejmowanie wektorów
{
	float VRes;

	VRes[0] = V1[0] - V2[0];
	VRes[1] = V1[1] - V2[1];
	VRes[2] = V1[2] - V2[2];

	return VRes; //wektor wynikowy
}

float Norm(Vector3f V) //tu chyba ok? ma zwracac modul wetkora
{
	float norm = sqrt(V[1]^2+V[2]^2+V[3]);
	return norm; //modul wektora
}
