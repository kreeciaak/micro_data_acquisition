/*
 * calculations.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */


#include <stdlib.h>
#include <math.h>
#include <calculations.h>
#include <vector.h>
#include <CalibrationConst.h>

typedef float Vector3f[3];
typedef float Matrix3f[3][3];

void RawToResult(Vector3f Acc, Vector3f Gyro, Vector3f Mag)// funckja zawierajaca wsyztskie obliczenia - argumety to surowe dane z czujnika
{
	Vector3f AccR, GyroR, MagR, RawAngles, GyroAngles, CFAngles, KalmanAngles;
	Macierz_Acc_Gyro_Mag[3][3] = RawDataCorrection(Acc, Gyro, Mag);
	//na tym etapie trzeba by to rozbic znowu na wektory, chyba ¿e da siê przes³ac po prostu 3 wektory z tamtej funkcji zamiast macierzy
	//Za³o¿ny ze nowe wekory bêd¹ nazywac siê np. AccR
	RawAngles = PitchRollYawMA(AccR, MagR);

	if (CFAngles[0] == 0 && CFAngles[1] == 0 && CFAngles[2] == 0 && KalmanAngles[0] == 0 && KalmanAngles[1] == 0 && KalmanAngles[2] == 0) //Wykonanie inicjalizacji tylko dla pierwszego wywolania funckji
	{
		CFAngles = RawAngles;
		KalmanAngles = RawAngles;
	}

	GyroAngles = GyroIntegral(GyroR);
	CFAngles = ComplementaryFilter(CFAngles,GyroAngles,RawAngles,weight); //weight pochodzi z calibration const
	KalmanAngles[0] = KalmanFilter();


	return //zwracana jest duza macierz 3-kolumnowa na t¹ chwile musi miec 7 wierszy (do Kalmana)
	//WSZYTSKIE WEKTORY Z WYNIKAMI PONI¯EJ RAWANGLES MUSZ¥ ZACHOWAC SWOJ¥ WARTOŒC DO NASTEPNEGO WYWOLANIA FUNKCJI BO ZACHODZI SUMOWANIE (CHYBA MUSZA BYC GLOBALNE)
}

void RawDataCorrection(Vector3f Acc, Vector3f Gyro, Vector3f Mag) //funkcja koryguj¹ca - argumenty to odczyty z czujnikow w postaci wektorów(wartosci rejestrow - int)
{
	Matrix3f AccCorr, MagCorr, GyroCorr, MagShift, GyroShift, AccShift, MagCalib, AccCalib;
	Vector3f Angles;

	AccCorr = RawDataOrientationCorrection(Acc,AccShift,AccCalib);
	MagCorr = RawDataOrientationCorrection(Mag,MagShift,MagCalib);
	GyroCorr = V3Subtract(Gyro,GyroShift);

	return //funcja zwraca 3 nowe wektory danych (takie jak wejsciowe) - moze byc macierz 3x3
}

void RawDataOrientationCorrection(Vector3f V, Vector3f VCorr, Matrix3f MCorr) //Argumenty - sutrowy wektor z czujnika (moze byc int), Wektor korekcyjny z calibration.h (float/double), macierz korekcji (float/dobule)
{
	float VBuff[3];
	float VRes;

	V3Subtract(V,VCorr,VBuff); //trzeba bedzie rzutowac wartosc z czujnika na jakis float?
	V3fTransform(MCorr,VBuff,VRes);

	return VRes; //zwrot - wektor z czujnika po korekcji (float/double)
}

PitchRollYawMA(Vector3f AccCorr, Vector3f MagCorr) //wstepne przeliczenie katow - argumenty to korygowane przyspieszenie i mag - float
{
	Vector3f Angles;
	Angles[0] = atan2(AccCorr[1],AccCorr[2]);
	Angles[1] = atan2(-AccCorr[0],Norm(AccCorr));

	double normA = Norm(AccCorr);
	double pitchA = -asin(AccCorr[0]/normA);
	double rollA = asin(AccCorr[1]/(cos(pitchA)*normA));

	double normM = Norm(MagCorr);
	double mx = MagCorr[0]/normM;
	double my = -MagCorr[1]/normM;
	double mz = MagCorr[2]/normM;

	double MX = mx*cos(pitchA)+mz*sin(pitchA);
	double MY = mx*sin(rollA)*sin(pitchA)+my*cos(rollA)-mz*sin(rollA)*cos(pitchA);

	Angles[3] = atan2(-MY,MX);

	for (int i=1;i<=3;i++)
	{
		Angles[i] = Angles[i] * RadToDegrees;
	}


	return Angles; // zwraca katy w stopniach - float wektor
}

GyroIntegral(Vector3f GyroCorr, Vector3f GyroAngles, float Timestamp) //Ca³kowanie prêdkosci k¹towej - argument predkosc katowa korygowana, dotychczasowy k¹t, próbka czasu
{
	for (int i=1;i<=3;i++)
	{
		GyroAngles[i] = GyroAngles[i] * RadToDegrees;
		GyroCorr = RegisterToDPS*GyroCorr;
		GyroAngles[i] += GyroCorr[i] * Timestamp;
	}

	return GyroAngles; // zwraca po calkowaniu zyroskopu w stopniach - wektor
}


ComplementaryFilter(Vector3f CFAngles, Vector3f GyroAngles, Vector3f RawAngles, float weight) // filtr komplementarny - poprzednia wartosc kata, katy z zyro, surowe katy z acce i waga
{
	for (int i=1;i<=3;i++)
	{
		CFAngles[i] = weight*(CFAngles[i] + GyroAngles[i]) + (1-weight)*RawAngles[i];
	}

	return CFAngles; //funkcja zwaraca k¹ty w stopniach - wektor
}

float KalmanFilter(float RawAngle, float NewGyro, float Timestamp) //funkcja musi pamiêtac sowje wartosci (bias, angle i macierz P), do tego potrzebna jest inicjalizacja tych zmiennych jako 0
{
	for (int i=1;i<=3;i++)
	{
		NewGyro = NewGyro - bias;
		angle += Timestamp * NewGyro;
	}


	    // Update estimation error covariance - Project the error covariance ahead
	    /* Step 2 */
	    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	    P[0][1] -= dt * P[1][1];
	    P[1][0] -= dt * P[1][1];
	    P[1][1] += Q_bias * dt;

	    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	    // Calculate Kalman gain - Compute the Kalman gain
	    /* Step 4 */
	    float S = P[0][0] + R_measure; // Estimate error
	    /* Step 5 */
	    float K[2]; // Kalman gain - This is a 2x1 vector
	    K[0] = P[0][0] / S;
	    K[1] = P[1][0] / S;

	    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
	    /* Step 3 */
	    float y = newAngle - angle; // Angle difference
	    /* Step 6 */
	    angle += K[0] * y;
	    bias += K[1] * y;

	    // Calculate estimation error covariance - Update the error covariance
	    /* Step 7 */
	    float P00_temp = P[0][0];
	    float P01_temp = P[0][1];

	    P[0][0] -= K[0] * P00_temp;
	    P[0][1] -= K[0] * P01_temp;
	    P[1][0] -= K[1] * P00_temp;
	    P[1][1] -= K[1] * P01_temp;

	    return angle;
}



