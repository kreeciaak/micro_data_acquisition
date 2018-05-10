/*
 * calculations.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */
#include <calculations.h>


void RawToResult(Vector3f Acc, Vector3f Gyro, Vector3f Mag, float *vResBuff)// funckja zawierajaca wsyztskie obliczenia - argumety to surowe dane z czujnika
{
	Vector3f AccR= {0.0,0.0,0.0}, GyroR= {0,0,0}, MagR= {0,0,0}, RawAngles= {0,0,0}, GyroAngles= {0,0,0};
	//statyczna inicjalizacja zmiennych
	static Vector3f CFAngles = {0,0,0};
	static Vector3f KalmanAngles = {0,0,0};

	RawDataOrientationCorrection(Acc,AccShift,AccCalib, AccR);
	RawDataOrientationCorrection(Mag,MagShift,MagCalib, MagR);
	V3Subtract(Gyro,GyroShift, GyroR);
	//na tym etapie trzeba by to rozbic znowu na wektory, chyba ¿e da siê przes³ac po prostu 3 wektory z tamtej funkcji zamiast macierzy
	//Za³o¿ny ze nowe wekory bêd¹ nazywac siê np. AccR
	PitchRollYawMA(AccR, MagR, RawAngles);


	GyroIntegral(GyroR, GyroAngles, 0.01);
	ComplementaryFilter(CFAngles,GyroAngles,RawAngles,weight); //weight pochodzi z calibration const
//	KalmanAngles[0] = KalmanFilter();

 //zwracana jest duza macierz 3-kolumnowa na t¹ chwile musi miec 7 wierszy (do Kalmana)
	//WSZYTSKIE WEKTORY Z WYNIKAMI PONI¯EJ RAWANGLES MUSZ¥ ZACHOWAC SWOJ¥ WARTOŒC DO NASTEPNEGO WYWOLANIA FUNKCJI BO ZACHODZI SUMOWANIE (CHYBA MUSZA BYC GLOBALNE)
}

void RawDataOrientationCorrection(Vector3f V, Vector3f VCorr, Matrix3f MCorr, float *VRes) //Argumenty - sutrowy wektor z czujnika (moze byc int), Wektor korekcyjny z calibration.h (float/double), macierz korekcji (float/dobule)
{
	Vector3f VBuff;

	V3Subtract(V,VCorr,VBuff); //trzeba bedzie rzutowac wartosc z czujnika na jakis float?
	V3fTransform(VBuff, MCorr, VRes);
}

void PitchRollYawMA(Vector3f AccCorr, Vector3f MagCorr, float *Angles) //wstepne przeliczenie katow - argumenty to korygowane przyspieszenie i mag - float
{
	Angles[0] = atan2f(AccCorr[1],AccCorr[2]);
	Angles[1] = atan2f(-AccCorr[0],Norm(AccCorr));

	float normA = Norm(AccCorr);
	float pitchA = -asinf(AccCorr[0]/normA);
	float rollA = asinf(AccCorr[1]/(cos(pitchA)*normA));
//
//	double normM = Norm(MagCorr);
//	double mx = MagCorr[0]/normM;
//	double my = -MagCorr[1]/normM;
//	double mz = MagCorr[2]/normM;
//
//	double MX = mx*cos(pitchA)+mz*sin(pitchA);
//	double MY = mx*sin(rollA)*sin(pitchA)+my*cos(rollA)-mz*sin(rollA)*cos(pitchA);
//
//	Angles[3] = atan2(-MY,MX);
//
//	for (int i=1;i<=3;i++)
//	{
//		Angles[i] = Angles[i] * RadToDegrees;
//	}
}

void GyroIntegral(Vector3f GyroCorr, float *GyroAngles, float Timestamp) //Ca³kowanie prêdkosci k¹towej - argument predkosc katowa korygowana, dotychczasowy k¹t, próbka czasu
{
	for (int i=1;i<=3;i++)
	{
		GyroAngles[i] = GyroAngles[i] * RadToDegrees;
		GyroCorr[i] = RegisterToDPS*GyroCorr[i];
		GyroAngles[i] += GyroCorr[i] * Timestamp;
	}
}


void ComplementaryFilter(float *CFAngles, Vector3f GyroAngles, Vector3f RawAngles, float weight) // filtr komplementarny - poprzednia wartosc kata, katy z zyro, surowe katy z acce i waga
{
	for (int i=1;i<=3;i++)
	{
		CFAngles[i] = weight*(CFAngles[i] + GyroAngles[i]) + (1-weight)*RawAngles[i];
	}
}

float KalmanFilter(float RawAngle, float NewGyro, float Timestamp) //funkcja musi pamiêtac sowje wartosci (bias, angle i macierz P), do tego potrzebna jest inicjalizacja tych zmiennych jako 0
{
	static float bias = 0;
	static float angle = 0;
	static Matrix3f P = {{0,0,0},{0,0,0},{0,0,0}};
	static float dt = 0.2;

	static float newAngle = 0; //?

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
