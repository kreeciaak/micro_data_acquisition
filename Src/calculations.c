/*
 * calculations.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */
#include <calculations.h>
//#include <CalibrationConst.h>


void RawToResult(Vector3f Acc, Vector3f Gyro, Vector3f Mag, Result vResBuff)
{
	Vector3f AccR= {0,0,0}, MagR= {0,0,0};
	Vector3f AccO = {}, GyroO = {};
	Vector3f AccLP = {};
	Vector3f AccN = {0,0,0}, GyroN = {0,0,0}, GyroORad = {0,0,0};
	Vector3f GyroAngles = {0,0,0};

	Vector3f RawAngles= {0,0,0}, RawAnglesDeg = {0,0,0}, KalmanAngles = {0,0,0}, MadgwickAngles = {0,0,0}, MadgwickAnglesDeg = {0,0,0};
	Vector3f GyroAnglesRad = {0,0,0}, CFAnglesRad = {0,0,0}, KalmanAnglesRad = {0,0,0};

	Vector3f CFAnglesLP = {}, KalmanAnglesLP ={}, MadgwickAnglesLP = {};

	Vector3f AccTGyro = {0,0,0}, AccTCF = {0,0,0}, AccTKF = {0,0,0}, AccTMF = {0,0,0};
	Vector3f AccGRGyro = {0,0,0}, AccGRCF = {0,0,0}, AccGRKF = {0,0,0}, AccGRMF = {0,0,0};
	Vector3f AccGRGyroLP = {0,0,0}, AccGRCFLP = {0,0,0}, AccGRKFLP = {0,0,0}, AccGRMFLP = {0,0,0};
	Vector3f VelRecGyro = {0,0,0}, VelRecCF = {0,0,0}, VelRecKF = {0,0,0}, VelRecMF = {0,0,0};
	Vector3f PosRecGyro = {0,0,0}, PosRecCF = {0,0,0}, PosRecKF = {0,0,0}, PosRecMF = {0,0,0};
	Matrix3f GyroRotM 				= {{0,0,0},{0,0,0},{0,0,0}},
			 GyroRotMInv 			= {{0,0,0},{0,0,0},{0,0,0}},
			 ComplimentaryRotM 		= {{0,0,0},{0,0,0},{0,0,0}},
			 ComplimentaryRotMInv 	= {{0,0,0},{0,0,0},{0,0,0}},
			 KalmankRotM 			= {{0,0,0},{0,0,0},{0,0,0}},
			 KalmankRotMInv 		= {{0,0,0},{0,0,0},{0,0,0}},
			 MadgwickRotM 			= {{0,0,0},{0,0,0},{0,0,0}},
			 MadgwickRotMInv 		= {{0,0,0},{0,0,0},{0,0,0}};

	//statyczna inicjalizacja zmiennych
	static Vector3f AccOff = {}, GyroOff = {}, MagOff = {}, AccOff_temp = {}, GyroOff_temp = {};
	static Vector3f LPfilAccO = {0.0,0.0,Gravity2GRange};
	static Vector3f LPAccGRGyroPrev = {0,0,0}, LPAccGRCFPrev = {0,0,0}, LPAccGRKFPrev = {0,0,0}, LPAccGRMFPrev = {0,0,0};

	static Vector3f LPKalmanAnglesPrev = {}, LPCFAgnlesPrev = {}, LPMadgwickAnglesPrev ={};
	static Vector3f GyroAnglesBuff = {};
	static Vector3f bias = {0,0,0};
	static PKalman P = {{{0,0},{0,0}},{{0,0},{0,0}},{{0,0},{0,0}}};

	static Vector3f VelRecGyroBuff = {0,0,0}, VelRecCFBuff = {0,0,0}, VelRecKFBuff = {0,0,0}, VelRecMFBuff = {0,0,0};
	static Vector3f PosRecGyroBuff = {0,0,0}, PosRecCFBuff = {0,0,0}, PosRecKFBuff = {0,0,0}, PosRecMFBuff = {0,0,0};

	static int FirstOffCnt = 0, NextOffCnt = 0, zeroMoveCnt = 0;
	static int state = 1;
	static int RotCnt1 = 0, RotCnt2 = 0;
	static float MY_n = 0, siny_n = 0;
	static float quaternion[4] = {1.0,0.0,0.0,0.0};
	static bool isFirst = true;


	static Vector3f CFAngles = {0,0,0};
//	static AvBuffer AccBuff = {}, GyroBuff ={}, MagBuff = {};







	/*---KOREKCJA SUROWYCH DANYCH Z CZUJNIKOW---*/
	//Korekcja do sfery
	RawDataOrientationCorrection(Acc,AccShift,AccCalib, AccR);
	RawDataOrientationCorrection(Mag,MagShift,MagCalib, MagR);

	//Korekcja offsetu
	switch(state)
	{
	case 1:

		V3Sum(AccR, AccOff);
		V3Sum(Gyro, GyroOff);
		FirstOffCnt++;

		if (FirstOffCnt >= FirstOffLimit)
		{
			V3DivideConst(AccOff, AccOff, FirstOffLimit);
			V3DivideConst(GyroOff, GyroOff, FirstOffLimit);
			FirstOffCnt = 0;
			state = 2;
		}
		break;

	case 2:

		AccOffCalc(AccR, AccOff, AccO);
		MagOffsetCalc(MagR, AccO, MagOff);
		FirstOffCnt++;

		if (FirstOffCnt >= FirstOffLimit)
		{
			V3DivideConst(MagOff, MagOff, FirstOffLimit);
			FirstOffCnt = 0;
			state = 0;
		}
		break;

	case 3:

		AccOffCalc(AccR, AccOff, AccO);

		if (fabs(Norm(VectorTo2PowSum(AccO)))<83.0)
		{
			V3Sum(AccR, AccOff_temp);
			V3Sum(Gyro, GyroOff_temp);
			NextOffCnt++;

			if (NextOffCnt >= NextOffLimit)
			{
				V3DivideConst(AccOff, AccOff, NextOffLimit);
				V3DivideConst(GyroOff, GyroOff, NextOffLimit);
				NextOffCnt = 0;
				state = 0;
			}
		}else{
			state = 0;
			break;
		}
		break;

	default:

		/*---OFFSETOWANIE WARTOSCI ZYRO I AKCE, DLA AKCE OBROT WEKTORA---*/
		AccOffCalc(AccR, AccOff, AccO);
		V3Subtract(Gyro, GyroOff, GyroO);


		/*---WYG£ADZENIE PRZEBIEGÓW FUNKCJI POPRZEZ FILTR DOLNOPRZEPUSTOWY PROGRAMOWY---*/
		lowPassFilter(AccO, LPfilAccO, AccLP, LowPassCoef);


		/*---ZAMIANA ZNAKU WEKTORA PRZYSPIESZENIA - DLA POKRYCIA OSI ZYRO I AKCELEROMETRU---*/
		changeSignOfVector(AccLP, AccLP);
		AccLP[2] = -AccLP[2];

		/*---WYZNACZENIE WARTOSCI PARAMETROW W JEDNOSTKACH UKLADU SI ---*/
		NormaliseUnits(AccLP, GyroO, AccN, GyroN);


		/*---WYZNACZANIE KATA OBROTU---*/
		//Brak filtra
		IntegrationReactangleMethod(GyroN, GyroAnglesBuff, GyroAngles, sampleTime);

		//Wspolny poczatek dla filtrow: Kalmana, komplementarnego
		PitchRollYawMA(AccLP, MagR, RawAngles, &MY_n, &RotCnt1);
		RadiansToDegrees(RawAngles, RawAnglesDeg);

		//Utworzenie kierunku osi x jako kierunku odniesienia
		V3Subtract(RawAnglesDeg, MagOff, RawAnglesDeg);
		V3Subtract(RawAnglesDeg, MagOff2, RawAnglesDeg);

		//Filtr komplementarny
		ComplementaryFilter(CFAngles, GyroN, RawAnglesDeg, weight, sampleTime);
		lowPassFilter(CFAngles, LPCFAgnlesPrev, CFAnglesLP, LowPassCoef);

		//Filtr Kalamana
		KalmanFilter(RawAnglesDeg, GyroN, sampleTime, KalmanAngles, bias, P, isFirst);
		lowPassFilter(KalmanAngles, LPKalmanAnglesPrev, KalmanAnglesLP, LowPassCoef);

		//MadgwickAHRs
		DegreesToRadians(GyroN, GyroORad);
		MadgwickAHRSupdate(GyroORad[0],GyroORad[1],GyroORad[2],AccO[0],AccO[1],AccO[2],MagR[0],MagR[2],MagR[1],quaternion);
		QuaternionToEulerAngle(quaternion, MadgwickAngles, &siny_n, &RotCnt2);
		//lowPassFilter(MadgwickAngles, LPMadgwickAnglesPrev, MadgwickAnglesLP, LowPassCoef);
		RadiansToDegrees(MadgwickAngles, MadgwickAnglesDeg);


		//if (fabs(Norm(VectorTo2PowSum(AccN))-GravityConst) >= 0.02) //---> DO BADANIA ZDJ¥C NAWIASY I SPRAWDZIC CZY ZADZAIALA BO NA MALEJ SKALI CZASAMI JEST JAK W BEZRUCHU I BIERZE MAG
		//{
			CFAngles[2] = GyroAngles[2];
			CFAnglesLP[2] = GyroAngles[2];

			KalmanAngles[2] = GyroAngles[2];
			KalmanAnglesLP[2] = GyroAngles[2];
		//}


		/*---WYZNACZENIE ODWROTNEJ MACIERZY OBROTOW DLA KAZDEGO Z FILTROW---*/
		//Brak filtra
		changeSignOfVector(GyroAngles, GyroAngles);
		DegreesToRadians(GyroAngles, GyroAnglesRad);
		RotationMatrixFromAngles(GyroAnglesRad, GyroRotM);
		M3fInvert(GyroRotM, GyroRotMInv);

		//Filtr komplementarny
		changeSignOfVector(CFAnglesLP, CFAnglesLP);
		DegreesToRadians(CFAnglesLP, CFAnglesRad);
		//changeSignOfVector(CFAngles, CFAngles);
		//DegreesToRadians(CFAngles, CFAnglesRad);
		RotationMatrixFromAngles(CFAnglesRad, ComplimentaryRotM);
		M3fInvert(ComplimentaryRotM, ComplimentaryRotMInv);

		//Filtr Kalmana
		changeSignOfVector(KalmanAnglesLP, KalmanAnglesLP);
		DegreesToRadians(KalmanAnglesLP, KalmanAnglesRad);
		//changeSignOfVector(KalmanAngles, KalmanAngles);
		//DegreesToRadians(KalmanAngles, KalmanAnglesRad);
		RotationMatrixFromAngles(KalmanAnglesRad, KalmankRotM);
		M3fInvert(KalmankRotM, KalmankRotMInv);

		//MadgwickAHRS
		RotationMatrixFromQuaternion(quaternion, MadgwickRotM);
		M3fInvert(MadgwickRotM, MadgwickRotMInv);


		/*---OBROT WEKTORA PRZYSPIESZNIA WZGLEDEM MACIERZY OBROTOW---*/
		//Brak filtra
		V3fTransform(AccN, GyroRotMInv, AccTGyro);

		//Filtr komplementarny
		V3fTransform(AccN, ComplimentaryRotMInv, AccTCF);

		//Filtr Kalmana
		V3fTransform(AccN, KalmankRotMInv, AccTKF);

		//MadgwickAHRS
		V3fTransform(AccN, MadgwickRotMInv, AccTMF);

		/*---WYKRYCIE BRAKU RUCHU UK£ADU---*/

		if (fabs(Norm(VectorTo2PowSum(AccN))-GravityConst) <= 0.04)
		{
			for (int i=0;i<=2;i++)
			{
				AccTGyro[i] = GravityVector[i];
				AccTCF[i] = GravityVector[i];
				AccTKF[i] = GravityVector[i];
				AccTMF[i] = GravityVector[i];;
			}
//			AccTKF[0] = GravityVector[0];
//			AccTKF[1] = GravityVector[1];
//			AccTKF[2] = GravityVector[2];
			zeroMoveCnt++;
		}else{
			zeroMoveCnt = 0;
		}

		if (zeroMoveCnt > ZeroMoveTrig)
		{
			for (int i=0;i<=2;i++)
			{
				VelRecGyro[i] = 0;
				VelRecCF[i] = 0;
				VelRecKF[i] = 0;
				VelRecMF[i] = 0;
			}
//			VelRecKF[0] = 0;
//			VelRecKF[1] = 0;
//			VelRecKF[2] = 0;
		}

		if (zeroMoveCnt > 2*ZeroMoveTrig)
		{
			state = 3;
			break;
		}


		/*---REDUKCJA SKï¿½ADOWEJ GRAWITACJI Z WEKTORA PRZYSPIESZENIA---*/
		//Brak filtra
		V3Subtract(AccTGyro, GravityVector, AccGRGyro);
		lowPassFilter(AccGRGyro, LPAccGRGyroPrev, AccGRGyroLP, LowPassCoef);

		//Filtr komplementarny
		V3Subtract(AccTCF, GravityVector, AccGRCF);
		lowPassFilter(AccGRCF, LPAccGRCFPrev, AccGRCFLP, LowPassCoef);

		//Filtr Kalmana
		V3Subtract(AccTKF, GravityVector, AccGRKF);
		lowPassFilter(AccGRKF, LPAccGRKFPrev, AccGRKFLP, LowPassCoef);

		//MadgwickAHRS
		V3Subtract(AccTMF, GravityVector, AccGRMF);
		lowPassFilter(AccGRMF, LPAccGRMFPrev, AccGRMFLP, LowPassCoef);


		/*---WYZNACZENIE PREDKOSCI RUCHU POJAZDU NA PODSTAWIE WEKTORA PRZYSPIESZENIA - 2 METODY CALKOWANIA---*/
		//Brak filtra
		//IntegrationReactangleMethod(AccGRGyro, VelRecGyroBuff, VelRecGyro, sampleTime);
		IntegrationReactangleMethod(AccGRGyroLP, VelRecGyroBuff, VelRecGyro, sampleTime);;

		//Filtr komplementarny
		//IntegrationReactangleMethod(AccGRCF, VelRecCFBuff, VelRecCF, sampleTime);
		IntegrationReactangleMethod(AccGRCFLP, VelRecCFBuff, VelRecCF, sampleTime);

		//Filtr Kalmana
		//IntegrationReactangleMethod(AccGRKF, VelRecKFBuff, VelRecKF, sampleTime);
		IntegrationReactangleMethod(AccGRKFLP, VelRecKFBuff, VelRecKF, sampleTime);

		//MadgwickAHRS
		//IntegrationReactangleMethod(AccGRMF, VelRecMFBuff, VelRecMF, sampleTime);
		IntegrationReactangleMethod(AccGRMFLP, VelRecMFBuff, VelRecMF, sampleTime);


		/*---WYZNACZENIE TRAJEKTORII RUCHU POJAZDU NA PODSTAWIE WEKTORA PREDKOSCI - 2 METODY CALKOWANIA---*/
		//Brak filtra
		IntegrationReactangleMethod(VelRecGyro, PosRecGyroBuff, PosRecGyro, sampleTime);

		//Filtr komplementarny
		IntegrationReactangleMethod(VelRecCF, PosRecCFBuff, PosRecCF, sampleTime);

		//Filtr Kalmana
		IntegrationReactangleMethod(VelRecKF, PosRecKFBuff, PosRecKF, sampleTime);

		//MadgwickAHRS
		IntegrationReactangleMethod(VelRecMF, PosRecMFBuff, PosRecMF, sampleTime);

		//addVector3fToMatrix(GyroN, vResBuff, 0);
		//addVector3fToMatrix(GyroAngles, vResBuff, 1);

		addVector3fToMatrix(AccR, vResBuff, 0);
		addVector3fToMatrix(AccO, vResBuff, 1);
		addVector3fToMatrix(AccLP, vResBuff, 2);
		addVector3fToMatrix(AccN, vResBuff, 3);
//		addVector3fToMatrix(AccTKF, vResBuff, 4);
		addVector3fToMatrix(RawAnglesDeg, vResBuff, 4);
		addVector3fToMatrix(KalmanAngles, vResBuff, 5);
		addVector3fToMatrix(AccTKF, vResBuff, 6);
		addVector3fToMatrix(AccGRKFLP, vResBuff, 7);
//		addVector3fToMatrix(AccGRKF, vResBuff, 7);
		addVector3fToMatrix(VelRecKF, vResBuff, 8);
		addVector3fToMatrix(PosRecKF, vResBuff, 9);
		//addVector3fToMatrix(PosABKF, vResBuff, 8);

		break;
	}

}

void NoMovementDetection(Vector3f Acc, Vector3f Vel, int *state, int *zeroMoveCnt)
{
	if (fabs(Norm(VectorTo2PowSum(Acc))-GravityConst) <= 0.04)
		{
			Acc[0] = GravityVector[0];
			Acc[1] = GravityVector[1];
			Acc[2] = GravityVector[2];
			*zeroMoveCnt = *zeroMoveCnt + 1;
		}else{
			*zeroMoveCnt = 0;
		}

		if (*zeroMoveCnt > ZeroMoveTrig)
		{
			Vel[0] = 0;
			Vel[1] = 0;
			Vel[2] = 0;
		}

		if (*zeroMoveCnt > 2*ZeroMoveTrig)
		{
			*state = 3;
		}
}

void XYorietationCalibration(Vector3f DataOutput)
{
	Vector3f VecAcc = {}, VecMag = {}, AccR = {}, MagR = {},  AnglesRad = {};
	int16_t DataAccel[3] = {}, DataMag[3] = {};
	static float MY_n;
	static int RotCnt1;

	LSM_Accel_GetXYZ(DataAccel);
	LSM_Mag_GetXYZ(DataMag);

	intToVector3f(DataAccel, VecAcc);
	intToVector3f(DataMag, VecMag);

	RawDataOrientationCorrection(VecAcc,AccShift,AccCalib, AccR);
	RawDataOrientationCorrection(VecMag,MagShift,MagCalib, MagR);

	PitchRollYawMA(AccR, MagR, AnglesRad, &MY_n, &RotCnt1);
	RadiansToDegrees(AnglesRad, DataOutput);
}

float ZorientationCorrection(Vector3f Acc, Vector3f Gyro, Vector3f Mag)
{
	Vector3f AccR= {0,0,0}, GyroR= {0,0,0}, MagR= {0,0,0}, AccN = {0,0,0}, GyroN = {0,0,0}, angleRad = {}, angleDeg = {};
	Vector3f RawAngles= {0,0,0}, RawAnglesDeg = {0,0,0}, KalmanAngles = {0,0,0}, KalmanAnglesRad = {0,0,0}, AccTKF = {0,0,0};
	Matrix3f KalmankRotM = {{0,0,0},{0,0,0},{0,0,0}};
	volatile float normA = 0;


	static Vector3f bias = {0,0,0};
	static PKalman P = {{{0,0},{0,0}},{{0,0},{0,0}},{{0,0},{0,0}}};
	static bool isFirst = true;
	static float MY_n = 0, sum_angle = 0, angle = 0;
	static int RotCnt1 = 0, cnt = 0;

	RawDataOrientationCorrection(Acc,AccShift,AccCalib, AccR);
	RawDataOrientationCorrection(Mag,MagShift,MagCalib, MagR);
	V3Subtract(Gyro, GyroShift, GyroR);
	NormaliseUnits(AccR, GyroR, AccN, GyroN);
	changeSignOfVector(GyroN, GyroN);

	PitchRollYawMA(AccN, MagR, RawAngles, &MY_n, &RotCnt1);
	RadiansToDegrees(RawAngles, RawAnglesDeg);
//	V3Subtract(RawAnglesDeg, MagRotation1, RawAnglesDeg);
//	V3Subtract(RawAnglesDeg, MagRotation2, RawAnglesDeg);
	KalmanFilter(RawAnglesDeg, GyroN, sampleTime, KalmanAngles, bias, P, isFirst);

	DegreesToRadians(KalmanAngles, KalmanAnglesRad);
	RotationMatrixFromAngles(KalmanAnglesRad, KalmankRotM);

	V3fTransform(AccN, KalmankRotM, AccTKF);

	normA = Norm(VectorTo2PowSum(AccTKF));

	if (normA >= 10.0)
	{
		angleRad[2] = atan2f(AccTKF[1]/normA, AccTKF[0]/normA);
		RadiansToDegrees(angleRad, angleDeg);

		sum_angle += angleDeg[2];
		cnt++;
		angle = sum_angle/cnt;

		return angle;
	}else{
		return angle;
	}

}

void AccOffCalc(Vector3f AccR, Vector3f AccOff, Vector3f AccO)
{
	AccO[0] = AccR[0] - AccOff[0];
	AccO[1] = AccR[1] - AccOff[1];
	AccO[2] = AccR[2] - (AccOff[2]-Gravity2GRange);
}

void MagOffsetCalc(Vector3f MagR, Vector3f AccO, Vector3f DataOutput)
{
	Vector3f AnglesRad = {}, AnglesDeg = {};
	static float MY_n;
	static int RotCnt1;

	PitchRollYawMA(AccO, MagR, AnglesRad, &MY_n, &RotCnt1);
	RadiansToDegrees(AnglesRad, AnglesDeg);

	DataOutput[0] = 0;
	DataOutput[1] = 0;
	DataOutput[2] += AnglesDeg[2];
}


void RawDataOrientationCorrection(Vector3f V, const Vector3f VCorr, const Matrix3f MCorr, float *VRes) //dzia³a w chuj
{
	Vector3f VBuff;

	V3Subtract(V,VCorr,VBuff);
	V3fTransform(VBuff, MCorr, VRes);
}


void PitchRollYawMA(Vector3f AccCorr, Vector3f MagCorr, float *Angles, float *MY_n, int *x) //wstepne przeliczenie katow - argumenty to korygowane przyspieszenie i mag - float
{

	Angles[0] = atan2f(AccCorr[1],AccCorr[2]);
	Angles[1] = atan2f(-AccCorr[0],Norm(VectorTo2PowSum(AccCorr)));

	float normA = Norm(VectorTo2PowSum(AccCorr));
	float pitchA = asinf(-AccCorr[0]/normA);
	float rollA = asinf(AccCorr[1]/(normA*cos(pitchA)));

	float normM = Norm(VectorTo2PowSum(MagCorr));
	float mx = MagCorr[0]/normM;
	float my = MagCorr[2]/normM;
	float mz = MagCorr[1]/normM;

	float MX = mx*cosf(pitchA)+mz*sinf(pitchA);
	float MY = mx*sinf(rollA)*sinf(pitchA)+my*cosf(rollA)-mz*sinf(rollA)*cosf(pitchA);
	Angles[2] = atan2f(MY,MX);

	if ((checkSignofValue(*MY_n)!=checkSignofValue(MY)) && (MX < 0)){
		if (MY>=0) {*x = *x +1;}
		if (MY<0) {*x = *x - 1;}
	}
	if (*x!=0){
		Angles[2] = atan2f(MY,MX) -2**x*PI;
		*MY_n = MY;
	}else{
		Angles[2] = atan2f(MY,MX);
		*MY_n = MY;
	}
}

void ComplementaryFilter(float *CFAngles, Vector3f GyroN, Vector3f RawAngles, float weight, float dt) // filtr komplementarny - poprzednia wartosc kata, katy z zyro, surowe katy z acce i waga
{
	for (int i=0;i<=2;i++)
	{
		CFAngles[i] = weight*(CFAngles[i] + GyroN[i]*dt) + (1-weight)*RawAngles[i];
	}
}

void KalmanFilter(Vector3f RawAngle, Vector3f NewGyro, float dt, float *KalmanAngles, float *bias, PKalman P, bool isFirst) //funkcja musi pamiÃªtac sowje wartosci (bias, angle i macierz P), do tego potrzebna jest inicjalizacja tych zmiennych jako 0
{
	volatile Vector3f S = {0,0,0}, y = {0,0,0}, P00_temp = {0,0,0}, P01_temp = {0,0,0};

	if (isFirst){
		cpyVector3f(RawAngle, KalmanAngles);
		isFirst = false;
	};

	for (int i=0;i<=2;i++)
	{
		NewGyro[i] = NewGyro[i] - bias[i];
		KalmanAngles[i] += dt * NewGyro[i];

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P[i][0][0] += dt * (dt*P[i][1][1] - P[i][0][1] - P[i][1][0] + Q_angle);
		P[i][0][1] -= dt * P[i][1][1];
		P[i][1][0] -= dt * P[i][1][1];
		P[i][1][1] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		S[i] = P[i][0][0] + R_measure; // Estimate error
		/* Step 5 */
		volatile float K[2][3]; // Kalman gain - This is a 2x1 vector
		K[0][i] = P[i][0][0] / S[i];
		K[1][i] = P[i][1][0] / S[i];

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		y[i] = RawAngle[i] - KalmanAngles[i]; // Angle difference
		/* Step 6 */
		//angles[i] += K[0][i] * y[i];
		KalmanAngles[i] += K[0][i] * y[i];
		bias[i] += K[1][i] * y[i];

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		P00_temp[i] = P[i][0][0];
		P01_temp[i] = P[i][0][1];

		P[i][0][0] -= K[0][i] * P00_temp[i];
		P[i][0][1] -= K[0][i] * P01_temp[i];
		P[i][1][0] -= K[1][i] * P00_temp[i];
		P[i][1][1] -= K[1][i] * P01_temp[i];

	}
}


void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *quaternion) {

	volatile float beta = betaDef;								// 2 * proportional gain (Kp)
	volatile float q0 = quaternion[0], q1 = quaternion[1], q2 = quaternion[2], q3 = quaternion[3];

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, quaternion);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	quaternion[0] = q0;
	quaternion[1] = q1;
	quaternion[2] = q2;
	quaternion[3] = q3;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *quaternion) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float beta = betaDef, q0 = quaternion[0], q1 = quaternion[1], q2 = quaternion[2], q3 = quaternion[3];

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	quaternion[0] = q0;
	quaternion[1] = q1;
	quaternion[2] = q2;
	quaternion[3] = q3;
}

Vector3f *test() {
	static Vector3f x = {0,0,0};
	return &x;
}


