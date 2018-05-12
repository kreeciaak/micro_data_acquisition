/*
 * calculations.c
 *
 *  Created on: 6 maj 2018
 *      Author: User
 */
#include <calculations.h>
//#include <CalibrationConst.h>


void RawToResult(Vector3f Acc, Vector3f Gyro, Vector3f Mag, float *vResBuff)
{
	Vector3f AccR= {0,0,0}, GyroR= {0,0,0}, MagR= {0,0,0}, RawAngles= {0,0,0}, KalmanAngles = {0,0,0}, Madgwickangles = {0,0,0};

	//statyczna inicjalizacja zmiennych
	static Vector3f CFAngles = {0,0,0}; //czy to sie przypadkiem nie nadpisze przy kolejnym wywolaniu?st
	static Vector3f KalmanAngles = {0,0,0};
	static Vector3f GyroAngles= {0,0,0};
	static float quaternion = {1.0,0.0,0.0,0.0};

	//korekcja odczytow
	RawDataOrientationCorrection(Acc,AccShift,AccCalib, AccR);
	RawDataOrientationCorrection(Mag,MagShift,MagCalib, MagR);
	V3Subtract(Gyro, GyroShift, GyroR);

	//wyznaczenie katow obrotu
	PitchRollYawMA(AccR, MagR, RawAngles);
	GyroIntegral(GyroR, GyroAngles, 0.01);
	ComplementaryFilter(CFAngles,GyroAngles,RawAngles,weight);
	KalmanFilter(RawAngles, GyroR, 0.01, KalmanAngles);
	MadgwickAHRSupdate(GyroR[0],GyroR[1],GyroR[2],AccR[0],AccR[1],AccR[2],MagR[0],MagR[1],MagR[2],quaternion);
	QuaterniontoEulerAngle(quaternion, Madgwickangles);

}

void RawDataOrientationCorrection(Vector3f V, Vector3f VCorr, Matrix3f MCorr, float *VRes) //Argumenty - sutrowy wektor z czujnika (moze byc int), Wektor korekcyjny z calibration.h (float/double), macierz korekcji (float/dobule)
{
	Vector3f VBuff;

	V3Subtract(V,VCorr,VBuff);
	V3fTransform(VBuff, MCorr, VRes);
}

void PitchRollYawMA(Vector3f AccCorr, Vector3f MagCorr, float *Angles) //wstepne przeliczenie katow - argumenty to korygowane przyspieszenie i mag - float
{
	Angles[0] = atan2f(AccCorr[1],AccCorr[2]);
	Angles[1] = atan2f(-AccCorr[0],Norm(AccCorr));

	float normA = Norm(AccCorr);
	float pitchA = -asinf(AccCorr[0]/normA);
	float rollA = asinf(AccCorr[1]/(cos(pitchA)*normA));

	double normM = Norm(MagCorr);
	double mx = MagCorr[0]/normM;
	double my = -MagCorr[1]/normM;
	double mz = MagCorr[2]/normM;

	double MX = mx*cos(pitchA)+mz*sin(pitchA);
	double MY = mx*sin(rollA)*sin(pitchA)+my*cos(rollA)-mz*sin(rollA)*cos(pitchA);

	Angles[2] = atan2(-MY,MX);

	for (int i=0;i<=2;i++)
	{
		Angles[i] = Angles[i] * RadToDegrees;
	}
}

void GyroIntegral(Vector3f GyroCorr, float *GyroAngles, float Timestamp) //Ca³kowanie prêdkosci k¹towej - argument predkosc katowa korygowana, dotychczasowy k¹t, próbka czasu
{
	for (int i=0;i<=2;i++)
	{
		GyroAngles[i] = GyroAngles[i] * RadToDegrees;
		GyroCorr[i] = RegisterToDPS * GyroCorr[i];
		GyroAngles[i] += GyroCorr[i] * Timestamp;
	}
}

void ComplementaryFilter(float *CFAngles, Vector3f GyroAngles, Vector3f RawAngles, float weight) // filtr komplementarny - poprzednia wartosc kata, katy z zyro, surowe katy z acce i waga
{
	for (int i=0;i<=2;i++)
	{
		CFAngles[i] = weight*(CFAngles[i] + GyroAngles[i]) + (1-weight)*RawAngles[i];
	}
}

void KalmanFilter(Vector3f RawAngle, Vector3f NewGyro, float dt, float *KalmanAngles) //funkcja musi pamiêtac sowje wartosci (bias, angle i macierz P), do tego potrzebna jest inicjalizacja tych zmiennych jako 0
{
	static Vector3f bias = {0,0,0};
	static Vector3f angles = {RawAngle[0],RawAngle[1],RawAngle[2]};
	static float P = {{0,0},{0,0},{0,0,0}};

	for (int i=0;i<=2;i++)
	{
		NewGyro[i] = RegisterToDPS*NewGyro[i] - bias[i];
		angles[i] += dt * NewGyro[i];

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P[0][0][i] += dt * (dt*P[1][1][i] - P[0][1][i] - P[1][0][i] + Q_angle);
		P[0][1][i] -= dt * P[1][1][i];
		P[1][0][i] -= dt * P[1][1][i];
		P[1][1][i] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		float S[i] = P[0][0][i] + R_measure; // Estimate error
		/* Step 5 */
		float K[2][3]; // Kalman gain - This is a 2x1 vector
		K[0][i] = P[0][0][i] / S[i];
		K[1][i] = P[1][0][i] / S[i];

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		float y[i] = RawAngle[i] - angles[i]; // Angle difference
		/* Step 6 */
		angles[i] += K[0][i] * y[i];
		bias[i] += K[1][i] * y[i];

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		float P00_temp[i] = P[0][0][i];
		float P01_temp[i] = P[0][1][i];

		P[0][0][i] -= K[0][i] * P00_temp[i];
		P[0][1][i] -= K[0][i] * P01_temp[i];
		P[1][0][i] -= K[1][i] * P00_temp[i];
		P[1][1][i] -= K[1][i] * P01_temp[i];

		KalmanAngles[i] = angles[i];
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

