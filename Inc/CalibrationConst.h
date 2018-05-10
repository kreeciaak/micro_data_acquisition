/*
 * CalibrationConst.h
 *
 *  Created on: 9 maj 2018
 *      Author: User
 */

#ifndef CALIBRATIONCONST_H_
#define CALIBRATIONCONST_H_

#define RadToDegrees 	57.29578
#define RegisterToDPS 	250/32768


Vector3f MagR, AccR, Mag, Acc, Gyro;

static const Vector3f MagShift = {
			-79.568073f,
			-43.539449f,
			-59.246582f
		};

static const Vector3f AccShift = {
		-82.776597f,
		175.771870f,
		647.287009f
		};

static const Vector3f GyroShift = {
		-81.166333f,
		40.882f,
		-39.596f
		};

static const Matrix3f MagCalib = {
			{0.983101f, 0.019143f, 0.008105f},
			{0.019143f, 1.012514f, -0.003008f},
			{0.008105f, -0.003008f, 0.960453f}
		};

static const Matrix3f AccCalib = {
			{0.984432f, 0.001300f, -0.001628f},
			{0.001300f, 1.009410f, 0.000265f},
			{-0.001628f, 0.000265f, 1.023268f}
		};


static const float weight = 0.98;
static const float Q_angle = 0.001f;
static const float Q_bias = 0.003f;
static const float R_measure = 0.03f;


#endif /* CALIBRATIONCONST_H_ */
