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

Matrix3f MagCalib, AccCalib;
Vector3f MagR, AccR, Mag, Acc, Gyro, MagShift, AccShift, GyroShift; //Plik do wpisywania wartoœci kalibracji - nie wiem jak to zapisac zeby by³o dobrze
float weight;

MagShift[0] = -79.568073f;
MagShift[1] = -43.539449f;
MagShift[2] = -59.246582f;

AccShift[0] = -82.776597f;
AccShift[1] = 175.771870f;
AccShift[2] = 647.287009f;

GyroShift[0] = -81.166333f;
GyroShift[1] = 40.882f;
GyroShift[2] = -39.596f;

MagCalib[0][0] = 0.983101f;
MagCalib[0][1] = 0.019143f;
MagCalib[0][2] = 0.008105f;

MagCalib[1][0] = 0.019143f;
MagCalib[1][1] = 1.012514f;
MagCalib[1][2] = -0.003008f;

MagCalib[2][0] = 0.008105f;
MagCalib[2][1] = -0.003008f;
MagCalib[2][2] = 0.960453f;

AccCalib[0][0] = 0.984432f;
AccCalib[0][1] = 0.001300f;
AccCalib[0][2] = -0.001628f;

AccCalib[1][0] = 0.001300f;
AccCalib[1][1] = 1.009410f;
AccCalib[1][2] = 0.000265f;

AccCalib[2][0] = -0.001628f;
AccCalib[2][1] = 0.000265f;
AccCalib[2][2] = 1.023268f;

weight = 0.98;

Q_angle = 0.001f;
Q_bias = 0.003f;
R_measure = 0.03f;



#endif /* CALIBRATIONCONST_H_ */
