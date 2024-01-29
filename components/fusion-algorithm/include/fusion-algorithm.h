//=============================================================================================
// SensorFusion.h
//=============================================================================================
//
// Madgwick's implementation of Mahony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
// 23/11/2017   Aster			Optimised time handling and melted in one library
//
//=============================================================================================
#pragma once

#include <math.h>
#include "esp_timer.h"
#include <string.h>
//--------------------------------------------------------------------------------------------
// Variable declaration


void computeAngles();

void
MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat);

void MahonyUpdate2(float gx, float gy, float gz, float ax, float ay, float az, float deltat);

void
MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat);

void MadgwickUpdate2(float gx, float gy, float gz, float ax, float ay, float az, float deltat);

// find initial Quaternios
// it is good practice to provide mean values from multiple measurements
bool initQuat(float ax, float ay, float az, float mx, float my, float mz);

//these values are already defined by arduino

float deltatUpdate();

float getRoll();

float getPitch();

float getYaw();

float getRollRadians();

float getPitchRadians();

float getYawRadians();

float *getQuat() ;

static float invSqrt(float x);

void vectorCross(float A[3], float B[3], float cross[3]);

void computeAngles();

