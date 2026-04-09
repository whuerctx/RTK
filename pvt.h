#pragma once
#include"struct.h"
using namespace std;

//检查星历是否有效
bool IsValid(const int Prn, const GNSSSys Sys, const GPSTIME* t, const GPSEPHREC* GPSEph);

//卫星钟差和钟速（含相对论改正）计算
int CompSatClkoff(const int Prn, const GNSSSys Sys, const GPSTIME* t, const GPSEPHREC* GPSEph, const GPSEPHREC* BDSEph, SATMIDRES* Mid);

// 牛顿迭代法求偏心异常
double ComputeEccentricAnomaly(double Mk, double e);

//GPS卫星位置、速度计算
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);

//BDS卫星位置、速度计算
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);

//卫星位置和速度的地球自转改正(pos为用户位置)
void EarthRotCorrect(SATMIDRES* Mid, double* pos);