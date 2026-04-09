#pragma once
#include"struct.h"
using namespace std;

//信号发射时刻卫星位置计算
void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA*Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double UserPos[3]);

//单点定位的函数
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, PPRESULT* ppresult);

//单点测速函数
void SPV(EPOCHOBSDATA* Epoch, PPRESULT* ppresult);