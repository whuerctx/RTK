#pragma once
#include"struct.h"
using namespace std;

//大地坐标到笛卡尔坐标
void BLHToXYZ(const BLH* blh, XYZ* xyz, const double a, const double e);

//笛卡尔坐标到大地坐标
void XYZToBLH(const XYZ* xyz, BLH* blh, const double a, const double e);

//测站地平坐标转换矩阵计算函数
void BLHToNEUMat(const BLH* blh, double* mat);

//笛卡尔坐标到测站地平坐标
void XYZToENU(const XYZ* Xr, const XYZ* Xs, ENU* enu, const double a, const double e);

//卫星高度角方位角计算函数
void CompSatElAz(const XYZ* Xr, const XYZ* Xs, double* Elev, double* Azim);

//定位误差计算函数
void CompEnudPos(const XYZ* X0, const XYZ* Xr, ENU* dEnu);

//打印坐标
void CoordinatePrint(BLH* blh);
void CoordinatePrint(XYZ* xyz);
void CoordinatePrint(ENU* enu);