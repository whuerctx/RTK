#pragma once
#include"struct.h"
using namespace std;

//对流层改正
double Hopfield(const double H, const double Elev);

//粗差探测
void DetectOutlier(EPOCHOBSDATA* Obs);

//电离层改正
double Klobutchar(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3]);