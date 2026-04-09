#pragma once
#include"struct.h"
using namespace std;

//单差观测值
void FormSDEpochObs(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs);

//周跳探测
void DetectCycleSlip(SDEPOCHOBS* Obs);