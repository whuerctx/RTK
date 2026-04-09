#pragma once
#include"struct.h"
#include"sockets.h"
#include"decode.h"
using namespace std;

//时间求差函数
double TimeDiff(const GPSTIME* t1, const GPSTIME* t2);

//获取时间同步数据
int GetSynObs(FILE* FBas, FILE* FRov, RAWDAT* Raw);
int GetSynObs(SOCKET* NetGps1, SOCKET* NetGps2, RAWDAT* Raw);
