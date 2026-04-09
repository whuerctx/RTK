#pragma once
#include"struct.h"
using namespace std;

// 通用时到儒略日
void CommonTimeToJdTime(const COMMONTIME* ct, JDTIME* jd);
// 通用时到简化儒略日
void CommonTimeToMjdTime(const COMMONTIME* ct, MJDTIME* mjd);
// 儒略日到通用时
void JdTimeToCommonTime(const JDTIME* jd, COMMONTIME* ct);
// 儒略日到简化儒略日
void JdTimeToMjdTime(const JDTIME* mjd, MJDTIME* jd);
// 简化儒略日到儒略日
void MjdTimeToJdTime(const MJDTIME* mjd, JDTIME* jd);
//简化儒略日到GPS时
void MjdTimeToGpsTime(const MJDTIME* mjd, GPSTIME* gt);
//GPS时到简化儒略日
void GpsTimeToMjdTime(const GPSTIME* gt, MJDTIME* mjd);
// 通用时到GPS时
void CommonTimeToGpsTime(const COMMONTIME* ct, GPSTIME* gt);
// GPS时到通用时
void GpsTimeToCommonTime(const GPSTIME* gt, COMMONTIME* ct);
//打印时间
void TimePrint(const COMMONTIME* ct);
void TimePrint(const JDTIME* jd);
void TimePrint(const MJDTIME* mjd);
void TimePrint(const GPSTIME* gt);