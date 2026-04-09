#include "time.h"
#include <cmath>
#include<iostream>;
#include<iomanip>
using namespace std;


// 通用时到儒略日
void CommonTimeToJdTime(const COMMONTIME* ct, JDTIME* jd)
{
    int year = ct->Year;
    int month = ct->Month;
    if (month <= 2)
    {
        year--;
        month += 12;
    }
    double commonfracday = (ct->Hour + ct->Minute / 60.0 + ct->Second / 3600.0) / 24.0;
    if (commonfracday <= 0.5)
    {
        jd->Days = int(365.25 * year) + int(30.6001 * (month + 1)) + ct->Day + 1720981;
        jd->FracDay = commonfracday + 0.5;
    }
    else
    {
        jd->Days = int(365.25 * year) + int(30.6001 * (month + 1)) + ct->Day + 1720981 + 1;
        jd->FracDay = commonfracday - 0.5;
    }
}

// 儒略日到通用时
void JdTimeToCommonTime(const JDTIME* jd, COMMONTIME* ct)
{
    int a = int(jd->Days + jd->FracDay + 0.5);
    int b = a + 1537;
    int C = int((b - 122.1) / 365.25);
    int d = int(365.25 * C);
    int e = int((b - d) / 30.6001);
    int day = b - d - int(30.6001 * e);
    int month = e - 1 - 12 * int(e / 14);
    int year = C - 4715 - int((7 + month) / 10);
    double seconds = (jd->FracDay + 0.5) * 24 * 3600;//小数天换算成秒
    int hour = seconds / 3600;
    int minute = (seconds - 3600 * hour) / 60;
    double second = seconds - 3600 * hour - 60 * minute;

    ct->Year = short(year);
    ct->Month = unsigned short(month);
    ct->Day = unsigned short(day);
    ct->Hour = unsigned short(hour);
    ct->Minute = unsigned short(minute);
    ct->Second = second;
}

// 通用时到简化儒略日
void CommonTimeToMjdTime(const COMMONTIME* ct, MJDTIME* mjd)
{
    int year = ct->Year;
    int month = ct->Month;
    if (month <= 2)
    {
        year--;
        month += 12;
    }

    mjd->Days = int(365.25 * year) + int(30.6001 * (month + 1)) + ct->Day + 1720981.5 - 2400000.5;
    mjd->FracDay = (ct->Hour + ct->Minute / 60.0 + ct->Second / 3600.0) / 24.0;
}

// 儒略日到简化儒略日
void JdTimeToMjdTime(const JDTIME* jd, MJDTIME* mjd)
{
    if (jd->FracDay <= 0.5)
    {
        mjd->Days = jd->Days - 2400000 - 1;
        mjd->FracDay = jd->FracDay + 0.5;
    }
    else
    {
        mjd->Days = jd->Days - 2400000;
        mjd->FracDay = jd->FracDay - 0.5;
    }

}

// 简化儒略日到儒略日
void MjdTimeToJdTime(const MJDTIME* mjd, JDTIME* jd)
{
    if (mjd->FracDay <= 0.5)
    {
        jd->Days = mjd->Days + 2400000;
        jd->FracDay = mjd->FracDay + 0.5;
    }
    else
    {
        jd->Days = mjd->Days + 2400000 + 1;
        jd->FracDay = mjd->FracDay - 0.5;
    }
}

//简化儒略日到GPS时
void MjdTimeToGpsTime(const MJDTIME* mjd, GPSTIME* gt)
{
    gt->Week = int((mjd->Days + mjd->FracDay - 44244) / 7);
    gt->SecOfWeek = (mjd->Days + mjd->FracDay - 44244 - gt->Week * 7) * 86400;
}

//GPS时到简化儒略日
void GpsTimeToMjdTime(const GPSTIME* gt, MJDTIME* mjd)
{
    mjd->Days = 44244 + gt->Week * 7 + int(gt->SecOfWeek / 86400);
    mjd->FracDay = gt->SecOfWeek / 86400 - int(gt->SecOfWeek / 86400);
}

// 通用时到GPS时
void CommonTimeToGpsTime(const COMMONTIME* ct, GPSTIME* gt)
{
    MJDTIME mjd;
    CommonTimeToMjdTime(ct, &mjd);
    MjdTimeToGpsTime(&mjd, gt);
}

// GPS时到通用时
void GpsTimeToCommonTime(const GPSTIME* gt, COMMONTIME* ct)
{
    MJDTIME mjd;
    JDTIME jd;
    GpsTimeToMjdTime(gt, &mjd);
    MjdTimeToJdTime(&mjd, &jd);
    JdTimeToCommonTime(&jd, ct);
}

//打印时间
void TimePrint(const COMMONTIME* ct)
{
    cout << ct->Year << "年" << ct->Month << "月" << ct->Day << "日" << ct->Hour << "时" << ct->Minute << "分" << ct->Second << "秒" << endl;
}
void TimePrint(const JDTIME* jd)
{
    cout << "整数天:" << jd->Days << endl;
    cout << "小数天:" << jd->FracDay << endl;
}
void TimePrint(const MJDTIME* mjd)
{
    cout << "整数天:" << mjd->Days << endl;
    cout << "小数天:" << mjd->FracDay << endl;
}
void TimePrint(const GPSTIME* gt)
{
    cout << gt->Week << "周" << gt->SecOfWeek << "秒" << endl;
}