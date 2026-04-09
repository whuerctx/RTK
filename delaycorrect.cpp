#include"delaycorrect.h"
#include<cmath>
#include<iostream>
using namespace std;

//对流层改正
double Hopfield(const double H, const double Elev)
{
    double elev = Elev * 180 / PI;
    // 检查测站高度是否在对流层范围内（0到11000米）
    if (H < -5000.0 || H > 11000.0)
    {
        return 0.0;
    }

    // 计算干成分
    double hd = 40136.0 + 148.72 * (T0 - 273.16);
    double T = T0 - 0.0065 * (H - H0);
    double p = p0 * pow(1.0 - 0.0000226 * (H - H0), 5.225);
    double RH = RH0 * exp(-0.0006396 * (H - H0));
    double Kd = 155.2e-7 * (p / T) * (hd - H);
    double delta_d = Kd / sin(sqrt(elev * elev + 6.25) * PI / 180.0);

    // 计算湿成分
    double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
    double Kw = 155.2e-7 * (4810.0 / (T * T)) * e * (hw - H);
    double delta_w = Kw / sin(sqrt(elev * elev + 2.25) * PI / 180.0);

    // 对流层改正
    double delta_trop = delta_d + delta_w;

    return delta_trop;
}

//粗差探测
void DetectOutlier(EPOCHOBSDATA* obs)
{
    double MW = 0, GF = 0, dMW = 0, dGF = 0, PIF = 0;
    for (int i = 0; i < obs->SatNum; i++)
    {
        double P1 = obs->SatObs[i].P[0];
        double P2 = obs->SatObs[i].P[1];
        double L1 = obs->SatObs[i].L[0];
        double L2 = obs->SatObs[i].L[1];
        if (P1 == 0 || P2 == 0 || L1 == 0 || L2 == 0)
        {
            obs->SatObs[i].Valid == false;
            continue;
        }
        obs->ComObs[i].Prn = obs->SatObs[i].Prn;
        obs->ComObs[i].Sys = obs->SatObs[i].System;

        if (obs->ComObs[i].Sys == GPS)
        {
            MW = (FG1_GPS * L1 - FG2_GPS * L2) / (FG1_GPS - FG2_GPS) - (FG1_GPS * P1 + FG2_GPS * P2) / (FG1_GPS + FG2_GPS);
        }
        if (obs->ComObs[i].Sys == BDS)
        {
            MW = (FG1_BDS * L1 - FG3_BDS * L2) / (FG1_BDS - FG3_BDS) - (FG1_BDS * P1 + FG3_BDS * P2) / (FG1_BDS + FG3_BDS);
        }
        GF = L1 - L2;
        dGF = fabs(GF - obs->ComObs[i].GF);
        dMW = fabs(MW - obs->ComObs[i].MW);
        //第一个历元的数据，设置为可用
        if (obs->ComObs[i].GF == 0 && obs->ComObs[i].MW == 0)
        {
            dGF = dMW = 0;
        }
        if (dGF > 0.05 || dMW > 3)//则有粗差
        {
            obs->SatObs[i].Valid == false;
            obs->ComObs[i].GF = GF;
            obs->ComObs[i].MW = MW;
            obs->ComObs[i].n = 0;
            continue;
        }
        else
        {
            obs->SatObs[i].Valid = true;
            obs->ComObs[i].GF = GF;
            obs->ComObs[i].n += 1;
            obs->ComObs[i].MW = ((obs->ComObs[i].n - 1) * obs->ComObs[i].MW + MW) / obs->ComObs[i].n;
            
            //计算位居IF组合
            if (obs->ComObs[i].Sys == GPS)
            {
                PIF = (FG1_GPS * FG1_GPS * P1 - FG2_GPS * FG2_GPS * P2) / (FG1_GPS * FG1_GPS - FG2_GPS * FG2_GPS);
            }
            if (obs->ComObs[i].Sys == BDS)
            {
                PIF = (FG1_BDS * FG1_BDS * P1 - FG3_BDS * FG3_BDS * P2) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
            }
            obs->ComObs[i].PIF = PIF;
        }
    }

}

//电离层改正
double Klobutchar(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3])
{
    return 0;
}