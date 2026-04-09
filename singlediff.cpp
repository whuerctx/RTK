#include"singlediff.h"
#include<cmath>
using namespace std;

//单差观测值(EpkA:基站，EpkB;流动站)
void FormSDEpochObs(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs)
{
    double AL[2], AP[2], BL[2], BP[2];
	SDObs->Time.Week = EpkB->Time.Week;
	SDObs->Time.SecOfWeek = EpkB->Time.SecOfWeek;
	int satnum = 0;
	for (int i = 0; i < EpkA->SatNum; i++)
	{
		for (int j = 0; j < EpkB->SatNum; j++)
		{
			if (EpkA->SatObs[i].System == EpkB->SatObs[j].System && EpkA->SatObs[i].Prn == EpkB->SatObs[j].Prn)
			{
                if (EpkA->SatObs[i].Valid == 1 && EpkA->SatPVT[i].Valid == 1 && EpkB->SatObs[j].Valid == 1 && EpkB->SatPVT[j].Valid == 1
                    &&EpkA->SatObs[i].half[0]!=0 && EpkA->SatObs[i].half[1] != 0 && EpkB->SatObs[i].half[0] != 0 && EpkB->SatObs[i].half[1] != 0)
                {
                    satnum++;
                    SDObs->SdSatObs[satnum - 1].System = EpkA->SatObs[i].System;
                    SDObs->SdSatObs[satnum - 1].Prn = EpkA->SatObs[i].Prn;
                    SDObs->SdSatObs[satnum - 1].nBas = i;
                    SDObs->SdSatObs[satnum - 1].nRov = j;
                    for (int k = 0; k < 2; k++)
                    {
                        AL[k] = EpkA->SatObs[i].L[k];
                        AP[k] = EpkA->SatObs[i].P[k];
                        BL[k] = EpkB->SatObs[j].L[k];
                        BP[k] = EpkB->SatObs[j].P[k];
                        SDObs->SdSatObs[satnum - 1].dL[k] = BL[k] - AL[k];
                        SDObs->SdSatObs[satnum - 1].dP[k] = BP[k] - AP[k];
                    }
                }
                else
                {
                    continue;
                }
			}
		}
	}
	SDObs->SatNum = satnum;
}

	

//周跳探测
void DetectCycleSlip(SDEPOCHOBS* Obs)
{
    double MW = 0, GF = 0, dMW = 0, dGF = 0;
    for (int i = 0; i < Obs->SatNum; i++)
    {
        double P1 = Obs->SdSatObs[i].dP[0];
        double P2 = Obs->SdSatObs[i].dP[1];
        double L1 = Obs->SdSatObs[i].dL[0];
        double L2 = Obs->SdSatObs[i].dL[1];
        if (P1 == 0 || P2 == 0 || L1 == 0 || L2 == 0)
        {
            Obs->SdSatObs[i].Valid = false;
            continue;
        }
        Obs->SdCObs[i].Prn = Obs->SdSatObs[i].Prn;
        Obs->SdCObs[i].Sys = Obs->SdSatObs[i].System;

        if (Obs->SdCObs[i].Sys == GPS)
        {
            MW = (FG1_GPS * L1 - FG2_GPS * L2) / (FG1_GPS - FG2_GPS) - (FG1_GPS * P1 + FG2_GPS * P2) / (FG1_GPS + FG2_GPS);
        }
        if (Obs->SdCObs[i].Sys == BDS)
        {
            MW = (FG1_BDS * L1 - FG3_BDS * L2) / (FG1_BDS - FG3_BDS) - (FG1_BDS * P1 + FG3_BDS * P2) / (FG1_BDS + FG3_BDS);
        }
        GF = L1 - L2;
        dGF = fabs(GF - Obs->SdCObs[i].GF);
        dMW = fabs(MW - Obs->SdCObs[i].MW);
        //第一个历元的数据，设置为可用
        if (Obs->SdCObs[i].GF == 0 && Obs->SdCObs[i].MW == 0)
        {
            dGF = dMW = 0;
        }
        if (dGF > 0.05 || dMW > 3)
        {
            Obs->SdSatObs[i].Valid == false;
            Obs->SdCObs[i].GF = GF;
            Obs->SdCObs[i].MW = MW;
            Obs->SdCObs[i].n = 0;
            continue;
        }
        else
        {
            Obs->SdSatObs[i].Valid = true;
            Obs->SdCObs[i].GF = GF;
            Obs->SdCObs[i].n += 1;
            Obs->SdCObs[i].MW = ((Obs->SdCObs[i].n - 1) * Obs->SdCObs[i].MW + MW) / Obs->SdCObs[i].n;

        }
    }
}