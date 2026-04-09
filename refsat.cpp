#include"refsat.h"
#include<iostream>
using namespace std;
void DetRefSat(const EPOCHOBSDATA* EpkRov, const EPOCHOBSDATA* EpkBas, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
    int i, sys;
    int Idx_MaxE[2] = { 0 }, Satnum[2] = { 0 };
    double MaxElev[2] = { 0.0 };

    /*清空上一历元的双差数据*/
    memset(DDObs, 0, sizeof(DDObs));

    /*每个导航系统都需要选取一颗卫星作为参考星*/
    for (i = 0; i < SDObs->SatNum; i++)
    {
        //利用单差数据判断是否有粗差、周跳或半周标记
        if (SDObs->SdSatObs[i].Valid != 1) continue;
        sys = SDObs->SdSatObs[i].System == GPS ? 0 : 1;
        /*分别对不同系统进行参考星的选取*/
        //SatPVT中卫星存储顺序与SatObs中一致，卫星索引号在单差结构体中已存储
        if (EpkRov->SatPVT[SDObs->SdSatObs[i].nRov].Valid != true || EpkBas->SatPVT[SDObs->SdSatObs[i].nBas].Valid != true)  continue;//判断卫星星历是否正常，卫星位置是否计算成功
        Satnum[sys] += 1;

        /*进行CN0判断(由于高度角可能受到
        /遮蔽的影响，不能肯定当卫星高度角越高时卫星的观测数据质量更好)*/
        if (EpkRov->SatPVT[SDObs->SdSatObs[i].nRov].Elevation > MaxElev[sys]
            && EpkBas->SatObs[SDObs->SdSatObs[i].nBas].cn0[0] > 40
            && EpkRov->SatObs[SDObs->SdSatObs[i].nRov].cn0[0] > 40
            /* &&EpkRov->SatObs[SDObs->SdSatObs[i].nRov].LockTime[0] > 10
             && EpkBas->SatObs[SDObs->SdSatObs[i].nBas].LockTime[0] > 10*/)
        {
            MaxElev[sys] = EpkRov->SatPVT[SDObs->SdSatObs[i].nRov].Elevation;
            Idx_MaxE[sys] = i;
        }
    }

    for (i = 0; i < 2; i++)
    {
        if (MaxElev[i] > 0.0)
        {
            DDObs->RefPrn[i] = SDObs->SdSatObs[Idx_MaxE[i]].Prn;
            DDObs->BasePos[i] = Idx_MaxE[i];
        }
        else
        {
            DDObs->RefPrn[i] = 0;
            DDObs->BasePos[i] = -1;
        }
        if (DDObs->RefPrn[i] != 0) DDObs->DDSatNum[i] = Satnum[i] - 1;
        else DDObs->DDSatNum[i] = 0;
    }
}