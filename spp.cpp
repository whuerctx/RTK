#include "spp.h"
#include"matrix.h"
#include"coordinate.h"
#include"pvt.h"
#include"delaycorrect.h"
#include<cmath>
#include <iostream>
#include<iomanip>
using namespace std;


//信号发射时刻卫星位置计算
void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double UserPos[3])
{
	GPSTIME t_tr_GPS;
	//遍历每颗卫星
	for (int i = 0; i < Epk->SatNum; i++)
	{
		t_tr_GPS.Week = Epk->Time.Week;
		t_tr_GPS.SecOfWeek = Epk->Time.SecOfWeek - Epk->ComObs[i].PIF / C_Light;//使用IF组合伪距
		CompSatClkoff(Epk->SatObs[i].Prn, Epk->SatObs[i].System, &t_tr_GPS, GPSEph, BDSEph, &Epk->SatPVT[i]);
		t_tr_GPS.SecOfWeek = Epk->Time.SecOfWeek - Epk->ComObs[i].PIF / C_Light - Epk->SatPVT[i].SatClkOft;

		//计算卫星位置，速度，钟差，钟速
		CompSatClkoff(Epk->SatObs[i].Prn, Epk->SatObs[i].System, &t_tr_GPS, GPSEph, BDSEph, &Epk->SatPVT[i]);
		if (Epk->SatObs[i].System == GPS)
		{
			CompGPSSatPVT(Epk->SatObs[i].Prn, &t_tr_GPS, GPSEph, &Epk->SatPVT[i]);
		}
		else if(Epk->SatObs[i].System == BDS)
		{
			CompBDSSatPVT(Epk->SatObs[i].Prn, &t_tr_GPS, BDSEph, &Epk->SatPVT[i]);
		}
		else
		{
			Epk->SatPVT[i].Valid = false;
		}

		if (Epk->SatPVT[i].Valid == false)continue;
		//地球自转改正
		EarthRotCorrect(&Epk->SatPVT[i], UserPos);

		//对流层改正
		//计算接收机的海拔高度
		//这里采用WGS84坐标系参数将笛卡尔坐标系转换成大地坐标系
		double a = 6378137;
		double f = 1 / 298.257223563;
		double e = sqrt(2 * f - f * f);
		XYZ xr = { UserPos[0],UserPos[1],UserPos[2] };
		BLH blh;
		XYZToBLH(&xr, &blh, a, e);
		double H = blh.height;
		//计算卫星的仰角
		XYZ xs = { Epk->SatPVT[i].SatPos[0] ,Epk->SatPVT[i].SatPos[1] ,Epk->SatPVT[i].SatPos[2] };
		double Elev = 0, Azim = 0;
		CompSatElAz(&xr, &xs, &Elev, &Azim);
		Epk->SatPVT[i].Elevation = Elev;
		Epk->SatPVT[i].Azimuth = Azim;
		Epk->SatPVT[i].TropCorr = Hopfield(H, Elev);
		////计算成功
		//Epk->SatPVT[i].Valid = true;
	}
}

//单点定位的函数
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, PPRESULT* ppresult)
{
	double delta_tG = 0, delta_tC = 0;//接收机钟差
	double B[MAXCHANNUM * 5] = {}, w[MAXCHANNUM] = {}, x[5] = {}, P[MAXCHANNUM * MAXCHANNUM] = {};
	double BT[MAXCHANNUM * 5], BTPB[25], BTPw[5], N[25];
	double BTPB_prime[16], N_prime[16], W[4], x_prime[4];
	double mid1[MAXCHANNUM * 5], mid2[MAXCHANNUM * 5], mid3[MAXCHANNUM];
	double V[MAXCHANNUM] = {}, Vmid[MAXCHANNUM] = {}, Bx[MAXCHANNUM], VT[MAXCHANNUM] = {}, VTPV[1];
	int kk = 0;
	int r = 0;//多余观测数
	int t = 4;//必要观测数
	double sigmap, PDOP;//定位单位权中误差

	//粗差与周跳探测
	DetectOutlier(Epoch);
	//设置初始位置
	double UserPos[3] = { 0, 0, 0 };
	double X[5] = { UserPos[0] ,UserPos[1] ,UserPos[2] ,delta_tG ,delta_tC };

	//计算信号发射时刻的卫星位置钟差
	ComputeSatPVTAtSignalTrans(Epoch, GPSEph, BDSEph, UserPos);

	//统计可用卫星数
	int b = 0, g = 0, j = 0;
	//用于记录可用卫星的序号
	int avaGPS[MAXCHANNUM] ;
	int avaBDS[MAXCHANNUM] ;
	int ava[MAXCHANNUM] ;
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if (Epoch->SatPVT[i].Valid == true && Epoch->SatObs[i].Valid == true)
		{
			if (Epoch->SatObs[i].System == GPS)
			{
				avaGPS[g] = i;
				g++;
			}
			if (Epoch->SatObs[i].System == BDS)
			{
				avaBDS[b] = i;
				b++;
			}
			ava[j] = i;
			j++;
		}
	}

	//计算必要观测数
	if (b == 0 || g == 0) t = 4;
	else t = 5;

	if (j < t)
	{
		//cout << "可用卫星数小于必要观测数，不能进行单点定位" << endl;
		return false;
	}


	//循环次数
	int v = 0;
	//可用卫星大于等于必要观测数，进行单点定位解算
	do
	{
		ComputeSatPVTAtSignalTrans(Epoch, GPSEph, BDSEph, UserPos);

		//计算B,w矩阵
		
		//GPS部分
		for (int i = 0 ; i < g ; i++)
		{
			double xsX = Epoch->SatPVT[avaGPS[i]].SatPos[0];
			double xsY = Epoch->SatPVT[avaGPS[i]].SatPos[1];
			double xsZ = Epoch->SatPVT[avaGPS[i]].SatPos[2];
			double xrX = X[0];
			double xrY = X[1];
			double xrZ = X[2];
			//计算几何距离
			double rou = sqrt((xrX - xsX) * (xrX - xsX) + (xrY - xsY) * (xrY - xsY) + (xrZ - xsZ) * (xrZ - xsZ));
			double pIF = Epoch->ComObs[avaGPS[i]].PIF;

			B[5 * i + 0] = (xrX - xsX) / rou;
			B[5 * i + 1] = (xrY - xsY) / rou;
			B[5 * i + 2] = (xrZ - xsZ) / rou;
			B[5 * i + 3] = 1;
			B[5 * i + 4] = 0;

			w[i] = pIF - (rou + X[3] - C_Light * Epoch->SatPVT[avaGPS[i]].SatClkOft + Epoch->SatPVT[avaGPS[i]].TropCorr);
		}
		//BDS部分
		for(int i = 0;i < b ; i++)
		{
			double xsX = Epoch->SatPVT[avaBDS[i]].SatPos[0];
			double xsY = Epoch->SatPVT[avaBDS[i]].SatPos[1];
			double xsZ = Epoch->SatPVT[avaBDS[i]].SatPos[2];
			double xrX = X[0];
			double xrY = X[1];
			double xrZ = X[2];
			//计算几何距离
			double rou = sqrt((xrX - xsX) * (xrX - xsX) + (xrY - xsY) * (xrY - xsY) + (xrZ - xsZ) * (xrZ - xsZ));
			double pIF = Epoch->ComObs[avaBDS[i]].PIF;
			int Prn = Epoch->SatObs[avaBDS[i]].Prn;
			double tgd = FG1_BDS * FG1_BDS / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS) * BDSEph[Prn - 1].TGD1*C_Light;

			B[5 * i + 0 + 5 * g] = (xrX - xsX) / rou;
			B[5 * i + 1 + 5 * g] = (xrY - xsY) / rou;
			B[5 * i + 2 + 5 * g] = (xrZ - xsZ) / rou;
			B[5 * i + 3 + 5 * g] = 0;
			B[5 * i + 4 + 5 * g] = 1;
			w[i + g] = pIF - (rou + X[4] - C_Light * Epoch->SatPVT[avaBDS[i]].SatClkOft + Epoch->SatPVT[avaBDS[i]].TropCorr + tgd);
		}
		
		//计算P矩阵，设置为单位阵
		for (int i = 0; i < j; i++)
		{
			P[i * j + i] = 1;
		}
		//Mat_print(B, j, 5);

		//Mat_print(w, j, 1);


		Mat_transpose(B, BT, j, 5);
		Mat_multiply(BT, P, B, mid1, BTPB, 5, j, j, j, j, 5);
		Mat_multiply(BT, P, w, mid2, BTPw, 5, j, j, j, j, 1);
		//Mat_print(BTPB,5,5);
		
		//如果GPS，BDS卫星数都不为0
		if (g != 0 && b != 0)
		{
			kk = Mat_inversion(BTPB, N, 5, 5);
			if (kk == -1)return false;
			Mat_multiply(N, BTPw, x, 5, 5, 5, 1);
		}

		//如果GPS卫星数和BDS卫星数有一个为0，矩阵重构
		else
		{
			//如果GPS卫星数为0,删去四行四列
			if (g == 0)
			{
				for (int i = 0; i < 3; i++)BTPB_prime[i] = BTPB[i];
				BTPB_prime[3] = BTPB[4];
				for (int i = 4; i < 7; i++)BTPB_prime[i] = BTPB[i + 1];
				BTPB_prime[7] = BTPB[9];
				for (int i = 8; i < 11; i++)BTPB_prime[i] = BTPB[i + 2];
				BTPB_prime[11] = BTPB[14];
				for (int i = 12; i < 15; i++)BTPB_prime[i] = BTPB[i + 8];
				BTPB_prime[15] = BTPB[24];
				for (int i = 0; i < 3; i++)W[i] = BTPw[i];
				W[3] = BTPw[4];

			}
			//如果BDS卫星数为0,删去五行五列
			if (b == 0)
			{
				for (int i = 0; i < 4; i++)BTPB_prime[i] = BTPB[i];
				for (int i = 4; i < 8; i++)BTPB_prime[i] = BTPB[i + 1];
				for (int i = 8; i < 12; i++)BTPB_prime[i] = BTPB[i + 2];
				for (int i = 12; i < 16; i++)BTPB_prime[i] = BTPB[i + 3];
				for (int i = 0; i < 4; i++)W[i] = BTPw[i];
			}
			kk = Mat_inversion(BTPB_prime, N_prime, 4, 4);
			if (kk == -1)return false;
			Mat_multiply(N_prime, W, x_prime, 4, 4, 4, 1);
			if (g == 0)
			{
				for (int i = 0; i < 3; i++)x[i] = x_prime[i];
				x[3] = 0;
				x[4] = x_prime[3];
			}
			if (b == 0)
			{
				for (int i = 0; i < 4; i++)x[i] = x_prime[i];
				x[4] = 0;
			}
		}
		Mat_add(X, x, X, 5, 1, 5, 1);
		UserPos[0] = X[0];
		UserPos[1] = X[1];
		UserPos[2] = X[2];
		
		v++;
		if (v > 10) return false;

	}while (fabs(x[0]) > 1e-8 && fabs(x[1]) > 1e-6 && fabs(x[2]) > 1e-8);
	//精度评定
	//计算单位权中误差
	Mat_multiply(B, x, Bx, j, 5, 5, 1);
	Mat_subtract(Bx, w, V, j, 1, j, 1);
	Mat_transpose(V, VT, j, 1);
	Mat_multiply(VT, P, V, mid3, VTPV, 1, j, j, j, j, 1);
	r = j - t;
	if (r == 0)sigmap = 999999;
	else sigmap = sqrt(VTPV[0] / r);
	if (g != 0 && b != 0)PDOP = sqrt(N[0] + N[6] + N[12]);
	else PDOP = sqrt(N_prime[0] + N_prime[5] + N_prime[10]);
	ppresult->Time.Week = Epoch->Time.Week;
	ppresult->Time.SecOfWeek = Epoch->Time.SecOfWeek;
	ppresult->Position[0] = UserPos[0];
	ppresult->Position[1] = UserPos[1];
	ppresult->Position[2] = UserPos[2];

	double ClkOft_bdssum = 0;
	double ClkOft_gpssum = 0;
	for (int i = 0; i < g; i++) ClkOft_gpssum += Epoch->SatPVT[avaGPS[i]].SatClkOft;
	for (int i = 0; i < b; i++) ClkOft_gpssum += Epoch->SatPVT[avaBDS[i]].SatClkOft;
	ppresult->RcvClkOft[0] = ClkOft_gpssum / g * C_Light;
	ppresult->RcvClkOft[1] = ClkOft_bdssum / b * C_Light;
	ppresult->RcvClkSft = (X[3] + X[4]) / 2;
	
	ppresult->PDOP = PDOP;
	ppresult->SigmaPos = sigmap;
	ppresult->GPSSatNum = g;
	ppresult->BDSSatNum = b;
	ppresult->AllSatNum = j;
	ppresult->IsSuccess = true;
	return true;
}


//单点测速函数
void SPV(EPOCHOBSDATA* Epoch, PPRESULT* ppresult)
{
	double B[MAXCHANNUM * 4] = {}, w[MAXCHANNUM] = {}, X[4] = {}, P[MAXCHANNUM * MAXCHANNUM] = {};
	double BT[MAXCHANNUM * 4], BTPB[16], BTPw[4], N[16];
	double mid1[MAXCHANNUM * 4], mid2[MAXCHANNUM * 4], mid3[MAXCHANNUM];
	double V[MAXCHANNUM] = {}, Vmid[MAXCHANNUM] = {}, Bx[MAXCHANNUM], VT[MAXCHANNUM] = {}, VTPV[1];
	double sigmav = 0;

	//用于记录可用卫星的序号
	int ava[MAXCHANNUM] = {};
	int j = 0;
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if (Epoch->SatPVT[i].Valid == true && Epoch->SatObs[i].Valid == true)
		{
			ava[j] = i;
			j++;
		}
	}

	for (int i = 0; i < j; i++)
	{
		double xsX = Epoch->SatPVT[ava[i]].SatPos[0];
		double xsY = Epoch->SatPVT[ava[i]].SatPos[1];
		double xsZ = Epoch->SatPVT[ava[i]].SatPos[2];
		double xsX_dot = Epoch->SatPVT[ava[i]].SatVel[0];
		double xsY_dot = Epoch->SatPVT[ava[i]].SatVel[1];
		double xsZ_dot = Epoch->SatPVT[ava[i]].SatVel[2];
		double xrX = ppresult->Position[0];
		double xrY = ppresult->Position[1];
		double xrZ = ppresult->Position[2];
		//计算几何距离
		double rou = sqrt((xrX - xsX) * (xrX - xsX) + (xrY - xsY) * (xrY - xsY) + (xrZ - xsZ) * (xrZ - xsZ));
		double rou_dot = ((xsX - xrX) * xsX_dot + (xsY - xrY) * xsY_dot + (xsZ - xrZ) * xsZ_dot) / rou;
		double dop = Epoch->SatObs[ava[i]].D[0];

		B[4 * i + 0] = (xrX - xsX) / rou;
		B[4 * i + 1] = (xrY - xsY) / rou;
		B[4 * i + 2] = (xrZ - xsZ) / rou;
		B[4 * i + 3] = 1;

		w[i] = dop - (rou_dot - C_Light * Epoch->SatPVT[ava[i]].SatClkSft);
	}

	//计算P矩阵，设置为单位阵
	for (int i = 0; i < j; i++)
	{
		P[i * j + i] = 1;
	}

	Mat_transpose(B, BT, j, 4);
	Mat_multiply(BT, P, B, mid1, BTPB, 4, j, j, j, j, 4);
	Mat_multiply(BT, P, w, mid2, BTPw, 4, j, j, j, j, 1);
	Mat_inversion(BTPB, N, 4, 4);
	Mat_multiply(N, BTPw, X, 4, 4, 4, 1);

	//精度评定
	//计算单位权中误差
	Mat_multiply(B, X, Bx, j, 4, 4, 1);
	Mat_subtract(Bx, w, V, j, 1, j, 1);
	Mat_transpose(V, VT, j, 1);
	Mat_multiply(VT, P, V, mid3, VTPV, 1, j, j, j, j, 1);
	double r = j - 4;
	if (r == 0)sigmav = 999999;
	else sigmav = sqrt(VTPV[0] / r);
	
	
	ppresult->Velocity[0] = X[0];
	ppresult->Velocity[1] = X[1];
	ppresult->Velocity[2] = X[2];
	ppresult->SigmaVel = sigmav;
}