#include"rtkfloat.h"
#include<cmath>
#include<iostream>
#include"vector.h"
#include"matrix.h"
using namespace std;



bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov)
{
	int i, j, k, m, row, dim, iter, SysId1, SysId2, ai, bi;
	int SatNum[2]={0}, Sats;
	double w[4 * MAXCHANNUM] = { 0.0 }, B[4 * MAXCHANNUM * (3 + MAXCHANNUM * 2)] = { 0.0 }, P[(4 * MAXCHANNUM) * (4 * MAXCHANNUM)] = { 0.0 }, BT[4 * MAXCHANNUM * (3 + MAXCHANNUM * 2)] = { 0.0 };
	double BTP[4 * MAXCHANNUM * (3 + MAXCHANNUM * 2)] = { 0.0 }, BTPw[3 + MAXCHANNUM * 2] = { 0.0 }, BTPB[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)] = { 0.0 };
	double N[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)] = { 0.0 }, V[4 * MAXCHANNUM] = { 0.0 }, Bx[4 * MAXCHANNUM] = { 0.0 }, VT[4 * MAXCHANNUM] = { 0.0 }, VTP[4 * MAXCHANNUM] = { 0.0 }, VTPV[1] = { 0.0 };
	double BasePos[3], S_Base_Sat[MAXCHANNUM], S_Rov_Refsat[2][4];
	double dPos[4], RovPos[3] = { 0.0 }, X[3 + MAXCHANNUM * 2] = { 0.0 }, x[3 + MAXCHANNUM * 2] = { 0.0 };
	double wl1, wl2;
	double deltaP = 0.5;
	double deltaL = 0.002;

	Raw->DDObs.bFixed = Raw->DDObs.Status = false;

	//检查参考星是否选取
	if (Raw->DDObs.BasePos[0] == -1 && Raw->DDObs.BasePos[1] == -1) 
	{
		cout << "未选取参考星" << endl;
		return false;
	}

	//设置基站坐标初始值
	if (Norm(Raw->BasEpk.Pos, 3) > 1.0)CopyArray(BasePos, Raw->BasEpk.Pos, 3);
	else if (Norm(Base->Position, 3) > 1.0)CopyArray(BasePos, Base->Position, 3);
	else
	{
		cout << "基站参考位置未知" << endl;
		return false;
	}

	//计算基站到卫星的站星距离
	for (i = 0; i < Raw->SdObs.SatNum; i++)
	{
		ai = Raw->SdObs.SdSatObs[i].nBas;
		bi = Raw->SdObs.SdSatObs[i].nRov;
		if (Raw->SdObs.SdSatObs[i].Valid < 1 || Raw->BasEpk.SatPVT[ai].Valid < 1 || Raw->RovEpk.SatPVT[bi].Valid < 1)
		{
			Raw->SdObs.SdSatObs[i].Valid = -1;
			continue;
		}
		if (Raw->SdObs.SdSatObs[i].System == GPS) SatNum[0]++;
		if (Raw->SdObs.SdSatObs[i].System == BDS) SatNum[1]++;
		Mat_subtract(BasePos, Raw->BasEpk.SatPVT[ai].SatPos, dPos, 1, 3, 1, 3);
		S_Base_Sat[i] = Norm(dPos, 3);
	}

	if ((SatNum[0] + SatNum[1]) < 6)
	{
		cout << "卫星数量不够，没有足够的双差观测值.GPS 卫星数:" << SatNum[0] << ", BDS 卫星数 :" << SatNum[1] << "." << endl;
		return false;
	}
	iter = 0;
	dim = 0;//计算浮点解的待估参数的维数
	row = 4 * (SatNum[0] -1 + SatNum[1] -1 );
	if (SatNum[0] < 2 || Raw->DDObs.BasePos[0] == -1)SatNum[0] = 0;
	else dim += (SatNum[0] - 1) * 2;
	if (SatNum[1] < 2 || Raw->DDObs.BasePos[1] == -1)SatNum[1] = 0;
	else dim += (SatNum[1] - 1) * 2;
	dim = dim + 3;//双差模糊度基线向量的总维数

	//设置流动站初始位置并初始化基线向量
	if (Norm(Raw->RovEpk.Pos, 3) > 1.0)Mat_subtract(Raw->RovEpk.Pos, BasePos, X, 1, 3, 1, 3);
	else if (Norm(Rov->Position, 3) > 1.0)Mat_subtract(Rov->Position, BasePos, X, 1, 3, 1, 3);
	//Mat_subtract(Rov->Position, BasePos, X, 1, 3, 1, 3);
	Mat_add(BasePos, X, RovPos, 3, 1, 3, 1);
	EmptyArray(x, 3);

	//用伪距初始化模期度
	for (Sats = 0, i = 0; i < Raw->SdObs.SatNum; i++)
	{
		SysId1 = Raw->SdObs.SdSatObs[i].System == GPS ? 0 : 1;
		if (Raw->SdObs.SdSatObs[i].Valid < 1)continue;
		if (Raw->DDObs.BasePos[SysId1] == -1 || SatNum[SysId1] == 0)continue;
		if (Raw->DDObs.BasePos[0] == i || Raw->DDObs.BasePos[1] == i)continue;
		wl1 = (SysId1 == 0 ? WL1_GPS : WL1_BDS);
		wl2 = (SysId1 == 0 ? WL2_GPS : WL3_BDS);
		X[3 + Sats * 2 + 0] = (Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.BasePos
			[SysId1]].dL[0] - Raw->SdObs.SdSatObs[i].dP[0] + Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[SysId1]].dP
			[0]) / wl1;
		X[3 + Sats * 2 + 1] = (Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.BasePos
			[SysId1]].dL[1] - Raw->SdObs.SdSatObs[i].dP[1] + Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[SysId1]].dP
			[1]) / wl2;
			Sats++;
	}
	do
	{
		Sats = 0;

		//流动站与参考星的距离
		for (i = 0; i < 2; i ++)
		{
			if (Raw->DDObs.BasePos[i] == -1 || Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[i]].Valid < 1)
			{
				Raw->DDObs.BasePos[i] = Raw->DDObs.RefPrn[i] = -1;
				continue;
			}
			ai = Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[i]].nRov;
			Mat_subtract(RovPos, Raw->RovEpk.SatPVT[ai].SatPos, S_Rov_Refsat[i], 1, 3, 1, 3);
			S_Rov_Refsat[i][3] = Norm(S_Rov_Refsat[i], 3);
		}

		//B矩阵和w矩阵和权阵
		for (k = i = 0; i < Raw->SdObs.SatNum; i++)
		{
			SysId1 = Raw->SdObs.SdSatObs[i].System == GPS ? 0 : 1;
			if (Raw->SdObs.SdSatObs[i].Valid < 1)continue;
			if (Raw -> DDObs.BasePos[SysId1] == -1|| SatNum[SysId1]== 0)continue;
			if (Raw->DDObs.BasePos[0] == i || Raw->DDObs.BasePos[1] == i)continue;

			bi = Raw->SdObs.SdSatObs[i].nRov;
			Mat_subtract(RovPos, Raw->RovEpk.SatPVT[bi].SatPos, dPos,1,3,1,3);
			dPos[3] = Norm(dPos, 3);//流动站和卫星的距离

			//B矩阵
			B[k * dim + 0] = B[(k + 1) * dim + 0] = B[(k + 2) * dim + 0] = B[(k + 3) * dim + 0] = dPos[0] / dPos[3] - S_Rov_Refsat[SysId1][0] / S_Rov_Refsat[SysId1][3];
			B[k * dim + 1] = B[(k + 1) * dim + 1] = B[(k + 2) * dim + 1] = B[(k + 3) * dim + 1] = dPos[1] / dPos[3] - S_Rov_Refsat[SysId1][1] / S_Rov_Refsat[SysId1][3];
			B[k * dim + 2] = B[(k + 1) * dim + 2] = B[(k + 2) * dim + 2] = B[(k + 3) * dim + 2] = dPos[2] / dPos[3] - S_Rov_Refsat[SysId1][2] / S_Rov_Refsat[SysId1][3];
			B[(k + 1) * dim + 3 + 2 * Sats + 0] = wl1 = (SysId1 == 0 ? WL1_GPS : WL1_BDS);
			B[(k + 3) * dim + 3 + 2 * Sats + 1] = wl2 = (SysId1 == 0 ? WL2_GPS : WL3_BDS);

			w[k + 0] = Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[SysId1]].dP[0]-(dPos[3] - S_Base_Sat[i] - S_Rov_Refsat[SysId1][3] + S_Base_Sat[Raw->DDObs.BasePos[SysId1]]);
			w[k + 1] = Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[SysId1]].dL[0] -(dPos[3] - S_Base_Sat[i] - S_Rov_Refsat[SysId1][3] + S_Base_Sat[Raw->DDObs.BasePos[SysId1]]) - X[3+ 2 * Sats + 0] * wl1;
			w[k + 2] = Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[SysId1]].dP[1] -(dPos[3] - S_Base_Sat[i] - S_Rov_Refsat[SysId1][3] + S_Base_Sat[Raw->DDObs.BasePos[SysId1]]);
			w[k + 3] = Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.BasePos[SysId1]].dL[1] -(dPos[3] - S_Base_Sat[i] - S_Rov_Refsat[SysId1][3] + S_Base_Sat[Raw->DDObs.BasePos[SysId1]]) - X[3+ 2 * Sats + 1] * wl2;

			//只算一次权阵
			if (iter == 0)
			{
				for (m = j = 0; j < Raw->SdObs.SatNum; j++)//双差伪距和相位的权阵
				{
					SysId2 = Raw->SdObs.SdSatObs[j].System == GPS ? 0 : 1;
					if (Raw->SdObs.SdSatObs[j].Valid < 1) continue;
					if (Raw->DDObs.BasePos[SysId2] == -1 || SatNum[SysId2] == 0)continue;
					if (Raw->DDObs.BasePos[0] == j || Raw->DDObs.BasePos[1] == j)continue;
					if (i == j)
					{
						P[(k + 0) * row + 4 * m + 0] = (SatNum[SysId2] - 1) / (2 * deltaP * deltaP) / SatNum[SysId2];
						P[(k + 1) * row + 4 * m + 1] = (SatNum[SysId2] - 1) / (2 * deltaL * deltaL) / SatNum[SysId2];
						P[(k + 2) * row + 4 * m + 2] = (SatNum[SysId2] - 1) / (2 * deltaP * deltaP) / SatNum[SysId2];
						P[(k + 3) * row + 4 * m + 3] = (SatNum[SysId2] - 1) / (2 * deltaL * deltaL) / SatNum[SysId2];
								
					}
					else
					{

						if (SysId1 == SysId2)
						{
							P[(k + 0) * row + 4 * m + 0] = -1 / (2 * deltaP * deltaP) / SatNum[SysId2];
							P[(k + 1) * row + 4 * m + 1] = -1 / (2 * deltaL * deltaL) / SatNum[SysId2];
							P[(k + 2) * row + 4 * m + 2] = -1 / (2 * deltaP * deltaP) / SatNum[SysId2];
							P[(k + 3) * row + 4 * m + 3] = -1 / (2 * deltaL * deltaL) / SatNum[SysId2];

						}
					}
					m++;	
				}		
			}
			Sats++;
			k += 4;
		}
		
		

		//Mat_print(B, k, dim);
		//Mat_print(P, k, k);
		Mat_transpose(B, BT, k, dim);
		Mat_multiply(BT, P, BTP, dim, k, k, k);
		Mat_multiply(BTP, B, BTPB, dim, k, k, dim);
		Mat_multiply(BTP, w, BTPw, dim, k, k, 1);
		//Mat_print(BTPB, dim, dim);

		if (Mat_inversion(BTPB, N, dim, dim) == 0)
		{
			if (iter > 2)
			{
				break;
			}
			else
			{
				cout << "求逆出现问题，卫星数:" << Sats << "." << endl;
				return false;
			}
		}

		if (k > dim)//观测方程数量大于未知数数量
		{
			Mat_multiply(N, BTPw, x, dim, dim, dim, 1);
		}

		Mat_add(X, x, X, dim, 1, dim, 1);
		Mat_add(BasePos, X, RovPos, 3, 1, 3, 1);
		iter++;
			
	} while (Norm(x, 3) > 1e-6 && iter < 6);

	Raw->DDObs.Status = true;
	//精度评定
	//单位权中误差
	Mat_multiply(B, x, Bx, row, dim, dim, 1);
	Mat_subtract(Bx, w, V, row, 1, row, 1);
	Mat_transpose(V, VT, row, 1);
	Mat_multiply(VT, P, VTP, 1, row, row, row);
	Mat_multiply(VTP, V, VTPV, 1, row, row, 1);
	Raw->DDObs.FloatSigma0 = sqrt(VTPV[0] / (row - dim - 1));
	
	CopyArray(Raw->DDObs.Qxx, N, dim* dim);

	Raw->DDObs.Sats = dim - 3;
	Raw->DDObs.DDSatNum[0] = 2 * SatNum[0];
	Raw->DDObs.DDSatNum[1] = 2 * SatNum[1];
	for (i = 0; i < 3; i++)Raw->DDObs.dPos[i] = X[i];
	for (i = 0; i < 3; i++)Raw->DDObs.BasPos[i] = BasePos[i];
	for (i = 0; i < 3; i++)Raw->DDObs.RovPos[i] = BasePos[i]*dPos[i];
	for (i = 0; i < dim - 3; i++)Raw->DDObs.FloatAmb[i] = X[i + 3];

}
