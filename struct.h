#pragma once

//常量
#define MAXCHANNUM 36
#define MAXRAWLEN 20480
#define MAXGPSNUM 32
#define MAXBDSNUM 63
#define POLYCRC32 0xEDB88320u /* CRC32 polynomial */
#define C_Light 299792458.0      /* Speed of light  [m/s]; IAU 1976  */

#define  FG1_GPS  1575.42E6             /* L1信号频率 */
#define  FG2_GPS  1227.60E6             /* L2信号频率 */
#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)

#define  FG1_BDS  1561.098E6               /* B1信号的基准频率 */
#define  FG2_BDS  1207.140E6               /* B2信号的基准频率 */
#define  FG3_BDS  1268.520E6               /* B3信号的基准频率 */
#define  WL1_BDS  (C_Light/FG1_BDS)
#define  WL2_BDS  (C_Light/FG2_BDS)
#define  WL3_BDS  (C_Light/FG3_BDS)       // 波长

#define PI 3.1415926535898
//GPS :
#define GPSmiu 3.986005e14//m^3/s^2(GM)
#define GPSomega 7.2921151467e-5//rad/s(地球自转角速度)
//BDS :
#define BDSmiu 3.986004418e14//m^3/s^2(GM)
#define BDSomega 7.2921150e-5//rad/s(地球自转角速度)

//相对论常数
#define F -4.442807633e-10//(s/(m^1/2))

#define H0 0 //海平面
#define T0 15+273.16 //温度
#define p0 1013.25 //气压
#define RH0 0.5 //相对湿度
#define hw 11000.0 //常数湿度高度

/*************************计时法与结构体定义****************************/

//通用计时法
struct COMMONTIME
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
};

//儒略日
struct JDTIME
{
	int Days;
	double FracDay;

	JDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	};
};

//简化儒略日
struct MJDTIME
{
	int Days;
	double FracDay;

	MJDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	};
};

//GPS时
struct GPSTIME
{
	unsigned short Week;
	double SecOfWeek;

	GPSTIME()
	{
		Week = 0;
		SecOfWeek = 0.0;
	};
};

/**********************空间直角坐标与结构体定义*************************/

//笛卡尔坐标
struct XYZ
{
	double x;
	double y;
	double z;
};

//大地坐标
struct BLH
{
	double longitude;
	double latitude;
	double height;
};

//测站地平坐标
struct ENU
{
	double dE;
	double dN;
	double dU;
};

/**********************卫星星历及观测数据等结构体*************************/

//导航卫星系统定义
enum GNSSSys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };

// 定义同步模式枚举
enum SYNC_MODE
{
	MODE_FILE,   // 文件模式
	MODE_SERIAL  // 串口模式
};

//每颗卫星的观测数据定义 
struct SATOBSDATA
{
	short Prn;
	GNSSSys System;
	double P[2], L[2], D[2], cn0[2], LockTime[2], half[2], CodeLock[2];
	bool Valid;
	SATOBSDATA()
	{
		Prn = 0;
		System = UNKS;
		for (int i = 0; i < 2; i++)
			P[i] = L[i] = D[i] = cn0[i] = LockTime[i] = half[i] = CodeLock[i] = 0.0;
		Valid = false;
	}
};

//每颗卫星位置、速度和钟差等的中间计算结果
struct SATMIDRES
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid;
	//false=没有星历或星历过期，true-计算成功
	SATMIDRES()
	{
		SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
		SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
		Elevation = PI / 2.0;
		Azimuth = 0.0;
		TropCorr = 0.0;
		SatClkOft = SatClkSft = 0.0;
		Tgd1 = Tgd2 = TropCorr = 0.0;
		Valid = false;
	}
};

//粗差探测数据结构体定义
struct MWGF
{
	short Prn;//卫星号
	GNSSSys Sys;
	double MW;
	double GF;
	double PIF;
	int n; //平滑计数
	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};

//每个历元的观测数据定义 
struct EPOCHOBSDATA
{
	GPSTIME Time;
	short SatNum;
	SATOBSDATA SatObs[MAXCHANNUM];
	MWGF ComObs[MAXCHANNUM];
	SATMIDRES SatPVT[MAXCHANNUM];
	double Pos[3];//BESTPOS
	EPOCHOBSDATA()
	{
		Time.Week = 0;
		Time.SecOfWeek = 0.0;
		SatNum = 0;
		Pos[0] = Pos[1] = Pos[2] = 0.0;
	}
};

//卫星星历结构体
struct GPSEPHREC
{
	short PRN;
	GNSSSys System;
	GPSTIME TOC, TOE;
	double ClkBias, ClkDrift, ClkDriftRate;
	double IODE, IODC;
	double SqrtA, M0, e, OMEGA, i0, omega;
	double Crs, Cuc, Cus, Cic, Cis, Crc;
	double DetlaN, OMEGADot, iDot;
	int SVHealth;
	double TGD1, TGD2;
	double SVAccuracy;
	GPSEPHREC()
	{
		PRN = -1;
		System = UNKS;
		ClkBias = ClkDrift = ClkDriftRate = 0.0;
		IODE = IODC = 0.0;
		SqrtA = M0 = e = OMEGA = i0 = omega = 0.0;
		Crs = Cuc = Cus = Cic = Cis = Crc = 0.0;
		DetlaN = OMEGADot = iDot = 0.0;
		TGD1 = TGD2 = 0.0;
		SVAccuracy = 0.0;
		SVHealth = -1;
	}
};

//每个历元的定位结果结构体定义
struct POSRES
{
	GPSTIME Time;
	double Pos[3], Vel[3];
	double PDOP, SigmaPos, SigmaVel;
	int SatNum;
	POSRES()
	{
		Pos[0] = Pos[1] = Pos[2] = 0.0;
		Vel[0] = Vel[1] = Vel[2] = 0.0;
		PDOP = SigmaPos = SigmaVel = 0.0;
		SatNum = 0;
	}
};

//每颗卫星的单差观测数据定义
struct SDSATOBS
{
	short    Prn;
	GNSSSys  System;
	short    Valid;
	double   dP[2], dL[2];   // m
	short    nBas, nRov;   // 存储单差观测值对应的基准和流动站的数值索引号

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKS;
		dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
		Valid = -1;
	}
};

//每个历元的单差观测数据定义
struct SDEPOCHOBS
{
	GPSTIME    Time;
	short      SatNum;
	SDSATOBS   SdSatObs[MAXCHANNUM];
	MWGF       SdCObs[MAXCHANNUM];

	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};

//双差相关的数据定义
struct DDCOBS
{
	int RefPrn[2], BasePos[2];         // 参考星卫星号与存储位置，0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // 待估的双差模糊度数量，0=GPS; 1=BDS
	double FloatAmb[MAXCHANNUM * 2];
	double FixedAmb[MAXCHANNUM * 4];  // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA浮点解中的模糊度残差
	float  FixRMS[2];                 // 固定解定位中rms误差
	double dPos[3];                   // 基线向量
	bool bFixed;                      // true为固定，false为未固定
	bool Status;
	double FloatSigma0;
	double FixedSigma0;
	double Qxx[(MAXCHANNUM * 2 + 3) * (MAXCHANNUM * 2 + 3)];
	double BasPos[3];
	double RovPos[3];

	DDCOBS()
	{
		int i;
		for (i = 0; i < 2; i++)
		{
			DDSatNum[i] = 0;    // 各卫星系统的双差数量
			BasePos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // 双差卫星总数
		dPos[0] = dPos[1] = dPos[2] = 0.0;
		ResAmb[0] = ResAmb[1] = FixRMS[1] = Ratio = 0.0;
		FixRMS[0] = 0.0;
		bFixed = false;
		for (i = 0; i < MAXCHANNUM * 2; i++)
		{
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
			FloatAmb[i] = 0.0;
		}
		FloatSigma0 = FixedSigma0 = 0.0;
		for (i = 0; i < (MAXCHANNUM * 2 + 3) * (MAXCHANNUM * 2 + 3); i++) Qxx[i] = 0.0;
		for (i = 0; i > 3; i++)BasPos[i] = RovPos[i] = 0.0;
	}

};

//RTK定位的数据定义
struct RAWDAT
{
	EPOCHOBSDATA BasEpk;
	EPOCHOBSDATA RovEpk;
	SDEPOCHOBS SdObs;
	DDCOBS DDObs;
	GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];
};

/* 每个历元单点定位和测速的结果及其精度指标 */
struct PPRESULT
{
	GPSTIME Time;
	double Position[3];
	double Velocity[3];
	double RcvClkOft[2];               /* 0 为GPS钟差; 1=BDS钟差 */
	double RcvClkSft;
	double PDOP, SigmaPos, SigmaVel;  // 精度指标
	short  GPSSatNum, BDSSatNum;      /* 单点定位使用的GPS卫星数 */
	short  AllSatNum;                /* 观测历元的所有卫星数   */
	bool   IsSuccess;                /* 单点定位是否成功, 1为成功, 0为失败 */

	PPRESULT()
	{
		for (int i = 0; i < 3; i++)		Position[i] = Velocity[i] = 0.0;
		RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
		PDOP = SigmaPos = SigmaVel = 999.9;
		GPSSatNum = BDSSatNum = AllSatNum = 0;
		IsSuccess = false;
	}
};