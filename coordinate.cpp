#include "coordinate.h"
#include "matrix.h"
#include<cmath>;
#include<iostream>;
#include<iomanip>;
using namespace std;

//大地坐标到笛卡尔坐标
void BLHToXYZ(const BLH* blh, XYZ* xyz, const double a, const double e)
{
	if (fabs(blh->longitude) < 0.000001 && fabs(blh->latitude) < 0.000001 && fabs(blh->height) < 0.000001)
	{
		xyz->x = 0.0;
		xyz->y = 0.0;
		xyz->z = 0.0;
		return;
	}
	double N = a / sqrt(1 - e * e * sin(blh->latitude) * sin(blh->latitude));
	xyz->x = (N + blh->height) * cos(blh->latitude) * cos(blh->longitude);
	xyz->y = (N + blh->height) * cos(blh->latitude) * sin(blh->longitude);
	xyz->z = (N * (1 - e * e) + blh->height) * sin(blh->latitude);
}

//笛卡尔坐标到大地坐标
void XYZToBLH(const XYZ* xyz, BLH* blh, const double a, const double e)
{
	if (fabs(xyz->x) < 0.000001 && fabs(xyz->y) < 0.000001 && fabs(xyz->z) < 0.000001)
	{
		blh->longitude = 0.0;
		blh->latitude = 0.0;
		blh->height = 0.0;
		return;
	}
	//求经度
	blh->longitude = atan2(xyz->y, xyz->x);

	//求纬度
	double R = sqrt(xyz->x * xyz->x + xyz->y * xyz->y + xyz->z * xyz->z);
	double fai = atan(xyz->z / sqrt(xyz->x * xyz->x + xyz->y * xyz->y));
	blh->latitude = 0;//初始时令纬度为0
	double B;//创建迭代中间变量
	double W;
	double N;

	do
	{
		B = blh->latitude;
		W = sqrt(1 - e * e * sin(blh->latitude) * sin(blh->latitude));
		blh->latitude = atan(tan(fai) * (1 + a * e * e * sin(B) / (xyz->z * W)));

	} while (fabs(B - blh->latitude) > 1e-10);

	//求高度
	N = a / W;
	blh->height = R * cos(fai) / cos(blh->latitude) - N;

}

//测站地平坐标转换矩阵计算函数
void BLHToNEUMat(const BLH* blh, double* Mat)
{
	Mat[0] = -sin(blh->longitude);
	Mat[1] = cos(blh->longitude);
	Mat[2] = 0;
	Mat[3] = -sin(blh->latitude) * cos(blh->longitude);
	Mat[4] = -sin(blh->latitude) * sin(blh->longitude);
	Mat[5] = cos(blh->latitude);
	Mat[6] = cos(blh->latitude) * cos(blh->longitude);
	Mat[7] = cos(blh->latitude) * sin(blh->longitude);
	Mat[8] = sin(blh->latitude);
}

//笛卡尔坐标到测站地平坐标
void XYZToENU(const XYZ* Xr, const XYZ* Xs, ENU* enu, const double a, const double e)
{
	BLH blh;
	double Mat[9];//转换矩阵
	XYZToBLH(Xr, &blh, a, e);
	BLHToNEUMat(&blh, Mat);
	double Dxyz[3] = { Xs->x - Xr->x,Xs->y - Xr->y,Xs->z - Xr->z };//笛卡尔坐标差矩阵
	double Enu[3];//测站地平坐标矩阵
	Mat_multiply(Mat, Dxyz, Enu, 3, 3, 3, 1);
	enu->dE = Enu[0];
	enu->dN = Enu[1];
	enu->dU = Enu[2];
}


//卫星高度角方位角计算函数
void CompSatElAz(const XYZ* Xr, const XYZ* Xs, double* Elev, double* Azim)
{
	ENU enu;
	//这里采用WGS84坐标系参数将笛卡尔坐标系转换成大地坐标系
	double a = 6378137;
	double f = 1 / 298.257223563;
	double e = sqrt(2 * f - f * f);
	XYZToENU(Xr, Xs, &enu, a, e);
	*Elev = atan(enu.dU / sqrt(enu.dE * enu.dE + enu.dN * enu.dN));
	*Azim = atan2(enu.dE, enu.dN);
}

//定位误差计算函数
void CompEnudPos(const XYZ* X0, const XYZ* Xr, ENU* dEnu)
{
	//这里采用WGS84坐标系参数将笛卡尔坐标系转换成大地坐标系
	double a = 6378137;
	double f = 1 / 298.257223563;
	double e = sqrt(2 * f - f * f);
	XYZToENU(X0, Xr, dEnu, a, e);
}

//打印坐标
void CoordinatePrint(BLH* blh)
{
	cout << "经度:" << blh->latitude << endl;
	cout << "纬度：" << blh->longitude << endl;
	cout << "高度：" << blh->height << endl;
}
void CoordinatePrint(XYZ* xyz)
{
	cout << "X:" << xyz->x << endl;
	cout << "Y:" << xyz->y << endl;
	cout << "Z:" << xyz->z << endl;
}
void CoordinatePrint(ENU* enu)
{
	cout << "东方向:" << enu->dE << endl;
	cout << "北方向:" << enu->dN << endl;
	cout << "天方向:" << enu->dU << endl;
}