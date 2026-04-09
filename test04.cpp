#include"getsynobs.h"
#include"coordinate.h"
#include<iostream>
#include<iomanip>
#include<fstream>
#include <sstream>
#include<vector>
#include <algorithm>
#include"struct.h"
#include"time.h"
#include"decode.h"
#include"delaycorrect.h"
#include"spp.h"
#include"pvt.h"
#include"rtkfloat.h"
#include"refsat.h"
#include"singlediff.h"
#include"rtkfixed.h"

#pragma warning(disable: 4996)
using namespace std;

//读取二进制文件
int test01()
{
	XYZ xyz1,xyz2;
	COMMONTIME ct;
	GPSTIME gt;
	int Status = 0;
	RAWDAT Raw;
	PPRESULT Base;
	PPRESULT Rov;
	FILE* FBas = fopen("oem719-202202021500-base.bin", "rb"); // 以二进制模式打开文件
	FILE* FRov = fopen("oem719-202202021500-rover.bin", "rb"); // 以二进制模式打开文件
	if (!FBas)
	{
		cout << "open error!" << endl;
		return -1;
	}
	if (!FRov)
	{
		cout << "open error!" << endl;;
		return -1;
	}

	//输出结果文件
	ofstream ofs;
	ofs.open("result_binary.txt", ios::out);
	

	ct.Year = 2025;
	ct.Month = 5;
	ct.Day = 10;
	ct.Hour = 19;
	ct.Minute = 0;
	ct.Second = 0.0;
	CommonTimeToGpsTime(&ct, &gt);

	while (Status != -1)
	{
		gt.SecOfWeek += 1;
		Status = GetSynObs(FBas, FRov, &Raw);
		cout << endl;
		if (Status == 0)
		{
			cout << "时间同步失败！" << endl;
			continue;
		}
		if (Status == 1)
		{
			cout << Raw.BasEpk.Time.Week << "周" << Raw.BasEpk.Time.SecOfWeek << "秒" << endl;
			if (SPP(&Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, &Rov))
			{
				SPV(&Raw.RovEpk, &Rov);
				if (SPP(&Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, &Base))
				{
					SPV(&Raw.BasEpk, &Base);
				}
				FormSDEpochObs(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs);
				DetectCycleSlip(&Raw.SdObs);
				DetRefSat(&Raw.RovEpk, &Raw.BasEpk, &Raw.SdObs, &Raw.DDObs);
				RTKFloat(&Raw, &Base, &Rov);
				RTKFixed(&Raw);
				if (Raw.DDObs.Status == true)
				{
					
					xyz1.x = Raw.DDObs.BasPos[2];
					xyz1.y = Raw.DDObs.BasPos[2];
					xyz1.z = Raw.DDObs.BasPos[2];
					xyz2.x = Raw.DDObs.RovPos[0];
					xyz2.y = Raw.DDObs.RovPos[1];
					xyz2.z = Raw.DDObs.RovPos[2];


					ofs << Raw.BasEpk.Time.Week << "  ";
					ofs << setiosflags(ios::fixed) << setprecision(3) << Raw.BasEpk.Time.SecOfWeek << "  ";
					ofs << setiosflags(ios::fixed) << setprecision(5) << Raw.DDObs.RovPos[0] << "  " << Raw.DDObs.RovPos[1] << "  " << Raw.DDObs.RovPos[2] << "  ";
					ofs << Raw.DDObs.bFixed << endl;
					//cout << setiosflags(ios::fixed) << setprecision(5) << Raw.DDObs.RovPos[0] << "  " << Raw.DDObs.RovPos[1] << "  " << Raw.DDObs.RovPos[2] << "  " << Raw.DDObs.bFixed << endl;
					//cout << setiosflags(ios::fixed) << setprecision(5) << Raw.DDObs.BasPos[0] << "  " << Raw.DDObs.BasPos[1] << "  " << Raw.DDObs.BasPos[2] << endl;
					cout << setiosflags(ios::fixed) << setprecision(5) << Raw.DDObs.dPos[0] << "  " << Raw.DDObs.dPos[1] << "  " << Raw.DDObs.dPos[2] << endl;	
				}
			}
			else
			{
				cout << "流动站单点定位失败" << endl;
			}

		}


	}
	fclose(FBas);
	fclose(FRov);
	ofs.close();
	return 0;
}

//实时流
int test02()
{
	XYZ xyz1, xyz2;
	BLH blh1, blh2;
	int Status = 0;
	RAWDAT Raw;
	PPRESULT Base;
	PPRESULT Rov;
	SOCKET  NetGps_bas;
	SOCKET  NetGps_rov;
	int count;
	if (OpenSocket(NetGps_bas, "8.148.22.229 ", 4002) == false)
	{
		printf("This ip & port was not opened.\n");
		return -1;
	}
	/*if (OpenSocket(NetGps_bas, "47.114.134.129 ", 7190) == false)
	{
		printf("This ip & port was not opened.\n");
		return -1;
	}*/
	if (OpenSocket(NetGps_rov, "8.148.22.229", 7002) == false)
	{
		printf("This ip & port was not opened.\n");
		return -1;
	}

	//输出结果文件
	ofstream ofs1;
	ofs1.open("result_base.txt", ios::out);
	ofstream ofs2;
	ofs2.open("result_rov.txt", ios::out);
	ofstream ofs3;
	ofs3.open("resultotal.txt", ios::out);

	count = 0;
	
	while (count <= 3600 * 8 + 10)
	{
		count++;
		Sleep(980);

		Status = GetSynObs(&NetGps_bas, &NetGps_bas, &Raw);
		cout << SPP(&Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, &Rov);
		cout << endl;
		if (Status == 0)
		{
			cout << "时间同步失败！" << endl;
			continue;
		}
		if (Status == 1)
		{
			cout << Raw.BasEpk.Time.Week << "周" << Raw.BasEpk.Time.SecOfWeek << "秒" << endl;
			if (SPP(&Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, &Rov))
			{
				SPV(&Raw.RovEpk, &Rov);
				if (SPP(&Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, &Base))
				{
					SPV(&Raw.BasEpk, &Base);
				}
				FormSDEpochObs(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs);
				DetectCycleSlip(&Raw.SdObs);
				DetRefSat(&Raw.RovEpk, &Raw.BasEpk, &Raw.SdObs, &Raw.DDObs);
				RTKFloat(&Raw, &Base, &Rov);
				RTKFixed(&Raw);
				if (Raw.DDObs.Status == true)
				{
					xyz1.x = Raw.DDObs.BasPos[2];
					xyz1.y = Raw.DDObs.BasPos[2];
					xyz1.z = Raw.DDObs.BasPos[2];
					xyz2.x = Raw.DDObs.RovPos[0];
					xyz2.y = Raw.DDObs.RovPos[1];
					xyz2.z = Raw.DDObs.RovPos[2];
					double a = 6378137;
					double f = 1 / 298.257223563;
					double e = sqrt(2 * f - f * f);
					XYZToBLH(&xyz1, &blh1, a, e);
					XYZToBLH(&xyz2, &blh2, a, e);
					ofs1 << Raw.BasEpk.Time.Week << "  ";
					ofs1 << setiosflags(ios::fixed) << setprecision(3) << Raw.BasEpk.Time.SecOfWeek << "  ";
					ofs1 << setiosflags(ios::fixed) << setprecision(8) << blh1.latitude * 180 / PI << "  " << blh1.longitude * 180 / PI << "  ";
					ofs1 << setiosflags(ios::fixed) << setprecision(3) << blh1.height - 15 << "  " << endl;;
					ofs2 << Raw.BasEpk.Time.Week << "  ";
					ofs2 << setiosflags(ios::fixed) << setprecision(3) << Raw.BasEpk.Time.SecOfWeek << "  ";
					ofs2 << setiosflags(ios::fixed) << setprecision(8) << blh2.latitude * 180 / PI << "  " << blh2.longitude * 180 / PI << "  ";
					ofs2 << setiosflags(ios::fixed) << setprecision(3) << blh2.height << "  ";
					ofs2 << Raw.DDObs.bFixed << endl;
					ofs3 << Raw.BasEpk.Time.Week << "  ";
					ofs3 << setiosflags(ios::fixed) << setprecision(3) << Raw.BasEpk.Time.SecOfWeek << "  ";
					ofs3 << setiosflags(ios::fixed) << setprecision(5) << xyz1.x << "  " << xyz1.y << "  " << xyz1.z << "  " << xyz2.x << "  " << xyz2.y << "  " << xyz2.z << "  ";
					ofs3 << Raw.DDObs.DDSatNum[0] / 2 << "  " << Raw.DDObs.DDSatNum[1] / 2 << "  ";
					ofs3 << Raw.DDObs.bFixed << endl;

					cout << setiosflags(ios::fixed) << setprecision(5) << Raw.DDObs.dPos[0] << "  " << Raw.DDObs.dPos[1] << "  " << Raw.DDObs.dPos[2] << endl;
				}
			}
			else
			{
				cout << "流动站单点定位失败" << endl;
			}

		}


	}
	CloseSocket(NetGps_bas);
	CloseSocket(NetGps_rov);
	ofs1.close();
	ofs2.close();
	ofs3.close();
	return 0;

}
//统计定位误差
int test03()
{
	ifstream ifs("resulttotal.txt");
	if (!ifs.is_open())
	{
		std::cerr << "无法打开文件!" << std::endl;
		return -1;
	}
	ofstream ofs("error.txt");
	if (!ofs.is_open())
	{
		std::cerr << "无法打开文件!" << std::endl;
		return -1;
	}
	string line;

	XYZ UserPos, BestPos;
	GPSTIME gt;
	int GPSSatNum, BDSSatNum, IsFixed;
	ENU dEnu;
	vector<ENU> DENU;
	vector<double> eastErrors, northErrors, upErrors;
	vector<int> gpsSatNum, bdsSatNum, isFixed;
	// 按行读取文件
	while (std::getline(ifs, line))
	{
		istringstream iss(line);

		// 读取数据到结构体
		if (iss >> gt.Week >> gt.SecOfWeek >> UserPos.x >> UserPos.y >> UserPos.z
			>> BestPos.x >> BestPos.y >> BestPos.z >> GPSSatNum >> BDSSatNum >> IsFixed)
		{
			CompEnudPos(&UserPos, &BestPos, &dEnu);
			DENU.push_back(dEnu);
			eastErrors.push_back(dEnu.dE);
			northErrors.push_back(dEnu.dN);
			upErrors.push_back(dEnu.dU);
			gpsSatNum.push_back(GPSSatNum);
			bdsSatNum.push_back(BDSSatNum);
			isFixed.push_back(IsFixed);
		}
		else
		{
			std::cerr << "数据格式错误！" << std::endl;
			return 1;
		}
	}
	ifs.close();
	cout << std::fixed << std::setprecision(5); // 设置小数点精度
	for (const auto& coord : DENU)
	{
		ofs << coord.dE << "   " << coord.dN << "   " << coord.dU << endl;
	}
	ofs.close();

	//统计精度
	// 计算精度指标的通用函数
	auto calculateStats = [](const vector<double>& errors, const string& direction) {
		if (errors.empty()) return;

		double sum = 0.0;
		for (double error : errors) {
			sum += error;
		}

		double mean = sum / errors.size();
		double rmse = 0.0, stddev = 0.0, minError = *min_element(errors.begin(), errors.end()), maxError = *max_element(errors.begin(), errors.end());
		for (double error : errors) {
			rmse += error * error;
			stddev += (error - mean) * (error - mean);
		}
		rmse = sqrt(rmse / errors.size());
		stddev = sqrt(stddev / errors.size());

		// 输出结果
		cout << direction << "方向:" << endl;
		cout << "  平均误差: " << mean << endl;
		cout << "  均方根误差 (RMSE): " << rmse << endl;
		cout << "  标准差: " << stddev << endl;
		cout << "  最大误差: " << maxError << endl;
		cout << "  最小误差: " << minError << endl;
		cout << endl;
		};

	// 分别统计东向、北向、垂向误差的精度指标
	calculateStats(eastErrors, "东向");
	calculateStats(northErrors, "北向");
	calculateStats(upErrors, "垂向");
	return 0;
}

int main()
{

	test01();
}
