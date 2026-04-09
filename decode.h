#pragma once
#include"struct.h"
using namespace std;

//NovOem7数据解码数据提取主函数
int DecodeNovOem7Dat(unsigned char buff[], int& Len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* RcvPos);

//Ushort类型数据提取
unsigned short U2(unsigned char* buff);

//Ulong类型数据提取
unsigned long U4(unsigned char* buff);

//float类型数据提取
float R4(unsigned char* buff);

//double类型数据提取
double R8(unsigned char* buff);

//实现CRC 校验
unsigned int crc32(const unsigned char* buff, int len);

//实现range的解码
void decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs);

//实现GPS的解码
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph);

//实现北斗的解码
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph);

//实现定位结果解码
void decode_bestpos(unsigned char* buff, POSRES* pos);