#include<iostream>
#include<iomanip>
#include"decode.h"
#include "coordinate.h"
using namespace std;

//Ushort类型数据提取
unsigned short U2(unsigned char* buff)
{
    unsigned short result;
    memcpy(&result, buff, 2);
    return result;
}

//Ulong类型数据提取
unsigned long U4(unsigned char* buff)
{
    unsigned long result;
    memcpy(&result, buff, 4);
    return result;
}

//float类型数据提取
float R4(unsigned char* buff)
{
    float result;
    memcpy(&result, buff, 4);
    return result;

}

//double类型数据提取
double R8(unsigned char* buff)
{
    double result;
    memcpy(&result, buff, 8);
    return result;
}

//实现CRC 校验
unsigned int crc32(const unsigned char* buff, int len)
{
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++)
    {
        crc ^= buff[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

//实现range的解码
void decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs)
{
    memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));
    unsigned short MessageLength = U2(buff + 8);//信息长度

	obs->Time.Week = U2(buff + 14);
    obs->Time.SecOfWeek = double(U4(buff + 16)) / 1000;
    unsigned char* m = buff + 28;//将首地址移至数据头后
    //读取各个卫星的观测数据
    int j ;
    int obsnum = U4(m);
    for (j=0,m; j < obsnum; j++, m += 44)
    {
        unsigned short prn = U2(m + 4);
        double psr = R8(m + 8);
        double adr = R8(m + 20);
        float dopp = R4(m + 32);
        float cn0 = R4(m + 36);
        float locktime = R4(m + 40);


        //获取卫星系统和信号类型
        unsigned long status = U4(m + 44);
        int sysid = (status >> 16) & 0x07;
        GNSSSys System;
        switch (sysid)
        {
        case 0:
            System = GPS;
            break;
        case 1:
            System = GLONASS;
            break;
        case 3:
            System = GALILEO;
            break;
        case 4:
            System = BDS;
            break;
        case 5:
            System = QZSS;
            break;
        default:
            System = UNKS;
            break;
        }
        if (System != GPS && System != BDS)continue;

        int sigid = (status >> 21) & 0x1F;

        int n=2;//频率序号
        if (System == GPS)
        {
            switch (sigid)
            {
            case 0: n = 0; break;
            case 9: n = 1; break;
            default: break;
            }
        }
        else if (System == BDS)
        {
            switch (sigid)
            {
            case 0: n = 0; break;
            case 4: n = 0; break;
            case 2: n = 1; break;
            case 6: n = 1; break;
            default: break;
            }
        }
        else continue;
        if (n != 0 && n != 1)continue;

        // 将当前解出来的obs放到对应数组
        int p = 0;//卫星序号
        for (int k = 0; k < MAXCHANNUM; k++)
        {
            //如果已存在
            if (obs->SatObs[k].Prn == prn && obs->SatObs[k].System == System)
            {
                p = k;
                break; 
            }
            //如果没有
            if (obs->SatObs[k].Prn == 0 && obs->SatObs[k].System == UNKS)
            {
                p = k;
                break;
            }
        }

        if ((p + 1) > obs->SatNum) obs->SatNum = p + 1;

        obs->SatObs[p].Prn = prn;
        obs->SatObs[p].System = System;
        obs->SatObs[p].P[n] = psr;
        double frep = 0;
        if (System == GPS)
        {
            if (n == 0)frep = FG1_GPS;
            if (n == 1)frep = FG2_GPS;
        }
        if (System == BDS)
        {
            if (n == 0)frep = FG1_BDS;
            if (n == 1)frep = FG3_BDS;
        }
        obs->SatObs[p].L[n] = -1.0 * adr * C_Light / frep;
        obs->SatObs[p].D[n] = -1.0 * dopp * C_Light / frep;
        obs->SatObs[p].cn0[n] = cn0;
        obs->SatObs[p].LockTime[n] = locktime;
        obs->SatObs[p].half[n] = (status >> 11) & 0x1;
        obs->SatObs[p].CodeLock[n] = (status >> 12) & 0x1;
    }
}

//实现GPS的解码
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph)
{
    short prn = 0;
    GPSEPHREC* geph;
    unsigned char* m = buff + 28;//将首地址移至数据头后
    if (buff == NULL) return -1;

    prn = short(U4(m));
    if (prn > 32 || prn < 1) return 0;//防止数组超限
    geph = eph + prn - 1;

    geph->System = GPS;
    geph->PRN = prn;
    geph->SVHealth = int(U4(m + 12));

    eph[prn - 1].IODE = double(U4(m + 16));
    eph[prn - 1].TOE.Week = unsigned short(U4(m + 24));
    eph[prn - 1].TOC.Week = unsigned short(U4(m + 28));
    eph[prn - 1].TOE.SecOfWeek = R8(m + 32);
    eph[prn - 1].SqrtA = sqrt(R8(m + 40));
    eph[prn - 1].DetlaN = R8(m + 48);
    eph[prn - 1].M0 = R8(m + 56);
    eph[prn - 1].e = R8(m + 64);
    eph[prn - 1].omega = R8(m + 72);
    eph[prn - 1].Cuc = R8(m + 80);
    eph[prn - 1].Cus = R8(m + 88);
    eph[prn - 1].Crc = R8(m + 96);
    eph[prn - 1].Crs = R8(m + 104);
    eph[prn - 1].Cic = R8(m + 112);
    eph[prn - 1].Cis = R8(m + 120);
    eph[prn - 1].i0 = R8(m + 128);
    eph[prn - 1].iDot = R8(m + 136);
    eph[prn - 1].OMEGA = R8(m + 144);
    eph[prn - 1].OMEGADot = R8(m + 152);
    eph[prn - 1].IODC = double(U4(m + 160));
    eph[prn - 1].TOC.SecOfWeek = R8(m + 164);
    eph[prn - 1].TGD1 = R8(m + 172);
    eph[prn - 1].ClkBias = R8(m + 180);
    eph[prn - 1].ClkDrift = R8(m + 188);
    eph[prn - 1].ClkDriftRate = R8(m + 196);
    eph[prn - 1].SVAccuracy = R8(m + 216);
    return 1;

}

//实现北斗的解码
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph)
{
    short prn = 0;
    unsigned char* m = buff + 28;//将首地址移至数据头后
    if (buff == NULL) return -1;

    prn = short(U4(m));
    if (prn > 63 || prn < 1) return 0;//防止数组超限

    eph[prn - 1].System = BDS;
    eph[prn - 1].PRN = prn;
    eph[prn - 1].TOE.Week = eph[prn - 1].TOC.Week = unsigned short(U4(m + 4));
    eph[prn - 1].SVAccuracy = R8(m + 8);
    eph[prn - 1].SVHealth = int(U4(m + 16));
    eph[prn - 1].TGD1 = R8(m + 20);
    eph[prn - 1].TGD2 = R8(m + 28);
    eph[prn - 1].TOC.SecOfWeek = double(U4(m + 40));
    eph[prn - 1].ClkBias = R8(m + 44);
    eph[prn - 1].ClkDrift = R8(m + 52);
    eph[prn - 1].ClkDriftRate = R8(m + 60);
    eph[prn - 1].IODE = double(U4(m + 68));
    eph[prn - 1].TOE.SecOfWeek = double(U4(m + 72));
    eph[prn - 1].SqrtA = R8(m + 76);
    eph[prn - 1].e = R8(m + 84);
    eph[prn - 1].omega = R8(m + 92);
    eph[prn - 1].DetlaN = R8(m + 100);
    eph[prn - 1].M0 = R8(m + 108);
    eph[prn - 1].OMEGA = R8(m + 116);
    eph[prn - 1].OMEGADot = R8(m + 124);
    eph[prn - 1].i0 = R8(m + 132);
    eph[prn - 1].iDot = R8(m + 140);
    eph[prn - 1].Cuc = R8(m + 148);
    eph[prn - 1].Cus = R8(m + 156);
    eph[prn - 1].Crc = R8(m + 164);
    eph[prn - 1].Crs = R8(m + 172);
    eph[prn - 1].Cic = R8(m + 180);
    eph[prn - 1].Cis = R8(m + 188);
    return 1;
}

//实现定位结果解码
void decode_bestpos(unsigned char* buff, POSRES* pos)
{ 
    unsigned short MessageLength = U2(buff + 8);//信息长度
    pos->Time.Week = U2(buff + 14);
    pos->Time.SecOfWeek = double(U4(buff + 16)) / 1000;
    unsigned char* m = buff + 28;//将首地址移至数据头后
    BLH blh;
    XYZ xyz;
    blh.latitude = R8(m + 8) * PI / 180;
    blh.longitude = R8(m + 16) * PI / 180;
    blh.height = R8(m + 24) + R4(m + 32);
    double a = 6378137;
    double f = 1 / 298.257223563;
    double e = sqrt(2 * f - f * f);
    BLHToXYZ(&blh, &xyz, a, e);
    pos->Pos[0] = xyz.x;
    pos->Pos[1] = xyz.y;
    pos->Pos[2] = xyz.z;
}

//NovOem7数据解码数据提取主函数
int DecodeNovOem7Dat(unsigned char buff[], int& Len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* RcvPos)
{
    unsigned char HeaderLgth;
    unsigned short MessageID, MessageLength;
    int i = 0;
    int j = 0;
    int test = 0;
    while (i < Len)
    {
        //如果剩余缓存数据不包含整个数据头，退出
        if (i + 28 > Len) break;

        if (buff[i] != 0xAA || buff[i + 1] != 0x44 || buff[i + 2] != 0x12)
        {
            i++;
            continue;
        }
        HeaderLgth = buff[i + 3];

        MessageID = U2(buff + i + 4);
        MessageLength = U2(buff + i + 8);

        //如果剩余缓存数据不包含整个数据，退出
        if (i + (unsigned short)HeaderLgth + MessageLength + 4 > Len) break;

        //进行CRC校验
        if (crc32(buff + i, HeaderLgth + MessageLength) != U4(buff + i + HeaderLgth + MessageLength))
        {
            i = i + 3;
            continue;
        }
        test = 0;
        switch (MessageID)
        {
        case 43:
            test = 1;
            decode_rangeb_oem7(buff + i, obs);  // 处理 RANGE
            break;
        case 7:
            decode_gpsephem(buff + i, geph);  // 处理 GPSEPHEM
            break;
        case 1696:
            decode_bdsephem(buff + i, beph);  // 处理 BDSEPHEMERIS
            break;
        case 42:
            decode_bestpos(buff + i, RcvPos);  //处理BESTPOS
            for (int kk = 0; kk < 3; kk++)
            {
                obs->Pos[kk] = RcvPos->Pos[kk];
            }
            break;
        default:
            break;
        }
        i += (HeaderLgth + MessageLength + 4);
        if (test == 1) break;
    }

    for (j = 0; j < Len - i; j++)
    {
        buff[j] = buff[i + j];
    }
    Len = j;

    return test;
}