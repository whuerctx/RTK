#include"getsynobs.h"
#include<cmath>
using namespace std;

//时间求差函数
double TimeDiff(const GPSTIME* t1, const GPSTIME* t2)
{
	double t = (t1->Week - t2->Week) * 604800.0 + (t1->SecOfWeek - t2->SecOfWeek);
	return t;
}

//获取时间同步数据
int GetSynObs(FILE* FBas, FILE* FRov, RAWDAT* Raw)
{
    //用于读流动站文件
    static unsigned char buff_rov[MAXRAWLEN], buff_bas[MAXRAWLEN];
    static int lenDecode_bas = 0, lenDecode_rov = 0;
    int lenRead_bas = 0, lenRead_rov = 0;
    int isBasObs = -1, isRovObs = -1;
    int timediff = 999;
    int isSyn = 0;
    POSRES BestPos;

    while (!feof(FRov))//先读流动站数据
    {
        lenRead_rov = fread(buff_rov + lenDecode_rov, sizeof(unsigned char), MAXRAWLEN - lenDecode_rov, FRov); // 从总文件中要读取的数据量
        if (lenRead_rov < MAXRAWLEN - lenDecode_rov)
        {
            isSyn = -1;
            return isSyn;
        }
        lenDecode_rov = lenDecode_rov + lenRead_rov;
        isRovObs = DecodeNovOem7Dat(buff_rov, lenDecode_rov, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &BestPos);
        if (isRovObs == 1) break;
    }

    GPSTIME RovTime;
    GPSTIME BasTime;
    RovTime = Raw->RovEpk.Time;
    BasTime = Raw->BasEpk.Time;
    
    timediff = TimeDiff(&RovTime, &BasTime);//基站的时间减去移动站时间
    if (fabs(timediff) <= 0.5)
    {
        isSyn = 1;
        return isSyn;
    }

    if (timediff > 0.5)
    {
        while (!feof(FBas))//再读基站数据，进行观测值时间匹配
        {
            lenRead_bas = fread(buff_bas + lenDecode_bas, sizeof(unsigned char), MAXRAWLEN - lenDecode_bas, FBas); // 从总文件中要读取的数据量
            if (lenRead_bas < MAXRAWLEN - lenDecode_bas)
            {
                isSyn = -1;
                return isSyn;
            }
            lenDecode_bas = lenDecode_bas + lenRead_bas;//左侧的lenDec大小为20480，右侧lenDec为上一循环剩余不足一个整message的数据量
            isBasObs = DecodeNovOem7Dat(buff_bas, lenDecode_bas, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &BestPos);
            if (isBasObs == 1)
            {
                BasTime = Raw->BasEpk.Time;
                timediff = TimeDiff(&RovTime, &BasTime);//基站的时间减去移动站时间
                // cout << timediff << " " << endl;
                if (fabs(timediff) <= 0.5)
                {
                    isSyn = 1;
                    break;
                }
                else if (timediff < -0.5)
                {
                    isSyn = 0;
                    break;
                }
            }
        }
    }
    return isSyn;

}


int GetSynObs(SOCKET* NetGps1, SOCKET* NetGps2, RAWDAT* Raw)
{
    static unsigned char baseBuf[MAXRAWLEN], roverBuf[MAXRAWLEN];
    static int baseLen = 0, roverLen = 0;
    static int baseDecoded = 0; // 标记基站数据是否已解码
    POSRES BestPos;

    // 1. 优先处理流动站数据
    int roverRecv = recv(*NetGps2, (char*)roverBuf + roverLen, sizeof(roverBuf) - roverLen, 0);
    if (roverRecv > 0) {
        roverLen += roverRecv;
        if (DecodeNovOem7Dat(roverBuf, roverLen, &Raw->RovEpk,
            Raw->GpsEph, Raw->BdsEph, &BestPos)) {
            roverLen = 0; // 解码成功重置缓冲区

            // 2. 检查与基站时间差
            if (baseDecoded) {
                double diff = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800.0
                    + (Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek);

                if (fabs(diff) <= 15) {
                    baseDecoded = 0; // 重置标记
                    return 1;  // 同步成功
                }
            }
        }
        else if (roverLen >= sizeof(roverBuf)) {
            roverLen = 0; // 防止缓冲区溢出
        }
    }

    // 3. 处理基站数据（仅在需要时更新）
    int baseRecv = recv(*NetGps1, (char*)baseBuf + baseLen, sizeof(baseBuf) - baseLen, 0);
    if (baseRecv > 0) {
        baseLen += baseRecv;
        if (DecodeNovOem7Dat(baseBuf, baseLen, &Raw->BasEpk,
            Raw->GpsEph, Raw->BdsEph, &BestPos)) {
            baseLen = 0;
            baseDecoded = 1; // 标记基站数据有效
        }
        else if (baseLen >= sizeof(baseBuf)) {
            baseLen = 0;
        }
    }

    // 错误处理
    if (roverRecv == SOCKET_ERROR || baseRecv == SOCKET_ERROR) {
        return -1;
    }
    return 0;
}