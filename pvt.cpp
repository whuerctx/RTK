#include "pvt.h"
#include"matrix.h"
#include <iostream>
using namespace std;

//检查星历是否有效
bool IsValid(const int Prn, const GNSSSys Sys, const GPSTIME* t, const GPSEPHREC* GPSEph)
{
    int prn = Prn - 1;
    double tk = (t->Week - GPSEph[prn].TOE.Week) * 604800.0 + (t->SecOfWeek - GPSEph[prn].TOE.SecOfWeek);
    if(Sys==GPS)
    {
        if (tk > 2 * 3600 + 5 * 60 || GPSEph[prn].SVHealth != 0) return false;
        else return true;
    }
    else if (Sys == BDS)
    {
        if (tk < 0 || tk > 3600 + 5 * 60 || GPSEph[prn].SVHealth != 0) return false;
        else return true;
    }
    else return false;
}


//卫星钟差和钟速（含相对论改正）计算
int CompSatClkoff(const int Prn, const GNSSSys Sys, const GPSTIME* t, const GPSEPHREC* GPSEph, const GPSEPHREC* BDSEph, SATMIDRES* Mid)
{
    double delta_tr, delta_tsv, delta_tr_dot, delta_tsv_dot;
    int prn = Prn - 1;
    if (Sys == GPS)
    {
        if (!IsValid(Prn, GPS, t, GPSEph))
        {
            Mid->Valid = false;
            return -1;
        }

        double A = GPSEph[prn].SqrtA * GPSEph[prn].SqrtA;
        double n0 = sqrt(GPSmiu / (A * A * A));
        double n = n0 + GPSEph[prn].DetlaN;
        double tk = (t->Week - GPSEph[prn].TOE.Week) * 604800.0 + (t->SecOfWeek - GPSEph[prn].TOE.SecOfWeek);
        double Mk = GPSEph[prn].M0 + n * tk;
        double Ek = ComputeEccentricAnomaly(Mk, GPSEph[prn].e);

        //钟差
        delta_tr = F * GPSEph[prn].e * GPSEph[prn].SqrtA * sin(Ek);
        double delta_t = (t->Week - GPSEph[prn].TOC.Week) * 604800.0 + (t->SecOfWeek - GPSEph[prn].TOC.SecOfWeek);
        delta_tsv = GPSEph[prn].ClkBias + GPSEph[prn].ClkDrift * delta_t + GPSEph[prn].ClkDriftRate * delta_t * delta_t + delta_tr;
		Mid->SatClkOft = delta_tsv;
        //钟速
        double Ek_dot = n / (1 - GPSEph[prn].e * cos(Ek));// 偏心角的变化率
        delta_tr_dot = F * GPSEph[prn].e * GPSEph[prn].SqrtA * cos(Ek) * Ek_dot;
        delta_tsv_dot = GPSEph[prn].ClkDrift + 2 * GPSEph[prn].ClkDriftRate * delta_t + delta_tr_dot;
        Mid->SatClkSft = delta_tsv_dot;

        return 0;
    }

    else if (Sys == BDS)
    {
        //统一时间基准
        GPSTIME T;
        if (t->SecOfWeek >= 14)
        {
            T.Week = t->Week - 1356;
            T.SecOfWeek = t->SecOfWeek - 14;
        }
        else
        {
            T.Week = t->Week - 1 - 1356;
            T.SecOfWeek = t->SecOfWeek - 14 + 604800;
        }

        if (!IsValid(Prn, BDS, &T, BDSEph))
        {
            Mid->Valid = false;
            return -1;
        }

        double A = BDSEph[prn].SqrtA * BDSEph[prn].SqrtA;
        double n0 = sqrt(BDSmiu / (A * A * A));
        double n = n0 + BDSEph[prn].DetlaN;
        double tk = (T.Week - BDSEph[prn].TOE.Week) * 604800.0 + (T.SecOfWeek - BDSEph[prn].TOE.SecOfWeek);
        double Mk = BDSEph[prn].M0 + n * tk;
        double Ek = ComputeEccentricAnomaly(Mk, BDSEph[prn].e);

        //钟差
        delta_tr = F * BDSEph[prn].e * BDSEph[prn].SqrtA * sin(Ek);
        double delta_t = (T.Week - BDSEph[prn].TOC.Week) * 604800.0 + (T.SecOfWeek - BDSEph[prn].TOC.SecOfWeek);
        delta_tsv = BDSEph[prn].ClkBias + BDSEph[prn].ClkDrift * delta_t + BDSEph[prn].ClkDriftRate * delta_t * delta_t + delta_tr;
        Mid->SatClkOft = delta_tsv;
        //钟速
        double Ek_dot = n / (1 - BDSEph[prn].e * cos(Ek));// 偏心角的变化率
        delta_tr_dot = F * BDSEph[prn].e * BDSEph[prn].SqrtA * cos(Ek) * Ek_dot;
        delta_tsv_dot = BDSEph[prn].ClkDrift + 2 * BDSEph[prn].ClkDriftRate * delta_t + delta_tr_dot;
        Mid->SatClkSft = delta_tsv_dot;

        return 0;
    }
    else return -1;
}

// 牛顿迭代法求偏心异常
double ComputeEccentricAnomaly(double Mk, double e)
{
    double Ek = Mk;  // 初始猜测值，通常可以选择MK作为初值
    double Ek_next;

    do {
        // 计算 f(Ek) 和 f'(Ek)
        double f_Ek = Ek - e * sin(Ek) - Mk;
        double f_prime_Ek = 1 - e * cos(Ek);

        // 计算下一个 E_k
        Ek_next = Ek - f_Ek / f_prime_Ek;

        // 判断是否收敛
        if (fabs(Ek_next - Ek) < 1e-12) {
            break;
        }

        Ek = Ek_next;  // 更新 E_k
    } while (true);
    return Ek;
}

//GPS卫星位置、速度计算
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
    int prn = Prn - 1;
    //检查星历数据是否有效
    if (!IsValid(Prn,GPS, t, Eph))
    {
        Mid->Valid = false;
        return -1;
    }

    //计算轨道长半轴
    double A = Eph[prn].SqrtA * Eph[prn].SqrtA;

    //计算平均运动角速度
    double n0 = sqrt(GPSmiu / (A * A * A));

    //计算信号发射时刻到星历历元的时间 tk
    double tk = (t->Week - Eph[prn].TOE.Week) * 604800.0 + (t->SecOfWeek - Eph[prn].TOE.SecOfWeek);
    //对平均运动角速度进行改正
    double n = n0 + Eph[prn].DetlaN;

    //计算平均近点角
    double Mk = Eph[prn].M0 + n * tk;

    //计算偏近点角Ek
    double Ek = ComputeEccentricAnomaly(Mk, Eph[prn].e);

    //计算真近点角
    double sinE = sin(Ek);
    double cosE = cos(Ek);
    double vk = atan2(sinE * sqrt(1 - Eph[prn].e * Eph[prn].e), cosE - Eph[prn].e);

    //计算升交角距
    double phik = vk + Eph[prn].omega;

    //计算二阶调和改正数
    double delta_uk = Eph[prn].Cuc * cos(2 * phik) + Eph[prn].Cus * sin(2 * phik); // 交角距改正数
    double delta_rk = Eph[prn].Crc * cos(2 * phik) + Eph[prn].Crs * sin(2 * phik); // 轨道半径改正数
    double delta_ik = Eph[prn].Cic * cos(2 * phik) + Eph[prn].Cis * sin(2 * phik); // 轨道倾角改正数

    //计算经过改正的升交角距
    double uk = phik + delta_uk;

    //计算经过改正的轨道半径
    double rk = A * (1 - Eph[prn].e * cosE) + delta_rk;

    //计算经过改正的轨道倾角
    double ik = Eph[prn].i0 + delta_ik + Eph[prn].iDot * tk;

    //计算卫星在轨道平面上的位置
    double xk_prime = rk * cos(uk);
    double yk_prime = rk * sin(uk);

    //计算改正后的升交点经度
    double Omega_k = Eph[prn].OMEGA + (Eph[prn].OMEGADot - GPSomega) * tk - GPSomega * Eph[prn].TOE.SecOfWeek;

    //计算在地固坐标系下的位置
    double cos_Omega_k = cos(Omega_k);
    double sin_Omega_k = sin(Omega_k);
    double cos_ik = cos(ik);
    double sin_ik = sin(ik);

    double xk = xk_prime * cos_Omega_k - yk_prime * cos_ik * sin_Omega_k;
    double yk = xk_prime * sin_Omega_k + yk_prime * cos_ik * cos_Omega_k;
    double zk = yk_prime * sin_ik;

    //更新卫星位置
	Mid->SatPos[0] = xk;
    Mid->SatPos[1] = yk;
    Mid->SatPos[2] = zk;


    //速度计算部分

    double Ek_dot = n / (1 - Eph[prn].e * cosE); // 偏心角的变化率

    double phik_dot = sqrt(1 - Eph[prn].e * Eph[prn].e) * Ek_dot / (1 - Eph[prn].e * cosE);

    double uk_dot = 2 * (Eph[prn].Cus * cos(2 * phik) - Eph[prn].Cuc * sin(2 * phik)) * phik_dot + phik_dot;
    double rk_dot = A * Eph[prn].e * sinE * Ek_dot + 2 * (Eph[prn].Crs * cos(2 * phik) - Eph[prn].Crc * sin(2 * phik)) * phik_dot;
    double Ik_dot = Eph[prn].iDot + 2 * (Eph[prn].Cis * cos(2 * phik) - Eph[prn].Cic * sin(2 * phik)) * phik_dot;
    double omega_k_dot = Eph[prn].OMEGADot - GPSomega;

    double R_dot[12] = { cos_Omega_k,-sin_Omega_k * cos_ik,-(xk_prime * sin_Omega_k + yk_prime * cos_Omega_k * cos_ik),yk_prime * sin_Omega_k * sin_ik,
    sin_Omega_k,cos_Omega_k * cos_ik,xk_prime * cos_Omega_k - yk_prime * sin_Omega_k * cos_ik,-yk_prime * cos_Omega_k * sin_ik,
    0,sin_ik,0,yk_prime * cos_ik };

    double xk_prime_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
    double yk_prime_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);

    double x_y_omega_ik[4] = { xk_prime_dot,yk_prime_dot,omega_k_dot,Ik_dot };
    double vel[3];
    Mat_multiply(R_dot, x_y_omega_ik, vel, 3, 4, 4, 1);

    //更新卫星速度
    Mid->SatVel[0] = vel[0];
    Mid->SatVel[1] = vel[1];
    Mid->SatVel[2] = vel[2];

    Mid->Valid = true;
    return 0; // 成功
}

//BDS卫星位置、速度计算
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
    //统一时间基准
    GPSTIME T;
    T.Week = t->Week - 1356;
    T.SecOfWeek = t->SecOfWeek - 14;

    int prn = Prn - 1;
    //检查星历数据是否有效
    if (!IsValid(Prn,BDS, &T, Eph))
    {
        Mid->Valid = false;
        return -1;
    }

    //计算轨道长半轴
    double A = Eph[prn].SqrtA * Eph[prn].SqrtA;

    //计算平均运动角速度
    double n0 = sqrt(BDSmiu / (A * A * A));

    //计算信号发射时刻到星历历元的时间 tk
    double tk = (T.Week - Eph[prn].TOE.Week) * 604800.0 + (T.SecOfWeek - Eph[prn].TOE.SecOfWeek);

    //对平均运动角速度进行改正
    double n = n0 + Eph[prn].DetlaN;

    //计算平均近点角
    double Mk = Eph[prn].M0 + n * tk;

    //计算偏近点角Ek
    double Ek = ComputeEccentricAnomaly(Mk, Eph[prn].e);

    //计算真近点角
    double sinE = sin(Ek);
    double cosE = cos(Ek);
    double vk = atan2(sinE * sqrt(1 - Eph[prn].e * Eph[prn].e), cosE - Eph[prn].e);

    //计算升交角距
    double phik = vk + Eph[prn].omega;

    //计算二阶调和改正数
    double delta_uk = Eph[prn].Cuc * cos(2 * phik) + Eph[prn].Cus * sin(2 * phik); // 交角距改正数
    double delta_rk = Eph[prn].Crc * cos(2 * phik) + Eph[prn].Crs * sin(2 * phik); // 轨道半径改正数
    double delta_ik = Eph[prn].Cic * cos(2 * phik) + Eph[prn].Cis * sin(2 * phik); // 轨道倾角改正数

    //计算经过改正的升交角距
    double uk = phik + delta_uk;

    //计算经过改正的轨道半径
    double rk = A * (1 - Eph[prn].e * cosE) + delta_rk;

    //计算经过改正的轨道倾角
    double ik = Eph[prn].i0 + delta_ik + Eph[prn].iDot * tk;

    //计算卫星在轨道平面上的位置
    double xk_prime = rk * cos(uk);
    double yk_prime = rk * sin(uk);

    //卫星识别
    if (Eph[prn].i0 < (30 * PI / 180))//轨道倾角<30.卫星为GEO
    {
        //计算改正后的升交点经度
        double Omega_k = Eph[prn].OMEGA + Eph[prn].OMEGADot * tk - BDSomega * Eph[prn].TOE.SecOfWeek;

        double cos_Omega_k = cos(Omega_k);
        double sin_Omega_k = sin(Omega_k);
        double cos_ik = cos(ik);
        double sin_ik = sin(ik);

        double xgk = xk_prime * cos_Omega_k - yk_prime * cos_ik * sin_Omega_k;
        double ygk = xk_prime * sin_Omega_k + yk_prime * cos_ik * cos_Omega_k;
        double zgk = yk_prime * sin_ik;

        double fai1 = BDSomega * tk;
        double fai2 = -5 * PI / 180;

        double cosf1 = cos(fai1);
        double sinf1 = sin(fai1);
        double cosf2 = cos(fai2);
        double sinf2 = sin(fai2);

        double RX[9] = { 1,0,0,0,cosf2,sinf2,0,-sinf2,cosf2 };
        double RZ[9] = { cosf1,sinf1,0,-sinf1,cosf1,0,0,0,1 };
        double xyzgk[3] = { xgk,ygk,zgk };
        double Mid0[9];//三矩阵相乘中间变量
        double pos[3];

        Mat_multiply(RZ, RX, xyzgk, Mid0, pos, 3, 3, 3, 3, 3, 1);

        //更新卫星位置
        Mid->SatPos[0] = pos[0];
        Mid->SatPos[1] = pos[1];
        Mid->SatPos[2] = pos[2];

        //速度计算部分
        //先对[XGK,YGK,ZGK]进行求导
        double Ek_dot = n / (1 - Eph[prn].e * cosE); // 偏心角的变化率

        double phik_dot = sqrt(1 - Eph[prn].e * Eph[prn].e) * Ek_dot / (1 - Eph[prn].e * cosE);

        double uk_dot = 2 * (Eph[prn].Cus * cos(2 * phik) - Eph[prn].Cuc * sin(2 * phik)) * phik_dot + phik_dot;
        double rk_dot = A * Eph[prn].e * sinE * Ek_dot + 2 * (Eph[prn].Crs * cos(2 * phik) - Eph[prn].Crc * sin(2 * phik)) * phik_dot;
        double Ik_dot = Eph[prn].iDot + 2 * (Eph[prn].Cis * cos(2 * phik) - Eph[prn].Cic * sin(2 * phik)) * phik_dot;
        double omega_k_dot = Eph[prn].OMEGADot ;

        double R_dot[12] = { cos_Omega_k,-sin_Omega_k * cos_ik,-(xk_prime * sin_Omega_k + yk_prime * cos_Omega_k * cos_ik),yk_prime * sin_Omega_k * sin_ik,
        sin_Omega_k,cos_Omega_k * cos_ik,xk_prime * cos_Omega_k - yk_prime * sin_Omega_k * cos_ik,-yk_prime * cos_Omega_k * sin_ik,
        0,sin_ik,0,yk_prime * cos_ik };

        double xk_prime_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
        double yk_prime_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);

        double x_y_omega_ik[4] = { xk_prime_dot,yk_prime_dot,omega_k_dot,Ik_dot };
        double xyzgk_dot[3];
        Mat_multiply(R_dot, x_y_omega_ik, xyzgk_dot, 3, 4, 4, 1);
        
        //再对RZ矩阵求导
        double RZ_dot[9] = { -BDSomega * sinf1,BDSomega * cosf1,0,-BDSomega * cosf1,-BDSomega * sinf1,0,0,0,0 };

        //求速度
        double Mid1[3];//求导的中间变量
        double Mid2[3];
        double Mid3[9];//矩阵乘法的中间变量
        double Mid4[9];
        double vel[3];

        Mat_multiply(RZ, RX, xyzgk_dot, Mid3, Mid1, 3, 3, 3, 3, 3, 1);
        Mat_multiply(RZ_dot, RX, xyzgk, Mid4, Mid2, 3, 3, 3, 3, 3, 1);
        Mat_add(Mid1, Mid2, vel, 3, 1, 3, 1);

        //更新卫星速度
        Mid->SatVel[0] = vel[0];
        Mid->SatVel[1] = vel[1];
        Mid->SatVel[2] = vel[2];

        Mid->Valid = true;
        return 0; // 成功
    }
    else//卫星为MEO/IGSO
    {
        //计算改正后的升交点经度
        double Omega_k = Eph[prn].OMEGA + (Eph[prn].OMEGADot - BDSomega) * tk - BDSomega * Eph[prn].TOE.SecOfWeek;

        //计算在地固坐标系下的位置
        double cos_Omega_k = cos(Omega_k);
        double sin_Omega_k = sin(Omega_k);
        double cos_ik = cos(ik);
        double sin_ik = sin(ik);

        double xk = xk_prime * cos_Omega_k - yk_prime * cos_ik * sin_Omega_k;
        double yk = xk_prime * sin_Omega_k + yk_prime * cos_ik * cos_Omega_k;
        double zk = yk_prime * sin_ik;
       
        //更新卫星位置
        Mid->SatPos[0] = xk;
        Mid->SatPos[1] = yk;
        Mid->SatPos[2] = zk;

        //速度计算部分

        double Ek_dot = n / (1 - Eph[prn].e * cosE); // 偏心角的变化率

        double phik_dot = sqrt(1 - Eph[prn].e * Eph[prn].e) * Ek_dot / (1 - Eph[prn].e * cosE);

        double uk_dot = 2 * (Eph[prn].Cus * cos(2 * phik) - Eph[prn].Cuc * sin(2 * phik)) * phik_dot + phik_dot;
        double rk_dot = A * Eph[prn].e * sinE * Ek_dot + 2 * (Eph[prn].Crs * cos(2 * phik) - Eph[prn].Crc * sin(2 * phik)) * phik_dot;
        double Ik_dot = Eph[prn].iDot + 2 * (Eph[prn].Cis * cos(2 * phik) - Eph[prn].Cic * sin(2 * phik)) * phik_dot;
        double omega_k_dot = Eph[prn].OMEGADot - BDSomega;

        double R_dot[12] = { cos_Omega_k,-sin_Omega_k * cos_ik,-(xk_prime * sin_Omega_k + yk_prime * cos_Omega_k * cos_ik),yk_prime * sin_Omega_k * sin_ik,
        sin_Omega_k,cos_Omega_k * cos_ik,xk_prime * cos_Omega_k - yk_prime * sin_Omega_k * cos_ik,-yk_prime * cos_Omega_k * sin_ik,
        0,sin_ik,0,yk_prime * cos_ik };

        double xk_prime_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
        double yk_prime_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);

        double x_y_omega_ik[4] = { xk_prime_dot,yk_prime_dot,omega_k_dot,Ik_dot };
        double vel[3];
        Mat_multiply(R_dot, x_y_omega_ik, vel, 3, 4, 4, 1);

        //更新卫星速度
        Mid->SatVel[0] = vel[0];
        Mid->SatVel[1] = vel[1];
        Mid->SatVel[2] = vel[2];

        Mid->Valid = true;
        return 0; // 成功
    }
}

//卫星位置和速度的地球自转改正(pos为用户位置)
void EarthRotCorrect(SATMIDRES* Mid, double* Pos)
{
    //计算几何距离
    double rou = sqrt((Mid->SatPos[0] - Pos[0]) * (Mid->SatPos[0] - Pos[0]) + (Mid->SatPos[1] - Pos[1]) *
        (Mid->SatPos[1] - Pos[1]) + (Mid->SatPos[2] - Pos[2]) * (Mid->SatPos[2] - Pos[2]));
    //计算信号传播时间
    double deltat = rou / C_Light;
    //计算地球旋转角度
    double arpha = GPSomega * deltat;
    //计算改正矩阵RZ
    double sina = sin(arpha);
    double cosa = cos(arpha);
    double RZ[9] = { cosa,sina,0,-sina,cosa,0,0,0,1 };
    double pos1[3] = { Mid->SatPos[0] ,Mid->SatPos[1] ,Mid->SatPos[2] };
    double vel1[3] = { Mid->SatVel[0] ,Mid->SatVel[1] ,Mid->SatVel[2] };
    double pos[3];
    double vel[3];
    Mat_multiply(RZ, pos1, pos, 3, 3, 3, 1);
    Mat_multiply(RZ, vel1, vel, 3, 3, 3, 1);

    //更新卫星位置
    Mid->SatPos[0] = pos[0];
    Mid->SatPos[1] = pos[1];
    Mid->SatPos[2] = pos[2];

    //更新卫星速度
    Mid->SatVel[0] = vel[0];
    Mid->SatVel[1] = vel[1];
    Mid->SatVel[2] = vel[2];
}