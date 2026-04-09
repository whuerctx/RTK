#include"rtkfixed.h"
#include"lambda.h"
#include"matrix.h"
#include"vector.h"
using namespace std;

bool RTKFixed(RAWDAT* Raw)
{

    double FixedAmb[MAXCHANNUM * 4]={0.0}, ResAmb[2] = { 0.0 }, Qxx[MAXCHANNUM * 2 * MAXCHANNUM * 2] = { 0.0 }, Qxy[MAXCHANNUM * 2 * 3] = { 0.0 },
        N[(MAXCHANNUM * 2 + 3) * (MAXCHANNUM * 2 + 3)] = { 0.0 }, QxyN[3 * MAXCHANNUM * 2] = { 0.0 }, dX[MAXCHANNUM * 4] = { 0.0 }, QxyNdX[3] = { 0.0 }, dX2[MAXCHANNUM * 4], QxyNdX2[3] = { 0.0 };


    for (int i = 0; i < Raw->DDObs.Sats; i++)
        for (int j = 0; j < Raw->DDObs.Sats; j++)
            Qxx[i + j * Raw->DDObs.Sats] = Raw->DDObs.Qxx[(i + 3) + (j + 3) * (Raw->DDObs.Sats + 3)];

    //进行模糊度固定
    if (lambda(Raw->DDObs.Sats, 2, Raw->DDObs.FloatAmb, Qxx, FixedAmb, ResAmb) != 0) return false;

    for (int i = 0; i < Raw->DDObs.Sats; i++) Raw->DDObs.FixedAmb[i] = FixedAmb[i];
    for (int i = 0; i < 2; i++) Raw->DDObs.ResAmb[i] = ResAmb[i];

    //Ratio检验
    Raw->DDObs.Ratio = Raw->DDObs.ResAmb[1] / Raw->DDObs.ResAmb[0];
    if (Raw->DDObs.Ratio >= 3)Raw->DDObs.bFixed = true;
    else return false;

    for (int j = 0; j < 3; j++)
        for (int i = 0; i < Raw->DDObs.Sats; i++)
            Qxy[i + j * Raw->DDObs.Sats] = Raw->DDObs.Qxx[j * (Raw->DDObs.Sats + 3) + (3 + i)];

    Mat_inversion(Qxx, N,Raw->DDObs.Sats, Raw->DDObs.Sats);
    Mat_multiply(Qxy, N, QxyN, 3, Raw->DDObs.Sats, Raw->DDObs.Sats, Raw->DDObs.Sats);
    Mat_subtract(Raw->DDObs.FloatAmb, FixedAmb, dX, Raw->DDObs.Sats, 1,Raw->DDObs.Sats, 1);
    Mat_multiply(QxyN, dX, QxyNdX, 3, Raw->DDObs.Sats, Raw->DDObs.Sats, 1);
    Mat_subtract(Raw->DDObs.dPos, QxyNdX, Raw->DDObs.dPos, 3, 1, 3, 1);
    for (int i = 0; i < 3; i++)Raw->DDObs.RovPos[i] = Raw->DDObs.BasPos[i] + Raw->DDObs.dPos[i];


    return 1;
}