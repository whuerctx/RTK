#pragma once
using namespace std;


//向量加法
void Vec_add(const double* v1, const double* v2, double* result, int size1, int size2);

//向量减法
void Vec_subtract(const double* v1, const double* v2, double* result, int size1, int size2);

//向量点积
double Vec_dot(const double* v1, const double* v2, int size1, int size2);

//向量叉积
void Vec_cross(const double* v1, const double* v2, double* result, int size1, int size2);

//打印向量
void Vec_print(const double* v, int size);

//向量取模
double Norm(const double* v, int size);

//复制向量
void CopyArray(double* v1, const double* v2, int size);

//清空向量
void EmptyArray(double* v, int size);