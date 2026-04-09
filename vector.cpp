#include "vector.h"
#include <iostream>
using namespace std;


//向量加法
void Vec_add(const double* v1, const double* v2, double* result, int size1, int size2)
{
    if (size1 == size2)
    {
        for (int i = 0; i < size1; ++i)
        {
            result[i] = v1[i] + v2[i];
        }
    }
    else
    {
        std::cerr << "Error: Vectors must be the same dimension for addition." << std::endl;
    }
}

//向量减法
void Vec_subtract(const double* v1, const double* v2, double* result, int size1, int size2)
{
    if (size1 == size2)
    {
        for (int i = 0; i < size1; ++i)
        {
            result[i] = v1[i] - v2[i];
        }
    }
    else
    {
        std::cerr << "Error: Vectors must be the same dimension for subtraction." << std::endl;
    }
}

//向量点积
double Vec_dot(const double* v1, const double* v2, int size1, int size2)
{
    if (size1 != size2)
    {
        std::cerr << "Error: Vectors must be the same dimension for dot product." << std::endl;
        return 0.0;
    }

    double result = 0.0;
    for (int i = 0; i < size1; ++i)
    {
        result += v1[i] * v2[i];
    }
    return result;
}

//向量叉积
void Vec_cross(const double* v1, const double* v2, double* result, int size1, int size2)
{
    if (size1 == 3 && size2 == 3)
    {
        // 仅支持三维向量
        result[0] = v1[1] * v2[2] - v1[2] * v2[1];
        result[1] = v1[2] * v2[0] - v1[0] * v2[2];
        result[2] = v1[0] * v2[1] - v1[1] * v2[0];
    }
    else
    {
        std::cerr << "Error：The cross product of vectors supports only three-dimensional vectors." << std::endl;
    }

}

//打印向量
void Vec_print(const double* v, int size)
{
    std::cout << "(";
    for (int i = 0; i < size; i++)
    {
        std::cout << v[i];
        if (i < size - 1) std::cout << ", ";
    }
    std::cout << ")" << std::endl;
}

//向量取模
double Norm(const double* v, int size)
{
    double sum = 0;
    for (int i = 0; i < size; i++)
    {
        sum += v[i] * v[i];
    }
    return sqrt(sum);
}

//复制向量
void CopyArray(double* v1, const double* v2, int size)
{
    for (int i = 0; i < size; i++)
    {
        v1[i] = v2[i];
    }
}

//清空向量
void EmptyArray(double* v, int size)
{
    for (int i = 0; i < size; i++)
    {
        v[i] = 0.0;
    }
}