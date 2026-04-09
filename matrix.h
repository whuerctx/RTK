#pragma once
using namespace std;

// ¾ØÕó¼Ó·¨
void Mat_add(const double* m1, const double* m2, double* result, int rows1, int cols1, int rows2, int cols2);

// ¾ØÕó¼õ·¨
void Mat_subtract(const double* m1, const double* m2, double* result, int rows1, int cols1, int rows2, int cols2);

// ¾ØÕó³Ë·¨
void Mat_multiply(const double* m1, const double* m2, double* result, int rows1, int cols1, int rows2, int cols2);

// ¾ØÕó³Ë·¨£¨ÈıÁ¬³Ë£©
void Mat_multiply(const double* m1, const double* m2, const double* m3, double* mid, double* result, int rows1, int cols1, int rows2, int cols2, int rows3, int cols3);

//¾ØÕóÇóÄæ
int Mat_inversion(const double* m, double* result, int rows, int cols);

// ¾ØÕó×ªÖÃ
void Mat_transpose(const double* m, double* result, int rows, int cols);

// ´òÓ¡¾ØÕó
void Mat_print(const double* m, int rows, int cols);