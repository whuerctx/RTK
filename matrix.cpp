#include "matrix.h"
#include <iostream>
using namespace std;


// 矩阵加法
void Mat_add(const double* m1, const double* m2, double* result, int rows1, int cols1, int rows2, int cols2)
{
	if (rows1 == rows2 && cols1 == cols2)
	{
		for (int i = 0; i < rows1; ++i)
		{
			for (int j = 0; j < cols1; ++j)
			{
				result[i * cols1 + j] = m1[i * cols1 + j] + m2[i * cols1 + j];
			}
		}
	}
	else
	{
		std::cerr << "Error: Matrices must be the same dimension for addition." << std::endl;
	}
}

// 矩阵减法
void Mat_subtract(const double* m1, const double* m2, double* result, int rows1, int cols1, int rows2, int cols2)
{
	if (rows1 == rows2 && cols1 == cols2)
	{
		for (int i = 0; i < rows1; ++i)
		{
			for (int j = 0; j < cols1; ++j)
			{
				result[i * cols1 + j] = m1[i * cols1 + j] - m2[i * cols1 + j];
			}
		}
	}
	else
	{
		std::cerr << "Error: Matrices must be the same dimension for subtraction." << std::endl;
	}
}

// 矩阵乘法
void Mat_multiply(const double* m1, const double* m2, double* result, int rows1, int cols1, int rows2, int cols2)
{
	if (cols1 == rows2)
	{
		for (int i = 0; i < rows1; ++i)
		{
			for (int j = 0; j < cols2; ++j)
			{
				result[i * cols2 + j] = 0;
				for (int k = 0; k < cols1; ++k)
				{
					result[i * cols2 + j] += m1[i * cols1 + k] * m2[k * cols2 + j];
				}
			}
		}
	}
	else
	{
		std::cerr << "Error!The two matrices do not satisfy the multiplication requirement!" << std::endl;
	}
}

// 矩阵乘法（三连乘）
void Mat_multiply(const double* m1, const double* m2, const double* m3, double* mid, double* result, int rows1, int cols1, int rows2, int cols2, int rows3, int cols3)
{
	Mat_multiply(m1, m2, mid, rows1, cols1, rows2, cols2);
	Mat_multiply(mid, m3, result, rows1, cols2, rows3, cols3);	
}

//矩阵求逆
int Mat_inversion(const double* m, double* result, int rows, int cols)
{
	int i, j, k, l, u, v, is[100], js[100];   /* matrix dimension <= 100 */
	double d, p;

	if (rows <= 0 || cols <= 0 || rows != cols)
	{
		std::cerr << "Error dimension in MatrixInv!" << std::endl;
		return -1;
	}

	/* 将输入矩阵赋值给输出矩阵result，下面对result矩阵求逆，m矩阵不变 */
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < rows; j++)
		{
			result[i * rows + j] = m[i * rows + j];
		}
	}

	for (k = 0; k < rows; k++)
	{
		d = 0.0;
		for (i = k; i < rows; i++)   /* 查找右下角方阵中主元素的位置 */
		{
			for (j = k; j < rows; j++)
			{
				l = rows * i + j;
				p = fabs(result[l]);
				if (p > d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d < 1E-15)   /* 主元素接近于0，矩阵不可逆 */
		{
			//std::cerr << "Divided by 0 in MatrixInv!" << std::endl;
			return 0;
		}

		if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
		{
			for (j = 0; j < rows; j++)
			{
				u = k * rows + j;
				v = is[k] * rows + j;
				p = result[u];
				result[u] = result[v];
				result[v] = p;
			}
		}

		if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
		{
			for (i = 0; i < rows; i++)
			{
				u = i * rows + k;
				v = i * rows + js[k];
				p = result[u];
				result[u] = result[v];
				result[v] = p;
			}
		}

		l = k * rows + k;
		result[l] = 1.0 / result[l];  /* 初等行变换 */
		for (j = 0; j < rows; j++)
		{
			if (j != k)
			{
				u = k * rows + j;
				result[u] = result[u] * result[l];
			}
		}
		for (i = 0; i < rows; i++)
		{
			if (i != k)
			{
				for (j = 0; j < rows; j++)
				{
					if (j != k)
					{
						u = i * rows + j;
						result[u] = result[u] - result[i * rows + k] * result[k * rows + j];
					}
				}
			}
		}
		for (i = 0; i < rows; i++)
		{
			if (i != k)
			{
				u = i * rows + k;
				result[u] = -result[u] * result[l];
			}
		}
	}

	for (k = rows - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
	{
		if (js[k] != k)
		{
			for (j = 0; j < rows; j++)
			{
				u = k * rows + j;
				v = js[k] * rows + j;
				p = result[u];
				result[u] = result[v];
				result[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i < rows; i++)
			{
				u = i * rows + k;
				v = is[k] + i * rows;
				p = result[u];
				result[u] = result[v];
				result[v] = p;
			}
		}
	}
	return 1;
}




// 矩阵转置
void Mat_transpose(const double* m, double* result, int rows, int cols)
{
	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			result[j * rows + i] = m[i * cols + j];
		}
	}
}

// 打印矩阵
void Mat_print(const double* m, int rows, int cols)
{
	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			printf("%20.6f ", m[i * cols + j]);
			//std::cout <<std::fixed<< m[i * cols + j] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}