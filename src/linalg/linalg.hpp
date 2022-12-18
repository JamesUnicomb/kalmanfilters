#ifndef _LINALG_HPP_
#define _LINALG_HPP_

#include <vector>

namespace linalg
{
void zero(std::vector<double>& a, int n1)
{
	int i;
	for(i = 0; i < n1; i++)
	{
		a[i] = 0.0;
	}
}

void zero(std::vector<std::vector<double>>& a, int n1, int n2)
{
	int i, j;
	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			a[i][j] = 0.0;
		}
	}
}

void matcopy(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b, int n1, int n2)
{
	// a is n1 x n2
	// b is n1 x n2
	int i, j;
	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			b[i][j] = a[i][j];
		}
	}
}

void transpose(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b, int n1, int n2)
{
	// a is n1 x n2
	// b is n2 x n1
	int i, j;
	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			b[j][i] = a[i][j];
		}
	}
}

void matvecmult(
	std::vector<std::vector<double>>& a, std::vector<double>& b, std::vector<double>& c, int n1, int n2)
{
	// a is n1 x n2
	// b is n2
	// c is n1
	int i, j;
	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			c[i] += a[i][j] * b[j];
		}
	}
}

void matmult(
	std::vector<std::vector<double>>& a,
	std::vector<std::vector<double>>& b,
	std::vector<std::vector<double>>& c,
	int n1,
	int n2,
	int n3)
{
	// a is n1 x n2
	// b is n2 x n3
	// c is n1 x n3
	int i, j, k;
	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n3; j++)
		{
			for(k = 0; k < n2; k++)
			{
				c[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

void matsubtract(
	std::vector<std::vector<double>>& a,
	std::vector<std::vector<double>>& b,
	std::vector<std::vector<double>>& c,
	int n1,
	int n2)
{
    // a is n1 x n2
	// b is n1 x n2
	// c is n1 x n2
	int i, j;
	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			c[i][j] = a[i][j] - b[i][j];
		}
	}
}

void inv33(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b)
{
	double det = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2]) -
				 a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
				 a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

	double invdet = 1.0 / det;

	b[0][0] = (a[1][1] * a[2][2] - a[2][1] * a[1][2]) * invdet;
	b[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * invdet;
	b[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invdet;
	b[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * invdet;
	b[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invdet;
	b[1][2] = (a[1][0] * a[0][2] - a[0][0] * a[1][2]) * invdet;
	b[2][0] = (a[1][0] * a[2][1] - a[2][0] * a[1][1]) * invdet;
	b[2][1] = (a[2][0] * a[0][1] - a[0][0] * a[2][1]) * invdet;
	b[2][2] = (a[0][0] * a[1][1] - a[1][0] * a[0][1]) * invdet;
}
}; // namespace linalg

#endif /* _LINALG_HPP_ */