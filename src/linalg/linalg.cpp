#include "linalg/linalg.hpp"

#include <vector>
#include <exception>
#include <cmath>

void linalg::setzero(std::vector<double>& a, int n1)
{
	int i;
	for(i = 0; i < n1; i++)
	{
		a[i] = 0.0;
	}
}

void linalg::setzero(std::vector<std::vector<double>>& a, int n1, int n2)
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

void linalg::matcopy(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b, int n1, int n2)
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

void linalg::transpose(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b, int n1, int n2)
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

void linalg::matvecmultacc(
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

void linalg::matvecmult(
	std::vector<std::vector<double>>& a, std::vector<double>& b, std::vector<double>& c, int n1, int n2)
{
	// a is n1 x n2
	// b is n2
	// c is n1
	setzero(c, n1);
	matvecmultacc(a, b, c, n1, n2);
}

void linalg::matmultacc(
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

void linalg::matmult(
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
	setzero(c, n1, n3);
	matmultacc(a, b, c, n1, n2, n3);
}

void linalg::matsubtract(
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

linalg::cholesky::cholesky(int n)
	: n(n)
{
	el = std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
}

linalg::cholesky::cholesky(std::vector<std::vector<double>>& a)
	: n(a.size())
	, el(a)
{
	dcmp(a);
}

void linalg::cholesky::inverse(std::vector<std::vector<double>>& ainv)
{
	int i, j, k;
	double sum;
	for(i = 0; i < n; i++)
		for(j = 0; j <= i; j++)
		{
			sum = (i == j ? 1. : 0.0);
			for(k = i - 1; k >= j; k--)
				sum -= el[i][k] * ainv[j][k];
			ainv[j][i] = sum / el[i][i];
		}
	for(i = n - 1; i >= 0; i--)
		for(j = 0; j <= i; j++)
		{
			sum = (i < j ? 0.0 : ainv[j][i]);
			for(k = i + 1; k < n; k++)
				sum -= el[k][i] * ainv[j][k];
			ainv[i][j] = ainv[j][i] = sum / el[i][i];
		}
}

void linalg::cholesky::dcmp(std::vector<std::vector<double>>& a)
{
	int i, j, k;
	double sum;
	for(i = 0; i < n; i++)
	{
		for(j = i; j < n; j++)
		{
			for(sum = el[i][j], k = i - 1; k >= 0; k--)
				sum -= el[i][k] * el[j][k];
			if(i == j)
			{
				if(sum <= 0.0)
					throw std::runtime_error("Cholesky failed");
				el[i][i] = sqrt(sum);
			}
			else
				el[j][i] = sum / el[i][i];
		}
	}
	for(i = 0; i < n; i++)
		for(j = 0; j < i; j++)
			el[j][i] = 0.;
}