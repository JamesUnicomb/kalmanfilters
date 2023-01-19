#include "linalg.hpp"
#include <vector>
#include <exception>
#include <cmath>

using namespace std;

linalg::cholesky::cholesky(int n)
	: n(n)
{
	el = Matrix(n, n, 0.0);
}

linalg::cholesky::cholesky(Matrix& a)
	: n(a.nrows())
	, el(a)
{
	dcmp();
}

void linalg::cholesky::inverse(Matrix& ainv)
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

void linalg::cholesky::dcmp()
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
					throw runtime_error("Cholesky failed");
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

void linalg::cholesky::dcmp(Matrix& a)
{
	el = a;
	dcmp();
}