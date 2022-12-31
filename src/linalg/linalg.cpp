#include "linalg/linalg.hpp"

#include <vector>
#include <exception>
#include <cmath>

using namespace std;

void linalg::setzero(vector<double>& a, int n1)
{
	int i;
	for(i = 0; i < n1; i++)
	{
		a[i] = 0.0;
	}
}

void linalg::setzero(vector<vector<double>>& a, int n1, int n2)
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

void linalg::matcopy(vector<vector<double>>& a, vector<vector<double>>& b, int n1, int n2)
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

void linalg::transpose(vector<vector<double>>& a, vector<vector<double>>& b, int n1, int n2)
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

void linalg::matvecmultacc(vector<vector<double>>& a, vector<double>& b, vector<double>& c, int n1, int n2)
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

void linalg::matvecmult(vector<vector<double>>& a, vector<double>& b, vector<double>& c, int n1, int n2)
{
	// a is n1 x n2
	// b is n2
	// c is n1
	setzero(c, n1);
	matvecmultacc(a, b, c, n1, n2);
}

void linalg::matmultacc(
	vector<vector<double>>& a, vector<vector<double>>& b, vector<vector<double>>& c, int n1, int n2, int n3)
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
	vector<vector<double>>& a, vector<vector<double>>& b, vector<vector<double>>& c, int n1, int n2, int n3)
{
	// a is n1 x n2
	// b is n2 x n3
	// c is n1 x n3
	setzero(c, n1, n3);
	matmultacc(a, b, c, n1, n2, n3);
}

void linalg::vecadd(vector<double>& a, vector<double>& b, vector<double>& c, int n1)
{
	// a is n1 x 1
	// b is n1 x 1
	// c is n1 x 1
	int i;
	for(i = 0; i < n1; i++)
	{
		c[i] = a[i] + b[i];
	}
}

void linalg::vecsubtract(vector<double>& a, vector<double>& b, vector<double>& c, int n1)
{
	// a is n1 x 1
	// b is n1 x 1
	// c is n1 x 1
	int i;
	for(i = 0; i < n1; i++)
	{
		c[i] = a[i] - b[i];
	}
}

void linalg::matadd(
	vector<vector<double>>& a, vector<vector<double>>& b, vector<vector<double>>& c, int n1, int n2)
{
	// a is n1 x n2
	// b is n1 x n2
	// c is n1 x n2
	int i, j;
	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			c[i][j] = a[i][j] + b[i][j];
		}
	}
}

void linalg::matsubtract(
	vector<vector<double>>& a, vector<vector<double>>& b, vector<vector<double>>& c, int n1, int n2)
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
	el = vector<vector<double>>(n, vector<double>(n, 0.0));
}

linalg::cholesky::cholesky(vector<vector<double>>& a)
	: n(a.size())
	, el(a)
{
	dcmp(a);
}

void linalg::cholesky::inverse(vector<vector<double>>& ainv)
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

void linalg::cholesky::dcmp(vector<vector<double>>& a)
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

void linalg::weightedsum(vector<double>& w, vector<vector<double>>& a, vector<double>& b, int n1, int n2)
{
	// w is n1 x 1
	// a is n1 x n2
	// b is n2 x 1
	int i, j;
	setzero(b, n1);

	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			b[j] += w[i] * a[i][j];
		}
	}
}

void linalg::weightedmult(
	vector<double>& w,
	vector<vector<double>>& a,
	vector<vector<double>>& b,
	vector<vector<double>>& c,
	int n1,
	int n2,
	int n3)
{
	// w is n1 x 1
	// a is n1 x n2
	// b is n1 x n3
	// c is n2 x n3
	int i, j, k;
	setzero(c, n2, n3);

	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			for(k = 0; k < n3; k++)
			{
				c[j][k] += w[i] * a[i][j] * b[i][k];
			}
		}
	}
}