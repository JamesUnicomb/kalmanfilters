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

void linalg::vecmult(double c, vector<double>& a, vector<double>& b, int n1)
{
	// a is n1 x 1
	// b is n1 x 1
	int i;
	for(i = 0; i < n1; i++)
	{
		b[i] = c * a[i];
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

void linalg::weightedsum(vector<double>& w, vector<vector<double>>& a, vector<double>& b, int n1, int n2)
{
	// w is n1 x 1
	// a is n1 x n2
	// b is n2 x 1
	int i, j;
	setzero(b, n2);

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
	vector<double>& abar,
	vector<vector<double>>& b,
	vector<double>& bbar,
	vector<vector<double>>& c,
	int n1,
	int n2,
	int n3)
{
	// w    is n1 x 1
	// a    is n1 x n2
	// abar is n2 x 1
	// b    is n1 x n3
	// bbar is n3 x 1
	// c    is n2 x n3
	int i, j, k;
	setzero(c, n2, n3);

	for(i = 0; i < n1; i++)
	{
		for(j = 0; j < n2; j++)
		{
			for(k = 0; k < n3; k++)
			{
				c[j][k] += w[i] * (a[i][j] - abar[j]) * (b[i][k] - bbar[k]);
			}
		}
	}
}