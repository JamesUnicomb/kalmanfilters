#include "linalg/linalg.hpp"

#include <vector>
#include <exception>
#include <cmath>

using namespace std;

void linalg::setzero(Vector& a)
{
	int i;
	for(i = 0; i < a.size(); i++)
	{
		a[i] = 0.0;
	}
}

void linalg::setzero(Matrix& a)
{
	int i, j;
	for(i = 0; i < a.nrows(); i++)
	{
		for(j = 0; j < a.ncols(); j++)
		{
			a[i][j] = 0.0;
		}
	}
}

double linalg::norm(Vector& a)
{
	int i;
	double s = 0.0;
	for(i = 0; i < a.size(); i++)
	{
		s += a[i] * a[i];
	}
	return sqrt(s);
}

void linalg::transpose(Matrix& a, Matrix& b)
{
	int i, j;
	for(i = 0; i < a.nrows(); i++)
	{
		for(j = 0; j < a.ncols(); j++)
		{
			b[j][i] = a[i][j];
		}
	}
}

void linalg::mult(double a, Vector& b, Vector& c)
{
	int i;
	for(i = 0; i < b.size(); i++)
	{
		c[i] = a * b[i];
	}
}

void linalg::mult(double a, Matrix& b, Matrix& c)
{
	int i, j;
	for(i = 0; i < b.nrows(); i++)
	{
		for(j = 0; j < b.ncols(); i++)
		{
			c[i][j] = a * b[i][j];
		}
	}
}

void linalg::mult(double a, double* b, Vector& c, int n)
{
	// copy free array to vector
	int i;
	for(i = 0; i < n; i++)
	{
		c[i] = a * b[i];
	}
}

void linalg::mult(Matrix& a, Vector& b, Vector& c)
{
	int i, j;
	setzero(c);
	for(i = 0; i < a.nrows(); i++)
	{
		for(j = 0; j < a.ncols(); j++)
		{
			c[i] += a[i][j] * b[j];
		}
	}
}

void linalg::mult(Matrix& a, Matrix& b, Matrix& c)
{
	int i, j, k;
	setzero(c);
	for(i = 0; i < a.nrows(); i++)
	{
		for(j = 0; j < b.ncols(); j++)
		{
			for(k = 0; k < a.ncols(); k++)
			{
				c[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

void linalg::add(Vector& a, Vector& b, Vector& c)
{
	int i;
	for(i = 0; i < a.size(); i++)
	{
		c[i] = a[i] + b[i];
	}
}

void linalg::add(Matrix& a, Matrix& b, Matrix& c)
{
	int i, j;
	for(i = 0; i < a.nrows(); i++)
	{
		for(j = 0; j < a.ncols(); j++)
		{
			c[i][j] = a[i][j] + b[i][j];
		}
	}
}

void linalg::subtract(Vector& a, Vector& b, Vector& c)
{
	int i;
	for(i = 0; i < a.size(); i++)
	{
		c[i] = a[i] - b[i];
	}
}

void linalg::subtract(Matrix& a, Matrix& b, Matrix& c)
{
	int i, j;
	for(i = 0; i < a.nrows(); i++)
	{
		for(j = 0; j < a.ncols(); j++)
		{
			c[i][j] = a[i][j] - b[i][j];
		}
	}
}

void linalg::weightedsum(Vector& w, std::vector<Vector>& a, Vector& b)
{
	int i, j;
	setzero(b);
	for(i = 0; i < a.size(); i++)
	{
		for(j = 0; j < b.size(); j++)
		{
			b[j] += w[i] * a[i][j];
		}
	}
}

void linalg::weightedmult(
	Vector& w, std::vector<Vector>& a, Vector& abar, std::vector<Vector>& b, Vector& bbar, Matrix& c)
{
	int i, j, k;
	setzero(c);
	for(i = 0; i < a.size(); i++)
	{
		for(j = 0; j < abar.size(); j++)
		{
			for(k = 0; k < bbar.size(); k++)
			{
				c[j][k] += w[i] * (a[i][j] - abar[j]) * (b[i][k] - bbar[k]);
			}
		}
	}
}