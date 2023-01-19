#include "linalg.hpp"
#include <exception>
#include <vector>

using namespace std;

linalg::Matrix::Matrix()
	: nn(0)
	, mm(0)
	, v(NULL)
{ }

linalg::Matrix::Matrix(vector<vector<double>>& a)
	: nn(a.size())
	, mm(a[0].size())
	, v(nn > 0 ? new double*[nn] : NULL)
{
	int i, j, nel = mm * nn;
	if(v)
		v[0] = nel > 0 ? new double[nel] : NULL;
	for(i = 1; i < nn; i++)
		v[i] = v[i - 1] + mm;
	for(i = 0; i < nn; i++)
		for(j = 0; j < mm; j++)
			v[i][j] = a[i][j];
}

linalg::Matrix::Matrix(int n, int m)
	: nn(n)
	, mm(m)
	, v(n > 0 ? new double*[n] : NULL)
{
	int i, nel = m * n;
	if(v)
		v[0] = nel > 0 ? new double[nel] : NULL;
	for(i = 1; i < n; i++)
		v[i] = v[i - 1] + m;
}

linalg::Matrix::Matrix(int n, int m, const double& a)
	: nn(n)
	, mm(m)
	, v(n > 0 ? new double*[n] : NULL)
{
	int i, j, nel = m * n;
	if(v)
		v[0] = nel > 0 ? new double[nel] : NULL;
	for(i = 1; i < n; i++)
		v[i] = v[i - 1] + m;
	for(i = 0; i < n; i++)
		for(j = 0; j < m; j++)
			v[i][j] = a;
}

linalg::Matrix::Matrix(int n, int m, const double* a)
	: nn(n)
	, mm(m)
	, v(n > 0 ? new double*[n] : NULL)
{
	int i, j, nel = m * n;
	if(v)
		v[0] = nel > 0 ? new double[nel] : NULL;
	for(i = 1; i < n; i++)
		v[i] = v[i - 1] + m;
	for(i = 0; i < n; i++)
		for(j = 0; j < m; j++)
			v[i][j] = *a++;
}

linalg::Matrix::Matrix(const linalg::Matrix& rhs)
	: nn(rhs.nn)
	, mm(rhs.mm)
	, v(nn > 0 ? new double*[nn] : NULL)
{
	int i, j, nel = mm * nn;
	if(v)
		v[0] = nel > 0 ? new double[nel] : NULL;
	for(i = 1; i < nn; i++)
		v[i] = v[i - 1] + mm;
	for(i = 0; i < nn; i++)
		for(j = 0; j < mm; j++)
			v[i][j] = rhs[i][j];
}

linalg::Matrix& linalg::Matrix::operator=(const linalg::Matrix& rhs)
// postcondition: normal assignment via copying has been performed;
//		if matrix and rhs were different sizes, matrix
//		has been resized to match the size of rhs
{
	if(this != &rhs)
	{
		int i, j, nel;
		if(nn != rhs.nn || mm != rhs.mm)
		{
			if(v != NULL)
			{
				delete[](v[0]);
				delete[](v);
			}
			nn = rhs.nn;
			mm = rhs.mm;
			v = nn > 0 ? new double*[nn] : NULL;
			nel = mm * nn;
			if(v)
				v[0] = nel > 0 ? new double[nel] : NULL;
			for(i = 1; i < nn; i++)
				v[i] = v[i - 1] + mm;
		}
		for(i = 0; i < nn; i++)
			for(j = 0; j < mm; j++)
				v[i][j] = rhs[i][j];
	}
	return *this;
}

linalg::Matrix& linalg::Matrix::operator=(const std::vector<std::vector<double>>& rhs)
{

	int i, j, nel;
	if(nn != rhs.size() || mm != rhs[0].size())
	{
		throw runtime_error("matrix dimensions do not match");
	}
	for(i = 0; i < nn; i++)
		for(j = 0; j < mm; j++)
			v[i][j] = rhs[i][j];

	return *this;
}

linalg::Matrix& linalg::Matrix::operator+=(const linalg::Matrix& rhs)
{
	if(this != &rhs)
	{
		int i, j, nel;
		if(nn != rhs.nn || mm != rhs.mm)
		{
			throw runtime_error("matrix dimensions do not match");
		}
		for(i = 0; i < nn; i++)
			for(j = 0; j < mm; j++)
				v[i][j] += rhs[i][j];
	}
	return *this;
}

void linalg::Matrix::resize(int newn, int newm)
{
	int i, nel;
	if(newn != nn || newm != mm)
	{
		if(v != NULL)
		{
			delete[](v[0]);
			delete[](v);
		}
		nn = newn;
		mm = newm;
		v = nn > 0 ? new double*[nn] : NULL;
		nel = mm * nn;
		if(v)
			v[0] = nel > 0 ? new double[nel] : NULL;
		for(i = 1; i < nn; i++)
			v[i] = v[i - 1] + mm;
	}
}

void linalg::Matrix::assign(int newn, int newm, const double& a)
{
	int i, j, nel;
	if(newn != nn || newm != mm)
	{
		if(v != NULL)
		{
			delete[](v[0]);
			delete[](v);
		}
		nn = newn;
		mm = newm;
		v = nn > 0 ? new double*[nn] : NULL;
		nel = mm * nn;
		if(v)
			v[0] = nel > 0 ? new double[nel] : NULL;
		for(i = 1; i < nn; i++)
			v[i] = v[i - 1] + mm;
	}
	for(i = 0; i < nn; i++)
		for(j = 0; j < mm; j++)
			v[i][j] = a;
}

std::vector<std::vector<double>> linalg::Matrix::tovec()
{
	int i, j;
	std::vector<std::vector<double>> out(nn, std::vector<double>(mm, 0.0));
	for(i = 0; i < nn; i++)
		for(j = 0; j < mm; j++)
			out[i][j] = v[i][j];
	return out;
}

linalg::Matrix::~Matrix()
{
	if(v != NULL)
	{
		delete[](v[0]);
		delete[](v);
	}
}