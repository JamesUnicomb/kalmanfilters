#include "linalg.hpp"
#include <exception>
#include <vector>

using namespace std;

linalg::Vector::Vector()
	: nn(0)
	, v(NULL)
{ }

linalg::Vector::Vector(vector<double>& a)
	: nn(a.size())
	, v(nn > 0 ? new double[nn] : NULL)
{
	int i;
	for(i = 0; i < nn; i++)
		v[i] = a[i];
}

linalg::Vector::Vector(int n)
	: nn(n)
	, v(n > 0 ? new double[n] : NULL)
{ }

linalg::Vector::Vector(int n, const double& a)
	: nn(n)
	, v(n > 0 ? new double[n] : NULL)
{
	for(int i = 0; i < n; i++)
		v[i] = a;
}

linalg::Vector::Vector(int n, const double* a)
	: nn(n)
	, v(n > 0 ? new double[n] : NULL)
{
	for(int i = 0; i < n; i++)
		v[i] = *a++;
}

linalg::Vector::Vector(const linalg::Vector& rhs)
	: nn(rhs.nn)
	, v(nn > 0 ? new double[nn] : NULL)
{
	for(int i = 0; i < nn; i++)
		v[i] = rhs[i];
}

linalg::Vector& linalg::Vector::operator=(const linalg::Vector& rhs)
// postcondition: normal assignment via copying has been performed;
//		if vector and rhs were different sizes, vector
//		has been resized to match the size of rhs
{
	if(this != &rhs)
	{
		if(nn != rhs.nn)
		{
			if(v != NULL)
				delete[](v);
			nn = rhs.nn;
			v = nn > 0 ? new double[nn] : NULL;
		}
		for(int i = 0; i < nn; i++)
			v[i] = rhs[i];
	}
	return *this;
}

linalg::Vector& linalg::Vector::operator=(const std::vector<double>& rhs)
{
	if(nn != rhs.size())
	{
		throw runtime_error("vector sizes do not match.");
	}
	for(int i = 0; i < nn; i++)
		v[i] = rhs[i];

	return *this;
}

linalg::Vector& linalg::Vector::operator+=(const linalg::Vector& rhs)
{
	if(this != &rhs)
	{
		if(nn != rhs.nn)
		{
			throw runtime_error("vector dimensions do not match");
		}
		for(int i = 0; i < nn; i++)
			v[i] += rhs[i];
	}
	return *this;
}

void linalg::Vector::resize(int newn)
{
	if(newn != nn)
	{
		if(v != NULL)
			delete[](v);
		nn = newn;
		v = nn > 0 ? new double[nn] : NULL;
	}
}

void linalg::Vector::assign(int newn, const double& a)
{
	if(newn != nn)
	{
		if(v != NULL)
			delete[](v);
		nn = newn;
		v = nn > 0 ? new double[nn] : NULL;
	}
	for(int i = 0; i < nn; i++)
		v[i] = a;
}

std::vector<double> linalg::Vector::tovec()
{
	int i;
	std::vector<double> out(nn, 0.0);
	for(i = 0; i < nn; i++)
		out[i] = v[i];
	return out;
}

linalg::Vector::~Vector()
{
	if(v != NULL)
		delete[](v);
}
