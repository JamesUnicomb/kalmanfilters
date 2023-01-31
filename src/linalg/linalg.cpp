#include "linalg/linalg.hpp"
#include "nr3.hpp"

#include <vector>
#include <exception>
#include <cmath>

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

linalg::SVD::SVD(Matrix& a)
	: n(a.nrows())
	, m(a.ncols())
	, u(a)
{
	v = Matrix(n, n, 0.0);
	w = Vector(n, 0.0);
	eps = numeric_limits<double>::epsilon();
	dcmp();
	reorder();
	tsh = 0.5 * sqrt(m + n + 1.) * w[0] * eps;
}

linalg::SVD::SVD(int n, int m)
	: n(n)
	, m(m)
{
	u = Matrix(n, m, 0.0);
	v = Matrix(n, n, 0.0);
	w = Vector(n, 0.0);
	eps = numeric_limits<double>::epsilon();
	dcmp();
	reorder();
	tsh = 0.5 * sqrt(m + n + 1.) * w[0] * eps;
}

double linalg::SVD::inv_condition()
{
	return (w[0] <= 0. || w[n - 1] <= 0.) ? 0. : w[n - 1] / w[0];
}

void linalg::SVD::dcmp()
{
	bool flag;
	int i, its, j, jj, k, l, nm;
	double anorm, c, f, g, h, s, scale, x, y, z;
	Vector rv1(n);
	g = scale = anorm = 0.0;
	for(i = 0; i < n; i++)
	{
		l = i + 2;
		rv1[i] = scale * g;
		g = s = scale = 0.0;
		if(i < m)
		{
			for(k = i; k < m; k++)
				scale += abs(u[k][i]);
			if(scale != 0.0)
			{
				for(k = i; k < m; k++)
				{
					u[k][i] /= scale;
					s += u[k][i] * u[k][i];
				}
				f = u[i][i];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				u[i][i] = f - g;
				for(j = l - 1; j < n; j++)
				{
					for(s = 0.0, k = i; k < m; k++)
						s += u[k][i] * u[k][j];
					f = s / h;
					for(k = i; k < m; k++)
						u[k][j] += f * u[k][i];
				}
				for(k = i; k < m; k++)
					u[k][i] *= scale;
			}
		}
		w[i] = scale * g;
		g = s = scale = 0.0;
		if(i + 1 <= m && i + 1 != n)
		{
			for(k = l - 1; k < n; k++)
				scale += abs(u[i][k]);
			if(scale != 0.0)
			{
				for(k = l - 1; k < n; k++)
				{
					u[i][k] /= scale;
					s += u[i][k] * u[i][k];
				}
				f = u[i][l - 1];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				u[i][l - 1] = f - g;
				for(k = l - 1; k < n; k++)
					rv1[k] = u[i][k] / h;
				for(j = l - 1; j < m; j++)
				{
					for(s = 0.0, k = l - 1; k < n; k++)
						s += u[j][k] * u[i][k];
					for(k = l - 1; k < n; k++)
						u[j][k] += s * rv1[k];
				}
				for(k = l - 1; k < n; k++)
					u[i][k] *= scale;
			}
		}
		anorm = MAX(anorm, (abs(w[i]) + abs(rv1[i])));
	}
	for(i = n - 1; i >= 0; i--)
	{
		if(i < n - 1)
		{
			if(g != 0.0)
			{
				for(j = l; j < n; j++)
					v[j][i] = (u[i][j] / u[i][l]) / g;
				for(j = l; j < n; j++)
				{
					for(s = 0.0, k = l; k < n; k++)
						s += u[i][k] * v[k][j];
					for(k = l; k < n; k++)
						v[k][j] += s * v[k][i];
				}
			}
			for(j = l; j < n; j++)
				v[i][j] = v[j][i] = 0.0;
		}
		v[i][i] = 1.0;
		g = rv1[i];
		l = i;
	}
	for(i = MIN(m, n) - 1; i >= 0; i--)
	{
		l = i + 1;
		g = w[i];
		for(j = l; j < n; j++)
			u[i][j] = 0.0;
		if(g != 0.0)
		{
			g = 1.0 / g;
			for(j = l; j < n; j++)
			{
				for(s = 0.0, k = l; k < m; k++)
					s += u[k][i] * u[k][j];
				f = (s / u[i][i]) * g;
				for(k = i; k < m; k++)
					u[k][j] += f * u[k][i];
			}
			for(j = i; j < m; j++)
				u[j][i] *= g;
		}
		else
			for(j = i; j < m; j++)
				u[j][i] = 0.0;
		++u[i][i];
	}
	for(k = n - 1; k >= 0; k--)
	{
		for(its = 0; its < 30; its++)
		{
			flag = true;
			for(l = k; l >= 0; l--)
			{
				nm = l - 1;
				if(l == 0 || abs(rv1[l]) <= eps * anorm)
				{
					flag = false;
					break;
				}
				if(abs(w[nm]) <= eps * anorm)
					break;
			}
			if(flag)
			{
				c = 0.0;
				s = 1.0;
				for(i = l; i < k + 1; i++)
				{
					f = s * rv1[i];
					rv1[i] = c * rv1[i];
					if(abs(f) <= eps * anorm)
						break;
					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0 / h;
					c = g * h;
					s = -f * h;
					for(j = 0; j < m; j++)
					{
						y = u[j][nm];
						z = u[j][i];
						u[j][nm] = y * c + z * s;
						u[j][i] = z * c - y * s;
					}
				}
			}
			z = w[k];
			if(l == k)
			{
				if(z < 0.0)
				{
					w[k] = -z;
					for(j = 0; j < n; j++)
						v[j][k] = -v[j][k];
				}
				break;
			}
			if(its == 29)
				throw runtime_error("no convergence in 30 svdcmp iterations");
			x = w[l];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
			g = pythag(f, 1.0);
			f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
			c = s = 1.0;
			for(j = l; j <= nm; j++)
			{
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;
				for(jj = 0; jj < n; jj++)
				{
					x = v[jj][j];
					z = v[jj][i];
					v[jj][j] = x * c + z * s;
					v[jj][i] = z * c - x * s;
				}
				z = pythag(f, h);
				w[j] = z;
				if(z)
				{
					z = 1.0 / z;
					c = f * z;
					s = h * z;
				}
				f = c * g + s * y;
				x = c * y - s * g;
				for(jj = 0; jj < m; jj++)
				{
					y = u[jj][j];
					z = u[jj][i];
					u[jj][j] = y * c + z * s;
					u[jj][i] = z * c - y * s;
				}
			}
			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
}

void linalg::SVD::dcmp(Matrix& a)
{
	u = a;
	dcmp();
	reorder();
	tsh = 0.5 * sqrt(m + n + 1.) * w[0] * eps;
}

void linalg::SVD::inverse(Matrix& ainv)
{
	int i, j, k;
	setzero(ainv);
	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			for(k = 0; k < n; k++)
			{
				ainv[i][j] += (v[i][k] / w[k]) * u[j][k];
			}
		}
	}
}

void linalg::SVD::reorder()
{
	int i, j, k, s, inc = 1;
	double sw;
	vector<double> su(m), sv(n);
	do
	{
		inc *= 3;
		inc++;
	} while(inc <= n);
	do
	{
		inc /= 3;
		for(i = inc; i < n; i++)
		{
			sw = w[i];
			for(k = 0; k < m; k++)
				su[k] = u[k][i];
			for(k = 0; k < n; k++)
				sv[k] = v[k][i];
			j = i;
			while(w[j - inc] < sw)
			{
				w[j] = w[j - inc];
				for(k = 0; k < m; k++)
					u[k][j] = u[k][j - inc];
				for(k = 0; k < n; k++)
					v[k][j] = v[k][j - inc];
				j -= inc;
				if(j < inc)
					break;
			}
			w[j] = sw;
			for(k = 0; k < m; k++)
				u[k][j] = su[k];
			for(k = 0; k < n; k++)
				v[k][j] = sv[k];
		}
	} while(inc > 1);
	for(k = 0; k < n; k++)
	{
		s = 0;
		for(i = 0; i < m; i++)
			if(u[i][k] < 0.)
				s++;
		for(j = 0; j < n; j++)
			if(v[j][k] < 0.)
				s++;
		if(s > (m + n) / 2)
		{
			for(i = 0; i < m; i++)
				u[i][k] = -u[i][k];
			for(j = 0; j < n; j++)
				v[j][k] = -v[j][k];
		}
	}
}

double linalg::SVD::pythag(const double a, const double b)
{
	double absa = abs(a), absb = abs(b);
	return (
		absa > absb ? absa * sqrt(1.0 + SQR(absb / absa))
					: (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + SQR(absa / absb))));
}

void linalg::SVD::sqrtm(Matrix& asqrtm)
{
	int i, j, k;
	setzero(asqrtm);
	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			for(k = 0; k < n; k++)
			{
				asqrtm[i][j] += v[i][k] * sqrt(w[k]) * v[j][k];
			}
		}
	}
}
