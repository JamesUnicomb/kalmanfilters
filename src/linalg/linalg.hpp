#ifndef _LINALG_HPP_
#define _LINALG_HPP_

#include <vector>
#include <exception>
#include <cmath>

namespace linalg
{
class Vector
{
private:
	int nn; // size of array. upper index is nn-1
	double* v;

public:
	Vector();
	Vector(std::vector<double>& a);
	explicit Vector(int n); // Zero-based array
	Vector(int n, const double& a); //initialize to constant value
	Vector(int n, const double* a); // Initialize to array
	Vector(const Vector& rhs); // Copy constructor
	Vector& operator=(const Vector& rhs); //assignment
	Vector& operator=(const std::vector<double>& rhs); //assignment from std::vector
	Vector& operator+=(const Vector& rhs); //accumulate
	inline double& operator[](const int i); //i'th element
	inline const double& operator[](const int i) const;
	inline int size() const;
	void resize(int newn); // resize (contents not preserved)
	void assign(int newn, const double& a); // resize and assign a constant value
	std::vector<double> tovec(); // to std::vector
	~Vector();
};

class Matrix
{
private:
	int nn;
	int mm;
	double** v;

public:
	Matrix();
	Matrix(std::vector<std::vector<double>>& a);
	Matrix(int n, int m); // Zero-based array
	Matrix(int n, int m, const double& a); //Initialize to constant
	Matrix(int n, int m, const double* a); // Initialize to array
	Matrix(const Matrix& rhs); // Copy constructor
	Matrix& operator=(const Matrix& rhs); //assignment
	Matrix& operator=(const std::vector<std::vector<double>>& rhs); //assignment from 2d std::vector
	Matrix& operator+=(const Matrix& rhs); //accumulate
	inline double* operator[](const int i); //subscripting: pointer to row i
	inline const double* operator[](const int i) const;
	inline int nrows() const;
	inline int ncols() const;
	void resize(int newn, int newm); // resize (contents not preserved)
	void assign(int newn, int newm, const double& a); // resize and assign a constant value
	std::vector<std::vector<double>> tovec(); // to 2d std::vector
	~Matrix();
};

void setzero(Vector& a);
void setzero(Matrix& a);
void transpose(Matrix& a, Matrix& b);
void mult(double a, Vector& b, Vector& c);
void mult(double a, Matrix& b, Matrix& c);
void mult(Matrix& a, Vector& b, Vector& c);
void mult(Matrix& a, Matrix& b, Matrix& c);
void add(Vector& a, Vector& b, Vector& c);
void add(Matrix& a, Matrix& b, Matrix& c);
void subtract(Vector& a, Vector& b, Vector& c);
void subtract(Matrix& a, Matrix& b, Matrix& c);
// decompositions
struct cholesky
{
	int n;
	Matrix el;
	cholesky(int n);
	cholesky(Matrix& a);
	void inverse(Matrix& ainv);
	void dcmp();
	void dcmp(Matrix& a);
};
struct SVD
{
	int m, n;
	Matrix u, v;
	Vector w;
	double eps, tsh;
	SVD(Matrix& a);
	SVD(int n, int m);
	double inv_condition();
	void dcmp();
	void dcmp(Matrix& a);
	void reorder();
	double pythag(const double a, const double b);
	void inverse(Matrix& ainv);
	void sqrtm(Matrix& asqrtm);
};
// statistial linear algebra routines
void weightedsum(Vector& w, Matrix& a, Vector& b);
void weightedmult(Vector& w, Matrix& a, Vector& abar, Matrix& b, Vector& bbar, Matrix& c);
}; // namespace linalg

#endif /* _LINALG_HPP_ */