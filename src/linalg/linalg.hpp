#ifndef _LINALG_HPP_
#define _LINALG_HPP_

#include <vector>
#include <exception>
#include <cmath>

namespace linalg
{
void setzero(std::vector<double>& a, int n1);
void setzero(std::vector<std::vector<double>>& a, int n1, int n2);
void matcopy(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b, int n1, int n2);
void transpose(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b, int n1, int n2);
void matvecmultacc(
	std::vector<std::vector<double>>& a, std::vector<double>& b, std::vector<double>& c, int n1, int n2);
void matvecmult(
	std::vector<std::vector<double>>& a, std::vector<double>& b, std::vector<double>& c, int n1, int n2);
void matmultacc(
	std::vector<std::vector<double>>& a,
	std::vector<std::vector<double>>& b,
	std::vector<std::vector<double>>& c,
	int n1,
	int n2,
	int n3);
void matmult(
	std::vector<std::vector<double>>& a,
	std::vector<std::vector<double>>& b,
	std::vector<std::vector<double>>& c,
	int n1,
	int n2,
	int n3);
void vecadd(std::vector<double>& a, std::vector<double>& b, std::vector<double>& c, int n1);
void vecsubtract(std::vector<double>& a, std::vector<double>& b, std::vector<double>& c, int n1);
void matadd(
	std::vector<std::vector<double>>& a,
	std::vector<std::vector<double>>& b,
	std::vector<std::vector<double>>& c,
	int n1,
	int n2);
void matsubtract(
	std::vector<std::vector<double>>& a,
	std::vector<std::vector<double>>& b,
	std::vector<std::vector<double>>& c,
	int n1,
	int n2);
// decompositions
struct cholesky
{
	int n;
	std::vector<std::vector<double>> el;
	cholesky(int n);
	cholesky(std::vector<std::vector<double>>& a);
	void inverse(std::vector<std::vector<double>>& ainv);
	void dcmp();
};
// statistial linear algebra routines
void weightedsum(
	std::vector<double>& w, std::vector<std::vector<double>>& a, std::vector<double>& b, int n1, int n2);
void weightedmult(
	std::vector<double>& w,
	std::vector<std::vector<double>>& a,
	std::vector<double>& abar,
	std::vector<std::vector<double>>& b,
	std::vector<double>& bbar,
	std::vector<std::vector<double>>& c,
	int n1,
	int n2,
	int n3);
}; // namespace linalg

#endif /* _LINALG_HPP_ */