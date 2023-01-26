#include "igrf.hpp"
#include "linalg/linalg.hpp"
#include <cmath>

using namespace linalg;

IGRF::IGRF(double year)
	: IGRFConsts(year){};

void IGRF::get_field(double lon, double lat, double alt, double& mx, double& my, double& mz)
{
	int n, m;
	Vector x(3);
	get_position(lon, lat, alt, x);

	double r = norm(x) / a;
	double rn = 1.0 / (r * r * r);
	double phi = atan2(x[1], x[0]);
	double theta = pi / 2.0 - atan(x[2] / sqrt(x[0] * x[0] + x[1] * x[1]));

	legendre(theta);
	for(n = 0; n < 14; n++)
	{
		C[n] = cos(n * phi);
		S[n] = sin(n * phi);
	}

	mx = 0.0, my = 0.0, mz = 0.0;
	double dP;

	for(n = 1; n < 14; n++)
	{
		mz += (n + 1) * P[n][0] * rn * g[n][0];
		mx += P[0][n + 1] * rn * g[n][0];

		for(m = 1; m <= n; m++)
		{
			mz += ((n + 1) * P[n][m] * rn * (g[n][m] * C[m] + h[n][m] * S[m]));
			mx += (P[m][n + 1] * rn * (g[n][m] * C[m] + h[n][m] * S[m]));

			if(theta == 0.0)
			{
				dP = P[n][m];
			}
			else if(theta == pi)
			{
				dP = -P[m][n + 1];
			}
			else
			{
				dP = P[n][m] / P[1][1];
			}

			my += (m * dP * rn * (g[n][m] * S[m] - h[n][m] * C[m]));
		}

		rn = rn / r;
	}

	// convert to micro tesla
	mx *= 1e-3;
	my *= 1e-3;
	mz *= 1e-3;
}

void IGRF::get_position(double lon, double lat, double alt, Vector& x)
{
	double v;
	v = a / sqrt(1 - e2 * sin(lat * (pi / 180.0)) * sin(lat * (pi / 180.0)));

	x[0] = (v + alt) * cos(lat * (pi / 180.0)) * cos(lon * (pi / 180.0));
	x[1] = (v + alt) * cos(lat * (pi / 180.0)) * sin(lon * (pi / 180.0));
	x[2] = (v * (1 - e2) + alt) * sin(lat * (pi / 180.0));
}

void IGRF::legendre(double theta)
{
	int m, n;

	double costh = cos(theta);
	double sinth = sqrt(1.0 - costh * costh);

	P[0][0] = 1.0;
	P[1][1] = sinth;

	for(m = 0; m <= 13; m++)
	{
		double p = sqrt(m + m + 1) * P[m][m];
		P[m + 1][m] = costh * p;

		if(m > 0)
		{
			P[m + 1][m + 1] = sinth * p / sqrt(m + m + 2);
		}

		for(n = m + 2; n <= 13; n++)
		{
			double d = n * n - m * m;
			double e = n + n - 1;
			P[n][m] = ((e * costh * P[n - 1][m] - sqrt(d - e) * P[n - 2][m]) / sqrt(d));
		}
	}

	P[0][2] = -P[1][1];
	P[1][2] = P[1][0];

	for(n = 2; n <= 13; n++)
	{
		P[0][n + 1] = -sqrt((n * n + n) / 2) * P[n][1];
		P[1][n + 1] = ((sqrt(2 * (n * n + n)) * P[n][0] - sqrt((n * n + n - 2)) * P[n][2]) / 2);

		for(m = 2; m < n; m++)
		{
			P[m][n + 1] =
				(0.5 *
				 (sqrt((n + m) * (n - m + 1)) * P[n][m - 1] - sqrt((n + m + 1) * (n - m)) * P[n][m + 1]));
		}

		P[n][n + 1] = sqrt(2 * n) * P[n][n - 1] / 2;
	}
}