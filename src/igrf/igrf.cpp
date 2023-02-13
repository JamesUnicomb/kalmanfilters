#include "igrf.hpp"
#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include <cmath>

using namespace linalg;

IGRF::IGRF(double year)
	: IGRFConsts(year){};

void IGRF::get_field(double lon, double lat, double alt, double& mx, double& my, double& mz)
{
	int n, m;
	double theta, r;

	geod2geoc(lat, alt, theta, r);
	double phi = lon * DEG_TO_RAD;
	double rn = pow(RE / r, 2.0);

	legendre(theta);
	for(n = 0; n <= 14; n++)
	{
		C[n] = cos(n * phi);
		S[n] = sin(n * phi);
	}

	mx = 0.0, my = 0.0, mz = 0.0;

	for(n = 1; n <= 13; n++)
	{
		for(m = 0; m <= n; m++)
		{
			mx += (rn * P[m][n + 1]) * (g[n][m] * C[m] + h[n][m] * S[m]) * (RE / r);
			my += (rn * m * P[n][m] * RE) * (h[n][m] * C[m] - g[n][m] * S[m]) / (r * sin(theta));
			mz += (rn * (RE / r) * (n + 1) * P[n][m]) * (g[n][m] * C[m] + h[n][m] * S[m]);
		}

		rn *= (RE / r);
	}

	geoc2geod(theta, r, mx, mz);

	// convert to micro tesla
	mx *= 1e-3;
	my *= 1e-3;
	mz *= 1e-3;
}

void IGRF::geod2geoc(double lat, double alt, double& theta, double& r)
{
	double a = WGS84_a;
	double b = a * sqrt(1 - WGS84_e2);
	double alpha = lat * DEG_TO_RAD;

	double sin_alpha_2 = SQR(sin(alpha));
	double cos_alpha_2 = SQR(cos(alpha));

	double tmp = alt * sqrt(a * a * cos_alpha_2 + b * b * sin_alpha_2);
	double beta = atan((tmp + b * b) / (tmp + a * a) * tan(alpha));
	theta = PI_OVER_TWO - beta;
	r = sqrt(
		alt * alt + 2.0 * tmp +
		a * a * (1 - (1 - pow(b / a, 4.0)) * sin_alpha_2) / (1 - (1 - pow(b / a, 2.0)) * sin_alpha_2));
}

void IGRF::geoc2geod(double theta, double r, double& mx, double& mz)
{
	double a = WGS84_a;
	double b = a * sqrt(1 - WGS84_e2);

	double E2 = 1.0 - (b / a) * (b / a);
	double E4 = E2 * E2;
	double E6 = E4 * E2;
	double E8 = E4 * E4;
	double OME2REQ = (1.0 - E2) * a;
	double A21 = (512.0 * E2 + 128.0 * E4 + 60.0 * E6 + 35.0 * E8) / 1024.0;
	double A22 = (E6 + E8) / 32.0;
	double A23 = -3.0 * (4.0 * E6 + 3.0 * E8) / 256.0;
	double A41 = -(64.0 * E4 + 48.0 * E6 + 35.0 * E8) / 1024.0;
	double A42 = (4.0 * E4 + 2.0 * E6 + E8) / 16.0;
	double A43 = 15.0 * E8 / 256.0;
	double A44 = -E8 / 16.0;
	double A61 = 3.0 * (4.0 * E6 + 5.0 * E8) / 1024.0;
	double A62 = -3.0 * (E6 + E8) / 32.0;
	double A63 = 35.0 * (4.0 * E6 + 3.0 * E8) / 768.0;
	double A81 = -5.0 * E8 / 2048.0;
	double A82 = 64.0 * E8 / 2048.0;
	double A83 = -252.0 * E8 / 2048.0;
	double A84 = 320.0 * E8 / 2048.0;

	double GCLAT = PI_OVER_TWO - theta;
	double SCL = sin(GCLAT);

	double RI = a / r;
	double A2 = RI * (A21 + RI * (A22 + RI * A23));
	double A4 = RI * (A41 + RI * (A42 + RI * (A43 + RI * A44)));
	double A6 = RI * (A61 + RI * (A62 + RI * A63));
	double A8 = RI * (A81 + RI * (A82 + RI * (A83 + RI * A84)));

	double CCL = sqrt(1 - SCL * SCL);
	double S2CL = 2.0 * SCL * CCL;
	double C2CL = 2.0 * CCL * CCL - 1.0;
	double S4CL = 2.0 * S2CL * C2CL;
	double C4CL = 2.0 * C2CL * C2CL - 1.0;
	double S8CL = 2.0 * S4CL * C4CL;
	double S6CL = S2CL * C4CL + C2CL * S4CL;

	double DLTCL = S2CL * A2 + S4CL * A4 + S6CL * A6 + S8CL * A8;
	double gdlat = DLTCL + GCLAT;

	double mxtmp = mx;
	double mztmp = mz;

	// magnetic components
	double psi = sin(gdlat) * sin(theta) - cos(gdlat) * cos(theta);
	mx = cos(psi) * mxtmp - sin(psi) * mztmp;
	mz = sin(psi) * mxtmp + cos(psi) * mztmp;
}

void IGRF::legendre(double theta)
{
	/* common derivations of the recursion relations for 
     * generating the associated legendre polynomials. 
	 * for a great introduction see:
	 * http://phys.uri.edu/nigh/NumRec/bookfpdf/f6-8.pdf
	 */
	int m, n;

	double costh = cos(theta);
	double sinth = sqrt(1.0 - costh * costh);

	P[0][0] = 1.0;
	P[1][0] = costh;
	P[1][1] = sinth;

	for(m = 0; m <= 14; m++)
	{
		double Ptmp = sqrt(m + m + 1) * P[m][m];
		P[m + 1][m] = costh * Ptmp;

		if(m > 0)
		{
			P[m + 1][m + 1] = sinth * Ptmp / sqrt(m + m + 2);
		}

		for(n = m + 2; n <= 14; n++)
		{
			double d = n * n - m * m;
			double e = n + n - 1;
			P[n][m] = ((e * costh * P[n - 1][m] - sqrt(d - e) * P[n - 2][m]) / sqrt(d));
		}
	}

	P[0][2] = -P[1][1];
	P[1][2] = P[1][0];

	for(n = 2; n < 14; n++)
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