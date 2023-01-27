#ifndef _IGRF_HPP_
#define _IGRF_HPP_

#include <math.h>
#include "linalg/linalg.hpp"

struct IGRFConsts
{
	double g[15][15];
	double h[15][15];
	double C[15];
	double S[15];
	double P[15][15];

	IGRFConsts(double year)
	{
		for(int n = 0; n < 14; n++)
		{
			C[n] = 0.0;
			S[n] = 0.0;
			for(int m = 0; m < 14; m++)
			{
				g[n][m] = 0.0;
				h[n][m] = 0.0;
			}
		}

		for(int n = 0; n <= 14; n++)
		{
			for(int m = 0; m <= 14; m++)
			{
				P[n][m] = 0.0;
			}
		}

		double dy = year - 2020.0;
		g[1][0] = -29404.8 + dy * 5.7;
		g[1][1] = -1450.9 + dy * 7.4;
		g[2][0] = -2499.6 + dy * -11.0;
		g[2][1] = 2982.0 + dy * -7.0;
		g[2][2] = 1677.0 + dy * -2.1;
		g[3][0] = 1363.2 + dy * 2.2;
		g[3][1] = -2381.2 + dy * -5.9;
		g[3][2] = 1236.2 + dy * 3.1;
		g[3][3] = 525.7 + dy * -12.0;
		g[4][0] = 903.0 + dy * -1.2;
		g[4][1] = 809.5 + dy * -1.6;
		g[4][2] = 86.3 + dy * -5.9;
		g[4][3] = -309.4 + dy * 5.2;
		g[4][4] = 48.0 + dy * -5.1;
		g[5][0] = -234.3 + dy * -0.3;
		g[5][1] = 363.2 + dy * 0.5;
		g[5][2] = 187.8 + dy * -0.6;
		g[5][3] = -140.7 + dy * 0.2;
		g[5][4] = -151.2 + dy * 1.3;
		g[5][5] = 13.5 + dy * 0.9;
		g[6][0] = 66.0 + dy * -0.5;
		g[6][1] = 65.5 + dy * -0.3;
		g[6][2] = 72.9 + dy * 0.4;
		g[6][3] = -121.5 + dy * 1.3;
		g[6][4] = -36.2 + dy * -1.4;
		g[6][5] = 13.5;
		g[6][6] = -64.7 + dy * 0.9;
		g[7][0] = 80.6 + dy * -0.1;
		g[7][1] = -76.7 + dy * -0.2;
		g[7][2] = -8.2;
		g[7][3] = 56.5 + dy * 0.7;
		g[7][4] = 15.8 + dy * 0.1;
		g[7][5] = 6.4 + dy * -0.5;
		g[7][6] = -7.2 + dy * -0.8;
		g[7][7] = 9.8 + dy * 0.8;
		g[8][0] = 23.7;
		g[8][1] = 9.7 + dy * 0.1;
		g[8][2] = -17.6 + dy * -0.1;
		g[8][3] = -0.5 + dy * 0.4;
		g[8][4] = -21.1 + dy * -0.1;
		g[8][5] = 15.3 + dy * 0.4;
		g[8][6] = 13.7 + dy * 0.3;
		g[8][7] = -16.5 + dy * -0.1;
		g[8][8] = -0.3 + dy * 0.4;
		g[9][0] = 5.0;
		g[9][1] = 8.4;
		g[9][2] = 2.9;
		g[9][3] = -1.5;
		g[9][4] = -1.1;
		g[9][5] = -13.2;
		g[9][6] = 1.1;
		g[9][7] = 8.8;
		g[9][8] = -9.3;
		g[9][9] = -11.9;
		g[10][0] = -1.9;
		g[10][1] = -6.2;
		g[10][2] = -0.1;
		g[10][3] = 1.7;
		g[10][4] = -0.9;
		g[10][5] = 0.7;
		g[10][6] = -0.9;
		g[10][7] = 1.9;
		g[10][8] = 1.4;
		g[10][9] = -2.4;
		g[10][10] = -3.8;
		g[11][0] = 3.0;
		g[11][1] = -1.4;
		g[11][2] = -2.5;
		g[11][3] = 2.3;
		g[11][4] = -0.9;
		g[11][5] = 0.3;
		g[11][6] = -0.7;
		g[11][7] = -0.1;
		g[11][8] = 1.4;
		g[11][9] = -0.6;
		g[11][10] = 0.2;
		g[11][11] = 3.1;
		g[12][0] = -2.0;
		g[12][1] = -0.1;
		g[12][2] = 0.5;
		g[12][3] = 1.3;
		g[12][4] = -1.2;
		g[12][5] = 0.7;
		g[12][6] = 0.3;
		g[12][7] = 0.5;
		g[12][8] = -0.3;
		g[12][9] = -0.5;
		g[12][10] = 0.1;
		g[12][11] = -1.1;
		g[12][12] = -0.3;
		g[13][0] = 0.1;
		g[13][1] = -0.9;
		g[13][2] = 0.5;
		g[13][3] = 0.7;
		g[13][4] = -0.3;
		g[13][5] = 0.8;
		g[13][6] = 0.0;
		g[13][7] = 0.8;
		g[13][8] = 0.0;
		g[13][9] = 0.4;
		g[13][10] = 0.1;
		g[13][11] = 0.5;
		g[13][12] = -0.5;
		g[13][13] = -0.4;

		h[1][1] = 4652.5 + dy * -25.9;
		h[2][1] = -2991.6 + dy * -30.2;
		h[2][2] = -734.6 + dy * -22.4;
		h[3][1] = -82.1 + dy * 6.0;
		h[3][2] = 241.9 + dy * -1.1;
		h[3][3] = -543.4 + dy * 0.5;
		h[4][1] = 281.9 + dy * -0.1;
		h[4][2] = -158.4 + dy * 6.5;
		h[4][3] = 199.7 + dy * 3.6;
		h[4][4] = -349.7 + dy * -5.0;
		h[5][1] = 47.7;
		h[5][2] = 208.3 + dy * 2.5;
		h[5][3] = -121.2 + dy * -0.6;
		h[5][4] = 32.3 + dy * 3.0;
		h[5][5] = 98.9 + dy * 0.3;
		h[6][1] = -19.1;
		h[6][2] = 25.1 + dy * -1.6;
		h[6][3] = 52.8 + dy * -1.3;
		h[6][4] = -64.5 + dy * 0.8;
		h[6][5] = 8.9;
		h[6][6] = 68.1 + dy * 1.0;
		h[7][1] = -51.5 + dy * 0.6;
		h[7][2] = -16.9 + dy * 0.6;
		h[7][3] = 2.2 + dy * -0.8;
		h[7][4] = 23.5 + dy * -0.2;
		h[7][5] = -2.2 + dy * -1.1;
		h[7][6] = -27.2 + dy * 0.1;
		h[7][7] = -1.8 + dy * 0.3;
		h[8][1] = 8.4 + dy * -0.2;
		h[8][2] = -15.3 + dy * 0.6;
		h[8][3] = 12.8 + dy * -0.2;
		h[8][4] = -11.7 + dy * 0.5;
		h[8][5] = 14.9 + dy * -0.3;
		h[8][6] = 3.6 + dy * -0.4;
		h[8][7] = -6.9 + dy * 0.5;
		h[8][8] = 2.8;
		h[9][1] = -23.4;
		h[9][2] = 11.0;
		h[9][3] = 9.8;
		h[9][4] = -5.1;
		h[9][5] = -6.3;
		h[9][6] = 7.8;
		h[9][7] = 0.4;
		h[9][8] = -1.4;
		h[9][9] = 9.6;
		h[10][1] = 3.4;
		h[10][2] = -0.2;
		h[10][3] = 3.6;
		h[10][4] = 4.8;
		h[10][5] = -8.6;
		h[10][6] = -0.1;
		h[10][7] = -4.3;
		h[10][8] = -3.4;
		h[10][9] = -0.1;
		h[10][10] = -8.8;
		h[11][1] = 0.0;
		h[11][2] = 2.5;
		h[11][3] = -0.6;
		h[11][4] = -0.4;
		h[11][5] = 0.6;
		h[11][6] = -0.2;
		h[11][7] = -1.7;
		h[11][8] = -1.6;
		h[11][9] = -3.0;
		h[11][10] = -2.0;
		h[11][11] = -2.6;
		h[12][1] = -1.2;
		h[12][2] = 0.5;
		h[12][3] = 1.4;
		h[12][4] = -1.8;
		h[12][5] = 0.1;
		h[12][6] = 0.8;
		h[12][7] = -0.2;
		h[12][8] = 0.6;
		h[12][9] = 0.2;
		h[12][10] = -0.9;
		h[12][11] = 0.0;
		h[12][12] = 0.5;
		h[13][1] = -0.9;
		h[13][2] = 0.6;
		h[13][3] = 1.4;
		h[13][4] = -0.4;
		h[13][5] = -1.3;
		h[13][6] = -0.1;
		h[13][7] = 0.3;
		h[13][8] = -0.1;
		h[13][9] = 0.5;
		h[13][10] = 0.5;
		h[13][11] = -0.4;
		h[13][12] = -0.4;
		h[13][13] = -0.6;
	}
};

class IGRF : public IGRFConsts
{
public:
	IGRF(double year);
	void get_field(double lon, double lat, double alt, double& mx, double& my, double& mz);
	void geod2geoc(double lat, double alt, double& theta, double& r);
	void geoc2geod(double theta, double r, double& mx, double& mz);
	void legendre(double theta);

private:
	double WGS84_e2 = 0.00669437999014;
	double WGS84_a = 6378.137;
	double d2r = 0.01745329251;
	double RE = 6371.2;
	const double pi = 3.14159265359;
};

#endif