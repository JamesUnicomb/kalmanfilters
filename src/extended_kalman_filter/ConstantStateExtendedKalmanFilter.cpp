#include "ConstantStateExtendedKalmanFilter.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

void ConstantStateExtendedKalmanFilter::update(const vector<double>& accel)
{
	// run trig functions once
	double sr, cr, cp, sp;
	sr = sin(state[0]);
	cr = cos(state[0]);
	sp = sin(state[1]);
	cp = cos(state[1]);

	// calculate innovation
    // y = z - h(x)
	innovation[0] = accel[0] - sp * g;
	innovation[1] = accel[1] + sr * cp * g;
	innovation[2] = accel[2] - cr * cp * g;

    // innovation uncertainty
    // S = dh/dx * sigma * dh/dx^T + R
    dh[0][0] = 0.0;
    dh[0][1] = cp * g;
    dh[1][0] = -cr * cp * g;
    dh[1][1] = sr * sp * g;
    dh[2][0] = -sr * cp * g;
    dh[2][1] = -cr * sp * g;

    transpose(dh, dhT, 3, 2);
    setzero(dhS, 3, 2);
    matmult(dh, state_unc, dhS, 3, 2, 2);
    setzero(innovation_unc, 3, 3);
    matmult(dhS, dhT, innovation_unc, 3, 2, 3);
    for (int i = 0; i < 3; i++) {
        innovation_unc[i][i] += measurement_unc;
    }

    // calculate kalman gain
    inv33(innovation_unc, innovation_unc_inv);
    setzero(dhTSinv, 2, 3);
    matmult(dhT, innovation_unc_inv, dhTSinv, 2, 3, 3);
    setzero(gain, 2, 3);
    matmult(state_unc, dhTSinv, gain, 2, 2, 3);

    // update
    setzero(dx, 3);
    matvecmult(gain, innovation, dx, 2, 3);
    for (int i = 0; i < 2; i++) {
        state[i] += dx[i];
    }

    setzero(khtmp, 2, 2);
    matmult(gain, dh, khtmp, 2, 3, 2);
    matsubtract(eye, khtmp, khtmp, 2, 2);
    matcopy(state_unc, tmpunc, 2, 2);
    setzero(state_unc, 2, 2);
    matmult(khtmp, tmpunc, state_unc, 2, 2, 2);
}

void ConstantStateExtendedKalmanFilter::predict(double dt)
{
    // x = f(x) ~ x
    // sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	for(int i = 0; i < 2; i++)
	{
		state_unc[i][i] += dt * process_unc;
	}
}