#include "ExtendedKalmanFilter.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

using namespace std;
using namespace linalg;

template <typename F, typename H, typename Z>
ExtendedKalmanFilter<F, H, Z>::ExtendedKalmanFilter(int statedim, int measuredim)
	: statedim(statedim)
	, measuredim(measuredim)
{
	int i;
	// N is state dimension
	// M is measurement dimension
	// state is Nx1 and sigma is NxN
	state = std::vector<double>(statedim, 0.0);
	state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	for(i = 0; i < statedim; i++)
	{
		state_unc[i][i] = 1.0;
	}

	// process and measurement uncertainty matrices
	process_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	measure_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

	// innovation is Nx1
	innovation = std::vector<double>(measuredim, 0.0);
	innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
	innovation_unc_inv = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

	// fill tmp matrices
	fjac = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	fjacT = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	hjac = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
	hjacT = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

	// kalman gain is NxM
	gain = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

	// update variables
	dx = std::vector<double>(statedim, 0.0);
	eye = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	for(i = 0; i < statedim; i++)
	{
		eye[i][i] = 1.0;
	}

	tmp = std::vector<std::vector<double>>(
		MAX(measuredim, statedim), std::vector<double>(MAX(measuredim, statedim), 0.0));
	tmpunc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
}

template <typename F, typename H, typename Z>
void ExtendedKalmanFilter<F, H, Z>::predict(double delta, F f)
{ 
    f(delta, state);
    f.derivs(delta, state, fjac);
    f.getProcessUncertainty(delta, process_unc);
    
    transpose(fjac, fjacT, statedim, statedim);

    matmult(fjac, state_unc, tmp, statedim, statedim, statedim);
    matmult(tmp, fjacT, state_unc, statedim, statedim, statedim);

    matadd(state_unc, process_unc, state_unc, statedim, statedim);
}

template <typename F, typename H, typename Z>
void ExtendedKalmanFilter<F, H, Z>::update(H h, const Z& z)
{
	h(state, z, innovation);
	h.derivs(state, hjac);
    h.getMeasurementUncertainty(measure_unc);

	transpose(hjac, hjacT, measuredim, statedim);
	matmult(hjac, state_unc, tmp, measuredim, statedim, statedim);
	matmult(tmp, hjacT, innovation_unc, measuredim, statedim, measuredim);
	matadd(innovation_unc, measure_unc, innovation_unc, measuredim, measuredim);

	// calculate kalman gain
	cholesky cho(innovation_unc);
	cho.inverse(innovation_unc_inv);
	matmult(hjacT, innovation_unc_inv, tmp, statedim, measuredim, measuredim);
	matmult(state_unc, tmp, gain, statedim, statedim, measuredim);

	// update
	matvecmult(gain, innovation, dx, statedim, measuredim);
	for(int i = 0; i < statedim; i++)
	{
		state[i] += dx[i];
	}

	matmult(gain, hjac, tmp, statedim, measuredim, statedim);
	matsubtract(eye, tmp, tmp, statedim, statedim);
	matcopy(state_unc, tmpunc, statedim, statedim);
	matmult(tmp, tmpunc, state_unc, statedim, statedim, statedim);
}
