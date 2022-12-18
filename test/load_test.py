import kalmanfilters
import numpy as np

def test_load_csekf():
    kf = kalmanfilters.ConstantStateExtendedKalmanFilter(0.025, 0.05)

    assert np.isclose(kf.process_unc, 0.025)
    assert np.isclose(kf.measurement_unc, 0.05)
    assert np.allclose(kf.state,[0.0, 0.0])
    assert np.allclose(kf.state_unc, [[1.0, 0.0], [0.0, 1.0]])