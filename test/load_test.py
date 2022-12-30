import kalmanfilters
import numpy as np


def test_load_cpekf():
    kf = kalmanfilters.cpekf(0.025)

    assert np.allclose(kf.state, [0.0, 0.0])
    assert np.allclose(kf.state_unc, (10.0 * np.eye(2)).tolist())


def test_load_cvekf():
    kf = kalmanfilters.cvekf(0.025)

    assert np.allclose(kf.state, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert np.allclose(kf.state_unc, (10.0 * np.eye(6)).tolist())
