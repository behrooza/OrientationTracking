import numpy as np
import pandas as pd

from transform.transform import transform


def _rotate(t, q, v):
    qv = t.multiply_quaternions(t.multiply_quaternions(q, t.vector_quaternion(v)),
                                t.inverse_quaternion(q))
    return qv[1:]


def test_inverse_quaternion_non_unit():
    t = transform()
    q = np.array([1.0, 2.0, 3.0, 4.0])
    np.testing.assert_allclose(t.multiply_quaternions(q, t.inverse_quaternion(q)),
                               [1, 0, 0, 0], atol=1e-12)


def test_yaw_correction_recovers_reference_heading():
    t = transform()
    identity = np.array([1.0, 0, 0, 0])
    mag_ref = np.array([0.3, -0.5, 1.0])
    for psi in [0.3, -1.0, 2.0]:
        q_psi = t.rotation_quaternion(psi, [0, 1, 0])
        mag_body = _rotate(t, t.inverse_quaternion(q_psi), mag_ref)
        q_yaw = t.yaw_correction(mag_body, identity, mag_ref, identity, alpha_y=0.0)
        np.testing.assert_allclose(_rotate(t, q_yaw, mag_body), mag_ref, atol=1e-9)


def test_transformations_run():
    rng = np.random.default_rng(0)
    n = 50
    acc = np.tile([0.3, 9.7, 0.5], (n, 1)) + rng.normal(0, 0.05, (n, 3))
    gyr = rng.normal(0, 0.01, (n, 3))
    mag = np.tile([20.0, -40.0, 5.0], (n, 1))
    ts = pd.Series(np.arange(n) * 10.0)

    acc_tilt, gyr_tilt = transform().transformation_v1(acc, gyr, ts, 0.98)
    assert acc_tilt.shape == gyr_tilt.shape == (n, 4)
    assert not np.isnan(acc_tilt).any()

    out = transform().transformation_v2(acc, gyr, mag, ts, 0.98, 0.98)
    assert all(o.shape == (n, 4) and not np.isnan(o).any() for o in out)
