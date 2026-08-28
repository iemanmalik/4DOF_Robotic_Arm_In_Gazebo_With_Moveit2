import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from forward_kinematics.fk import forward_kinematics


def test_fk_zero_configuration():

    q = np.zeros(4)

    T, transforms = forward_kinematics(q)

    assert T.shape == (4, 4)

    assert len(transforms) == 4

    # Homogeneous transformation bottom row
    assert np.allclose(
        T[3, :],
        [0, 0, 0, 1]
    )


def test_fk_identity_rotation():

    q = np.zeros(4)

    T, _ = forward_kinematics(q)

    R = T[:3, :3]

    # Rotation matrix must remain orthonormal
    assert np.allclose(
        R @ R.T,
        np.eye(3),
        atol=1e-6
    )


if __name__ == "__main__":
    test_fk_zero_configuration()
    test_fk_identity_rotation()

    print("FK tests PASSED")