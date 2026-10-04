from controller.thruster_controller_functions import compute_thruster_allocation_matrix
import pytest
import numpy as np


thrusters = [{'id': 1, 'name': 'front_left_lateral', 'position_m': [-0.2, 0.2, 0.0], 'direction': [0.7071, 0.7071, 0.0], 'topic': '/thruster/thruster_fll'}, {'id': 2, 'name': 'front_right_lateral', 'position_m': [0.2, 0.2, 0.0], 'direction': [-0.7071, 0.7071, 0.0], 'topic': '/thruster/thruster_frl'}, {'id': 3, 'name': 'rear_left_lateral', 'position_m': [-0.2, -0.2, 0.0], 'direction': [-0.7071, 0.7071, 0.0], 'topic': '/thruster/thruster_rll'}, {'id': 4, 'name': 'rear_right_lateral', 'position_m': [0.2, -0.2, 0.0], 'direction': [0.7071, 0.7071, 0.0], 'topic': '/thruster/thruster_rrl'}, {'id': 5, 'name': 'front_left_vertical', 'position_m': [-0.2, 0.2, 0.0], 'direction': [0.0, 0.0, 1.0], 'topic': '/thruster/thruster_flv'}, {'id': 6, 'name': 'front_right_vertical', 'position_m': [0.2, 0.2, 0.0], 'direction': [0.0, 0.0, 1.0], 'topic': '/thruster/thruster_frv'}, {'id': 7, 'name': 'rear_left_vertical', 'position_m': [-0.2, -0.2, 0.0], 'direction': [0.0, 0.0, 1.0], 'topic': '/thruster/thruster_rlv'}, {'id': 8, 'name': 'rear_right_vertical', 'position_m': [0.2, -0.2, 0.0], 'direction': [0.0, 0.0, 1.0], 'topic': '/thruster/thruster_rrv'}]
correct_alloc_matrix = np.array([
    [0.70710678, -0.70710678, -0.70710678, 0.70710678, 0.0, 0.0, 0.0, 0.0],
    [0.70710678, 0.70710678, 0.70710678, 0.70710678, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    [0.0, 0.0, -0.0, -0.0, 0.2, 0.2, -0.2, -0.2],
    [0.0, -0.0, 0.0, 0.0, 0.2, -0.2, 0.2, -0.2],
    [-0.28284271, 0.28284271, -0.28284271, 0.28284271, 0.0, 0.0, 0.0, 0.0],
])


def test_compute_thruster_allocation_matrix():
    alloc_matrix = compute_thruster_allocation_matrix(thrusters)
    assert alloc_matrix.shape == (6, 8)
    np.testing.assert_allclose(alloc_matrix, correct_alloc_matrix, atol=1e-6)

if __name__ == "__main__":
    pytest.main()