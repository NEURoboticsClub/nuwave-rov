import numpy as np

def compute_thruster_allocation_matrix(thrusters):

    AllocMatrix = np.zeros((6, len(thrusters)))
    
    for indx, thruster in enumerate(thrusters):
        pos_m = np.array(thruster['position_m'], dtype=float)
        dir = np.array(thruster['direction'], dtype=float)

        norm = np.linalg.norm(dir)
        if norm == 0.0:
            raise ValueError(f"Thruster {indx} has a zero-length direction vector in config")
        dir = dir / norm # normalize the direction vector

        # Linear Force Contribution
        AllocMatrix[0:3, indx] = dir
        AllocMatrix[3:6, indx] = np.cross(pos_m, dir) 
    return AllocMatrix
    