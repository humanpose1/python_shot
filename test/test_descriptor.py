import pytest
import numpy as np
import handcrafted_descriptor as hd

def test_descriptor():
    pcd = np.random.randn(50000, 3).astype(float)
    normals = np.random.randn(50000, 3).astype(float)
    normals = normals / np.linalg.norm(normals, axis=1).reshape(50000, 1)
    descr = hd.compute_shot(pcd, normals, pcd[::300], normals[::300], 1)


