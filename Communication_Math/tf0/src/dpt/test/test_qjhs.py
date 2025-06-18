import math
import random
import numpy as np
from dpt.qjhs import gd, mnth, object

def test_gd_simple():
    y = gd(6, 9, math.pi/4)
    assert math.isclose(y, 1.761813534551234, rel_tol=1e-9)

def test_mnth_converges():
    random.seed(0)
    np.random.seed(0)
    v = mnth(object, 9, 100, 0.95, 1e-3, 10, 6, 1.53, math.pi/4, 7, 10)
    assert math.isclose(v, 8.780273860709002, rel_tol=1e-9)
