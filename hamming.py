#coding:utf-8
import numpy as np
from pylab import *

N = 17

hammingWindow = np.hamming(N)

plot(hammingWindow)
axis((0, N-1, 0, 1))

show()