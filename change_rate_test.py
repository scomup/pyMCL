#!/usr/bin/python
# coding: UTF-8

# Probabilistic Robotics (pp.206 Table 8.3)

import random
from math import *
import matplotlib.pyplot as plt
import numpy as np
import random

def decision(probability):
    return random.random() < probability

"""
If the short-term likelihood is better or equal to the long-term likelihood, 
no random sample is added. However, if the short-termlikelihood is worse than
the long-term one, random samples are added in proportion to the quotient of 
these values.


s: likelihood
w_fast: short-term likelihood
w_slow: long-term likelihood
prob: The probability of random sample
"""
alpha_slow = 0.001
alpha_fast = 0.1
s = np.random.normal(0.5, 0.01, 1000)
noise = np.array([decision(0.005)  for i in range(s.shape[0])])
s -= noise * np.abs(np.random.normal(0., 1, 1000))
s = (s>0)*s
w_slow = []
w_slow.append(0.5)
w_fast = []
w_fast.append(0.5)
[w_slow.append(w_slow[i-1] + alpha_slow * (s[i] - w_slow[i-1]))  for i in range(1,s.shape[0]) ]
[w_fast.append(w_fast[i-1] + alpha_fast * (s[i] - w_fast[i-1]))  for i in range(1,s.shape[0]) ]

prob = [max(0.0, 1.0 - w_fast[i]/(w_slow[i]+0.000000001))  for i in range(0,s.shape[0]) ]

plt.subplot(2, 1, 1)
plt.plot(s)
plt.plot(w_slow)
plt.plot(w_fast)
plt.subplot(2, 1, 2)
plt.bar(range(len(prob)),prob)
plt.show()
print np.std(s)
