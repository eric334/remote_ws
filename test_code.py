#!/usr/bin/env python3

import numpy as np

coords = np.array([1,2],dtype=np.uint16)

print(bytes(coords))

print(len(bytes(coords)))