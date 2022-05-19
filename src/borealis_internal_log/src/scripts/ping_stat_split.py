from audioop import avgpp
import subprocess
import re
import time
import platform
import matplotlib.pyplot as plt
import numpy as np



out = """Reply from 127.0.1.1, 64 bytes in 0.01ms
Reply from 127.0.1.1, 64 bytes in 0.03ms

Round Trip Times min/avg/max is 0.01/0.02/0.03 ms"""

out = out.split('/') # ms
print(out)
avg = out[3]

print(avg)
