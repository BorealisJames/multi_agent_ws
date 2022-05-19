import subprocess
import re
import time
import platform
import matplotlib.pyplot as plt
import numpy as np



out = """wlo1      IEEE 802.11  ESSID:"SUTD_LAB"  
          Mode:Managed  Frequency:5.26 GHz  Access Point: B8:3A:5A:BF:98:32   
          Bit Rate=866.7 Mb/s   Tx-Power=22 dBm   
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Encryption key:off
          Power Management:on
          Link Quality=59/70  Signal level=-51 dBm  
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:13  Invalid misc:0   Missed beacon:0 """


out = out.split() # Mb/s
bit_rate = out[11] # Mb/s
Tx_power = out[13] # dbm
signal_level = out[29] # dBm
bit_rate = bit_rate.split('=')[1]
Tx_power = Tx_power.split('=')[1]
signal_level = signal_level.split('=')[1]

print(bit_rate)
print(Tx_power)
print(signal_level)
