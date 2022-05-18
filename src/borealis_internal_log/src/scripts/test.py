import subprocess
import re
import time
import platform
import matplotlib.pyplot as plt
import numpy as np

def read_data_from_cmd():
	if platform.system() == 'Linux':
		p = subprocess.Popen("iwconfig", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	elif platform.system() == 'Windows':
		p = subprocess.Popen("netsh wlan show interfaces", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	else:
		raise Exception('reached else of if statement')
	out = p.stdout.read().decode()
	print(out)
	if platform.system() == 'Linux':
		m = re.findall('(wlan[0-9]+).*?Signal level=(-[0-9]+) dBm', out, re.DOTALL)
		print(m)
	elif platform.system() == 'Windows':
		m = re.findall('Name.*?:.*?([A-z0-9 ]*).*?Signal.*?:.*?([0-9]*)%', out, re.DOTALL)
	else:
		raise Exception('reached else of if statement')

	p.communicate()

	return m

print(read_data_from_cmd())
