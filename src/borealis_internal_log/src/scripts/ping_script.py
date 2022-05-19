from pythonping import ping

out = ping('192.168.1.62', verbose=False, count=1, size=36, timeout=8)
print(out)
