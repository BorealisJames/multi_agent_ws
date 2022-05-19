import psutil

output_cpu = psutil.cpu_percent(0.5) # avg cpu usage for 0.5 seconds
output_ram = psutil.virtual_memory()[2] # ram usage 

print(output_cpu)