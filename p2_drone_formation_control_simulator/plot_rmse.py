import matplotlib.pyplot as plt

timestamps = []
values = []

with open('copter_3_rmse.txt','r') as f:
    for line in f:
        timestamp, value = line.strip().split(',')
        timestamps.append(float(timestamp))
        values.append(float(value))

plt.figure()
plt.plot(timestamps, values, label='RMSE from Copter 3 with 50% package loss')
plt.xlabel('Time in s')
plt.ylabel('RMSE')
plt.title('RMSE from Copter 3 with 50% package loss')
plt.legend()
plt.show()