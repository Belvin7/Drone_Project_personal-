import matplotlib.pyplot as plt
import numpy as np

timestamps1 = []
values1 = []
losses1 = []
timestamps2 = []
values2 = []
losses2 = []



with open('copter_2_rmse_50.txt','r') as f:
    for line in f:
        timestamp, rms, loss = line.strip().split(',')
        timestamps1.append(float(timestamp))
        values1.append(float(rms))
        if loss=='False':
            losses1.append(0.4)
        else:
            losses1.append(2.5)

with open('copter_3_rmse_50.txt','r') as f:
    for line in f:
        timestamp, rms, loss = line.strip().split(',')
        timestamps2.append(float(timestamp))
        values2.append(float(rms))
        if loss == 'False':
            losses2.append(0.4)
        else:
            losses2.append(3)


timestamps1 = timestamps1[:len(timestamps1)//3]
timestamps2 = timestamps2[:len(timestamps2)//3]
values1 = values1[:len(values1)//3]
values2 = values2[:len(values2)//3]
losses1 = losses1[:len(losses1)//3]
losses2 = losses2[:len(losses2)//3]

plt.figure()

plt.subplot(2, 2, 1)
plt.plot(timestamps1,values1)
#plt.plot(timestamps1,losses1, ':' )
plt.xlabel('Time in s')
plt.ylabel('RMSE')
plt.title('RMSE from Copter 2 with 50% package loss')
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(timestamps2,values2)
#plt.plot(timestamps2, losses2, linestyle=":"  )
plt.xlabel('Time in s')
plt.ylabel('RMSE')
plt.title('RMSE from Copter 3 with 50% package loss')
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(timestamps1,values1)
plt.plot(timestamps1,losses1, ':' )
plt.xlabel('Time in s')
plt.ylabel('RMSE')
plt.title('RMSE from Copter 2 with 50% package loss as dotted line')
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(timestamps2,values2)
plt.plot(timestamps2, losses2, linestyle=":"  )
plt.xlabel('Time in s')
plt.ylabel('RMSE')
plt.title('RMSE from Copter 3 with 50% package loss as dotted line')
plt.legend()

plt.show()