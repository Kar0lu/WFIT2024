import json
import numpy as np
from ahrs.common.orientation import q_prod, q_conj, acc2q, am2q, q2R, q_rot
import matplotlib.pyplot as plt

#otwarcie zapisanej w JSON-ie trasy
with open('acc.json', 'r') as file:
    acc_data = json.load(file)
with open('gyr.json', 'r') as file:
    gyr_data = json.load(file)
#ahrs = imufusion.Ahrs() #przydatne rzeczy
acc_array = np.array([[d["x"], d["y"], d["z"]] for d in acc_data])
gyr_array = np.array([[d["qx"], d["qy"], d["qz"], d["qw"]] for d in gyr_data])

if not  len(acc_array) == len(gyr_array):
    if len(acc_array) > len(gyr_array):
        dl = len(acc_array) - len(gyr_array)
        acc_array = acc_array[0:len(acc_array) - dl]
    else:
        dl = len(gyr_array) - len(acc_array)
        gyr_array = gyr_array[0:len(gyr_array) - dl]
dt = (acc_data[-1]["time"] / len(acc_data))
#print(dt)
acc = []
for i in range(len(acc_array)):
    acc.append(q_rot(q_conj(gyr_array[i]), acc_array[i]))
acc = np.array(acc)
vel = []
for i in range(len(acc)):
    if (i == 0):
        vel.append([acc[i][0]*dt, acc[i][1]*dt, acc[i][2]*dt])
    else:
        vel.append([acc[i-1][0] + acc[i][0]*dt, acc[i-1][1] + acc[i][1]*dt, acc[i-1][2] + acc[i][2]*dt])
vel = np.array(vel)
pos = []
for i in range(len(vel)):
    if (i == 0):
        pos.append([acc[i][0]*dt*dt*0.5, acc[i][1]*dt*dt*0.5, acc[i][2]*dt*dt*0.5])
    else:
        pos.append([pos[i-1][0] + vel[i-1][0]*dt + acc[i][0]*dt*dt*0.5, pos[i-1][1] + vel[i][1]*dt + acc[i][1]*dt*dt*0.5, pos[i-1][2] + vel[i][2]*dt + acc[i][2]*dt*dt*0.5])
pos = np.array(pos)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label='Sensor Trajectory')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.legend()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)
plt.show()