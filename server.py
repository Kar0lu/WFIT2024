from flask import Flask, request, jsonify, send_file
import json
import numpy as np
import matplotlib.pyplot as plt

import ahrs.filters
from ahrs.common.orientation import q_prod, q_conj, acc2q, am2q, q2R, q_rot

from mpl_toolkits.mplot3d import Axes3D

app = Flask(__name__)

acc_data = []
gyr_data = []

@app.route('/data', methods=['POST'])
def data():
    if request.method == "POST":
        data = request.get_json()
        for d in data['payload']:
            if d.get("name") == "accelerometer":
                if(len(acc_data)):
                    acc_data.append({"time": (d["time"] * 1e-9) - (acc_data[0]["time"]), "x": d["values"]["x"], "y": d["values"]["y"], "z": d["values"]["z"]})
                else:
                    acc_data.append({"time": d["time"] * 1e-9, "x": d["values"]["x"], "y": d["values"]["y"], "z": d["values"]["z"]})
            if d.get("name") == "orientation":
                if(len(gyr_data)):
                    gyr_data.append({"time": (d["time"] * 1e-9) - (gyr_data[0]["time"]), "qx": d["values"]["qx"], "qy": d["values"]["qy"], "qz": d["values"]["qz"], "qw": d["values"]["qw"]})
                else:
                    gyr_data.append({"time": d["time"] * 1e-9, "qx": d["values"]["qx"], "qy": d["values"]["qy"], "qz": d["values"]["qz"], "qw": d["values"]["qw"]})
        return jsonify({"status": "success"}), 200
    else:
        return jsonify({"status": "method not allowed"}), 405

@app.route('/get', methods=["GET"])
def get():
    acc_data[0]["time"] = 0
    gyr_data[0]["time"] = 0
    if request.method == "GET":
        with open('acc.json', 'w') as file:
            json.dump(acc_data, file, indent=4)
        with open('gyr.json', 'w') as file:
            json.dump(gyr_data, file, indent=4)
        acc_data.clear()
        gyr_data.clear()
        return jsonify({"status": "success"}), 200
'''
@app.route('/plot', methods=["GET"])
def plot():
    if request.method == "GET":
        # Read data from acc.json and gyr.json
        with open('acc.json', 'r') as file:
            acc_data = json.load(file)
        with open('gyr.json', 'r') as file:
            gyr_data = json.load(file)
        
        # acc_array = np.array([[d["time"], d["x"], d["y"], d["z"]] for d in acc_data])
        # gyr_array = np.array([[d["time"], d["x"], d["y"], d["z"]] for d in gyr_data])
        
        # plot_acceleration(acc_array)
        # plot_gyroscope(gyr_array)
        
        pos = calculate_position(acc_data, gyr_data)
        # plot_trajectory(pos)
        # return send_file('trajectory.png', mimetype='image/png')
        return "xd"

def calculate_position(acc_data, gyr_data):

    quat  = np.zeros((len(acc_data), 4), dtype=np.float64)
    dt = acc_data[-1]["time"] / len(acc_data)
    time = [t*dt for t in range(0, len(acc_data))]

    acc = []
    gyr = []

    # gyr = np.zeros(3, dtype=np.float64)

    # for t in range(0, len(acc_data)):
    #     acc = np.array([[acc_data[t]["x"], acc_data[t]["y"], acc_data[t]["z"]]])
    # acc = acc.mean(axis=0)
    
    mahony = ahrs.filters.Mahony(Kp=1, Ki=0,KpInit=1, frequency=1/dt)
    # q = np.array([1.0,0.0,0.0,0.0], dtype=np.float64)
    q = np.tile([1., 0., 0., 0.], (len(acc_data), 1)) 
    # print(q)
    # for i in range(0, len(acc_data)):
    #     q = mahony.updateIMU(q, gyr=gyr, acc=acc)
    
    for t in range(0, len(acc_data)):
        acc.append(np.array([acc_data[t]["x"], acc_data[t]["y"], acc_data[t]["z"]]))
        gyr.append(np.array([gyr_data[t]["x"], gyr_data[t]["y"], gyr_data[t]["z"]])*np.pi/180)

    for t in range(1, len(acc_data)):
        # print(q[t-1],gyr[t],acc[t])
        q[t]=mahony.updateIMU(q[t-1],gyr=gyr[t],acc=acc[t])

    acc = []
    for i in range(0, len(acc_data)):
        acc.append(q_rot(q_conj(q[i]), np.array([acc_data[i]['x'], acc_data[i]['y'], acc_data[i]['z']])))
    acc = np.array(acc)
    acc = acc - np.array([0,0,0.0000981])
    
    vel = np.zeros(acc.shape)
    for t in range(1,vel.shape[0]):
        vel[t,:] = vel[t-1,:] + acc[t,:]*dt
    
    pos = np.zeros(vel.shape)
    for t in range(1,pos.shape[0]):
        pos[t,:] = pos[t-1,:] + vel[t,:]*dt
    
    posPlot = pos
    quatPlot = quat

    print(pos)
    print(quat)

    extraTime = 20
    onesVector = np.ones(int(extraTime*(1/dt)))

    # Create 6 DOF animation
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='3d') # Axe3D object
    ax.plot(posPlot[:,0],posPlot[:,1],posPlot[:,2])
    # min_, max_ = np.min(np.min(posPlot,axis=0)), np.max(np.max(posPlot,axis=0))
    # ax.set_xlim(-0.01,0.01)
    # ax.set_ylim(-0.01,0.01)
    ax.set_zlim(-0.1,0.1)
    ax.set_title("trajectory")
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_zlabel("z position (m)")
    
    plt.show()


def plot_acceleration(acc_data):
    times = acc_data[:, 0] * 1e-9  # Convert time from nanoseconds to seconds
    plt.figure()
    plt.plot(times, acc_data[:, 1], label='X Acceleration')
    plt.plot(times, acc_data[:, 2], label='Y Acceleration')
    plt.plot(times, acc_data[:, 3], label='Z Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.title('Acceleration Over Time')
    plt.legend()
    plt.savefig('acceleration.png')

def plot_gyroscope(gyr_data):
    times = gyr_data[:, 0] * 1e-9  # Convert time from nanoseconds to seconds
    plt.figure()
    plt.plot(times, gyr_data[:, 1], label='X Angular Rate')
    plt.plot(times, gyr_data[:, 2], label='Y Angular Rate')
    plt.plot(times, gyr_data[:, 3], label='Z Angular Rate')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Rate (rad/s)')
    plt.title('Gyroscope Data Over Time')
    plt.legend()
    plt.savefig('gyroscope.png')

def plot_trajectory(position):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(position[:, 0], position[:, 1], position[:, 2], label='Sensor Trajectory')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.legend()
    
    plt.savefig('trajectory.png')
'''

if __name__ == '__main__':
    app.run(debug=True, port=5000, host="0.0.0.0")
