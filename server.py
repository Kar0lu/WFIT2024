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

# Method to receive sensor data on server
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

# Method to save sensor data to json file
@app.route('/get', methods=["GET"])
def get():
    acc_data[0]["time"] = 0
    gyr_data[0]["time"] = 0
    if request.method == "GET":
        with open('resource/acc.json', 'w') as file:
            json.dump(acc_data, file, indent=4)
        with open('resource/gyr.json', 'w') as file:
            json.dump(gyr_data, file, indent=4)
        acc_data.clear()
        gyr_data.clear()
        return jsonify({"status": "success"}), 200

if __name__ == '__main__':
    app.run(debug=True, port=5000, host="0.0.0.0")
