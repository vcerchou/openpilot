#!/usr/bin/env python3
from flask import Flask, render_template, request
import subprocess
import json

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        latitude = float(request.form['latitude'])
        longitude = float(request.form['longitude'])
        waypoints = [(longitude, latitude)]

        dest = {
            "latitude": latitude,
            "longitude": longitude
        }
        params = {
            "NavDestination": json.dumps(dest),
            "NavDestinationWaypoints": json.dumps(waypoints)
        }
        with open('params.json', 'w') as f:
            json.dump(params, f)

        subprocess.run(['python3', 'openpilot/selfdrive/navd/set_destination.py'], check=True)

        return 'Waypoints done'

    return render_template('index.html')

def main():
  app.run(host="0.0.0.0", port=6000)

if __name__ == '__main__':
  main()
