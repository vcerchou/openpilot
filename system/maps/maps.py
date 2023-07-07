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

        # 向set_destination.py写入waypoints
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

        # 运行set_destination.py
        subprocess.run(['python3', 'openpilot/selfdrive/navd/set_destination.py'], check=True)

        return 'Waypoints设置成功！'

    return render_template('index.html')

if __name__ == '__main__':
    app.run(port=6000)
