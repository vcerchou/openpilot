#!/usr/bin/env python3
import json
import sys

from common.params import Params

if __name__ == "__main__":
    params = Params()

    # set from google maps url
    if len(sys.argv) > 1:
        coords = sys.argv[1].split("/@")[-1].split("/")[0].split(",")
        dest = {
            "latitude": float(coords[0]),
            "longitude": float(coords[1])
        }
        params.put("NavDestination", json.dumps(dest))
        params.remove("NavDestinationWaypoints")
    else:
        print("Setting to My Home")
        dest = {
            "latitude": 32.63177381227431,
            "longitude": 110.7961083984375,
        }
        params.put("NavDestination", json.dumps(dest))

        choice = input("Enter 1 to set default waypoints or 2 to enter custom waypoints: ")

        if choice == "1":
            waypoints = [
                (110.7961083984375, 32.63177381227431),
            ]
        elif choice == "2":
            custom_waypoints = input("Enter custom waypoints (latitude,longitude): ")
            waypoints = [tuple(map(float, waypoint.split(","))) for waypoint in custom_waypoints.split()]
        else:
            print("Invalid choice. Setting default waypoints.")
            waypoints = [
                (110.7961083984375, 32.63177381227431),
            ]

        params.put("NavDestinationWaypoints", json.dumps(waypoints))

        print(dest)
        print(waypoints)