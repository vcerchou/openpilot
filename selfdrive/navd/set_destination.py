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

    waypoints = [
      # (110.7961083984375, 32.63177381227431),
      (110.69993784822455,32.64445861763176),
    ]
    params.put("NavDestinationWaypoints", json.dumps(waypoints))

    print(dest)
    print(waypoints)
