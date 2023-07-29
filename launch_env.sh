#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="8.2"
fi

if [ -z "$PASSIVE" ]; then
  export PASSIVE="1"
fi

export STAGING_ROOT="/data/safe_staging"
export API_HOST="https://mypilot.com.cn/myPilot"
export ATHENA_HOST="wss://mypilot.com.cn"
export MAPBOX_TOKEN="pk.eyJ1IjoiZ29kenFoIiwiYSI6ImNrdmJzcHdhcGFxYnIycG56MDhqaTl6eWMifQ.-HU94mfF8jFLAMA08m8L2A"