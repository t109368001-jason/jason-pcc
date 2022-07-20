#!/bin/bash
set -ex

bash ./scripts/app/JPCCAppSegmentation.sh &
bash ./scripts/app/JPCCAppSegmentation[N-2048].sh &
