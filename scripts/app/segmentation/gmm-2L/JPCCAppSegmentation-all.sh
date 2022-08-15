#!/bin/bash
set -ex

bash ./scripts/app/segmentation/gmm-2L/JPCCAppSegmentation.sh &
bash ./scripts/app/segmentation/gmm-2L/JPCCAppSegmentation[A-0.00328-N-1400].sh &
bash ./scripts/app/segmentation/gmm-2L/JPCCAppSegmentation[STGT-5.984134206].sh &
