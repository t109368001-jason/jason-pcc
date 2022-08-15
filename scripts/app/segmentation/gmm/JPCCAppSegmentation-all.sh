#!/bin/bash
set -ex

bash ./scripts/app/segmentation/gmm/JPCCAppSegmentation.sh &
bash ./scripts/app/segmentation/gmm/JPCCAppSegmentation[A-0.00328-N-1400].sh &
bash ./scripts/app/segmentation/gmm/JPCCAppSegmentation[STGT-5.984134206].sh &
bash ./scripts/app/segmentation/gmm/JPCCAppSegmentation[A-0.000255-N-18090].sh &
