#!/bin/bash
set -ex

bash ./scripts/app/JPCCAppSegmentation.sh &
bash ./scripts/app/JPCCAppSegmentation[A-0.00164-N-2800].sh &
bash ./scripts/app/JPCCAppSegmentation[STGT-5.984134206].sh &
bash ./scripts/app/JPCCAppSegmentation[A-0.000328-N-14000].sh &
bash ./scripts/app/JPCCAppSegmentation[A-0.000255-N-18090].sh &
