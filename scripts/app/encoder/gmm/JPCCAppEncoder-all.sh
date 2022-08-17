#!/bin/bash
set -ex

bash ./scripts/app/encoder/gmm/JPCCAppEncoder.sh &
bash ./scripts/app/encoder/gmm/JPCCAppEncoder[A-0.00328-N-1400].sh &
bash ./scripts/app/encoder/gmm/JPCCAppEncoder[STGT-5.984134206].sh &
bash ./scripts/app/encoder/gmm/JPCCAppEncoder[A-0.000255-N-18090].sh &
