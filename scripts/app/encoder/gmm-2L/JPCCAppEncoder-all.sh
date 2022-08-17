#!/bin/bash
set -ex

bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder.sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[A-0.00328-N-1400].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[STGT-5.984134206].sh &
