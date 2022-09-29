#!/bin/bash
set -ex

bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[seg][tmc3-80].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[seg][tmc3-85].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[seg][tmc3-90].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[seg][tmc3-95].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[seg][tmc3-100].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[tmc3-80].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[tmc3-85].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[tmc3-90].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[tmc3-95].sh &
bash ./scripts/app/encoder/gmm-2L/JPCCAppEncoder[tmc3-100].sh &
