#!/bin/bash
set -ex

bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-40-40].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-50-50].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-60-60].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-70-70].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-80-80].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-100][tmc3-50-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-200][tmc3-50-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-300][tmc3-50-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-400][tmc3-50-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-500][tmc3-50-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-40-40].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-50-50].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-60-60].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-70-70].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-80-80].sh &
