#!/bin/bash
set -ex

bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-53-53].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-60-60].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-65-65].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-68-68].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-100][tmc3-71-71].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-100][tmc3-53-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-200][tmc3-60-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-300][tmc3-65-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-400][tmc3-68-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split-500][tmc3-71-0].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-53-53].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-60-60].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-65-65].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-68-68].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-71-71].sh &
