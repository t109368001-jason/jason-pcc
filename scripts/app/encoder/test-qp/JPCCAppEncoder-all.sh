#!/bin/bash
set -ex

bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg][tmc3-40].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg][tmc3-50].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg][tmc3-60].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg][tmc3-70].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg][tmc3-80].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split][resolution-100].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split][resolution-200].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split][resolution-300].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split][resolution-400].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[seg-split][resolution-500].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-40].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-50].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-60].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-70].sh &
bash ./scripts/app/encoder/test-qp/JPCCAppEncoder[tmc3-80].sh &
