#!/bin/bash
set -ex

bash ./scripts/app/JPCCAppAnalyzer.sh &
bash ./scripts/app/JPCCAppAnalyzer[ROR-N-5].sh &
bash ./scripts/app/JPCCAppAnalyzer[CR-R-100000][CR-Z-5000].sh &
bash ./scripts/app/JPCCAppAnalyzer[CR-R-100000][CR-Z-5000][ROR-N-5].sh &
