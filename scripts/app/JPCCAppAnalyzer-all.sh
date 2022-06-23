#!/bin/bash
set -ex

bash ./scripts/app/JPCCAppAnalyzer[CR-R-100000][ROR-R-500][ROR-N-1].sh &
bash ./scripts/app/JPCCAppAnalyzer[CR-R-100000][ROR-R-500][ROR-N-5].sh &
bash ./scripts/app/JPCCAppAnalyzer[CR-R-100000][CR-Z-5000][ROR-R-500][ROR-N-1].sh &
bash ./scripts/app/JPCCAppAnalyzer[CR-R-100000][CR-Z-5000][ROR-R-500][ROR-N-5].sh &
