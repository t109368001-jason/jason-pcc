#!/bin/bash
set -ex

bash JPCCAppAnalyzer[CR-R-100000][ROR-R-500][ROR-N-1].sh.sh &
bash JPCCAppAnalyzer[CR-R-100000][ROR-R-500][ROR-N-5].sh &
bash JPCCAppAnalyzer[CR-R-100000][CR-Z-5000][ROR-R-500][ROR-N-1].sh.sh &
bash JPCCAppAnalyzer[CR-R-100000][CR-Z-5000][ROR-R-500][ROR-N-5].sh.sh &
