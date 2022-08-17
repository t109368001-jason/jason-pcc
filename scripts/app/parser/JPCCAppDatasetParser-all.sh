#!/bin/bash
set -ex

bash ./scripts/app/parser/JPCCAppDatasetParser.sh &
bash ./scripts/app/parser/JPCCAppDatasetParser-XYZ.sh &
