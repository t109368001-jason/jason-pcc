#!/bin/bash
set -ex

APP_PARALLEL=${APP_PARALLEL:=false}

./bin/JPCCAppDatasetParser --app.parallel ${APP_PARALLEL} \
  --configs ./cfg/app/DatasetParser/ATS-20211214-Mid-100-to-ply.cfg

./bin/JPCCAppDatasetParser --app.parallel ${APP_PARALLEL} \
  --configs ./cfg/app/DatasetParser/longdress-to-longdress-xyz.cfg

./bin/JPCCAppDatasetParser --app.parallel ${APP_PARALLEL} \
  --configs ./cfg/app/DatasetParser/loot-to-loot-xyz.cfg

./bin/JPCCAppDatasetParser --app.parallel ${APP_PARALLEL} \
  --configs ./cfg/app/DatasetParser/redandblack-to-redandblack-xyz.cfg

./bin/JPCCAppDatasetParser --app.parallel ${APP_PARALLEL} \
  --configs ./cfg/app/DatasetParser/soldier-to-soldier-xyz.cfg
