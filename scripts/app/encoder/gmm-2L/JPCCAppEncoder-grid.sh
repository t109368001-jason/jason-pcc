#!/bin/bash
set -e

K_LIST=(
  2
  #  3
  #  4
  #  5
)
ALPHA_N_LIST=(
  "0.00395,350"
  #  "0.01310,350"
  #  "0.00198,700"
  #  "0.00656,700"
  #  "0.00099,1400"
  #  "0.00328,1400"
)
NULL_STATIC_THRESHOLD_LIST=(
  -8.976201309
  -7.978845608
  -5.984134206)
STATIC_THRESHOLD_THRESHOLD_LIST=(
  0.8802
  0.6049
  0.1350
)

OUTPUT_FOLDER_PREFIX=../../dataset/

if [ "${#K_LIST[@]}" -gt 1 ]; then
  echo "cannot run more then 1 k in K_LIST"
  exit 1
fi

if [ "${#ALPHA_N_LIST[@]}" -gt 1 ]; then
  echo "cannot run more then 1 alpha and n in ALPHA_N_LIST"
  exit 1
fi

for K in "${K_LIST[@]}"; do
  for ALPHA_N in "${ALPHA_N_LIST[@]}"; do
    IFS="," read -r -a TMP_ARRAY <<<"${ALPHA_N}"
    ALPHA=${TMP_ARRAY[0]}
    N=${TMP_ARRAY[1]}
    for NULL_STATIC_THRESHOLD in "${NULL_STATIC_THRESHOLD_LIST[@]}"; do
      for STATIC_THRESHOLD in "${STATIC_THRESHOLD_THRESHOLD_LIST[@]}"; do
        OUTPUT_FOLDER=converted-grid/ZX-XS-20220707-ctc-gmm-segmentation[K-${K}][A-${ALPHA}-N-${N}][STGT-${NULL_STATIC_THRESHOLD}][DTLE-${STATIC_THRESHOLD}]/

        mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

        ./bin/JPCCAppEncoder \
          --app.parallel false \
          --outputDataset.folder "${OUTPUT_FOLDER}" \
          --jpccGMMSegmentationParameter.k "${K}" \
          --jpccGMMSegmentationParameter.k "${K}" \
          --jpccGMMSegmentationParameter.alpha "${ALPHA}" \
          --jpccGMMSegmentationParameter.nTrain "${N}" \
          --jpccGMMSegmentationParameter.alpha 0.000255 \
          --jpccGMMSegmentationParameter.nTrain 18090 \
          --jpccGMMSegmentationParameter.nullStaticThreshold 0 \
          --jpccGMMSegmentationParameter.nullStaticThreshold "${NULL_STATIC_THRESHOLD}" \
          --jpccGMMSegmentationParameter.staticThreshold "${STATIC_THRESHOLD}" \
          --jpccGMMSegmentationParameter.staticThreshold "${STATIC_THRESHOLD}" \
          --configs cfg/app/Segmentation/ctc-raw.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log" &
      done
    done
  done
done
