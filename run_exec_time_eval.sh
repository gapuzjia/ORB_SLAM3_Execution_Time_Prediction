#!/bin/bash
set -u

DATE="$(date +"%Y-%m-%d_%H-%M-%S")"

DATASETS=(MH01 MH02 MH03 MH04 MH05 V101 V102 V103 V201 V202 V203)

dataset_dir() {
  case "$1" in
    MH01) echo "MH_01_easy" ;;
    MH02) echo "MH_02_easy" ;;
    MH03) echo "MH_03_medium" ;;
    MH04) echo "MH_04_difficult" ;;
    MH05) echo "MH_05_difficult" ;;
    V101) echo "V1_01_easy" ;;
    V102) echo "V1_02_medium" ;;
    V103) echo "V1_03_difficult" ;;
    V201) echo "V2_01_easy" ;;
    V202) echo "V2_02_medium" ;;
    V203) echo "V2_03_difficult" ;;
  esac
}

NUM_RUNS=10

for ds in "${DATASETS[@]}"; do
  folder=$(dataset_dir "$ds")

  for ((run=1; run<=NUM_RUNS; run++)); do

    echo "=== Running $ds (folder $folder), run $run ==="
    echo "COMMAND = ./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC_oasis.yaml ./Datasets/EuRoc/$folder ./Examples/Stereo-Inertial/EuRoC_TimeStamps/${ds}.txt dataset-${ds}_stereo_inertial"

    if ! ./Examples/Stereo-Inertial/stereo_inertial_euroc \
      ./Vocabulary/ORBvoc.txt \
      ./Examples/Stereo-Inertial/EuRoC_oasis.yaml \
      ./Datasets/EuRoc/$folder \
      ./Examples/Stereo-Inertial/EuRoC_TimeStamps/${ds}.txt \
      dataset-${ds}_stereo_inertial
    then
	echo "CRASHED on dataset $ds run $run (continuing)"
    fi
    
    OUT="${DATE}${ds}run${run}"
    mkdir -p "$OUT"
    
    RENAME_EXEC_TIME_EVAL="exec_time_eval_${ds}run${run}.txt"
    
    if [ -f exec_time_eval.txt ]; then
        cp exec_time_eval.txt "$RENAME_EXEC_TIME_EVAL"
	mv -f "$RENAME_EXEC_TIME_EVAL" "$OUT/"
    fi


    mv LocalMapTimeStats.txt TrackingTimeStats.txt LBA_Stats.txt ExecMean.txt \
       SessionInfo.txt 2>/dev/null "$OUT" || true

    mv cellManager.txt map_points.csv \
       f_dataset-${ds}_stereo_inertial.txt \
       kf_dataset-${ds}_stereo_inertial.txt \
       2>/dev/null "$OUT" || true

    echo "Saved → $OUT"
    echo

  done
done

echo "All done."

