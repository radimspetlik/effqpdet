#!/usr/bin/env bash

# if script is not run with an argument
if [ "$#" -ne 1 ] || ! [[ "$1" == "release" || "$1" == "debug" ]]; then
  echo "Usage: $0 release|debug"
  exit 1
fi

apptainer exec \
  --cleanenv \
  --bind /mnt/data/vrg/spetlrad/data:/mnt/data/vrg/spetlrad/data \
  --bind /etc/localtime:/etc/localtime \
  /mnt/data/vrg/spetlrad/data/singularity/ptbd_pop.sif \
  bash -c "cmake -Wno-dev -DCMAKE_BUILD_TYPE=${1^} . && make clean && make -j 24"