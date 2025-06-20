#!/bin/bash

# Function to determine the appropriate program based on dataset name
determine_program() {
  local dataset_name=$1
  local program=""

  # Check if dataset_name is in DATASET_NAMES_GT
  for dataset in "${DATASET_NAMES_GT[@]}"; do
    if [[ "$dataset" == "$dataset_name" ]]; then
      program="./peak_detection_no_triggers ./correlation_response_measurement"
      break
    fi
  done

  # If program is still not set, check DATASET_NAMES_NOGT
  if [[ -z "$program" ]]; then
    for dataset in "${DATASET_NAMES_NOGT[@]}"; do
      if [[ "$dataset" == "$dataset_name" ]]; then
        program="./peak_detection_no_triggers"
        break
      fi
    done
  fi

  echo "$program"
}