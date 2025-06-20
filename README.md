# Efficient Real-time Quadcopter Propeller Detection and Attribute Estimation

This repository contains the official C++ implementation of the paper **"Efficient Real-time Quadcopter Propeller Detection and Attribute Estimation with High-resolution Event Camera"** accepted to Scandinavian Conference on Image Analysis, to appear in proceedings of the conference. The code detects rotating propellers in high-resolution event camera streams and estimates their angular speed as well as the relative pitch and roll of the propeller plane.

## Overview

The detector processes asynchronous events from a Metavision-compatible camera. Events are binned into blocks of size `2^k` and modeled as Poisson processes. Real-time bursts in the inter-event statistics reveal rotating propellers. Once a propeller is detected, its angular speed is computed from inter-burst intervals, and an ellipse fit over accumulated events yields pitch and roll estimates.


## Repository Layout

- `main.cpp` – application entry point that loads a YAML configuration and runs the detector.
- `QuadcopterPropellerDetector_lib/` – library implementing the Poisson based burst detection and ellipse fitting algorithms.
- `config.yaml` / `config_win_detector.yaml` – example configuration files used to specify input recordings, output paths and processing parameters.
- `compile_build.sh` – script used on our environment for building inside an Apptainer container.
- `Google_tests/` – minimal Google Test setup.

## Building

The project uses CMake (≥3.26) and requires a C++17 compiler. Dependencies include
[Metavision SDK](https://www.prophesee.ai/metavision/), Boost, yaml-cpp, HDF5 and
Protobuf. 

Before any attempt to build, one has to correct all absolute paths in all `CMakeLists.txt` files.

Linking local HDF5 libraries and MetavisionSDK is a pain. I am willing to help any poor soul wishing to do so.

On Linux the following commands build the executable `qpdet`:

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

A helper script `compile_build.sh` is also provided for building inside an Apptainer/Singularity container.

## Running

The detector is invoked with a YAML configuration file:

```bash
./qpdet config.yaml
```

The configuration specifies the path to a RAW event recording (`recording_filepath`),
and controls window size, burst-detection thresholds, and whether results are visualized or stored for analysis.

## Dataset

The repository is accompanied by a dataset of quadcopter propeller recordings covering rotational speeds from 1100–8200 RPM and tilt angles of 0°, 10°, and 90°. 

The dataset is stored [here](https://ptak.felk.cvut.cz/personal/spetlrad/propeller_dataset.tar.gz).

Our detector achieves near-perfect detection accuracy with an average real‑time factor of 0.94 on a single CPU core.

## Paper

```
@inproceedings{author2024efficient,
  title     = {Efficient Real-time Quadcopter Propeller Detection and Attribute Estimation with High-resolution Event Camera},
  author    = {Spetlik, Radim and Uhrova, Tereza and Matas, Jiri},
  booktitle = "Image Analysis",
  year      = {2025}
}
```

Please cite the paper if you use this code or dataset.

