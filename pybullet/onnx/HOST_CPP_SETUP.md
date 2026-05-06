# Host-side C++ ONNX Runtime setup

This directory already contains:

- The model: `onnx/marl_agent.onnx`
- A Python inference path: `onnx/run_inference.py`
- A C++ inference driver: `onnx/run_random_inference.cpp`
- A C++ benchmark driver: `onnx/benchmark_inference.cpp`
- A host build script: `onnx/build_host.sh`

What is missing on this Raspberry Pi 5 right now:

- `g++`
- `cmake`
- ONNX Runtime C/C++ headers such as `onnxruntime_cxx_api.h`

The current Python virtualenv already includes the runtime shared library:

- `onnx/venv/lib/python3.11/site-packages/onnxruntime/capi/libonnxruntime.so.1.25.1`

That is enough to run Python inference, but not enough to compile C++ because the wheel does not ship the development headers.

## Install requirements on the host

Install build tools:

```bash
sudo apt update
sudo apt install -y build-essential cmake
```

Install ONNX Runtime headers on the host using an aarch64 ONNX Runtime SDK package.

You need a directory that looks like:

```text
<onnxruntime-root>/
  include/onnxruntime_cxx_api.h
  include/onnxruntime_c_api.h
  lib/libonnxruntime.so
```

Two practical options:

1. Install a full ONNX Runtime aarch64 SDK on the host and point `ONNXRUNTIME_ROOT` to it.
2. Reuse the Python-wheel runtime library already in `onnx/venv/.../capi/`, and provide only the headers via `ONNXRUNTIME_INCLUDE_DIR`.

## Build

If you have a full SDK:

```bash
ONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-<version> ./onnx/build_host.sh
```

If you want to reuse the library from the Python wheel:

```bash
ONNXRUNTIME_INCLUDE_DIR=/path/to/onnxruntime/include ./onnx/build_host.sh
```

The script defaults `ONNXRUNTIME_LIB_DIR` to:

```text
onnx/venv/lib/python3.11/site-packages/onnxruntime/capi
```

## Run

```bash
./onnx/build-host/run_random_inference ./onnx/marl_agent.onnx
```

Benchmark multiple session configurations:

```bash
./onnx/build-host/benchmark_inference ./onnx/marl_agent.onnx \
  --iterations 400 \
  --warmup 80 \
  --intra-threads 1,2,4 \
  --inter-threads 1,2 \
  --modes sequential,parallel \
  --spinning on,off
```

## What to install in Docker later

If inference will run inside the container, the container needs the same runtime pieces:

- compiler and CMake only if you build inside the image
- ONNX Runtime headers only if you compile inside the image
- ONNX Runtime shared library at runtime

For deployment images, the clean pattern is:

1. Build the C++ binary in a builder stage or on the host.
2. Copy the binary, model, and `libonnxruntime.so` into the runtime image.

If you instead compile inside the container, install the ONNX Runtime SDK in the Docker image as well.
