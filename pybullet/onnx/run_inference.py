#!/usr/bin/env python3
"""
Run one ONNX Runtime inference for the exported deterministic actor.

This script reads input metadata from the model and creates correctly shaped
dummy inputs automatically, so it is useful for quick validation on CPU-only
targets such as Raspberry Pi.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import numpy as np
import onnxruntime as ort


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="Path to the exported .onnx model")
    parser.add_argument("--batch", type=int, default=1, help="Batch size for dummy inference")
    parser.add_argument(
        "--random",
        action="store_true",
        help="Use random float inputs for non-mask tensors instead of zeros",
    )
    parser.add_argument(
        "--intra-threads",
        type=int,
        default=0,
        help="ONNX Runtime intra-op thread count (0 = runtime default)",
    )
    parser.add_argument(
        "--inter-threads",
        type=int,
        default=0,
        help="ONNX Runtime inter-op thread count (0 = runtime default)",
    )
    parser.add_argument(
        "--parallel",
        action="store_true",
        help="Use ORT_PARALLEL execution mode instead of ORT_SEQUENTIAL",
    )
    parser.add_argument(
        "--disable-spinning",
        action="store_true",
        help="Disable ONNX Runtime worker-thread spinning",
    )
    return parser.parse_args()


def resolve_dim(dim: int | str | None, batch: int) -> int:
    if isinstance(dim, int):
        return batch if dim <= 0 else dim
    return batch


def resolve_shape(shape: Iterable[int | str | None], batch: int) -> list[int]:
    resolved = [resolve_dim(dim, batch) for dim in shape]
    if resolved:
        resolved[0] = batch
    return resolved


def dtype_from_ort(type_str: str):
    mapping = {
        "tensor(float)": np.float32,
        "tensor(double)": np.float64,
        "tensor(int64)": np.int64,
        "tensor(int32)": np.int32,
        "tensor(bool)": np.bool_,
    }
    if type_str not in mapping:
        raise ValueError(f"Unsupported ONNX input type: {type_str}")
    return mapping[type_str]


def make_input(name: str, shape: list[int], dtype, use_random: bool) -> np.ndarray:
    if name == "move_mask":
        return np.ones(shape, dtype=np.float32)
    if np.issubdtype(dtype, np.floating):
        if use_random:
            rng = np.random.default_rng(42)
            return rng.uniform(-1.0, 1.0, size=shape).astype(dtype)
        return np.zeros(shape, dtype=dtype)
    if np.issubdtype(dtype, np.integer):
        return np.zeros(shape, dtype=dtype)
    if dtype == np.bool_:
        return np.zeros(shape, dtype=dtype)
    raise ValueError(f"Unsupported dtype for input {name}: {dtype}")


def main() -> None:
    args = parse_args()
    model_path = Path(args.model).resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = args.intra_threads
    session_options.inter_op_num_threads = args.inter_threads
    session_options.execution_mode = (
        ort.ExecutionMode.ORT_PARALLEL if args.parallel else ort.ExecutionMode.ORT_SEQUENTIAL
    )
    if args.disable_spinning:
        session_options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        session_options.add_session_config_entry("session.inter_op.allow_spinning", "0")

    session = ort.InferenceSession(
        model_path.as_posix(),
        sess_options=session_options,
        providers=["CPUExecutionProvider"],
    )

    print(f"model={model_path}")
    print(f"providers={session.get_providers()}")

    feeds = {}
    print("inputs:")
    for meta in session.get_inputs():
        dtype = dtype_from_ort(meta.type)
        shape = resolve_shape(meta.shape, args.batch)
        arr = make_input(meta.name, shape, dtype, args.random)
        feeds[meta.name] = arr
        print(f"  {meta.name}: shape={shape} dtype={arr.dtype}")

    print("outputs:")
    for meta in session.get_outputs():
        print(f"  {meta.name}: shape={meta.shape} type={meta.type}")

    outputs = session.run(None, feeds)
    for idx, output in enumerate(outputs):
        print(f"output[{idx}] shape={list(output.shape)} dtype={output.dtype}")
        print(output)
        if output.size >= 3:
            flat = output.reshape(-1, output.shape[-1])
            first = flat[0]
            if first.shape[0] >= 3:
                print(
                    f"first action -> vx={float(first[0]):.6f} "
                    f"vy={float(first[1]):.6f} yaw_rate={float(first[2]):.6f}"
                )


if __name__ == "__main__":
    main()
