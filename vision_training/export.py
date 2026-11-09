#!/usr/bin/env python3
"""Export a detection checkpoint with verified class mapping and model metadata."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import shutil

try:
    from .common import class_names, environment, new_output, read_yaml, sha256, write_json
except ImportError:
    from common import class_names, environment, new_output, read_yaml, sha256, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--data', required=True, help='class mapping must match checkpoint')
    parser.add_argument('--format', choices=['pt', 'onnx'], default='onnx')
    parser.add_argument('--output', required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--iou', type=float, default=0.45)
    args = parser.parse_args()
    from ultralytics import YOLO
    weights = Path(args.weights).resolve()
    model = YOLO(str(weights))
    names = class_names(model.names)
    if model.task != 'detect' or names != class_names(read_yaml(args.data)['names']):
        raise ValueError('Detection checkpoint and data.yaml class mapping must agree')
    if args.imgsz <= 0 or args.imgsz % 32:
        raise ValueError('imgsz must be a positive multiple of 32')
    output = new_output(args.output)
    # Export from a copy: do not write adjacent to the source training checkpoint.
    copied = output / 'model.pt'
    shutil.copy2(weights, copied)
    result = copied
    if args.format == 'onnx':
        result = Path(YOLO(str(copied)).export(
            format='onnx', imgsz=args.imgsz, batch=1, dynamic=False, half=False,
            simplify=False, opset=17, device=args.device))
        import onnx
        graph = onnx.load(str(result))
        onnx.checker.check_model(graph)
        # Verify that the engine sees the same mapping after export, not just sidecar text.
        exported = YOLO(str(result), task='detect')
        import numpy as np
        exported.predict(np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8),
                         imgsz=args.imgsz, device='cpu', verbose=False)
        if class_names(exported.names) != names:
            raise ValueError('Exported ONNX class mapping mismatch')
    write_json(output / 'metadata.json', dict(
        schema_version=1, version=args.version, model_file=result.name,
        format=args.format, sha256=sha256(result), source_weights_sha256=sha256(weights),
        names=names, imgsz=args.imgsz, task='detect',
        inference=dict(conf=args.conf, iou=args.iou, batch=1, half=False),
        created_utc=datetime.now(timezone.utc).isoformat(), environment=environment(),
        tensorrt='Build and validate on target GPU/software; PC engine is not portable to NX'))
    print(f'Exported model and class metadata: {output}')


if __name__ == '__main__':
    main()
