#!/usr/bin/env python3
"""Report actual runtime packages and test CUDA with a real tensor operation."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.common import environment, write_json  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output')
    args = parser.parse_args()
    result = environment()
    try:
        result['nvidia_smi'] = subprocess.check_output([
            'nvidia-smi', '--query-gpu=name,driver_version,memory.total',
            '--format=csv,noheader'], text=True).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        result['nvidia_smi_error'] = str(exc)
    import torch
    import cv2
    result.update(torch_cuda_runtime=torch.version.cuda, cuda_available=torch.cuda.is_available(),
                  opencv=cv2.__version__)
    if torch.cuda.is_available():
        a = torch.arange(16, device='cuda', dtype=torch.float32).reshape(4, 4)
        actual = (a @ a.T).cpu()
        expected = (a.cpu() @ a.cpu().T)
        torch.testing.assert_close(actual, expected)
        torch.cuda.synchronize()
        result.update(gpu=torch.cuda.get_device_name(0), cuda_matmul='PASS')
    else:
        result['cuda_matmul'] = 'NOT VERIFIED'
    print(json.dumps(result, indent=2))
    if args.output:
        write_json(args.output, result)


if __name__ == '__main__':
    main()
