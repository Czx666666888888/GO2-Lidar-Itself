#!/usr/bin/env python3
"""Inference-only CUDA memory/shape probe; never trains or exports weights."""
import argparse
import gc
import json
from pathlib import Path
import sys
import time
import numpy as np
import torch
from ultralytics import YOLO
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from vision_training.common import sha256, write_json


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights',type=Path,required=True);p.add_argument('--image',type=Path,required=True)
    p.add_argument('--square',action='store_true');p.add_argument('--imgsz',type=int,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    digest=sha256(args.weights)
    torch.cuda.empty_cache();torch.cuda.reset_peak_memory_stats()
    model=YOLO(str(args.weights))
    source=np.zeros((args.imgsz,args.imgsz,3),np.uint8) if args.square else str(args.image)
    kwargs=dict(imgsz=args.imgsz,device=0,batch=1,half=False,verbose=False,save=False)
    for _ in range(5):model.predict(source,**kwargs)
    gc.collect();torch.cuda.empty_cache();torch.cuda.reset_peak_memory_stats()
    shapes=[]
    handle=model.predictor.model.model.register_forward_pre_hook(lambda module,inputs:shapes.append(list(inputs[0].shape)))
    times=[]
    for _ in range(20):
        torch.cuda.synchronize();started=time.perf_counter()
        result=model.predict(source,**kwargs)[0]
        torch.cuda.synchronize();times.append((time.perf_counter()-started)*1000)
        del result
    handle.remove()
    assert sha256(args.weights)==digest
    assert shapes and all(s[:2]==[1,3] and max(s[2:])==args.imgsz and all(x%32==0 for x in s[2:]) for s in shapes)
    report=dict(imgsz=args.imgsz,input_shapes=sorted({tuple(s) for s in shapes}),images=1,repeats=20,warmup=5,
        batch=1,fp16=bool(model.predictor.model.fp16),square_input=args.square,gpu=torch.cuda.get_device_name(0),
        peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
        peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20,
        mean_wall_ms=float(np.mean(times)),p95_wall_ms=float(np.percentile(times,95)),
        weights_sha256=digest,status='PASS',training_memory='NOT VERIFIED: inference only',
        scope='single image, recorded actual auto-padding tensor, PyTorch allocator; excludes driver/context/other process VRAM')
    write_json(args.output,report);print(json.dumps(report,indent=2))

if __name__=='__main__':main()
