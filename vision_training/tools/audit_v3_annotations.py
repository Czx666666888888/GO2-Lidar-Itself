#!/usr/bin/env python3
"""Audit round-three GT size coverage and render diagnostic annotation previews."""
import sys
import json
from pathlib import Path
from collections import Counter,defaultdict
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_training.dataset import label_for, validate_label
from vision_training.common import write_json
base=Path(__file__).resolve().parents[1]; root=base/'datasets/science_targets_mixed_v3'; out=base/'reports/science_targets_mixed_v3'
records=json.loads((root/'preparation.json').read_text())['records']
sizes=defaultdict(list); groups=Counter(); candidates=[]
for r in records:
 if r.get('annotation_round')!=3:continue
 image=cv2.imread(str(root/r['image']));h,w=image.shape[:2];boxes=validate_label(label_for(root/r['image']),3)
 groups[(r['capture_group'],r['annotation_status'])]+=1
 for c,x,y,bw,bh in boxes:sizes[(r['capture_group'],int(c))].append(min(bw*w,bh*h)*640/max(w,h))
 if r.get('visually_blurred_block') or len(boxes)>1:candidates.append((r,image,boxes))
summary={}
for (g,c),v in sizes.items():summary.setdefault(g,{})[['color_block','rock_compact','rock_spiky'][c]]=dict(boxes=len(v),short_side_lt16=sum(x<16 for x in v),median_short_side_at640=float(np.median(v)))
write_json(out/'round3_gt_sizes.json',dict(groups={g:{s:groups[(g,s)] for s in ('positive','negative')} for g,s in groups},sizes=summary))
# Balance readable blur and multi-target inspection, preserve full image aspect ratio.
blur=[x for x in candidates if x[0].get('visually_blurred_block')]
multi=[x for x in candidates if len(x[2])>1 and x[0]['source_type']=='phone_real']
selected=blur[::max(1,len(blur)//6)][:6]+multi[::max(1,len(multi)//6)][:6]
tiles=[]
for r,im,boxes in selected:
 h,w=im.shape[:2]
 for c,x,y,bw,bh in boxes:
  a=(int((x-bw/2)*w),int((y-bh/2)*h));b=(int((x+bw/2)*w),int((y+bh/2)*h))
  cv2.rectangle(im,a,b,(0,255,0),2);cv2.putText(im,str(int(c)),a,cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
 tile=np.full((390,480,3),255,np.uint8);scaled=cv2.resize(im,(480,round(h*480/w)));tile[:scaled.shape[0]]=scaled
 cv2.putText(tile,r['filename'],(3,380),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1);tiles.append(tile)
while len(tiles)%3:tiles.append(np.full((390,480,3),255,np.uint8))
cv2.imwrite(str(out/'round3_annotation_preview.jpg'),np.concatenate([np.concatenate(tiles[i:i+3],axis=1) for i in range(0,len(tiles),3)]))
print(json.dumps(summary,indent=2))
