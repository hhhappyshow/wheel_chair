"""Offline detector comparison on saved camera images; no device access."""
import argparse
import json
from pathlib import Path
import time

import cv2
from ultralytics import YOLO

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--image', required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--models', nargs='+', required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
image = cv2.imread(a.image)
rows = []
for weight in a.models:
    model = YOLO(weight)
    for size in (640, 960):
        began = time.perf_counter()
        result = model.predict(image, device='cpu', conf=.15, imgsz=size,
                               retina_masks=True, verbose=False)[0]
        row = dict(model=weight, imgsz=size, wall_s=time.perf_counter()-began,
                   detections=[dict(name=model.names[int(b.cls.item())],
                                    confidence=float(b.conf.item()),
                                    xyxy=b.xyxy[0].tolist()) for b in result.boxes])
        rows.append(row)
        cv2.imwrite(str(a.output / f'{Path(weight).stem}_{size}.png'), result.plot())
        (a.output / 'results.json').write_text(json.dumps(rows, indent=2))
        print(json.dumps(row), flush=True)
