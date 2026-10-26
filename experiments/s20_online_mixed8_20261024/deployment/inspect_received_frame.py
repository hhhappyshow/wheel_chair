"""Run the existing YOLO weights on a received real RGB frame; no robot access."""
import argparse
import json
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True)
    a=p.parse_args()
    from ultralytics import YOLO
    model=YOLO(str(a.weights))
    began=time.monotonic()
    prediction=model.predict(str(a.capture/'rgb_last.png'),device='cpu',verbose=False)[0]
    rows=[]
    for box in prediction.boxes:
        index=int(box.cls.item())
        rows.append(dict(class_id=index,label=prediction.names[index],
            confidence=float(box.conf.item()),xyxy=box.xyxy[0].tolist()))
    result=dict(source='actual received D555 last RGB frame',weights=str(a.weights),
        detections=rows,inference_wall_s=time.monotonic()-began,
        device='cpu',motion_sent=False,io_written=False,
        geometric_pose_estimated=False)
    prediction.save(filename=str(a.capture/'yolo_last.png'))
    (a.capture/'yolo_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
