"""Official pretrained float PULP-DroNet v3 worker for the AirStack HTTP bridge."""
import argparse
import json
import os
from pathlib import Path
import struct
import time
import urllib.request
import urllib.error

import cv2
import numpy as np
import torch

from .model import dronet, Depthwise_Separable
from .policy import preprocess, Policy, trajectory, HeadingReference


def fetch(server):
    with urllib.request.urlopen(server+'/frame',timeout=3) as r:
        data=r.read()
    if len(data)<4:raise ValueError('Truncated frame')
    size=struct.unpack('!I',data[:4])[0]
    meta=json.loads(data[4:4+size]);payload=data[4+size:]
    frame=cv2.imdecode(np.frombuffer(payload[:meta['jpeg_bytes']],np.uint8),cv2.IMREAD_COLOR)
    if frame is None:raise ValueError('Invalid RGB JPEG')
    return meta,frame


def post(server,path,payload):
    req=urllib.request.Request(server+path,data=json.dumps(payload).encode(),
                               headers={'Content-Type':'application/json'},method='POST')
    with urllib.request.urlopen(req,timeout=3) as r:return json.load(r)


def save_preview(bgr,gray,roi,info):
    folder=os.environ.get('WS2_INFERENCE_DIR')
    if not folder:return
    out=Path(folder);out.mkdir(exist_ok=True,parents=True)
    display=np.zeros((480,960,3),np.uint8)
    image=bgr.copy();x,y,w,h=roi;cv2.rectangle(image,(x,y),(x+w,y+h),(0,220,255),2)
    display[:300,:480]=cv2.resize(image,(480,300))
    display[:300,480:780]=cv2.cvtColor(cv2.resize(gray,(300,300)),cv2.COLOR_GRAY2BGR)
    lines=['PULP-DroNet v3 | camera RGB -> grayscale crop -> steering + collision',
           f"Steering {info['steering']:+.3f} | predicted collision probability {info['collision_probability']:.3f}",
           f"Command {info['speed']:.3f} m/s | yaw {np.degrees(info['yaw_rate']):+.1f} deg/s | inference {info['inference_ms']:.1f} ms",
           f"Frame {info['sequence']} | angular crop {info['input_fov_deg']:.0f} deg | trajectory accepted {info['accepted']}"]
    for i,line in enumerate(lines):cv2.putText(display,line,(12,335+35*i),cv2.FONT_HERSHEY_SIMPLEX,.57,(235,235,235),1,cv2.LINE_AA)
    ok,jpg=cv2.imencode('.jpg',display,[cv2.IMWRITE_JPEG_QUALITY,85])
    if ok:
        tmp=out/'pulp_dronet.jpg.tmp';tmp.write_bytes(jpg.tobytes());tmp.replace(out/'pulp_dronet.jpg')
    tmp=out/'pulp_dronet.json.tmp';tmp.write_text(json.dumps(info));tmp.replace(out/'pulp_dronet.json')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--server',default='http://127.0.0.1:8765')
    p.add_argument('--variant',choices=['resblock','tiny'],default='resblock')
    p.add_argument('--weights',help='Optional checkpoint override; strict architecture loading is always enabled')
    p.add_argument('--execute',action='store_true');p.add_argument('--headless',action='store_true')
    p.add_argument('--maximum-speed',type=float,default=.35)
    p.add_argument('--yaw-gain',type=float,default=120.)
    p.add_argument('--smoothing',type=float,default=.3)
    p.add_argument('--input-fov',type=float,default=60.)
    p.add_argument('--trajectory-horizon',type=float,default=2.)
    p.add_argument('--rate',type=float,default=10.)
    p.add_argument('--yaw-reference',choices=['trajectory','integrated'],default='integrated')
    args=p.parse_args()
    torch.set_num_threads(2)
    # This 320k-parameter model is fast on CPU; avoids unnecessary GPU contention.
    model=(dronet() if args.variant=='resblock' else dronet(depth_mult=.125,block_class=Depthwise_Separable,bypass=False)).eval()
    if not args.weights:
        filename='pulp-dronet-v3-resblock-1.0.pth' if args.variant=='resblock' else 'tiny-pulp-dronet-v3-dw-pw-0.125.pth'
        args.weights=str(Path(__file__).parents[1]/'tiny-pulp-dronet-v3/model'/filename)
    model.load_state_dict(torch.load(args.weights,map_location='cpu',weights_only=True),strict=True)
    policy=Policy(args.maximum_speed,args.yaw_gain,args.smoothing)
    heading_reference=HeadingReference()
    sequence=None;last=0.;errors=0
    print(f'PULP-DroNet v3: official float {args.variant} checkpoint; RGB-only reactive policy',flush=True)
    try:
        while True:
            time.sleep(max(0,1/args.rate-(time.monotonic()-last)))
            try:meta,bgr=fetch(args.server)
            except (urllib.error.URLError,TimeoutError) as exc:
                errors+=1
                if errors>=10:raise RuntimeError('Bridge unavailable for 10 consecutive reads') from exc
                time.sleep(.2);continue
            errors=0
            if meta['sequence']==sequence:time.sleep(.01);continue
            sequence=meta['sequence'];last=time.monotonic()
            gray,tensor,roi=preprocess(bgr,meta['k'],args.input_fov)
            start=time.monotonic()
            with torch.inference_mode():steer,collision=model(torch.from_numpy(tensor))
            latency=(time.monotonic()-start)*1000
            steer,collision=float(steer.item()),float(collision.item())
            speed,yaw=policy.step(steer,collision)
            points=trajectory(meta['t_world_camera'],speed,yaw,args.trajectory_horizon)
            measured_heading=float(points[0,3])
            reference=heading_reference.update(measured_heading,yaw,float(meta['stamp']))
            if args.yaw_reference=='integrated':
                # Heading is already integrated in time. Do not integrate it
                # again through distance-timed waypoints: that becomes unstable
                # near zero forward speed (the controller has a velocity floor).
                points[:,3]=reference
            response=post(args.server,'/trajectory',{'command_index':0,'velocity':max(speed,.001),
                          'execute':args.execute,'replace':True,'waypoints':points.tolist()})
            info=dict(planner='pulp_dronet',model_variant=args.variant,run_id=os.environ.get('WS2_RUN_ID'),wall_time=time.time(),
                      sequence=sequence,sim_stamp=meta['stamp'],inference_ms=latency,steering=steer,
                      collision_probability=collision,speed=speed,yaw_rate=yaw,input_fov_deg=args.input_fov,
                      actual_input_fov_deg=float(np.degrees(2*np.arctan(roi[2]/(2*meta['k'][0])))),
                      yaw_reference_mode=args.yaw_reference,reference_heading_rad=reference,
                      camera_heading_rad=float(np.arctan2(meta['t_world_camera'][6],meta['t_world_camera'][2])),
                      crop_roi=roi,accepted=response.get('accepted',False))
            save_preview(bgr,gray,roi,info)
            print(json.dumps(info),flush=True)
    except KeyboardInterrupt:pass
    finally:
        if args.execute:
            try:post(args.server,'/pause',{})
            except (OSError,ValueError):pass


if __name__=='__main__':main()
