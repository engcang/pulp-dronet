"""Image adaptation and continuous DroNet outputs -> AirStack trajectory.

No depth, GT geometry, target waypoint or external obstacle detector is used.
Positive model steering means positive FLU yaw (left).
"""
import math
import numpy as np


def preprocess(bgr, camera_k=None, field_of_view=60.):
    """Square central rectilinear view, grayscale uint8 -> float32 [0,1].

    Upstream consumes a 200x200 center crop from its Himax camera. Our ZED
    has a different focal length; select an explicit, recorded angular crop.
    No histogram equalization or per-image normalization.
    """
    import cv2  # Worker runtime dependency; controller contract tests need no OpenCV.
    h, w = bgr.shape[:2]
    focal = float(camera_k[0]) if camera_k is not None else w/2
    side = min(h, w, max(2, round(2*focal*math.tan(math.radians(field_of_view)/2))))
    x, y = (w-side)//2, (h-side)//2
    gray = cv2.cvtColor(bgr[y:y+side, x:x+side], cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (200, 200), interpolation=cv2.INTER_AREA if side>200 else cv2.INTER_LINEAR)
    return gray, gray.astype(np.float32)[None, None]/255., [x,y,side,side]


class Policy:
    def __init__(self, maximum_speed=.35, yaw_gain=120., smoothing=.3):
        self.maximum_speed, self.yaw_gain = maximum_speed, math.radians(yaw_gain)
        self.smoothing = smoothing
        self.speed = self.yaw = 0.

    def step(self, steering, collision):
        if not math.isfinite(steering) or not math.isfinite(collision):
            raise ValueError('Nonfinite network output')
        # Original IoTJ navigation controller: v=(1-p)*v_max, yaw=steer*120deg/s,
        # alpha=.3 old + .7 new. Clamp only physical command bounds.
        target_speed = (1-float(np.clip(collision,0,1)))*self.maximum_speed
        target_yaw = float(np.clip(steering,-1,1))*self.yaw_gain
        a = self.smoothing
        self.speed = a*self.speed+(1-a)*target_speed
        self.yaw = a*self.yaw+(1-a)*target_yaw
        return self.speed, self.yaw


def trajectory(camera_pose, speed, yaw_rate, horizon=2., count=21):
    pose = np.asarray(camera_pose,dtype=float).reshape(4,4)
    heading = math.atan2(pose[1,2],pose[0,2])
    t = np.linspace(0,horizon,count)
    if abs(yaw_rate)<1e-6:
        forward, left = speed*t, np.zeros_like(t)
    else:
        forward = speed*np.sin(yaw_rate*t)/yaw_rate
        left = speed*(1-np.cos(yaw_rate*t))/yaw_rate
    x = pose[0,3]+forward*math.cos(heading)-left*math.sin(heading)
    y = pose[1,3]+forward*math.sin(heading)+left*math.cos(heading)
    return np.column_stack((x,y,np.full(count,pose[2,3]),heading+yaw_rate*t))


class HeadingReference:
    """Convert yaw *rate* to a persistent heading reference in simulation time.

    Reanchoring every new trajectory to measured yaw loses accumulated rotation
    under tracking lag. Bound the reference error to prevent integral windup.
    This is a control-interface conversion, not another navigation policy.
    """
    def __init__(self):
        self.heading = self.stamp = None

    def update(self, measured, yaw_rate, stamp):
        if self.heading is None:self.heading=measured
        dt=0. if self.stamp is None else max(0.,min(.5,stamp-self.stamp))
        self.stamp=stamp
        self.heading+=yaw_rate*dt
        error=math.atan2(math.sin(self.heading-measured),math.cos(self.heading-measured))
        self.heading=measured+float(np.clip(error,-math.pi/4,math.pi/4))
        return self.heading
