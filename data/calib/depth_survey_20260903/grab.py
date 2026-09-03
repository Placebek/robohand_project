import sys, time
sys.path.insert(0,"/home/jake/Downloads/robohand")
import cv2, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from suction.config import DEPTH_TOPIC
SP="/tmp/claude-1000/-home-jake-Downloads-robohand/5b60d780-36b4-426b-adf1-b2445fd5b59b/scratchpad/"

class T(Node):
    def __init__(self):
        super().__init__("grab"); self.b=CvBridge(); self.frames=[]
        self.create_subscription(Image, DEPTH_TOPIC,
            lambda m: self.frames.append(self.b.imgmsg_to_cv2(m,"passthrough")), 10)
rclpy.init(); t=T()
end=time.monotonic()+8
while time.monotonic()<end and len(t.frames)<40: rclpy.spin_once(t,timeout_sec=0.1)
t.destroy_node(); rclpy.shutdown()

if not t.frames: print("кадров нет"); sys.exit(1)
d = t.frames[-1]
v = d[d>0]
print(f"кадров набрано: {len(t.frames)}")
print(f"валидных пикселей: {v.size}/{d.size} = {100*v.size/d.size:.1f}%")
if v.size==0: sys.exit(0)
print(f"глубина мм: min={v.min()} max={v.max()} median={np.median(v):.0f}")
print()
print("гистограмма по дальностям:")
for lo,hi in [(1,200),(200,300),(300,400),(400,500),(500,700),(700,1000),(1000,5000)]:
    n=int(((d>=lo)&(d<hi)).sum())
    if n: print(f"  {lo:>4}-{hi:<5} мм : {n:>6} px ({100*n/d.size:>4.1f}%)")
print()
print("карта валидности 8x8 (доля валидных пикселей в блоке, %):")
h,w = d.shape
for by in range(8):
    row=[]
    for bx in range(8):
        blk = d[by*h//8:(by+1)*h//8, bx*w//8:(bx+1)*w//8]
        row.append(f"{100*(blk>0).mean():>4.0f}")
    print("   "+" ".join(row))
print()
print("медианная глубина по блокам 8x8 (мм, '-' = нет данных):")
for by in range(8):
    row=[]
    for bx in range(8):
        blk = d[by*h//8:(by+1)*h//8, bx*w//8:(bx+1)*w//8]
        vv = blk[blk>0]
        row.append(f"{np.median(vv):>5.0f}" if vv.size>20 else "    -")
    print("   "+" ".join(row))

# стабильность во времени по центральному блоку
c = [f[h//2-40:h//2+40, w//2-40:w//2+40] for f in t.frames]
meds = [np.median(x[x>0]) for x in c if (x>0).sum()>100]
if len(meds)>3:
    print(f"\nцентр кадра по {len(meds)} кадрам: median={np.median(meds):.0f} мм, "
          f"размах={max(meds)-min(meds):.0f} мм, std={np.std(meds):.2f} мм")
np.save(SP+"depth.npy", d)
