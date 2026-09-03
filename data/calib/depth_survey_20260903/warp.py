import sys, time
sys.path.insert(0,"/home/jake/Downloads/robohand")
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from suction.config import DEPTH_TOPIC
SP="/tmp/claude-1000/-home-jake-Downloads-robohand/5b60d780-36b4-426b-adf1-b2445fd5b59b/scratchpad/"

class T(Node):
    def __init__(self):
        super().__init__("warp"); self.b=CvBridge(); self.f=[]
        self.create_subscription(Image, DEPTH_TOPIC,
            lambda m: self.f.append(self.b.imgmsg_to_cv2(m,"passthrough").astype(np.float32)), 10)
rclpy.init(); t=T(); end=time.monotonic()+8
while time.monotonic()<end and len(t.f)<50: rclpy.spin_once(t,timeout_sec=0.05)
t.destroy_node(); rclpy.shutdown()

A=np.stack(t.f); A[A==0]=np.nan
d=np.nanmedian(A,axis=0)                     # усреднение убирает временной шум
np.save(SP+"depth_avg.npy", d)
print(f"усреднено кадров: {len(t.f)}")

K=np.array([[584.1,0,320.0],[0,583.1,274.8],[0,0,1]])
h,w=d.shape
vs,us=np.mgrid[0:h,0:w].astype(float)
surf=np.isfinite(d)&(d>400)&(d<500)
print(f"поверхность: {surf.sum()} px, дистанция {np.nanmedian(d[surf]):.0f} мм")

X=(us-K[0,2])*d/K[0,0]; Y=(vs-K[1,2])*d/K[1,1]; Z=d
# плоскость строим ТОЛЬКО по центру кадра
cen = surf & (np.abs(us-K[0,2])<120) & (np.abs(vs-K[1,2])<90)
P=np.stack([X[cen],Y[cen],Z[cen]],axis=1)
c=P.mean(axis=0); _,_,vt=np.linalg.svd(P-c,full_matrices=False); n=vt[2]
print(f"плоскость по центру: {cen.sum()} px")

# невязка по ВСЕМУ полю относительно этой плоскости
res=((X-c[0])*n[0]+(Y-c[1])*n[1]+(Z-c[2])*n[2])
res[~surf]=np.nan

print("\nневязка к центральной плоскости, мм (сетка 8x8):")
for by in range(8):
    row=[]
    for bx in range(8):
        blk=res[by*h//8:(by+1)*h//8, bx*w//8:(bx+1)*w//8]
        v=blk[np.isfinite(blk)]
        row.append(f"{np.median(v):+6.1f}" if v.size>50 else "     -")
    print("  "+" ".join(row))

# радиальная ли это систематика?
r=np.sqrt((us-K[0,2])**2+(vs-K[1,2])**2)
m=surf&np.isfinite(res)
print("\nневязка по кольцам от главной точки:")
for lo,hi in [(0,60),(60,120),(120,180),(180,240),(240,300),(300,400)]:
    k=m&(r>=lo)&(r<hi)
    if k.sum()>200:
        print(f"  r={lo:>3}-{hi:<3} px : median={np.median(res[k]):+6.2f} мм  "
              f"std={np.std(res[k]):5.2f}  n={k.sum()}")
rr=r[m]; ee=res[m]
print(f"\nкорреляция невязки с r^2: {np.corrcoef(rr**2, ee)[0,1]:+.3f}")
print("  (|corr| > 0.5 -> радиальная систематика камеры, а не форма листа)")
