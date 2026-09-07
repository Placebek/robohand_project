import sys, time
from pathlib import Path
sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())))
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from suction.config import DEPTH_TOPIC

class T(Node):
    def __init__(self):
        super().__init__("noise"); self.b=CvBridge(); self.f=[]; self.stamps=[]
        self.create_subscription(Image, DEPTH_TOPIC, self.cb, 50)
    def cb(self,m):
        self.f.append(self.b.imgmsg_to_cv2(m,"passthrough").copy())
        self.stamps.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
rclpy.init(); t=T()
end=time.monotonic()+10
while time.monotonic()<end and len(t.f)<60: rclpy.spin_once(t,timeout_sec=0.05)
t.destroy_node(); rclpy.shutdown()

F=t.f
print(f"кадров: {len(F)}")
dup = sum(1 for i in range(1,len(F)) if np.array_equal(F[i],F[i-1]))
print(f"пар подряд идущих БИТ-В-БИТ одинаковых кадров: {dup} из {len(F)-1}")

uniq_stamps = len(set(t.stamps))
print(f"уникальных header.stamp: {uniq_stamps} из {len(t.stamps)}")

st=np.array(sorted(set(t.stamps)))
if len(st)>2:
    dt=np.diff(st)
    print(f"интервал по header.stamp: median={np.median(dt)*1000:.1f} мс "
          f"-> {1/np.median(dt):.1f} Гц")

A=np.stack(F).astype(np.float32)
valid_all = (A>0).all(axis=0)
print(f"\nпикселей, валидных во ВСЕХ {len(F)} кадрах: {valid_all.sum()}")
if valid_all.sum()>100:
    stds = A[:, valid_all].std(axis=0)
    print(f"ПОПИКСЕЛЬНЫЙ шум по времени (мм):")
    print(f"  median={np.median(stds):.3f}  mean={stds.mean():.3f}  "
          f"p95={np.percentile(stds,95):.3f}  max={stds.max():.3f}")
    print(f"  доля пикселей с нулевым разбросом: {100*(stds==0).mean():.1f}%")

    # квантование: какие вообще значения встречаются
    px = A[:, valid_all][:, :5]
    print(f"\nпримеры рядов 5 пикселей по кадрам (первые 8 кадров):")
    for j in range(5):
        print("   ", " ".join(f"{int(px[i,j]):4d}" for i in range(min(8,len(F)))))
