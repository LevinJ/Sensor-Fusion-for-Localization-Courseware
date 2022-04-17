from poseinfo import PoseInfo
import numpy as np


src = []
src.append([1, 1, 1])
src.append([2, 5, 2])
src.append([3, 3, 3])
src = np.array(src)

ypr = np.zeros(3)
t= [2, 5, 0]
Tts = PoseInfo().construct_fromyprt(ypr = ypr, t= t)

tgt = []
for s in src:
    t = Tts.project_point(s)
    tgt.append(t)
tgt = np.array(tgt)
print(tgt)
