import matplotlib.pyplot as plt
import pandas as pd
import glob
import sys
import numpy as np

DATASET = 'harmonics'

df = pd.DataFrame()

pos  = [0]
vel  = [0]
act  = [0]
time = [0]

for path in sorted(glob.glob(f"datasets/raw/{DATASET}/*")):
    df = pd.read_csv(path)

    pos.extend(np.array(df["position"]) + pos[-1] - df["position"][0])
    vel.extend(df["speed"])
    act.extend(df["torque"])
    
    time.extend(np.array(df["time"]) + time[-1] - df["time"][0])

pos.pop(0)
vel.pop(0)
act.pop(0)
time.pop(0)

plt.plot(pos)
plt.xlim()
plt.show()

plt.plot(vel)
plt.xlim()
plt.show()