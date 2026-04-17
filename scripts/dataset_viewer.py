import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import glob
import sys
import numpy as np

name = sys.argv[1]
paths = glob.glob(f"datasets/{name}/*")

for path in sorted(paths):
  df = pd.read_csv(path)
  T = np.array(df["time"])
  x = np.array(df["torque"])
  y = np.array(df["position"])
  dy = np.array(df["speed"])

  fig = plt.figure(figsize=(10, 6))
  gs = gridspec.GridSpec(3, 2, width_ratios=[1, 2])

  ax1 = fig.add_subplot(gs[0, 0])
  ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)
  ax3 = fig.add_subplot(gs[2, 0], sharex=ax1)

  ax_big = fig.add_subplot(gs[:, 1])

  ax1.plot(T, x)
  ax1.grid()
  ax1.set_title("Torque")
  ax2.plot(T, y - y[0])
  ax2.set_title("Position")
  ax2.grid()
  ax3.plot(T, dy)
  ax3.set_title("Speed")
  ax3.grid()

  ax_big.plot(dy, x, lw=.5)
  ax_big.grid()
  ax_big.set_title("Гистерезис")

  fig.tight_layout()

  plt.get_current_fig_manager().full_screen_toggle()
  plt.show()
    
