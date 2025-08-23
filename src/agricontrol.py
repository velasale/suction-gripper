import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FFMpegWriter

path = '/media/alejo/Elements/Alejo - Air Pressure Servoing/Joes Videos/'
trial = '20240911__stf_high__offset_2__seq_1__rep_todo/'

# --- Load the three CSV files ---
df1 = pd.read_csv(path + trial + "gripper-pressure-sc1.csv")
df2 = pd.read_csv(path + trial + "gripper-pressure-sc2.csv")
df3 = pd.read_csv(path + trial + "gripper-pressure-sc3.csv")

# Keep common length
n = min(len(df1), len(df2), len(df3))
t1, m1 = df1['Time'][:n].values, df1['data'][:n].values
t2, m2 = df2['Time'][:n].values, df2['data'][:n].values
t3, m3 = df3['Time'][:n].values, df3['data'][:n].values

# Use first signal’s timestamps as reference
timestamps = t1
elapsed_time = timestamps - timestamps[0]  # <--- elapsed time in seconds
magnitudes = np.vstack([m1, m2, m3]).T

magnitudes = magnitudes / 10  # hPa → kPa

# Angles for the 3 vectors
angles_deg = [60, 180, 300]
angles_rad = np.deg2rad(angles_deg)
unit_vectors = np.array([[np.cos(a), np.sin(a)] for a in angles_rad])
colors = ['r', 'g', 'b']

# --- Setup figure with 2 subplots ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
fig.tight_layout(pad=4)

# --- Subplot 1: Vector plot ---
ax1.set_aspect('equal')
max_val = np.max(magnitudes)
ax1.set_xlim(-max_val*1.2, max_val*1.2)
ax1.set_ylim(-max_val*1.2, max_val*1.2)
ax1.set_title("Air Pressure Vectors")
ax1.grid(True)


script_dir = os.path.dirname(os.path.abspath(__file__))
img_path = os.path.join(script_dir, '..', 'media', 'gripper_topview.png')
img_path = os.path.normpath(img_path)  # cleans up the path
img = plt.imread(img_path)  
ax1.imshow(img, extent=[-max_val*1.2, max_val*1.2, -max_val*1.2, max_val*1.2], aspect='auto', zorder=0, alpha=0.5)

quiver = ax1.quiver([0,0,0], [0,0,0], [0,0,0], [0,0,0],
                    angles='xy', scale_units='xy', scale=1, color=colors)

texts = [ax1.text(0, 0, '', color=c, fontsize=10, weight='bold') for c in colors]

# Sum vector
quiver_sum = ax1.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1, color='k', width=0.02)
txt_sum = ax1.text(0, 0, '', color='k', fontsize=10, weight='bold')

# --- Perpendicular vector ---
quiver_perp = ax1.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1, 
                         color='purple', width=0.02, linestyle='dashed')
txt_perp = ax1.text(0, 0, '', color='purple', fontsize=10, weight='bold')



# --- Subplot 2: Time series ---
ax2.plot(elapsed_time, m1/10, color='r', label='scA')
ax2.plot(elapsed_time, m2/10, color='g', label='scB')
ax2.plot(elapsed_time, m3/10, color='b', label='scC')
ax2.set_xlim(elapsed_time[0], elapsed_time[-1])
ax2.set_ylim(np.min(magnitudes)*0.9, np.max(magnitudes)*1.1)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Pressure [kPa]")
ax2.legend()
ax2.grid(True)

# Vertical line showing current time
time_marker = ax2.axvline(timestamps[0], color='k', linestyle='--')

# --- Update function ---
def update(frame):
    mags = magnitudes[frame]
    U = [m * uv[0] for m, uv in zip(mags, unit_vectors)]
    V = [m * uv[1] for m, uv in zip(mags, unit_vectors)]    
    
    quiver.set_UVC(U, V)   

    # Update labels near the vector tips
    for i, txt in enumerate(texts):

        if i == 0:
            cup = 'A'
            offset = 0
        elif i == 1:
            cup = 'B'
            offset = 2
        else:
            cup = 'C'
            offset = 0
        txt.set_position((U[i], V[i]+offset))
        txt.set_text(f"sc{cup}: {mags[i]:.2f} kPa")
    
    U_sum = sum(U)
    V_sum = sum(V)
    quiver_sum.set_UVC([U_sum], [V_sum])
    txt_sum.set_position((U_sum, V_sum))
    txt_sum.set_text(f"Sum: {np.linalg.norm([U_sum, V_sum]):.1f} kPa")

    # Perpendicular vector
    U_perp = -V_sum
    V_perp = U_sum
    quiver_perp.set_UVC([U_perp], [V_perp])
    txt_perp.set_position((U_perp + 0.02*max_val, V_perp + 0.02*max_val))
    txt_perp.set_text(f"$\omega$ (⊥ Sum)")

    # ax1.set_title(f"Timestamp: {elapsed_time[frame]:.3f}")

    # Move vertical line
    time_marker.set_xdata(elapsed_time[frame])

    return quiver, quiver_sum, quiver_perp, time_marker, *texts, txt_sum, txt_perp

# Compute intervals in ms from timestamps
dt = np.diff(timestamps, prepend=timestamps[0])  # seconds
intervals = (dt * 1000).astype(int)

# --- Animate ---
speed_factor = 4

# Compute intervals in ms from timestamps
dt = np.diff(timestamps, prepend=timestamps[0])  # seconds
intervals = (dt * 1000).astype(int)

# Slow down by 4×
interval_slow = intervals.mean() * speed_factor

ani = animation.FuncAnimation(fig, update, frames=n,
                              interval=interval_slow, blit=True)

# Save video
writer = FFMpegWriter(fps=30 / speed_factor, metadata=dict(artist="Me"), bitrate=1800)
ani.save("vectors_with_timeseries.mp4", writer=writer)

plt.show()
