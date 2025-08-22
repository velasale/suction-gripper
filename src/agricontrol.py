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

img = plt.imread('/path/to/your/image.png')  

quiver = ax1.quiver([0,0,0], [0,0,0], [0,0,0], [0,0,0],
                    angles='xy', scale_units='xy', scale=1, color=colors)

texts = [ax1.text(0, 0, '', color=c, fontsize=10, weight='bold') for c in colors]

# Sum vector
quiver_sum = ax1.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1, color='k', width=0.02)
txt_sum = ax1.text(0, 0, '', color='k', fontsize=10, weight='bold')


# --- Subplot 2: Time series ---
ax2.plot(elapsed_time, m1, color='r', label='scA')
ax2.plot(elapsed_time, m2, color='g', label='scB')
ax2.plot(elapsed_time, m3, color='b', label='scC')
ax2.set_xlim(elapsed_time[0], elapsed_time[-1])
ax2.set_ylim(np.min(magnitudes)*0.9, np.max(magnitudes)*1.1)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Pressure [hPa]")
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
            offset = 15
        else:
            cup = 'C'
            offset = 0
        txt.set_position((U[i], V[i]+offset))
        txt.set_text(f"sc{cup}: {mags[i]:.1f} hPa")
    
    U_sum = sum(U)
    V_sum = sum(V)
    quiver_sum.set_UVC([U_sum], [V_sum])

    txt_sum.set_position((U_sum, V_sum))
    txt_sum.set_text(f"Sum: {np.linalg.norm([U_sum, V_sum]):.1f} hPa")

    ax1.set_title(f"Timestamp: {elapsed_time[frame]:.3f}")

    # Move vertical line
    time_marker.set_xdata(elapsed_time[frame])

    return quiver, quiver_sum, time_marker, *texts, txt_sum

# Compute intervals in ms from timestamps
dt = np.diff(timestamps, prepend=timestamps[0])  # seconds
intervals = (dt * 1000).astype(int)

# --- Animate ---
ani = animation.FuncAnimation(fig, update, frames=n,
                              interval=intervals.mean(), blit=True)

# Save video
writer = FFMpegWriter(fps=30, metadata=dict(artist="Me"), bitrate=1800)
ani.save("vectors_with_timeseries.mp4", writer=writer)

plt.show()
