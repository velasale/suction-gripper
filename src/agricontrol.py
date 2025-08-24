import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from matplotlib.animation import FFMpegWriter
import glob
from PIL import Image


# ----------------------- IMPORT DATA ------------------------------------#
environment = 'windows'
if environment == 'ubuntu':
    path = r'/media/alejo/Elements/Alejo - Air Pressure Servoing/Joes Videos/'
else:
    path = r'D:/Alejo - Air Pressure Servoing/Joes Videos/'

# folder = 'Alejo - Air Pressure Servoing/Joes Videos/'
# trial = r'20240911__stf_high__offset_2__seq_1__rep_todo/'
trial = r'20240911__stf_high__offset_3__seq_1__rep_todo/'

# --- Images from fixed cam ---
image_folder = os.path.join(path, trial, 'pngs_fixed_cam/')
image_files = sorted(
    glob.glob(os.path.join(image_folder, '*.png')),
    key=lambda f: int(os.path.splitext(os.path.basename(f))[0])
)

# Extract timestamps from filenames (divide by 10 for seconds)
img_timestamps = np.array([
    int(os.path.splitext(os.path.basename(f))[0]) / 10
    for f in image_files
])

# --- Load Air Pressure CSV files ---
df1 = pd.read_csv(path + trial + "gripper-pressure-sc1.csv")
df2 = pd.read_csv(path + trial + "gripper-pressure-sc2.csv")
df3 = pd.read_csv(path + trial + "gripper-pressure-sc3.csv")

# Find global min/max
t_min = min(df1['Time'].min(), df2['Time'].min(), df3['Time'].min())
t_max = max(df1['Time'].max(), df2['Time'].max(), df3['Time'].max())

# Choose step as the smallest median dt across signals
dt = min(
    np.median(np.diff(df1['Time'])),
    np.median(np.diff(df2['Time'])),
    np.median(np.diff(df3['Time']))
)

t_uniform = np.arange(t_min, t_max, dt)

p1_uniform = np.interp(t_uniform, df1['Time'], df1['data'])
p2_uniform = np.interp(t_uniform, df2['Time'], df2['data'])
p3_uniform = np.interp(t_uniform, df3['Time'], df3['data'])

# --- Interpolate onto uniform grid ---
m1 = np.interp(t_uniform, df1['Time'], df1['data'])
m2 = np.interp(t_uniform, df2['Time'], df2['data'])
m3 = np.interp(t_uniform, df3['Time'], df3['data'])

# Use resampled timeline
elapsed_time = t_uniform - t_uniform[0]
n = len(elapsed_time)

magnitudes = np.vstack([m1, m2, m3]).T
magnitudes = magnitudes / 10  # hPa → kPa

# -------------------------- FIGURES SETUP -----------------------------#
# Angles for the 3 vectors
angles_deg = [60, 180, 300]
angles_rad = np.deg2rad(angles_deg)
unit_vectors = np.array([[np.cos(a), np.sin(a)] for a in angles_rad])
colors = ['r', 'g', 'b']

# --- Setup figure with 3 subplots ---
fig, (ax2, ax1, ax3) = plt.subplots(1, 3, figsize=(18, 6))
fig.tight_layout(pad=4)

# --- Subplot 1: Vector plot ---
ax1.set_aspect('equal')
max_val = np.max(magnitudes)
ax1.set_xlim(-max_val*1.2, max_val*1.2)
ax1.set_ylim(-max_val*1.2, max_val*1.2)
ax1.set_title("Air Pressure Vectors")

# Remove box and put axes in the middle
ax1.spines['left'].set_position('center')
ax1.spines['bottom'].set_position('center')
ax1.spines['right'].set_color('none')
ax1.spines['top'].set_color('none')
ax1.xaxis.set_ticks_position('bottom')
ax1.yaxis.set_ticks_position('left')
ax1.grid(False)  # optional: remove grid

# Circles to emulate suction cup engagement
angles = [60, 180, -60]  # degrees
r = 70
coords = [(r*np.cos(np.deg2rad(a)), r*np.sin(np.deg2rad(a))) for a in angles]
colors = ['red', 'orange', 'blue']
circles = []
for (x, y), color in zip(coords, colors):
    c = patches.Circle((x, y), radius=25, color=color, alpha=0.3, visible=False)
    ax1.add_patch(c)
    circles.append(c)

# Dashed line between cups (initially invisible)
cup_line = Line2D([0,0], [0,0], color='gray', linestyle='--', linewidth=1.5, visible=False)
ax1.add_line(cup_line)

projection_line = Line2D([0,0], [0,0], color='purple', linestyle='--', linewidth=1.5, visible=False)
ax1.add_line(projection_line)

# Background image
script_dir = os.path.dirname(os.path.abspath(__file__))
img_path = os.path.join(script_dir, '..', 'media', 'gripper_topview.png')
img_path = os.path.normpath(img_path)  # cleans up the path
img = plt.imread(img_path)
ax1.imshow(img, extent=[-max_val*1.2, max_val*1.2, -max_val*1.2, max_val*1.2], aspect='auto', zorder=0, alpha=0.5)

quiver = ax1.quiver([0,0,0], [0,0,0], [0,0,0], [0,0,0],
                    angles='xy', scale_units='xy', scale=1, color=colors)

texts = [ax1.text(0, 0, '', color=c, fontsize=10, weight='bold') for c in colors]

# THETA (Sum vector)
quiver_sum = ax1.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1, color='k', width=0.02)
txt_sum = ax1.text(0, 0, '', color='k', fontsize=10, weight='bold')

# OMEGA (Perpendicular vector)
quiver_perp = ax1.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1,
                         color='purple', width=0.02, linestyle='dashed')
txt_perp = ax1.text(0, 0, '', color='purple', fontsize=10, weight='bold')


# --- Subplot 2: Time series ---
ax2.plot(elapsed_time, m1/10, color='r', label='scA')
ax2.plot(elapsed_time, m2/10, color='orange', label='scB')
ax2.plot(elapsed_time, m3/10, color='b', label='scC')
ax2.set_xlim(elapsed_time[0], elapsed_time[-1])
ax2.set_ylim(0, np.max(magnitudes)*1.1)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Air Pressure [kPa]")
# Horizontal line at threshold (green dashed)
threshold_kPa = 60
ax2.axhline(y=threshold_kPa, color='green', linestyle='--', linewidth=1.5, label='Engagement Threshold')

# Vertical line showing current time
time_marker = ax2.axvline(elapsed_time[0], color='k', linestyle='--')

ax2.legend()
ax2.grid(True)


# --- Subplot 3: Camera display ---
ax3.axis('off')

# Load one sample image to get dimensions
sample_img = Image.open(image_files[0])
h, w = sample_img.size[1], sample_img.size[0]

img_display = ax3.imshow(sample_img, aspect='equal', extent=[0, w, h, 0])  # extent matches pixel grid
ax3.set_xlim(0, w)
ax3.set_ylim(h, 0)  # flip y to keep image orientation
ax3.set_title("Camera View")


# -------------------------- UPDATE FUNCTION ---------------------------------
def update(frame):
    mags = magnitudes[frame]
    U = [m * uv[0] for m, uv in zip(mags, unit_vectors)]
    V = [m * uv[1] for m, uv in zip(mags, unit_vectors)]

    quiver.set_UVC(U, V)

    # --- Update vector labels ---
    for i, txt in enumerate(texts):
        cup = ['A', 'B', 'C'][i]
        offset = 2 if i == 1 else 0
        txt.set_position((U[i], V[i] + offset))
        txt.set_text(f"sc{cup}: {mags[i]:.2f} kPa")

    # --- Update SUM vector ---
    U_sum = sum(U)
    V_sum = sum(V)
    quiver_sum.set_UVC([U_sum], [V_sum])
    txt_sum.set_position((U_sum, V_sum))
    txt_sum.set_text(f"Sum: {np.linalg.norm([U_sum, V_sum]):.1f} kPa")

    # --- Update circle visibility and get active cups ---
    threshold = 60.0  # kPa
    active_indices = []
    for i, c in enumerate(circles):
        if mags[i] < threshold:
            c.set_visible(True)
            active_indices.append(i)
        else:
            c.set_visible(False)

    # --- Determine omega vector origin ---
    origin_x, origin_y = 0, 0
    cup_line.set_visible(False)
    projection_line.set_visible(False)

    if len(active_indices) == 1:
        # One cup active
        origin_x, origin_y = coords[active_indices[0]]

    elif len(active_indices) == 2:
        # Two cups active
        idx1, idx2 = active_indices
        x1, y1 = coords[idx1]
        x2, y2 = coords[idx2]

        # Show dashed line between cups
        cup_line.set_data([x1, x2], [y1, y2])
        cup_line.set_visible(True)

        line_vec = np.array([x2 - x1, y2 - y1])
        line_start = np.array([x1, y1])
        sum_vec = np.array([U_sum, V_sum])

        # Intersection of sum vector with line between cups
        A = np.column_stack((line_vec, -sum_vec))
        b = -line_start
        try:
            s, t = np.linalg.lstsq(A, b, rcond=None)[0]
        except np.linalg.LinAlgError:
            s = 0.5
        s = np.clip(s, 0, 1)
        origin_vec = line_start + s * line_vec
        origin_x, origin_y = origin_vec

        # Projection line along sum vector from intersection
        projection_line.set_data([origin_x, origin_x + U_sum], [origin_y, origin_y + V_sum])
        projection_line.set_visible(True)

    # --- Omega (perpendicular) vector ---
    U_perp = -V_sum
    V_perp = U_sum
    quiver_perp.set_offsets([[origin_x, origin_y]])
    quiver_perp.set_UVC([U_perp], [V_perp])

    # Place omega text at tip of vector
    tip_x = origin_x + U_perp
    tip_y = origin_y + V_perp
    txt_perp.set_position((tip_x + 2, tip_y + 2))  # small offset to avoid overlap
    txt_perp.set_text("$\\omega$ (⊥ Sum)")

    # --- Move vertical line in time series ---
    time_marker.set_xdata(elapsed_time[frame])

    # --- Update camera image ---
    t = elapsed_time[frame]
    idx = np.argmin(np.abs(img_timestamps - t))
    img = Image.open(image_files[idx])
    img_display.set_data(np.array(img))

    return quiver, quiver_sum, quiver_perp, time_marker, *texts, txt_sum, txt_perp, img_display, cup_line, projection_line


# Compute intervals in ms from timestamps
dt = np.diff(elapsed_time, prepend=elapsed_time[0])  # seconds
intervals = (dt * 1000).astype(int)

# --- Animate ---
# Compute intervals in ms from timestamps
dt = np.diff(elapsed_time, prepend=elapsed_time[0])  # seconds
intervals = (dt * 1000).astype(int)
# Slow down by 4×
speed_factor = 4
interval_slow = intervals.mean() * speed_factor
ani = animation.FuncAnimation(fig, update, frames=n, interval=interval_slow, blit=True)
# Save video
writer = FFMpegWriter(fps=30 / speed_factor, metadata=dict(artist="Me"), bitrate=1800)
ani.save("vectors_with_timeseries.mp4", writer=writer)

plt.show()
